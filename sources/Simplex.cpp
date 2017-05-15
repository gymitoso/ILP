#include <Eigen>
#include "Simplex.h"
#include "exception.h"

using namespace Eigen;

/**
 * Construtor
 *
 * @param int mode Pode ser: SIMPLEX_MINIMIZE, SIMPLEX_MAXIMIZE
 * @param const VectorXd &objectiveFunction Os coeficientes da fun��o objetivo.
 * @param const VectorXd &relations Os sinais de rela��o das restri��es {0 -> <=; 1 -> >=; 2 -> =}.
 * @param const MatrixXd &constraints Matriz com todas as restri��es.
 * @returns Simplex
 */
Simplex::Simplex(int mode, const VectorXd &objectiveFunction, const MatrixXd &constraints, const VectorXd &relations) {
	__int64 constantColumn, temp;
	this->foundSolution = false;
	this->optimum = 0;
	this->numberOfVariables = objectiveFunction.rows();

	/*
		Valida as entradas
	*/
	if (mode != SIMPLEX_MINIMIZE && mode != SIMPLEX_MAXIMIZE) {
		throw(new Exception("Simplex: modo inv�lido!."));
	}

	if (objectiveFunction.rows() < 1) {
		throw(new Exception("Simplex: Deve conter pelo menos um coeficiente na fun��o objetivo."));
	}

	if (constraints.rows() < 1) {
		throw(new Exception("Simplex: Deve ter pelo menos uma restri��o."));
	}

	if (constraints.cols() != objectiveFunction.rows() + 1) {
		throw(new Exception("Simplex: n�mero de coeficientes da fun��o objetivo diferente do n�mero de coeficientes de restri��o."));
	}

	if (relations.rows() != constraints.rows()) {
		throw(new Exception("Simplex: n�mero de rela��es diferentes do n�mero de restri��es."));
	}

	for (__int64 i = 0; i < this->numberOfVariables; i++) {
		if (objectiveFunction(i) == 0) {
			throw(new Exception("Simplex: Um dos coeficientes da fun��o objetivo � zero."));
		}
	}

	temp = constraints.cols() - 1;
	for (__int64 i = 0; i < constraints.rows(); i++) {
		if (constraints(i, temp) < 0) {
			throw(new Exception("Simplex: Todo lado direito da tabela de restri��es deve ser n�o negativo."));
		}
	}

	/*
		Constr�i o tableau
	*/
	int numberOfArtificials = 0;
    for(__int64 i = 0; i < relations.rows(); i++) {
        if(relations(i) != 0) {
            numberOfArtificials++;
        }
    }
    if(numberOfArtificials > 0) {
        this->tableau.resize(constraints.rows() + 2, this->numberOfVariables + constraints.rows() + numberOfArtificials + 1);

        this->tableau <<    MatrixXd::Zero(1, constraints.rows() + this->numberOfVariables + numberOfArtificials + 1),
                            -objectiveFunction.transpose(), MatrixXd::Zero(1, constraints.rows() + numberOfArtificials + 1),
                            constraints.leftCols(this->numberOfVariables),	MatrixXd::Identity(constraints.rows(), constraints.rows()), MatrixXd::Zero(constraints.rows(), numberOfArtificials), constraints.rightCols(1);
        temp = 0;
        for(__int64 i = 2; i < this->tableau.rows(); i++) {
            if(relations(i-2) == 1) {
                this->tableau(i, this->numberOfVariables + i - 2) = -1;
                this->tableau.row(0)+=this->tableau.row(i)*relations(i-2);
                this->tableau(i, this->numberOfVariables + constraints.rows() + temp) = 1;
                temp++;
            } else if (relations(i-2) == 2) {
                removeColumn(this->numberOfVariables + i - 2);
                this->tableau.row(0)+=this->tableau.row(i)*relations(i-2);
                this->tableau(i, this->numberOfVariables + constraints.rows() + temp) = 1;
                temp++;
            }
        }
    } else {
        this->tableau.resize(constraints.rows() + 1, this->numberOfVariables + constraints.rows() + 1);

		this->tableau <<	-objectiveFunction.transpose(), MatrixXd::Zero(1, constraints.rows() + 1),
							constraints.leftCols(this->numberOfVariables),	MatrixXd::Identity(constraints.rows(), constraints.rows()), constraints.rightCols(1);
    }
    /*
        Primeira fase Simplex
    */
    if(numberOfArtificials > 0) {
        //caso a minimiza��o n�o seja 0, n�o existe solu��o para a PLI
        if (!this->simplexSolver(this->numberOfVariables, SIMPLEX_MINIMIZE, FIRST_PHASE) || this->tableau(0, this->tableau.cols() - 1) != 0) {
            return;	// Sem solu��o
        }
        //remove a primeira linha criada para cancelar as vari�veis artificiais
        removeRow(0);
        //remove as colunas das vari�veis artificiais
        for(__int64  i = 0; i < numberOfArtificials; i++) {
            removeColumn(this->tableau.cols() - 2);
        }
    }

    /*
        Segunda fase Simplex
    */
    if (!this->simplexSolver(this->numberOfVariables, mode, SECOND_PHASE)) {
			return;	// Sem solu��o
    }

	/*
		Busca solu��o
	*/
	constantColumn = this->tableau.cols() - 1;
	this->solution.resize(this->numberOfVariables);

	for (__int64 i = 0; i < this->numberOfVariables; i++) {
        temp = this->getPivotRow(i);
        if (temp > 0) {
            // Variavel basica
            this->solution(i) = this->tableau(temp, constantColumn);
        } else {
            // Variavel n�o basica
            this->solution(i) = 0;
        }
    }
    this->foundSolution = true;
    this->optimum = this->tableau(0, constantColumn);
}

/**
 * Retorna true se a solu��o foi encontrada.
 * Retorna false caso contr�rio.
 *
 * @returns boolean
 */
bool Simplex::hasSolution() {
	return this->foundSolution;
}

/**
 * Retorna o valor �timo da fun��o objetivo maximizado ou minimizado
 *
 * @returns double
 */
double Simplex::getOptimum() {
	return this->optimum;
}

/**
 * Retorna o valor das vari�veis para a solu��o encontrada.
 *
 * return VectorXd
 */
VectorXd Simplex::getSolution() {
	return this->solution;
}

/**
 * Busca na matriz tableau a solu��o.
 *
 * @param __int64 variableNum (O n�mero de vari�veis).
 * @param int mode (Se � para maximizar ou minimizar).
 * @param int phase (Se � a primeira ou segunda fase do m�todo simplex).
 * @returns bool Retorna true se uma solu��o foi encontrada, false caso n�o seja.
 */
bool Simplex::simplexSolver(__int64 variableNum, int mode, int phase) {
	MatrixXd::Index pivotColumn;
	__int64 pivotRow;

	while (true) {
		/*
			Busca a coluna pivotal
		*/
		if (mode == SIMPLEX_MAXIMIZE) {
            this->tableau.row(0).minCoeff(&pivotColumn);
            if (this->tableau(0, pivotColumn) >= 0) {
                //se o menor valor for maior ou igual a zero ent�o a solu��o foi encontrada
                break;
            }
        } else {
             this->tableau.row(0).leftCols(this->tableau.cols()-1).maxCoeff(&pivotColumn);
             if (this->tableau(0, pivotColumn) <= 0) {
                //se o maior valor for menor ou igual a zero ent�o a solu��o foi encontrada
                break;
            }
        }

		/*
			Busca a linha pivotal
		*/
		pivotRow = this->findPivot_min(pivotColumn, phase);
		if (pivotRow == -1) {
			//sem solu��o
			return false;
		}

		/*
			Opera��o com o piv�
		*/
		this->tableau.row(pivotRow) /= this->tableau(pivotRow, pivotColumn);
		this->tableau(pivotRow, pivotColumn) = 1;	// Para problemas de precis�o
		for (int i = 0; i < this->tableau.rows(); i++) {
			if (i == pivotRow) continue;

			this->tableau.row(i) -= this->tableau.row(pivotRow) * this->tableau(i, pivotColumn);
			this->tableau(i, pivotColumn) = 0;	// Para problemas de precis�o
		}
	}

	return true;
}

/**
 * Busca pela linha pivotal a partir da coluna pivotal
 * Tenta achar a menor propor��o (ratio) n�o negativo.
 * Retorna -1 se todas as propor��es s�o negativas ou os candidatos a piv� sejam 0.
 *
 * @param __int64 column coluna pivotal
 * @param int phase indica a fase do m�todo simplex
 * @returns __int64 Retorna o �ndice da linha pivotal ou -1 se n�o achou.
 */
__int64 Simplex::findPivot_min(__int64 column, int phase) {
	__int64 minIndex = -1;
	__int64 constantColumn = this->tableau.cols() - 1;
	double minRatio = 0;
	double minConstant = 0;	// Para "0/negativo < 0/positivo".
	double ratio;
	__int64 i = 1;
	__int64 rowNum = this->tableau.rows();

    if(phase == FIRST_PHASE) {
        i++;
    }

	for (i; i < rowNum; i++) {
		if (this->tableau(i, column) == 0) {
			continue;
		}

		ratio = this->tableau(i, constantColumn) / this->tableau(i, column);
		if (ratio < 0) {
			//A propor��o deve ser n�o negativo
			continue;
		}

		if (minIndex == -1) {
			// Primeiro candidato a piv�
			minIndex = i;
			minRatio = ratio;
			minConstant = this->tableau(i, constantColumn);
		} else {
			if (ratio == 0 && ratio == minRatio) {
				// 0/negativo < 0/positivo
				if (this->tableau(i, constantColumn) < minConstant) {
					minIndex = i;
					minRatio = ratio;
					minConstant = this->tableau(i, constantColumn);
				}
			} else if (ratio < minRatio) {
				minIndex = i;
				minRatio = ratio;
				minConstant = this->tableau(i, constantColumn);
			}
		}
	}

	return minIndex;
}

/**
 * Retorna a linha que possui o valor 1, sendo os outros valores 0, da coluna passada como par�metro.
 * Caso contr�rio retorna -1.
 * M�todo utilizado para contru��o da solu��o.
 *
 * @param __int64 column
 * @returns __int64
 */
__int64 Simplex::getPivotRow(__int64 column) {
	__int64 one_row = -1;

	for (__int64 i = 1; i < this->tableau.rows(); i++) {
		if (this->tableau(i, column) == 1) {
			if (one_row >= 0) {
				return -1;
			} else {
				one_row = i;
				continue;
			}
		} else if (this->tableau(i, column) != 0) {
			return -1;
		}
	}

	return one_row;
}

/**
 * M�todo para remover determinada linha do tableau
 *
 * @param __int64 rowToRemove
 * @returns void
 */
void Simplex::removeRow(__int64 rowToRemove) {
    __int64 numRows = this->tableau.rows()-1;
    __int64 numCols = this->tableau.cols();

    if(rowToRemove < numRows)
        this->tableau.block(rowToRemove,0,numRows-rowToRemove,numCols) = this->tableau.block(rowToRemove+1,0,numRows-rowToRemove,numCols);

    this->tableau.conservativeResize(numRows,numCols);
}

/**
 * M�todo para remover determinada coluna do tableau
 *
 * @param __int64 rowToRemove
 * @returns void
 */
void Simplex::removeColumn(__int64 colToRemove) {
    __int64 numRows = this->tableau.rows();
    __int64 numCols = this->tableau.cols()-1;

    if( colToRemove < numCols )
        this->tableau.block(0,colToRemove,numRows,numCols-colToRemove) = this->tableau.block(0,colToRemove+1,numRows,numCols-colToRemove);

    this->tableau.conservativeResize(numRows,numCols);
}
