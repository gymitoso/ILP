#include "../headers/CuttingPlane.h"
#include <Eigen>
#include <cmath>

using namespace Eigen;

/**
 * @desc Construtor
 *
 * @param Problem *ilp problema de Programação Linear Inteira a ser resolvido pelo planos de corte.
 * @param int mode Pode ser: MINIMIZE, MAXIMIZE
 */
CuttingPlane::CuttingPlane(Problem *ilp, int mode) {
    this->mode = mode;
    this->foundSolution = false;
    this->numberOfVariables = ilp->getObjectiveFunction().rows();

    this->solver = new Simplex(this->mode, ilp->getObjectiveFunction(), ilp->getConstraints(), ilp->getRelations());

    while(true) {
        if(!this->solver->hasSolution()) {
            break;
        }

        if(this->isIntegerSolution(this->solver->getSolution())) {
            this->foundSolution = true;
            this->optimum = this->solver->getOptimum();
            this->solution = this->solver->getSolution();
            break;
        }

        if(!gomoryCut(this->solver->getTableau())) {
            break;
        }

        this->solver = new Simplex(this->mode, ilp->getObjectiveFunction(), ilp->getConstraints(), ilp->getRelations(), this->cuts);
    }
}

/**
 * @desc Função para realizar o corte no espaço de solução
 * @param MatrixXd contém o tableau a ser realizado o corte
 * @returns bool true se o corte foi inserido ou false se inválido
 */
bool CuttingPlane::gomoryCut(MatrixXd tableau) {
    double intPart, floatPart;
    long long j, rowToCut;
    VectorXd cut = VectorXd::Zero(tableau.cols());

    rowToCut = getCutRow(tableau);
    if(rowToCut == -1) {
        return false;
    }

    for (j = 0; j < tableau.cols(); j++) {
        if(tableau(rowToCut,j) >= 0 ) {
            cut(j) = modf(tableau(rowToCut,j), &intPart);
        } else {
            floatPart = modf(tableau(rowToCut,j), &intPart);
            if(floatPart != 0){
                cut(j) = 1 + floatPart;
            } else {
                cut(j) = 0;
            }
        }
    }

    addCut(cut);

    return true;
}

/**
 * @desc Função para retornar a linha a ser realizado o corte
 * @param MatrixXd contém o tableau a ser realizado o corte
 * @returns long long a linha a ser realizado o corte
 */
long long CuttingPlane::getCutRow(MatrixXd tableau) {
    double intPart;
    long long i, j, row = -1;

    //busca linha a ser realizado o corte
    for (j = 0; j < this->numberOfVariables; j++) {
        for (i = 1; i < tableau.rows(); i++) {
            if (tableau(i, j) == 1) {
                if (row >= 0) {
                    row = -1;
                    break;
                } else {
                    row = i;
                    continue;
                }
            } else if (tableau(i, j) != 0) {
                row = -1;
                break;
            }
        }
        if(row != -1
                && modf(tableau(row, tableau.cols()-1), &intPart) > 0.00001
                && modf(tableau(row, tableau.cols()-1), &intPart) < 0.99) {
            break;
        }
        row = -1;
    }

    return row;
}

/**
 * @desc Função para adicionar o novo corte a matriz de cortes
 * @param VectorXd corte a ser adicionado
 * @returns void
 */
void CuttingPlane::addCut(VectorXd cut) {
    this->cuts.conservativeResize(this->cuts.rows()+1, cut.rows());
    this->cuts.col(this->cuts.cols()-1) = this->cuts.col(this->cuts.cols()-2);
    this->cuts.col(this->cuts.cols()-2) = VectorXd::Zero(this->cuts.rows());
    this->cuts.row(this->cuts.rows()-1) = cut;
}

/**
 * @desc Função para verificar se a solução é inteira
 * @param VectorXd contém a solução a ser verificada
 * @returns bool true se for a solução é inteira
 */
bool CuttingPlane::isIntegerSolution(VectorXd solution) {
    double intPart;
    for (long long i = 0; i < solution.rows(); i++) {
        if(modf(solution(i), &intPart) > 0.00001 && modf(solution(i), &intPart) < 0.99) {
            return false;
        }
    }
    return true;
}

/**
 * @desc Retorna true se a solução foi encontrada.
 * @desc Retorna false caso contrário.
 *
 * @returns boolean
 */
bool CuttingPlane::hasSolution() {
    return this->foundSolution;
}

/**
 * @desc Retorna o valor ótimo da funcao objetivo maximizado ou minimizado com valores inteiros
 *
 * @returns double
 */
double CuttingPlane::getOptimum() {
    return this->optimum;
}

/**
 * @desc Retorna o valor das variáveis para a solução encontrada.
 *
 * @returns VectorXd
 */
VectorXd CuttingPlane::getSolution() {
    return this->solution;
}
