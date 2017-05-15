#include "BranchBound.h"
#include <Eigen>
#include <cmath>

using namespace Eigen;

/**
 * Construtor
 *
 * @param Problem *pli problema de Programa��o Linear Inteira a ser resolvido pelo branch-and-bound.
 * @param int mode Pode ser: MINIMIZE, MAXIMIZE
 */
BranchBound::BranchBound(Problem *pli, int mode) {
    this->mode = mode;
    this->root = new Node();
    this->root->pli = pli;
    this->optimum = 0;
    this->findSolutions(root);
}

/**
 * Busca por todas as solu��es
 *
 * @param Node node cont�m o problema a ser resolvido.
 * @returns void
 */
void BranchBound::findSolutions(Node *node) {
    node->solver = new Simplex(this->mode, node->pli->getObjectiveFunction(), node->pli->getConstraints(), node->pli->getRelations());
    //verifica se o problema possui solu��o
    if(node->solver->hasSolution()) {
        //verifica se a solu��o possui n�meros reais
        //escolher m�todo para escolha do branch
        __int64 pos = this->findRealNumber(node->solver->getSolution());
        if(pos != -1) {
            double intPart, temp;
            VectorXd newConstraint;

            //busca a parte fracional do n�mero
            temp = node->solver->getSolution()(pos);
            modf(temp, &intPart);

            node->left = new Node();
            node->left->pli = new Problem(node->pli->getObjectiveFunction(), node->pli->getConstraints(), node->pli->getRelations());
            //cria uma nova restri��o baseado no limite inferior
            newConstraint = VectorXd::Zero(node->pli->getConstraints().cols());
            newConstraint(pos) = 1;
            newConstraint(newConstraint.rows()-1) = intPart;
            node->left->pli->addConstraint(newConstraint, 0);
            this->findSolutions(node->left);

            node->right = new Node();
            node->right->pli = new Problem(node->pli->getObjectiveFunction(), node->pli->getConstraints(), node->pli->getRelations());
            //cria uma nova restri��o baseado no limite superior
            newConstraint(newConstraint.rows()-1)++;
            node->right->pli->addConstraint(newConstraint, 1);
            this->findSolutions(node->right);
        } else {
            //verifica se a solu��o econtrada � melhor que a atual
            if(node->solver->getOptimum() > this->optimum) {
                this->foundSolution = true;
                this->optimum = node->solver->getOptimum();
                this->solution = node->solver->getSolution();
            }
        }
    }
}

/**
 * Busca por um n�mero Real
 *
 * @param VectorXd vectorToSearch vetor ao qual a busca ser� realizada
 * @returns __int64 Retorna o �ndice da coluna ou -1 se n�o achou.
 */
__int64 BranchBound::findRealNumber(VectorXd vectorToSearch) {
    double intPart;
    for (__int64 i = 0; i < vectorToSearch.rows(); i++) {
        if(modf(vectorToSearch(i), &intPart) != 0) {
            return i;
        }
    }
    return -1;
}

/**
 * Retorna true se a solu��o foi encontrada.
 * Retorna false caso contr�rio.
 *
 * @returns boolean
 */
bool BranchBound::hasSolution() {
	return this->foundSolution;
}


/**
 * Retorna o valor �timo da fun��o objetivo maximizado ou minimizado com valores inteiros
 *
 * @returns double
 */
double BranchBound::getOptimum() {
	return this->optimum;
}

/**
 * Retorna o valor das vari�veis para a solu��o encontrada.
 *
 * return VectorXd
 */
VectorXd BranchBound::getSolution() {
	return this->solution;
}
