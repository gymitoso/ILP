#include "../headers/BranchBound.h"
#include <Eigen>
#include <cmath>
#include <limits>

using namespace Eigen;

/**
 * @desc Construtor
 *
 * @param Problem *ilp problema de Programação Linear Inteira a ser resolvido pelo branch-and-bound.
 * @param int mode Pode ser: MINIMIZE, MAXIMIZE
 */
BranchBound::BranchBound(Problem *ilp, int mode) {
    this->mode = mode;
    this->root = new Node();
    this->root->ilp = ilp;
    this->foundSolution = false;
    if(mode == MAXIMIZE) {
        this->optimum = numeric_limits<double>::min();
    } else {
        this->optimum = numeric_limits<double>::max();
    }
    this->findSolutions(root);
}

/**
 * @desc Busca por todas as soluções
 *
 * @param Node node contém o problema a ser resolvido.
 * @returns void
 */
void BranchBound::findSolutions(Node *node) {
    //variável para verificar se a restrição foi adicionada
    bool check;

    node->solver = new Simplex(this->mode, node->ilp->getObjectiveFunction(), node->ilp->getConstraints(), node->ilp->getRelations());

    //verifica se o problema possui solução e se ela é melhor que a atual
    if(node->solver->hasSolution() && this->isBetterSolution(node->solver->getOptimum())) {

        //método de branch/ramificação
        long long pos = this->findBranch(node->solver->getSolution());

        if(pos != -1) {
            double intPart, temp;
            VectorXd newConstraint;
            //busca a parte fracional do número
            temp = node->solver->getSolution()(pos);
            modf(temp, &intPart);

            node->left = new Node();
            node->left->ilp = new Problem(node->ilp->getObjectiveFunction(), node->ilp->getConstraints(), node->ilp->getRelations());

            //cria uma nova restrição baseado no limite inferior
            newConstraint = VectorXd::Zero(node->ilp->getConstraints().cols());
            newConstraint(pos) = 1;
            newConstraint(newConstraint.rows()-1) = intPart;

            //caso a parte inteira seja 0 a relação é de igualdade
            if(intPart == 0) {
                check = node->left->ilp->addConstraint(newConstraint, 2);
            } else {
                check = node->left->ilp->addConstraint(newConstraint, 0);
            }

            if(check) {
                this->findSolutions(node->left);
            }

            node->right = new Node();
            node->right->ilp = new Problem(node->ilp->getObjectiveFunction(), node->ilp->getConstraints(), node->ilp->getRelations());

            //cria uma nova restrição baseado no limite superior
            newConstraint(newConstraint.rows()-1) = intPart + 1;
            check = node->right->ilp->addConstraint(newConstraint, 1);

            if(check) {
                this->findSolutions(node->right);
            }
        } else {
            this->foundSolution = true;
            this->optimum = node->solver->getOptimum();
            this->solution = node->solver->getSolution();
        }
    }
}

/**
 * @desc Busca por um número Real para ramificar
 * @desc Foi utilizado a tecnica de Variante de Dakin
 *
 * @param VectorXd vectorToSearch vetor ao qual a busca sera realizada
 * @returns __int64 Retorna o indice da coluna ou -1 se não achou.
 */
long long BranchBound::findBranch(VectorXd vectorToSearch) {
    double intPart, floatPart = 0;
    int temp = -1;
    for (long long i = 0; i < vectorToSearch.rows(); i++) {
        if(modf(vectorToSearch(i), &intPart) > floatPart && modf(vectorToSearch(i), &intPart) < 0.99) {
            floatPart = modf(vectorToSearch(i), &intPart);
            temp = i;
        }
    }
    return temp;
}

/**
 * @desc Verifica se a solução atual é melhor
 *
 * @param double Valor a ser verificado
 * @returns bool true se for melhor e false se não
 */
bool BranchBound::isBetterSolution(double optimumFound) {
    if(this->mode == MAXIMIZE && optimumFound > this->optimum) {
        return true;
    } else if(this->mode == MINIMIZE && optimumFound < this->optimum){
        return true;
    }
    return false;
}

/**
 * @desc Retorna true se a solução foi encontrada.
 * @desc Retorna false caso contrário.
 *
 * @returns boolean
 */
bool BranchBound::hasSolution() {
    return this->foundSolution;
}


/**
 * @desc Retorna o valor ótimo da funcao objetivo maximizado ou minimizado com valores inteiros
 *
 * @returns double
 */
double BranchBound::getOptimum() {
    return this->optimum;
}

/**
 * @desc Retorna o valor das variáveis para a solução encontrada.
 *
 * @returns VectorXd
 */
VectorXd BranchBound::getSolution() {
    return this->solution;
}
