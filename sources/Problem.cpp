#include "..\headers\Problem.h"

using namespace Eigen;

/**
 * @desc Construtor
 *
 * @param const VectorXd &objectiveFunction Os coeficientes da função objetivo.
 * @param const VectorXd &relations Os sinais de relação das restrições {0 -> <=; 1 -> >=; 2 -> =}.
 * @param const MatrixXd &constraints Matriz com todas as restrições.
 */
Problem::Problem(const VectorXd &objectiveFunction, const MatrixXd &constraints, const VectorXd &relations) {
    this->objectiveFunction = objectiveFunction;
    this->constraints = constraints;
    this->relations = relations;
}

/**
 * @desc Retorna as restrições do problema
 *
 * @returns MatrixXd
 */
MatrixXd Problem::getConstraints() {
    return this->constraints;
}

/**
 * @desc Retorna as relações das restrições do problema
 *
 * @returns VectorXd
 */
VectorXd Problem::getRelations() {
    return this->relations;
}

/**
 * @desc Retorna a função objetivo do problema
 *
 * @returns VectorXd
 */
VectorXd Problem::getObjectiveFunction() {
    return this->objectiveFunction;
}

/**
 * @desc Adiciona uma nova função de restrição e sua relação
 *
 * @returns bool true se adicionou a restricao ou false caso a restrição já exista
 */
bool Problem::addConstraint(VectorXd constraint, int relation) {
    long long temp;
    for(temp = 0; temp < this->constraints.rows(); temp++) {
        if(((this->constraints.row(temp) - constraint.transpose()).norm() == 0) && this->relations(temp) == relation) {
            return false;
        }
    }
    this->constraints.conservativeResize(this->constraints.rows()+1, this->constraints.cols());
    this->constraints.row(this->constraints.rows()-1) = constraint;
    this->relations.conservativeResize(this->relations.rows()+1);
    this->relations(this->relations.rows()-1) = relation;
    return true;
}
