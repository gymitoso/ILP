#include "Problem.h"
#include <Eigen>

using namespace Eigen;

/**
 * Construtor
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
 * Retorna as restrições do problema
 *
 * @returns MatrixXd
 */
 MatrixXd Problem::getConstraints() {
     return this->constraints;
 }

 /**
 * Retorna as relações das restrições do problema
 *
 * @returns VectorXd
 */
 VectorXd Problem::getRelations() {
     return this->relations;
 }

 /**
 * Retorna a função objetivo do problema
 *
 * @returns VectorXd
 */
 VectorXd Problem::getObjectiveFunction() {
     return this->objectiveFunction;
 }

 /**
 * Adiciona uma função de restrição e sua relação
 *
 * @returns void
 */
 void Problem::addConstraint(VectorXd constraint, int relation) {
     this->constraints.conservativeResize(this->constraints.rows()+1, this->constraints.cols());
     this->constraints.row(this->constraints.rows()-1) = constraint;
     this->relations.conservativeResize(this->relations.rows()+1);
     this->relations(this->relations.rows()-1) = relation;
 }


