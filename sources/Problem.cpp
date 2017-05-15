#include "Problem.h"
#include <Eigen>

using namespace Eigen;

/**
 * Construtor
 *
 * @param const VectorXd &objectiveFunction Os coeficientes da fun��o objetivo.
 * @param const VectorXd &relations Os sinais de rela��o das restri��es {0 -> <=; 1 -> >=; 2 -> =}.
 * @param const MatrixXd &constraints Matriz com todas as restri��es.
 */
 Problem::Problem(const VectorXd &objectiveFunction, const MatrixXd &constraints, const VectorXd &relations) {
     this->objectiveFunction = objectiveFunction;
     this->constraints = constraints;
     this->relations = relations;
 }

 /**
 * Retorna as restri��es do problema
 *
 * @returns MatrixXd
 */
 MatrixXd Problem::getConstraints() {
     return this->constraints;
 }

 /**
 * Retorna as rela��es das restri��es do problema
 *
 * @returns VectorXd
 */
 VectorXd Problem::getRelations() {
     return this->relations;
 }

 /**
 * Retorna a fun��o objetivo do problema
 *
 * @returns VectorXd
 */
 VectorXd Problem::getObjectiveFunction() {
     return this->objectiveFunction;
 }

 /**
 * Adiciona uma fun��o de restri��o e sua rela��o
 *
 * @returns void
 */
 void Problem::addConstraint(VectorXd constraint, int relation) {
     this->constraints.conservativeResize(this->constraints.rows()+1, this->constraints.cols());
     this->constraints.row(this->constraints.rows()-1) = constraint;
     this->relations.conservativeResize(this->relations.rows()+1);
     this->relations(this->relations.rows()-1) = relation;
 }


