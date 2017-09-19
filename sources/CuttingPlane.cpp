#include "..\headers\CuttingPlane.h"
#include <Eigen>

using namespace Eigen;

/**
 * @desc Construtor
 *
 * @param Problem *ilp problema de Programação Linear Inteira a ser resolvido pelo planos de corte.
 * @param int mode Pode ser: MINIMIZE, MAXIMIZE
 */
CuttingPlane::CuttingPlane(Problem *ilp, int mode) {
    this->mode = mode;
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
