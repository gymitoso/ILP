#pragma once

#include <Eigen>

using namespace Eigen;

#define SIMPLEX_MINIMIZE 1
#define SIMPLEX_MAXIMIZE 2

#define FIRST_PHASE 1
#define SECOND_PHASE 2

class Simplex {
    private:
        MatrixXd tableau;
        bool foundSolution;
        double optimum;
        VectorXd solution;
        long long numberOfVariables;

        bool simplexSolver(long long  variableNum, int mode, int phase);
        long long findPivot(long long column, int phase, bool bland);
        long long getPivotRow(long long column);
        void removeRow(long long rowToRemove);
        void removeColumn(long long colToRemove);
        void isValidEntry(int mode, const VectorXd &objectiveFunction,
            const MatrixXd &constraints, const VectorXd &relations);
        void searchSolution();
        int buildTableau(int mode, const VectorXd &objectiveFunction,
            const MatrixXd &constraints, const VectorXd &relations);
        int buildTableauWithCuts(int mode, const VectorXd &objectiveFunction,
            const MatrixXd &constraints, const VectorXd &relations,
            const MatrixXd &cuts);
        double adjustPrecision(double value);

    public:
        Simplex(int mode, const VectorXd &objectiveFunction,
            const MatrixXd &constraints, const VectorXd &relations);
        Simplex(int mode, const VectorXd &objectiveFunction,
            const MatrixXd &constraints, const VectorXd &relations,
            const MatrixXd &cuts);
        bool hasSolution();
        double getOptimum();
        VectorXd getSolution();
        MatrixXd getTableau();
};
