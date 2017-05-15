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
        __int64 numberOfVariables;

        bool simplexSolver(__int64 variableNum, int mode, int phase);
        __int64 findPivot_min(__int64 column, int phase);
        __int64 getPivotRow(__int64 column);
        void removeRow(__int64 rowToRemove);
        void removeColumn(__int64 colToRemove);

    protected:

    public:
        Simplex(int mode, const VectorXd &objectiveFunction, const MatrixXd &constraints, const VectorXd &relations);
        bool hasSolution();
        double getOptimum();
        VectorXd getSolution();
};
