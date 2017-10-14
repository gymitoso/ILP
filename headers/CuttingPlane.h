#pragma once

#include <iostream>
#include "Simplex.h"
#include "Problem.h"
#include <Eigen>

#define MINIMIZE 1
#define MAXIMIZE 2

using namespace std;

class CuttingPlane {
    private:
        int mode;
        long long numberOfVariables;
        bool foundSolution;
        double optimum;
        VectorXd solution;
        MatrixXd cuts;
        Simplex *solver;

        bool gomoryCut(MatrixXd tableau);
        void addCut(VectorXd cut);
        long long getCutRow(MatrixXd tableau);
        bool isIntegerSolution(VectorXd solution);

    public:
        CuttingPlane(Problem *ilp, int mode);
        bool hasSolution();
        double getOptimum();
        VectorXd getSolution();
};
