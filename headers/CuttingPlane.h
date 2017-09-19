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
        bool foundSolution;
        double optimum;
        VectorXd solution;

        VectorXd gomoryCut(VectorXd vectorToCut);

    public:
        CuttingPlane(Problem *ilp, int mode);
        bool hasSolution();
        double getOptimum();
        VectorXd getSolution();
}
