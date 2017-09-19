#pragma once

#include <iostream>
#include <string>
#include <fstream>
#include "Problem.h"

using namespace std;

class Interpreter {
    private:
        Problem *pli;
        int mode;
        MatrixXd constraints;
        VectorXd objectiveFunction;
        VectorXd relations;
        ifstream in;

        void getObjectiveAndMode(string line);
        bool getConstraint(string line);

    public:
        Interpreter(const string fileName);
        Problem* getProblem();
        int getMode();
};
