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

        void readFile();
        void getObjectiveAndMode(string line);
        bool getConstraint(string line);
        void readLPFile();
        bool getLPConstraint(string line);
        bool getLPBound(string line);
        void getLPObjectiveAndMode(string line);

    public:
        Interpreter(const string fileName);
        Problem* getProblem();
        int getMode();
};
