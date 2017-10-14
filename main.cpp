#include "headers/Interpreter.h"
#include "headers/BranchBound.h"
#include "headers/CuttingPlane.h"
#include "headers/Exception.h"
#include <time.h>

using namespace std;

int main(int argc, char* argv[]) {

    BranchBound *bb = NULL;
    CuttingPlane *cp = NULL;
    Interpreter *interpreter = NULL;
    clock_t time[2];

    try {

        if(argc < 2) {
            throw(new Exception("Digite o nome do arquivo de entrada!"));
        }

        interpreter = new Interpreter(argv[1]);

        time[0] = clock();

        bb = new BranchBound(interpreter->getProblem(), interpreter->getMode());

        time[1] = clock();

        double totalTime = (time[1] - time[0]) * 1000.00 / CLOCKS_PER_SEC;

        cout << "----------Branch and Bound---------" << endl;
        if (bb->hasSolution()) {
            cout << "Valor otimizado: " << bb->getOptimum() << endl;
            cout << "Solucao: [" << bb->getSolution().transpose() << "]"<< endl;
            cout << "Tempo: " << totalTime << "ms" << endl;
        } else {
            cout << "Solucao nao encontrada" << endl;
        }

        time[0] = clock();

        cp = new CuttingPlane(interpreter->getProblem(), interpreter->getMode());

        time[1] = clock();

        totalTime = (time[1] - time[0]) * 1000.00 / CLOCKS_PER_SEC;

        cout << "----------Planos de Corte---------" << endl;
        if (cp->hasSolution()) {
            cout << "Valor otimizado: " << cp->getOptimum() << endl;
            cout << "Solucao: [" << cp->getSolution().transpose() << "]"<< endl;
            cout << "Tempo: " << totalTime << "ms" << endl;
        } else {
            cout << "Solucao nao encontrada" << endl;
        }

    } catch (Exception *ex) {
        ex->print();
    }

    delete bb;
    delete cp;
    delete interpreter;

    return 0;
}
