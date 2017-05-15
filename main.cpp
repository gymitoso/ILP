#include <Eigen>
#include "BranchBound.h"
#include "exception.h"

using namespace std;
using namespace Eigen;

int main() {

	Problem *pli = NULL;
	BranchBound *bb = NULL;

	MatrixXd constraints(4, 3);
	VectorXd objectiveFunction(2);
	VectorXd relations(4);

	try {
		/*
			Problema de maximização
		*/
		objectiveFunction <<	11,
								14;

		constraints <<		1,	1,	17,
							3,	7,	63,
							3,	5,	48,
							3,  1,  30;

        relations << 0, 0,  0,  0;


        pli = new Problem(objectiveFunction, constraints, relations);

        bb = new BranchBound(pli, MAXIMIZE);

		if (bb->hasSolution()) {
			cout << "O valor otimizado e: " << bb->getOptimum() << endl;
			cout << "A solucao e: " << bb->getSolution().transpose() << endl;
		} else {
			cout << "PLI sem solucao." << endl;
		}
	} catch (Exception *ex) {
		ex->print();
	}

	delete bb;

	return 0;
}

