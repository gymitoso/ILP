#include <Eigen>

using namespace Eigen;

class Problem {
     private:
         MatrixXd constraints;
         VectorXd objectiveFunction;
         VectorXd relations;

     public:
         Problem(const VectorXd &objectiveFunction, const MatrixXd &constraints, const VectorXd &relations);
         VectorXd getObjectiveFunction();
         VectorXd getRelations();
         MatrixXd getConstraints();
         void addConstraint(VectorXd constraint, int relation);
};
