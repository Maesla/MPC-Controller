#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

struct Result {

    vector<double> x;
    vector<double> y;
    vector<double> steer;
    vector<double> acceleration;
};

class MPC {
 public:
  MPC();

  virtual ~MPC();

  Result Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

};

#endif /* MPC_H */
