#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd actuators,Eigen::VectorXd coeffs, double latency);


  /*********************************************************
   * Add weights for tuning.
   *********************************************************/
  static std::vector<double> weights;
};

#endif /* MPC_H */
