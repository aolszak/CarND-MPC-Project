#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

const int latency_multiplier = 2; // 100ms latency which equals 2*dt

struct SolveVector {
  vector<double> x; // position x
  vector<double> y; // position y
  vector<double> steering; // steering value
  vector<double> throttle; // speed value <-1,1>
};

class MPC {
 public:
  
  MPC();
  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  SolveVector Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  
  double previous_steering {0};
  double previous_throttle {0};
};

#endif /* MPC_H */
