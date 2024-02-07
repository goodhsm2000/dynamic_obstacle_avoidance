#ifndef PROBABILITY_COST_FUNCTION_H
#define PROBABILITY_COST_FUNCTION_H

#include <base_local_planner/trajectory_cost_function.h>
#include <vector>

using namespace std;

namespace base_local_planner {

/**
 * This class provides a cost based on collision probability with dynamic obstacles.
 */
class ProbabilityCostFunction: public base_local_planner::TrajectoryCostFunction {
public:

  ProbabilityCostFunction() {}
  ~ProbabilityCostFunction() {}

  void setDirectionProbability(std::vector<double> & arr);
  double scoreTrajectory(Trajectory &traj);
  bool prepare();

private:

  std::vector<double> vec_;
  double cost_;
};

} /* namespace base_local_planner */
#endif /* PROBABILITY_COST_FUNCTION_H */