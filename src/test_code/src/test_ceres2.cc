#include "ceres/ceres.h"
#include "glog/logging.h"
#include <iostream>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

// A templated cost functor that implements the residual r = 10 -
// x. The method operator() is templated so that we can then use an
// automatic differentiation wrapper around it to generate its
// derivatives.
struct CostFunctor {
  template <typename T>
  bool operator()(const T* const x, T* residual) const {
    residual[0] = 10.0 - x[0] * x[0];
    residual[1] = 10.0 - x[1];
    return true;
  }
};

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  // The variable to solve for with its initial value. It will be
  // mutated in place by the solver.
  double x[2] = {0.5,20};
//   const double initial_x = x;
  std::cout<< "before: " << std::endl;
  for(auto& ch : x)
    std::cout << ch << " ";
   std::cout << std::endl;  
  // Build the problem.
  Problem problem;

  // Set up the only cost function (also known as residual). This uses
  // auto-differentiation to obtain the derivative (jacobian).
  CostFunction* cost_function =
      new AutoDiffCostFunction<CostFunctor, 2, 2>(new CostFunctor);
  problem.AddResidualBlock(cost_function, nullptr, x);

  // Run the solver!
  Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  Solver::Summary summary;
  Solve(options, &problem, &summary);

//   std::cout << summary.BriefReport() << "\n";
  std::cout << "after: " << std::endl;
  for(auto& ch : x)
    std::cout << ch << " ";
  std::cout << std::endl;
  return 0;
}

