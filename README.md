## 结构
1. 建立 functor，定义变量残差
2. new 一个 functor 给 cost_function
3. 加入problem

### example
```
struct NumericDiffCostFunctor {
  bool operator()(const double* const x, double* residual) const { // 变量，残差
    residual[0] = 10.0 - x[0];
    return true;
  }
};

CostFunction* cost_function =
  new NumericDiffCostFunction<NumericDiffCostFunctor, ceres::CENTRAL, 1, 1>(); //变量，残差维度
problem.AddResidualBlock(cost_function, nullptr, &x);
```

curve fitting
```
struct ExponentialResidual {
  ExponentialResidual(double x, double y) : x_(x), y_(y) {}
  template <typename T>
  bool operator()(const T* const m, const T* const c, T* residual) const {
    residual[0] = y_ - exp(m[0] * x_ + c[0]);
    return true;
  }
 private:
  const double x_;
  const double y_;
};
int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  const double initial_m = 0.0;
  const double initial_c = 0.0;
  double m = initial_m;
  double c = initial_c;
  ceres::Problem problem;
  for (int i = 0; i < kNumObservations; ++i) {
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<ExponentialResidual, 1, 1, 1>(
            data[2 * i], data[2 * i + 1]),
        nullptr,
        &m,
        &c);
  }

  ceres::Solver::Options options;
  options.max_num_iterations = 25;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";
  std::cout << "Initial m: " << initial_m << " c: " << initial_c << "\n";
  std::cout << "Final   m: " << m << " c: " << c << "\n";
  return 0;
}
```
