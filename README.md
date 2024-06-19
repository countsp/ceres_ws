## 结构

```
struct Factor
{

}

static ceres::CostFunction *Create()
{
    return (new ceres::AutoDiffCostFunction<Factor,a,b,c>new Factor());

}
```

### example
```
struct NumericDiffCostFunctor {
  bool operator()(const double* const x, double* residual) const {
    residual[0] = 10.0 - x[0];
    return true;
  }
};

CostFunction* cost_function =
  new NumericDiffCostFunction<NumericDiffCostFunctor, ceres::CENTRAL, 1, 1>();
problem.AddResidualBlock(cost_function, nullptr, &x);
```

curve fitting
```
struct ExponentialResidual {
  ExponentialResidual(double x, double y)
      : x_(x), y_(y) {}

  template <typename T>
  bool operator()(const T* const m, const T* const c, T* residual) const {
    residual[0] = y_ - exp(m[0] * x_ + c[0]);
    return true;
  }

 private:
  // Observations for a sample.
  const double x_;
  const double y_;
};

double m = 0.0;
double c = 0.0;

Problem problem;
for (int i = 0; i < kNumObservations; ++i) {
  CostFunction* cost_function =
       new AutoDiffCostFunction<ExponentialResidual, 1, 1, 1>
           (data[2 * i], data[2 * i + 1]);
  problem.AddResidualBlock(cost_function, nullptr, &m, &c);
}

```
