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
A-LOAM

```

struct LidarEdgeFactor
{
	LidarEdgeFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_,
					Eigen::Vector3d last_point_b_, double s_)
		: curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_), s(s_) {}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{

		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> lpa{T(last_point_a.x()), T(last_point_a.y()), T(last_point_a.z())};
		Eigen::Matrix<T, 3, 1> lpb{T(last_point_b.x()), T(last_point_b.y()), T(last_point_b.z())};

		//Eigen::Quaternion<T> q_last_curr{q[3], T(s) * q[0], T(s) * q[1], T(s) * q[2]};
		Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
		Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
		q_last_curr = q_identity.slerp(T(s), q_last_curr);
		Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2]};

		Eigen::Matrix<T, 3, 1> lp;
		lp = q_last_curr * cp + t_last_curr;

		Eigen::Matrix<T, 3, 1> nu = (lp - lpa).cross(lp - lpb);
		Eigen::Matrix<T, 3, 1> de = lpa - lpb;

		residual[0] = nu.x() / de.norm();
		residual[1] = nu.y() / de.norm();
		residual[2] = nu.z() / de.norm();

		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_a_,
									   const Eigen::Vector3d last_point_b_, const double s_)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarEdgeFactor, 3, 4, 3>(
			new LidarEdgeFactor(curr_point_, last_point_a_, last_point_b_, s_)));     //3：残差3维 4：q为4元数 3：vector3d
	}

	Eigen::Vector3d curr_point, last_point_a, last_point_b;
	double s;
};
```
