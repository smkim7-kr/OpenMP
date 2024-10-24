# OpenMP use cases

## GLIM [cloud_covariance_estimation.cpp(line 57)](https://github.com/koide3/glim/blob/857a86daa9e545e2eab883ca9f844791ed55cf3e/src/glim/common/cloud_covariance_estimation.cpp#L57)

```cpp
  // Precompute pt * pt.transpose()
  std::vector<Eigen::Matrix4d> pt_cross(points.size());
  if (gtsam_points::is_omp_default()) {
#pragma omp parallel for num_threads(num_threads) schedule(guided, 64)
    for (int i = 0; i < points.size(); i++) {
      pt_cross[i] = points[i] * points[i].transpose();
    }
  } else {
#ifdef GTSAM_POINTS_USE_TBB
    tbb::parallel_for(tbb::blocked_range<int>(0, points.size(), 64), [&](const tbb::blocked_range<int>& range) {
      for (int i = range.begin(); i < range.end(); i++) {
        pt_cross[i] = points[i] * points[i].transpose();
      }
    });
#else
    std::cerr << "error : TBB is not enabled" << std::endl;
    abort();
#endif
  }
```