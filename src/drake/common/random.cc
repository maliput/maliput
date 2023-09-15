#include "maliput/drake/common/random.h"


namespace maliput::drake {
template <typename T>
T CalcProbabilityDensity(RandomDistribution distribution,
                         const Eigen::Ref<const VectorX<T>>& x) {
  switch (distribution) {
    case RandomDistribution::kUniform: {
      for (int i = 0; i < x.rows(); ++i) {
        if (x(i) < 0.0 || x(i) > 1.0) {
          return T(0.);
        }
      }
      return T(1.);
    }
    case RandomDistribution::kGaussian: {
      return ((-0.5 * x.array() * x.array()).exp() / std::sqrt(2 * M_PI))
          .prod();
    }
    case RandomDistribution::kExponential: {
      for (int i = 0; i < x.rows(); ++i) {
        if (x(i) < 0.0) {
          return T(0.);
        }
      }
      return (-x.array()).exp().prod();
    }
  }
  MALIPUT_DRAKE_UNREACHABLE();
}

// TODO(jwnimmer-tri) Use DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_...
// here, once we can break the dependency cycle.
template double CalcProbabilityDensity<double>(
    RandomDistribution, const Eigen::Ref<const VectorX<double>>&);
}  // namespace maliput::drake

