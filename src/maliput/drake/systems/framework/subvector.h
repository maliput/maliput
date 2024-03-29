#pragma once

#include <stdexcept>

#include "maliput/drake/common/default_scalars.h"
#include "maliput/drake/common/drake_copyable.h"
#include "maliput/drake/common/drake_deprecated.h"
#include "maliput/drake/common/drake_throw.h"
#include "maliput/drake/systems/framework/vector_base.h"

namespace maliput::drake {
namespace systems {

/// Subvector is a concrete class template that implements
/// VectorBase by providing a sliced view of a VectorBase.
///
/// @tparam_default_scalar
template <typename T>
class Subvector final : public VectorBase<T> {
 public:
  // Subvector objects are neither copyable nor moveable.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Subvector)

  /// Constructs a subvector of vector that consists of num_elements starting
  /// at first_element.
  /// @param vector The vector to slice.  Must not be nullptr. Must remain
  ///               valid for the lifetime of this object.
  Subvector(VectorBase<T>* vector, int first_element, int num_elements)
      : vector_(vector), first_element_(first_element), num_elements_(num_elements) {
    if (vector_ == nullptr) {
      throw std::logic_error("Cannot create Subvector of a nullptr vector.");
    }
    if ((first_element < 0) || (num_elements < 0) || (first_element + num_elements > vector->size())) {
      std::ostringstream oss;
      oss << "Subvector range [" << first_element << ", ";
      oss << first_element + num_elements << ") falls outside the";
      oss << " valid range [" << 0 << ", " << vector->size() << ").";
      throw std::logic_error(oss.str());
    }
  }

  int size() const final { return num_elements_; }

 private:
  const T& DoGetAtIndexUnchecked(int index) const final {
    MALIPUT_DRAKE_ASSERT(index < size());
    return (*vector_)[first_element_ + index];
  }

  T& DoGetAtIndexUnchecked(int index) final {
    MALIPUT_DRAKE_ASSERT(index < size());
    return (*vector_)[first_element_ + index];
  }

  const T& DoGetAtIndexChecked(int index) const final {
    if (index >= size()) {
      this->ThrowOutOfRange(index);
    }
    return (*vector_)[first_element_ + index];
  }

  T& DoGetAtIndexChecked(int index) final {
    if (index >= size()) {
      this->ThrowOutOfRange(index);
    }
    return (*vector_)[first_element_ + index];
  }

  VectorBase<T>* vector_{nullptr};
  int first_element_{0};
  int num_elements_{0};
};

}  // namespace systems
}  // namespace maliput::drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(class ::maliput::drake::systems::Subvector)
