#include "maliput/drake/systems/framework/output_port.h"

namespace maliput::drake::systems {

template <typename T>
void OutputPort<T>::CheckValidAllocation(const AbstractValue& proposed) const {
  if (this->get_data_type() != kVectorValued) return;  // Nothing we can check for an abstract port.

  const auto* const proposed_vec = proposed.maybe_get_value<BasicVector<T>>();
  if (proposed_vec == nullptr) {
    std::ostringstream oss;
    oss << "OutputPort::Allocate(): expected BasicVector output type ";
    oss << "but got " << proposed.GetNiceTypeName();
    oss << " for " << GetFullDescription() << ".";
    throw std::logic_error(oss.str());
  }

  const int proposed_size = proposed_vec->get_value().size();
  if (proposed_size != this->size()) {
    std::ostringstream oss;
    oss << "OutputPort::Allocate(): expected vector output type of ";
    oss << "size " << this->size() << " but got a vector of size ";
    oss << proposed_size << " for ";
    oss << GetFullDescription() << ".";
    throw std::logic_error(oss.str());
  }
}

}  // namespace maliput::drake::systems

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(class ::maliput::drake::systems::OutputPort)
