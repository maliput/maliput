#include "maliput/drake/systems/framework/leaf_output_port.h"

namespace maliput::drake::systems {

template <typename T>
void LeafOutputPort<T>::ThrowIfInvalidPortValueType(const Context<T>& context, const AbstractValue& proposed) const {
  const CacheEntryValue& cache_value = cache_entry().get_cache_entry_value(context);
  const AbstractValue& value = cache_value.PeekAbstractValueOrThrow();

  if (proposed.type_info() != value.type_info()) {
    std::ostringstream oss;
    oss << "OutputPort::Calc(): expected output type ";
    oss << value.GetNiceTypeName();
    oss << " but got ";
    oss << proposed.GetNiceTypeName();
    oss << " for ";
    oss << PortBase::GetFullDescription();
    throw std::logic_error(oss.str());
  }
}

}  // namespace maliput::drake::systems

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(class ::maliput::drake::systems::LeafOutputPort)
