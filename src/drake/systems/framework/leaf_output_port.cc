#include "maliput/drake/systems/framework/leaf_output_port.h"

namespace maliput::drake::systems {

template <typename T>
void LeafOutputPort<T>::ThrowIfInvalidPortValueType(
    const Context<T>& context, const AbstractValue& proposed) const {
  const CacheEntryValue& cache_value =
      cache_entry().get_cache_entry_value(context);
  const AbstractValue& value = cache_value.PeekAbstractValueOrThrow();

  if (proposed.type_info() != value.type_info()) {
    throw std::logic_error(
        fmt::format("OutputPort::Calc(): expected output type {} "
                    "but got {} for {}.",
                    value.GetNiceTypeName(), proposed.GetNiceTypeName(),
                    PortBase::GetFullDescription()));
  }
}

}  // namespace maliput::drake::systems

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::maliput::drake::systems::LeafOutputPort)
