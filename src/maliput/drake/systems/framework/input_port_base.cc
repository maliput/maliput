#include "maliput/drake/systems/framework/input_port_base.h"

#include <stdexcept>
#include <utility>

#include "maliput/drake/common/drake_assert.h"

namespace maliput::drake {
namespace systems {

InputPortBase::InputPortBase(internal::SystemMessageInterface* owning_system, internal::SystemId owning_system_id,
                             std::string name, InputPortIndex index, DependencyTicket ticket, PortDataType data_type,
                             int size, const std::optional<RandomDistribution>& random_type, EvalAbstractCallback eval)
    : PortBase("Input", owning_system, owning_system_id, std::move(name), index, ticket, data_type, size),
      eval_(std::move(eval)),
      random_type_(random_type) {
  if (is_random() && data_type != kVectorValued) {
    throw std::logic_error("Random input ports must be vector valued.");
  }
  MALIPUT_DRAKE_DEMAND(eval_ != nullptr);
}

InputPortBase::~InputPortBase() = default;

void InputPortBase::ThrowRequiredMissing() const {
  std::ostringstream oss;
  oss << "InputPort::Eval(): required ";
  oss << GetFullDescription();
  oss << " is not connected";
  throw std::logic_error(oss.str());
}

}  // namespace systems
}  // namespace maliput::drake
