#include "maliput/drake/systems/framework/port_base.h"

#include <utility>

#include "maliput/drake/common/drake_assert.h"
#include "maliput/drake/common/nice_type_name.h"

namespace maliput::drake {
namespace systems {

PortBase::PortBase(const char* kind_string, internal::SystemMessageInterface* owning_system,
                   internal::SystemId owning_system_id, std::string name, int index, DependencyTicket ticket,
                   PortDataType data_type, int size)
    : kind_string_(kind_string),
      owning_system_(*owning_system),
      owning_system_id_(owning_system_id),
      index_(index),
      ticket_(ticket),
      data_type_(data_type),
      size_(size),
      name_(std::move(name)) {
  MALIPUT_DRAKE_DEMAND(kind_string != nullptr);
  MALIPUT_DRAKE_DEMAND(owning_system != nullptr);
  MALIPUT_DRAKE_DEMAND(owning_system_id.is_valid());
  MALIPUT_DRAKE_DEMAND(!name_.empty());
}

PortBase::~PortBase() = default;

std::string PortBase::GetFullDescription() const {
  std::stringstream oss;
  oss << kind_string_ << "Port[" << index_ << "]";
  oss << " (" << name_ << ") of System ";
  oss << get_system_interface().GetSystemPathname();
  oss << NiceTypeName::RemoveNamespaces(get_system_interface().GetSystemType());
  return oss.str();
}

void PortBase::ThrowValidateContextMismatch() const {
  std::ostringstream oss;
  oss << kind_string_ << "Port: The Context given as an argument was";
  oss << " not created for this GetFullDescription()";
  throw std::logic_error(oss.str());
}

void PortBase::ThrowBadCast(const std::string& value_typename, const std::string& eval_typename) const {
  std::ostringstream oss;
  oss << kind_string_ << "Port::Eval(): wrong value type ";
  oss << eval_typename << "specified;";
  oss << "actual type was " << value_typename << " for ";
  oss << GetFullDescription();
  throw std::logic_error(oss.str());
}

}  // namespace systems
}  // namespace maliput::drake
