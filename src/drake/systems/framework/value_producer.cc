#include "maliput/drake/systems/framework/value_producer.h"

#include <stdexcept>

#include "maliput/drake/common/nice_type_name.h"

namespace maliput::drake {
namespace systems {

ValueProducer::ValueProducer() = default;

ValueProducer::ValueProducer(AllocateCallback allocate, CalcCallback calc)
    : allocate_(std::move(allocate)), calc_(std::move(calc)) {
  if (allocate_ == nullptr) {
    throw std::logic_error("Cannot create a ValueProducer with a null AllocateCallback");
  }
  if (calc_ == nullptr) {
    throw std::logic_error("Cannot create a ValueProducer with a null Calc");
  }
}

ValueProducer::~ValueProducer() = default;

bool ValueProducer::is_valid() const { return (allocate_ != nullptr) && (calc_ != nullptr); }

void ValueProducer::NoopCalc(const ContextBase&, AbstractValue*) {}

std::unique_ptr<AbstractValue> ValueProducer::Allocate() const {
  if (allocate_ == nullptr) {
    throw std::logic_error("ValueProducer cannot invoke a null AllocateCallback");
  }
  return allocate_();
}

void ValueProducer::Calc(const ContextBase& context, AbstractValue* output) const {
  if (output == nullptr) {
    throw std::logic_error("ValueProducer output was nullptr");
  }
  if (calc_ == nullptr) {
    throw std::logic_error("ValueProducer cannot invoke a null CalcCallback");
  }
  return calc_(context, output);
}

void ValueProducer::ThrowBadNull() { throw std::logic_error("ValueProducer was given a null callback pointer"); }

void ValueProducer::ThrowBadCast(const std::type_info& actual_type, const std::type_info& desired_type) {
  std::ostringstream oss;
  oss << "ValueProducer cannot cast a ";
  oss << NiceTypeName::Get(actual_type);
  oss << " to a ";
  oss << NiceTypeName::Get(desired_type);
  throw std::logic_error(oss.str());
}

}  // namespace systems
}  // namespace maliput::drake
