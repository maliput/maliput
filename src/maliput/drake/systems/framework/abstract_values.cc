#include "maliput/drake/systems/framework/abstract_values.h"

#include <utility>

namespace maliput::drake {
namespace systems {

AbstractValues::~AbstractValues() {}

AbstractValues::AbstractValues() {}

AbstractValues::AbstractValues(std::vector<std::unique_ptr<AbstractValue>>&& data) : owned_data_(std::move(data)) {
  for (auto& datum : owned_data_) {
    data_.push_back(datum.get());
  }
}

AbstractValues::AbstractValues(const std::vector<AbstractValue*>& data) : data_(data) {}

AbstractValues::AbstractValues(std::unique_ptr<AbstractValue> datum) : AbstractValues() {
  data_.push_back(datum.get());
  owned_data_.push_back(std::move(datum));
}

int AbstractValues::size() const { return static_cast<int>(data_.size()); }

const AbstractValue& AbstractValues::get_value(int index) const {
  MALIPUT_DRAKE_ASSERT(index >= 0 && index < size());
  MALIPUT_DRAKE_ASSERT(data_[index] != nullptr);
  return *data_[index];
}

AbstractValue& AbstractValues::get_mutable_value(int index) {
  MALIPUT_DRAKE_ASSERT(index >= 0 && index < size());
  MALIPUT_DRAKE_ASSERT(data_[index] != nullptr);
  return *data_[index];
}

void AbstractValues::SetFrom(const AbstractValues& other) {
  MALIPUT_DRAKE_ASSERT(size() == other.size());
  for (int i = 0; i < size(); i++) {
    MALIPUT_DRAKE_ASSERT(data_[i] != nullptr);
    data_[i]->SetFrom(other.get_value(i));
  }
}

std::unique_ptr<AbstractValues> AbstractValues::Clone() const {
  std::vector<std::unique_ptr<AbstractValue>> cloned_data;
  cloned_data.reserve(data_.size());
  for (const AbstractValue* datum : data_) {
    cloned_data.push_back(datum->Clone());
  }
  return std::make_unique<AbstractValues>(std::move(cloned_data));
}

}  // namespace systems
}  // namespace maliput::drake
