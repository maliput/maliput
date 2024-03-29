#include "maliput/drake/systems/framework/system_scalar_converter.h"

#include <sstream>
#include <stdexcept>

#include "maliput/drake/common/default_scalars.h"
#include "maliput/drake/common/hash.h"
#include "maliput/drake/common/nice_type_name.h"

using std::pair;
using std::type_index;
using std::type_info;

namespace maliput::drake {
namespace systems {

SystemScalarConverter::Key::Key(const type_info& t_info, const type_info& u_info)
    : pair<type_index, type_index>(t_info, u_info) {}

size_t SystemScalarConverter::KeyHasher::operator()(const Key& key) const {
  maliput::drake::DefaultHasher hasher;
  using maliput::drake::hash_append;
  hash_append(hasher, std::hash<std::type_index>{}(key.first));
  hash_append(hasher, std::hash<std::type_index>{}(key.second));
  return static_cast<size_t>(hasher);
}

SystemScalarConverter::SystemScalarConverter() = default;

void SystemScalarConverter::Insert(const std::type_info& t_info, const std::type_info& u_info,
                                   const ErasedConverterFunc& converter) {
  const auto& key = Key{t_info, u_info};
  const auto& insert_result = funcs_.insert({key, converter});
  MALIPUT_DRAKE_ASSERT(insert_result.second);
}

template <typename T, typename U>
void SystemScalarConverter::Remove() {
  funcs_.erase(Key(typeid(T), typeid(U)));
}

const SystemScalarConverter::ErasedConverterFunc* SystemScalarConverter::Find(const std::type_info& t_info,
                                                                              const std::type_info& u_info) const {
  const auto& key = Key{t_info, u_info};
  auto iter = funcs_.find(key);
  if (iter != funcs_.end()) {
    return &(iter->second);
  } else {
    return nullptr;
  }
}

void SystemScalarConverter::RemoveUnlessAlsoSupportedBy(const SystemScalarConverter& other) {
  // Remove the items from `funcs_` whose key is absent from `other`.
  // (This would use erase_if, if we had it.)
  for (auto iter = funcs_.begin(); iter != funcs_.end();) {
    const Key& our_key = iter->first;
    if (other.funcs_.count(our_key) == 0) {
      iter = funcs_.erase(iter);
    } else {
      ++iter;
    }
  }
}

namespace system_scalar_converter_internal {

void ThrowConversionMismatch(const type_info& s_t_info, const type_info& s_u_info, const type_info& other_info) {
  std::ostringstream oss;
  oss << "SystemScalarConverter was configured to convert a ";
  oss << NiceTypeName::Get(s_u_info) << " into a ";
  oss << NiceTypeName::Get(s_t_info) << " but was called with a ";
  oss << NiceTypeName::Get(other_info) << "at runtime";
  throw std::runtime_error(oss.str());
}

}  // namespace system_scalar_converter_internal

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS((&SystemScalarConverter::Remove<T, U>))

}  // namespace systems
}  // namespace maliput::drake
