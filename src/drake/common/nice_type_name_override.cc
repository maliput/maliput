#include "maliput/drake/common/nice_type_name_override.h"

#include "maliput/drake/common/drake_assert.h"
#include "maliput/drake/common/never_destroyed.h"

namespace maliput::drake {
namespace internal {

namespace {
NiceTypeNamePtrOverride& ptr_override() {
  static never_destroyed<NiceTypeNamePtrOverride> value;
  return value.access();
}
}  // namespace

void SetNiceTypeNamePtrOverride(NiceTypeNamePtrOverride new_ptr_override) {
  MALIPUT_DRAKE_DEMAND(ptr_override() == nullptr);
  MALIPUT_DRAKE_DEMAND(new_ptr_override != nullptr);
  ptr_override() = new_ptr_override;
}

const NiceTypeNamePtrOverride& GetNiceTypeNamePtrOverride() {
  return ptr_override();
}

}  // namespace internal
}  // namespace maliput::drake
