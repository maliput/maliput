#include "maliput/drake/common/value.h"

#include <atomic>
#include <sstream>

#include "maliput/drake/common/text_logging.h"

namespace maliput::drake {

namespace internal {
int ReportZeroHash(const std::type_info& detail) {
  // Log a debug message noting the possible performance impairment.  Elevate
  // it to a warning the first time it happens -- but only elevate it at most
  // once per process, to avoid spamming.
  static std::atomic<bool> g_has_warned{false};
  const bool has_warned = g_has_warned.exchange(true);
  const std::string bad_class = NiceTypeName::Get(detail);
  std::ostringstream oss;
  oss << "The " << bad_class << "class is incompatible with the typename";
  oss << " hasher that provides the";
  oss << " type-erasure checking for AbstractValue casts, most likely because";
  oss << " the problematic class mixes template parameters with nested";
  oss << " classes or non-type template parameters.";
  //
  oss << " As a result, operations on Value<" << bad_class << "> will suffer";
  oss << " from slightly impaired performance.";
  //
  oss << " If the problem relates to nested classes, you may be able to resolve it";
  oss << " by un-nesting the class in question.";
  //
  oss << " If the problem relates to a single non-type template parameter, you may";
  oss << " be able to resolve it by adding 'using NonTypeTemplateParameter = ...'.";
  oss << " See drake/common/test/value_test.cc for an example.";
  if (!has_warned) {
    log()->warn(oss.str() +
                " This is the first instance of an impaired T within this process."
                " Additional instances will not be warned about, but you may set"
                " the maliput::drake::log() level to 'debug' to see all instances.");
  } else {
    log()->debug(oss.str());
  }
  return 0;
}
}  // namespace internal

AbstractValue::~AbstractValue() = default;

std::string AbstractValue::GetNiceTypeName() const {
  return NiceTypeName::Canonicalize(NiceTypeName::Demangle(type_info().name()));
}

void AbstractValue::ThrowCastError(const std::string& requested_type) const {
  std::ostringstream oss;
  oss << "AbstractValue: a request to cast to '" << requested_type << "' ";
  oss << "the actual type was '" << GetNiceTypeName() << "'.";
  throw std::logic_error(oss.str());
}

}  // namespace maliput::drake
