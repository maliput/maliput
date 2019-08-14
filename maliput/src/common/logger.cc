#include "maliput/common/logger.h"

namespace maliput {

Logger* log() {
  // TODO(agalbachicar)   Create a custom maliput implementation.
  return drake::log();
}

std::string set_log_level(const std::string& level) {
  // TODO(agalbachicar)   Create a custom maliput implementation.
  return drake::logging::set_log_level(level);
}

}  // namespace maliput
