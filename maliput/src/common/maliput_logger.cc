#include "maliput/common/maliput_logger.h"

#include "maliput/common/maliput_never_destroyed.h"

namespace maliput {
namespace logging {}  // namespace logging

logging::Logger* maliput_log() {
  maliput::common::never_destroyed<std::unique_ptr<logging::Logger>> g_logger;
  return g_logger.access().get();
}

}  // namespace maliput
