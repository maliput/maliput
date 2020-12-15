#include <memory>
#include "plugin/plugin_base.h"

namespace maliput {
namespace plugin {

class Plugin1 : public PluginBase {
 public:
  std::string BackendName() const override { return "Plugin1"; }
};

// the class factories
extern "C" std::unique_ptr<PluginBase> create() { return std::make_unique<Plugin1>(); }

}  // namespace plugin
}  // namespace maliput
