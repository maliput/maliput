#include <memory>
#include "plugin/plugin_base.h"

namespace maliput {
namespace plugin {

class Plugin2 : public PluginBase {
 public:
  std::string BackendName() const override { return "Plugin2"; }
};

// the class factories
extern "C" std::unique_ptr<PluginBase> create() { return std::make_unique<Plugin2>(); }

}  // namespace plugin
}  // namespace maliput
