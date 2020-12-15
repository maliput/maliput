#include "plugin/plugin_base.h"

namespace maliput {
namespace plugin {

class Plugin2 : public PluginBase {
 public:
  std::string BackendName() const override { return "Plugin2"; }
};

// the class factories
extern "C" PluginBase* create() { return new Plugin2; }
extern "C" void destroy(PluginBase* p) { delete p; }

}  // namespace plugin
}  // namespace maliput
