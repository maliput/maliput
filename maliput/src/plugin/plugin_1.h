#include "plugin/plugin_base.h"

namespace maliput {
namespace plugin {

class Plugin1 : public PluginBase {
  public:
    std::string BackendName() const override {
      return "Plugin1";
    }
};

// the class factories
extern "C" PluginBase* create() {
 return new Plugin1;
}
extern "C" void destroy(PluginBase* p) {
 delete p;
}

} // namespace plugin
} // namespace maliput
