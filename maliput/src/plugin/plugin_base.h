#include <string>

namespace maliput {
namespace plugin {

class PluginBase {
  public:
    virtual std::string BackendName() const = 0;
    virtual ~PluginBase() = default;
};

// the types of the class factories
typedef PluginBase* create_t();
typedef void destroy_t(PluginBase*);

} // namespace plugin
} // namespace maliput
