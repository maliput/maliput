#include <string>

namespace maliput {
namespace plugin {

class PluginBase {
 public:
  virtual std::string BackendName() const = 0;
  virtual ~PluginBase() = default;
};

}  // namespace plugin
}  // namespace maliput
