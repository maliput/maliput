#include <dlfcn.h>
#include <memory>
#include <stdexcept>
#include <string>

namespace maliput {
namespace plugin {

template <typename PluginBaseT>
class Loader {
 public:
  Loader(const char* file) {
    // load library
    lhandle = dlopen(file, RTLD_LAZY);
    if (!lhandle) {
      throw std::runtime_error("Cannot load library: " + std::string(dlerror()));
    }
    // reset errors
    dlerror();

    // load the symbols
    creator = (create_t*)dlsym(lhandle, "create");
    const char* dlsym_error = dlerror();
    if (dlsym_error) {
      throw std::runtime_error("Cannot load symbol create: " + std::string(dlsym_error));
    }
  }

  std::unique_ptr<PluginBaseT> GetInstance() { return creator(); }

  ~Loader() { dlclose(lhandle); }

 private:
  // the types of the class factories
  typedef std::unique_ptr<PluginBaseT> create_t();

  create_t* creator;
  void* lhandle{nullptr};
};

}  // namespace plugin
}  // namespace maliput
