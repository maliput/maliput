#include <dlfcn.h>
#include <stdexcept>
#include <string>

namespace maliput {
namespace plugin {

template <typename PluginBase>
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
    destroyer = (destroy_t*)dlsym(lhandle, "destroy");
    dlsym_error = dlerror();
    if (dlsym_error) {
      throw std::runtime_error("Cannot load symbol destroy: " + std::string(dlsym_error));
    }
  }

  PluginBase* GetInstance() { return creator(); }

  void DestroyInstance(PluginBase* instance) { destroyer(instance); }

  ~Loader() { dlclose(lhandle); }

 private:
  // the types of the class factories
  typedef PluginBase* create_t();
  typedef void destroy_t(PluginBase*);

  create_t* creator{nullptr};
  destroy_t* destroyer{nullptr};
  void* lhandle{nullptr};
};

}  // namespace plugin
}  // namespace maliput
