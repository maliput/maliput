#include <string>

namespace maliput {
namespace plugin {

/// Returns the complete path to the `lib_name` library.
/// The macro this method uses is provided as a compile definition in the
/// CMakeLists.txt file.
/// @param lib_name Name of the dynamic library file.
std::string GetPathToLib(const std::string& lib_name);

}  // namespace plugin
}  // namespace maliput
