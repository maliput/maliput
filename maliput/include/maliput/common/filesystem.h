// clang-format off
#pragma once

#include <string>

namespace maliput {
namespace common {

class Path
{
public:

    Path() = default;
    Path(std::string path_);

    void set_path(const std::string& path_) noexcept;
    void set_as_temp();
    const std::string &get_path() const noexcept;
    bool is_file() const noexcept;
    bool is_directory() const noexcept;
    bool is_absolute() const noexcept;
    bool exists() const noexcept;
    void append(const std::string& path);

private:

    void normalize();

    std::string path;
};

class Filesystem
{
public:

    /* Returns false if path is not a file or file doesn't exists.
    Returns true otherwhise. */
    static bool remove_file(const Path& path);
    /* Returns false if path is not a directory or directory doesn't exists.
    Returns true otherwise. */
    static bool remove_directory(const Path& path);
    /* Returns false if it couldn't create the directory. False otherwise.*/
    static bool create_directory(const Path& path);
    /* Returns current working directory */
    static Path get_cwd();
    /* Reads the whole file into a string. Returns false if file could not be
    opened. True otherwise */
    static bool read_as_string(const Path &path, std::string &read_to);
    /* The same thing as mkdir -p <path> */
    static bool create_directory_recursive(const Path &path);
    /* Returns the environment variable path. Empty string if it doesn't
    exists*/
    static std::string get_env_path(const std::string &env_var);
};

} // namespace common
} // namespace maliput
// clang-format on
