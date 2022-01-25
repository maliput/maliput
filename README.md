
| GCC | Sanitizers(Clang) | Scan-Build |
| --------- | --------- | -------- |
|[![gcc](https://github.com/ToyotaResearchInstitute/maliput/actions/workflows/build.yml/badge.svg)](https://github.com/ToyotaResearchInstitute/maliput/actions/workflows/build.yml) | [![clang](https://github.com/ToyotaResearchInstitute/maliput/actions/workflows/sanitizers.yml/badge.svg)](https://github.com/ToyotaResearchInstitute/maliput/actions/workflows/sanitizers.yml) | [![scan_build](https://github.com/ToyotaResearchInstitute/maliput/actions/workflows/scan_build.yml/badge.svg)](https://github.com/ToyotaResearchInstitute/maliput/actions/workflows/scan_build.yml) |

# Maliput

Road network runtime interface.

## Build

1. Setup a development workspace as described [here](https://github.com/ToyotaResearchInstitute/maliput_documentation/blob/main/docs/installation_quickstart.rst).

2. Bring up your development workspace:

```sh
cd path/to/my/workspace
source ./bringup
```

3. Build maliput packages and their dependencies:

  - If not building drake from source:

   ```sh
   colcon build --packages-up-to maliput
   ```

  - If building drake from source:

   ```sh
   colcon build --cmake-args -DWITH_PYTHON_VERSION=3 --packages-up-to maliput
   ```
