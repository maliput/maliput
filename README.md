
[![GCC](https://github.com/maliput/maliput/actions/workflows/build.yml/badge.svg)](https://github.com/maliput/maliput/actions/workflows/build.yml)

# Maliput

## Description

A C++ runtime API describing a Road Network model for use in agent and traffic simulations. It guarantees a continuous description of the road geometry and supports dynamic environments with varying rules states.

For a full overview of Maliput capabilities please visit https://maliput.readthedocs.io/en/latest/maliput_overview.html.

## API Documentation

Refer to [Maliput's Online API Documentation](https://maliput.readthedocs.io/en/latest/html/deps/maliput/html/annotated.html).

## Examples

[Getting Started](https://maliput.readthedocs.io/en/latest/getting_started.html) page is a good place for starting to see the Maliput's capabilities.

There are a couple of packages where the Maliput's API is exercised.
 - [maliput_integration](https://github.com/maliput/maliput_integration): Concentrates applications created for maliput. See [maliput_integration's tutorials](https://maliput.readthedocs.io/en/latest/html/deps/maliput_integration/html/integration_tutorials.html).
 - [delphyne_demos](https://github.com/maliput/delphyne_demos): Contains demos based on delphyne, which is an agent simulation framework that uses `maliput` as the road network model. See [delphyne_demos](https://github.com/maliput/delphyne_demos).


## Installation

### Supported platforms

Ubuntu Focal Fossa 20.04 LTS.

### Binary Installation on Ubuntu

See [Installation Docs](https://maliput.readthedocs.io/en/latest/installation.html#binary-installation-on-ubuntu).

### Source Installation on Ubuntu

#### Prerequisites

```
sudo apt install python3-rosdep python3-colcon-common-extensions
```

#### Build

1. Create colcon workspace if you don't have one yet.
    ```sh
    mkdir colcon_ws/src -p
    ```

2. Clone this repository in the `src` folder
    ```sh
    cd colcon_ws/src
    git clone https://github.com/maliput/maliput.git
    ```

3. Install package dependencies via `rosdep`
    ```
    export ROS_DISTRO=foxy
    ```
    ```sh
    rosdep update
    rosdep install -i -y --rosdistro $ROS_DISTRO --from-paths src
    ```

4. Build the package
    ```sh
    colcon build --packages-up-to maliput
    ```

    **Note**: To build documentation a `-BUILD_DOCS` cmake flag is required:
    ```sh
    colcon build --packages-select maliput --cmake-args " -DBUILD_DOCS=On"
    ```

For further info refer to [Source Installation on Ubuntu](https://maliput.readthedocs.io/en/latest/installation.html#source-installation-on-ubuntu)

#### For development

It is recommended to follow the guidelines for setting up a development workspace as described [here](https://maliput.readthedocs.io/en/latest/developer_setup.html).

## Contributing

Please see [CONTRIBUTING](https://maliput.readthedocs.io/en/latest/contributing.html) page.

## License

[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://github.com/maliput/maliput/blob/main/LICENSE)
