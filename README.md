
[![GCC](https://github.com/maliput/maliput/actions/workflows/build.yml/badge.svg)](https://github.com/maliput/maliput/actions/workflows/build.yml)

# Maliput

A C++ runtime API describing a Road Network model for use in agent and traffic simulations. It guarantees a continuous description of the road geometry and supports dynamic environments with varying rules states.

Please visit https://maliput.readthedocs.org for more information.

## Installation

### Supported platforms

Ubuntu Focal Fossa 20.04 LTS.

### Binary Installation on Ubuntu

See [Installation page](https://maliput.readthedocs.io/en/latest/installation.html#binary-installation-on-ubuntu) for adding the ROS2 repositories to your source list.

Install maliput package:
```sh
sudo apt install ros-foxy-maliput
```

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

#### For development

It is recommended to follow the guidelines for setting up a development workspace as described [here](https://maliput.readthedocs.io/en/latest/developer_setup.html).

## Contributing

Please see [CONTRIBUTING](https://maliput.readthedocs.io/en/latest/contributing.html) page.

## License

This library is licensed under BSD 3-Clause. See also the [LICENSE](https://github.com/maliput/maliput/blob/main/LICENSE) file.
