# DevContainers

- [DevContainers](#devcontainers)
  - [Bazel-Zen](#bazel-zen)
      - [Image Details](#image-details)
      - [Getting Started](#getting-started)
  - [ROS-Zen](#ros-zen)

## Bazel-Zen

:warning: This is as yet an unsupported workflow! YMMV.

#### Image Details

* Base Image: `focal` (ubuntu)
* Bazel:
  * Installed via Bazelisk.
  * `bazel` and `bazelisk` invocations both work (a bash alias supports this).
  * The bazel version is configured via `./bazel/devcontainer.json` [1]

[1] Override if necessary. At a later date, we might configure the bazel version via a marker file at the project root.

#### Getting Started

* [Install VSCode](https://code.visualstudio.com/docs/setup/linux#_debian-and-ubuntu-based-distributions)
* Open the project in VSCode
* CTRL-SHIFT-P &rarr; Reopen in Container
* Open a terminal in the container and run

```
(docker) zen@bazel-zen:/workspaces/maliput$ bazel build //...
```

## ROS-Zen

TODO
