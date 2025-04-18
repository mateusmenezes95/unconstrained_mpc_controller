# Unconstrained MPC Controller <!-- omit in toc -->

The **unconstrained_mpc_controller** package provides a Model Predictive Control (MPC) implementation for MIMO systems within the [ros2_control](https://control.ros.org/rolling/doc/getting_started/getting_started.html#architecture) framework. It predicts future plant outputs using a discrete-time state-space model and computes optimal control actions over a finite prediction horizon, applying only the first action at each step. The controller is implemented as a ROS 2 controller plugin, interfacing with hardware via command and state interfaces. Core components include parameter validation, matrix and vector manipulation utilities, and real-time reference tracking. The design is modular, supporting easy integration, configuration, and testing with ROS 2 hardware and simulation environments. 

> **Note:** No constraints are enforced on the control inputs or outputs.This is the main reason to employ the term "unconstrained" in the package name and structure.

## Table of Contents <!-- omit in toc -->

- [Usage premises](#usage-premises)
- [Installing dependencies](#installing-dependencies)
  - [Tools](#tools)
  - [Installing source dependencies](#installing-source-dependencies)
  - [Installing binary dependencies](#installing-binary-dependencies)
- [Building](#building)
- [Usage](#usage)
  - [ROS interfaces](#ros-interfaces)
    - [Parameters](#parameters)
    - [Subscribers](#subscribers)
    - [Publishers](#publishers)
- [Unit Tests](#unit-tests)
- [Contributing](#contributing)
- [Bugs \& Feature Requests](#bugs--feature-requests)

## Usage premises

This package relies on the [ros2_control] framework, which is a set of packages that provide a standardized way to control hardware in ROS 2. Before using this package, you should be familiar with the concepts presented in the [ros2_control architecture]. In addition to the architecture, it is highly recommended to see the contents in the [ros2_control resources] page, which contains a list of resources to help you get started with the framework.

Apart from the ros2_control framework, users should be familiar with Model Predictive Control (MPC) algorithms, particularly for MIMO systems using a discrete-time state-space model. To see the math and control concepts used in this controller, refer to the page [Unconstrained MPC Theory](doc/unconstrained-mpc-theory.md)

## Installing dependencies

The steps below will help you to install the dependencies of the package.

> **Note:** The steps thereafter assumes that you have installed ROS 2 Jazzy Jalisco. If you have not installed it yet, please follow the instructions [here](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html).

### Tools

It is advised to install the following tools to help you with the installation and building of the package:

- [vcstool](https://github.com/dirk-thomas/vcstool?tab=readme-ov-file#how-to-install-vcstool): Used to clone source dependencies.
- [rosdep](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Rosdep.html#rosdep-installation): Used to install binary dependencies.
- [colcon-mixin-repository](https://github.com/colcon/colcon-mixin-repository?tab=readme-ov-file#how-to-fetch-the-information)

### Installing source dependencies

```bash
cd ~/<your_workspace>/src
vcs import unconstrained_mpc_controller/dependencies.repos
```

### Installing binary dependencies

```bash
rosdep update
sudo apt install update
cd ~/<your_workspace>
rosdep install --from-paths src/unconstrained_mpc_controller --ignore-src -r -y --rosdistro jazzy
```

> run rosdep install --help to see all the options available.

## Building

Once you have cloned the repository in your workspace, you can build it following the steps thereafter:

```bash
cd ~/<your_workspace>
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to unconstrained_mpc_controller --event-handlers console_direct+ --mixin compile-commands
```

> **Note:** The `--mixin compile-commands` option is used to generate the `compile_commands.json` file, which is useful for IDEs like VSCode and CLion. If you are not using an IDE, you can omit this option.

## Usage

`ros2_control` controllers are ROS 2 nodes that can not run on their own. First of all, the hardware that you want to control must provide command and state interfaces. After that, you need to create configuration files and launch files to describe how the controller should be loaded and how it should interact with the hardware. These interactions are managed by the ros2_control framework.

As start point, you can check the [unconstrained_mpc_controller_demo] which is a demo package that shows how to use the controller with the BlueROV2 simulated. The demo package contains all the necessary configuration files and launch files to run the controller with the simulated robot. You can also check the [unit tests](test/test_unconstrained_mpc_controller.cpp) to get a better understanding of the code base and how to use the controller.

### ROS interfaces

#### Parameters

> TODO 1

#### Subscribers

> TODO

#### Publishers

> TODO

## Unit Tests

To run the unit tests, build the package with the `--mixin build-testing-on` option:

```bash
cd ~/<your_workspace>
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to unconstrained_mpc_controller --event-handlers console_direct+ --mixin build-testing-on
```

Then, source the workspace:

```bash
source ~/your_workspace/install/setup.bash
colcon test --packages-select unconstrained_mpc_controller --event-handlers console_direct+
```

## Contributing

Contributions are welcome! If you want to contribute to the package, please follow the steps below:

1. Fork the repository.
2. Create a new branch for your feature or bug fix.
3. Make your changes following the [ROS 2 style guide](https://index.ros.org/doc/ros2/Contributing/Code-Style/) for C++ code. Try to keep the code style consistent with the rest of the package.
4. Make sure to run the unit tests and check that they pass.
5. Make sure to run the linters and check that they pass.
6. Commit the changes with a clear message. Use the [Seven Rules of a Great Commit Message](https://chris.beams.io/posts/git-commit/) as a guide.
7. Push your changes to your forked repository.
8. Create a pull request to this repository.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker]

[ros2_control]: https://control.ros.org/rolling/index.html
[ros2_control architecture]: https://control.ros.org/rolling/doc/getting_started/getting_started.html#architecture
[ros2_control resources]: https://control.ros.org/rolling/doc/resources/resources.html
[Issue Tracker]: https://github.com/mateusmenezes95/unconstrained_mpc_controller/issues
[unconstrained_mpc_controller_demo]: https://github.com/mateusmenezes95/unconstrained_mpc_controller_demo
