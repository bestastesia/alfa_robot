alfa_robot_description
==========================================

Robot description package for alfa_robot.

![Licence](https://img.shields.io/badge/License-Apache-2.0-blue.svg)

# Quick Start Instructions

If you are familiar with ROS 2, here are the quick-and-dirty build instructions.

  ```
  cd $COLCON_WS
  sudo apt-get update
  sudo apt-get upgrade
  git clone git@github.com:alfa_robot_description/alfa_robot_description.git src/alfa_robot_description
  vcs import src --input src/alfa_robot_description/alfa_robot_description.humble.repos
  vcs import src --input src/alfa_robot_description/alfa_robot_description.humble.upstream.repos
  source /opt/ros/humble/setup.bash
  rosdep install --ignore-src --from-paths src -y -r
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release    # Faster and more efficient build type
  cd ..
  ```
If you end up with missing dependencies, install them using commands from [Setup ROS Workspace](#setup-ros-workspace) section.

# How to use this Package and ROS Introduction

 - [Workflow With Docker](#workflow-with-docker)
   * [Quick Start Using ROS with Docker (RosTeamWorkspace)](#quick-start-using-ros-with-docker-rosteamworkspace)
 - [Install and Build](#install-and-build)
   * [Install ROS Humble and Development Tooling](#install-ros-humble-and-development-tooling)
   * [Setup ROS Workspace](#setup-ros-workspace)
   * [Configure and Build Workspace](#configure-and-build-workspace)
 - [Running Executables](#running-executables)
   * [Using the Local Workspace](#using-the-local-workspace)
 - [Testing and Linting](#testing-and-linting)
 - [Creating a new ROS 2 Package](#creating-a-new-ros2-package)
 - [References](#references)

## Workflow With [Docker](https://docs.docker.com/)

> **NOTE:** If you do not use Docker in the current workflow you can skip this section and jump to [Install and Build](#install-and-build)

We usually use a separate [Docker](https://docs.docker.com/) container for each of the projects/workspaces we work on.
An internal tool from [Stogl Robotics](https://b-robotized.com) called [Ros Team Workspace (RTW)](https://rtw.b-robotized.com) simplifies the creation and work with  Docker based workspaces.
The tool is targeted toward developers.

Installation of docker depends on the operating system you are using. Instructions can be found here: [Windows](https://docs.docker.com/desktop/install/windows-install/), [Mac](https://docs.docker.com/desktop/install/mac-install/) and [Linux](https://docs.docker.com/desktop/install/linux-install/).

### Quick Start Using ROS with Docker (ros_team_workspace)

Using [Ros Team Workspace (RTW)](https://rtw.b-robotized.com) you can easily with the following command:
```
setup-ros-workspace-docker WS_FOLDER_NAME ROS_DISTRO
```
and then after sourcing the new workspace with the `_WS_FOLDER_NAME` command, you can switch to the workspace with the:
```
rtw_switch_to_docker
```
command.

## Install and Build

### Install ROS Humble and Development Tooling

These instructions assume you are running Ubuntu 20.04:

1. [Install ROS 2 Humble](https://index.ros.org/doc/ros2/Installation/Humble/Linux-Install-Debians/).
   You can stop following along with the tutorial after you complete the section titled: [Environment setup](https://index.ros.org/doc/ros2/Installation/Humble/Linux-Install-Debians/#environment-setup).
   Make sure you setup your environment with:
   ```
   source /opt/ros/humble/setup.bash
   ```

   > **NOTE:** You may want to add that line to your `~/.bashrc`

   > **NOTE:** There is also a `zsh` version of the setup script.

1. [Install ROS 2 Build Tools](https://index.ros.org/doc/ros2/Installation/Humble/Linux-Development-Setup/#install-development-tools-and-ros-tools).
   You do not need to build ROS 2 from source.
   Simply install the tooling under the section titled "Install development tools and ROS tools".

1. Install `ccache`:
   ```
   sudo apt install ccache
   ```

1. Setup `colcon mixin` [Reference](https://github.com/colcon/colcon-mixin-repository) for convenience commands.
   ```
   sudo apt install python3-colcon-mixin
   colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
   colcon mixin update default
   ```

### Setup ROS Workspace

1. Create a colcon workspace:
   ```
   export COLCON_WS=~/workspace/ros_ws_humble
   mkdir -p $COLCON_WS/src
   ```

   > **NOTE:** Feel free to change `~/workspace/ros_ws_humble` to whatever absolute path you want.

   > **NOTE:** Over time you will probably have multiple ROS workspaces, so it makes sense to them all in a subfolder.
     Also, it is good practice to put the ROS version in the name of the workspace, for different tests you could just add a suffix to the base name `ros_ws_humble`.

1. Download the required repositories and install package dependencies:
   ```
   cd $COLCON_WS
   git clone git@github.com:alfa_robot_description/alfa_robot_description.git src/alfa_robot_description
   vcs import src --input src/alfa_robot_description/alfa_robot_description.humble.repos
   vcs import src --input src/alfa_robot_description/alfa_robot_description.humble.repos
   rosdep install --ignore-src --from-paths src -y -r       # install also is there are unreleased packages
   ```

   Sometimes packages do not list all their dependencies so `rosdep` will not install everything.
   If you are getting missing dependency errors, try manually install the following packages:
   ```
   sudo apt install ros2-humble-forward_command_controller ros2-humble-joint_state_broadcaster ros2-humble-joint_trajectory_controller ros2-humble-xacro
   ```

### Configure and Build Workspace:
To configure and build workspace execute following commands:
  ```
  cd $COLCON_WS
  colcon build --symlink-install --mixin rel-with-deb-info compile-commands ccache
  ```

## Running Executable

See `README.md` files of the packages for information regarding running executables.

<Add here some concrete data about current repository>

### Using the Local Workspace

To use the local workspace you have to source it by using local setup script:
  ```
  source $COLCON_WS/install/local_setup.bash
  ```
Since there are many errors one unintentionally do with wrong sourcing, please check also [Notes on Sourcing ROS Workspace](#notes-on-sourcing-ros-workspace).

#### Notes on Sourcing ROS Workspace

Sourcing of a workspace appends the binary and resource directories to appropriate environment variables.
It is important that you do not run the build command in the same terminal that you have previously sourced your local workspace.
This can cause dependency resolution issues.
Here is some advice copied from [Official ROS Workspace Tutorial](https://index.ros.org/doc/ros2/Tutorials/Workspace/Creating-A-Workspace/) on this:

Before sourcing the overlay, it is very important that you open a new terminal, separate from the one where you built the workspace.
Sourcing an overlay in the same terminal where you built, or likewise building where an overlay is sourced, may create complex issues.

Sourcing the local_setup of the overlay will only add the packages available in the overlay to your environment.
`setup` sources the overlay as well as the underlay it was created in, allowing you to utilize both workspaces.

So, sourcing your main ROS 2 installation’s setup and then the dev_ws overlay’s local_setup, like you just did, is the same as just sourcing dev_ws’s setup, because that includes the environment of the underlay it was created in.


## Testing and Linting

To test the packages packages built from source, use the following command with [colcon](https://colcon.readthedocs.io/en/released/).
In order to run tests and linters you will have had to already built the workspace.
To run the tests use following commands:
  ```
  cd $COLCON_WS
  colcon test
  colcon test-result
  ```

There are `--mixin` arguments that can be used to control testing with linters, specifically `linters-only` and `linters-skip`.

## Creating a new ROS 2 Package

If you need to create a new ROS 2 package it is helpful to start with the official boilerplate for a ROS 2 package.
The command `ros2 pkg` can be used to generate the boilerplate details.
For example to create a new ROS 2 package called `example_package` with a node called `example_node` and library called `example_library` use this command:
  ```
  ros2 pkg create --build-type ament_cmake --node-name example_node --library-name example_library example_package
  ```

## References

Here are some useful references for developing with ROS 2:

 - [Official ROS 2 Tutorials](https://index.ros.org/doc/ros2/Tutorials/)
   * [Launchfile](https://index.ros.org/doc/ros2/Tutorials/Launch-Files/Creating-Launch-Files/)
   * [Package](https://index.ros.org/doc/ros2/Tutorials/Creating-Your-First-ROS2-Package/)
   * [Parameters](https://index.ros.org/doc/ros2/Tutorials/Parameters/Understanding-ROS2-Parameters/)
   * [Workspace](https://index.ros.org/doc/ros2/Tutorials/Workspace/Creating-A-Workspace/)
 - [Example ROS packages](https://github.com/ros2/examples)
 - [Colcon Documentation](https://colcon.readthedocs.io/en/released/#)
 - [ROS 2 Design Documentation](https://design.ros2.org/)
 - [ROS 2 Launch Architecture](https://github.com/ros2/launch/blob/master/launch/doc/source/architecture.rst)

Descriptions, meshes, and visualization files for the robots and environments. Corresponding launch and test files are also stored here.

The structure and files in this package are generated using [RosTeamWorkspace script setup-robot-description](https://rtw.b-robotized.com/master/use-cases/ros_packages/setup_robot_description_package.html). You can use the same script to generate initial files for other robots.

## General details about robot description packages

This description package follows, as much as possible, the recommendations from [ROS-Industrial Consortium about robot support packages](https://wiki.ros.org/Industrial/Tutorials/WorkingWithRosIndustrialRobotSupportPackages).
The general package structure is the following:

```
<manufacturer|robot_name>_description/             # Robot's description files
├── [CMakeLists.txt]                               # if ament_cmake is used (recommended)
├── package.xml
├── [setup.py]                                     # if amend_python is used
├── [setup.cfg]                                    # if amend_python is used
├── config/                                        # general YAML files for a robot
│   └── <robot_name>_<someting_specific>.yaml
├── launch/                                        # launch files related to testing robots' description
│   └── test_<robot_name>_description.launch.py
├── meshes/                                        # meshes used in <robot_name>_macro.urdf.xacro
│   ├── collision
│   │   └── <robot_name|robot_model>               # meshes are sorted by robot name or model
│   │       ├── <link_xy>.stl
│   │       └── ...
│   └── visual
│       └── <robot_name|robot_model>
│           ├── <link_xy>.dae
│           └── ...
├── rviz/                                          # rviz display configurations
│   └── <robot_name>_default.rviz
└── urdf/                                          # URDF file for the robot
    ├── common.xacro                               # Common XACRO definitions
    ├── <robot_name>.urdf.xacro                    # Main URDF for a robot - loads macro and other files
    └── <robot_name|robot_model>
        ├── <robot_name>_macro.xacro               # Macro file of the robot - can add prefix, define origin, etc.
        └── <robot_name>_macro.ros2_control.xacro  # URDF-part used to configure ros2_control

```

### Testing the validity of robot description

1. Go to the root of your workspace folder (there where `src`, `build`, `install` and `log` files are).
2. Install the package by calling `colcon build --symlink-install --packages-select alfa_robot_description`
3. (Re-)Source environment `source install/setup.bash`


> **NOTE:** If you use [RosTeamWorkspace (RTW)](https://rtw.b-robotized.com) than instead of the previous three steps, use `cb alfa_robot_description` command.

Now, launch description test:
```
ros2 launch alfa_robot_description view_alfa_robot.launch.xml
```
or
```
ros2 launch alfa_robot_description view_alfa_robot.launch.py
```

If there are no issues with the description, two windows are opened: `rviz2` and `Joint State Publisher`.
Rviz2 visualizes the robot's state and Joint state Publisher to changes joint values using sliders or generates random but valid configurations.

Descriptions, meshes, and visualization files for the robots and environments. Corresponding launch and test files are also stored here.

The structure and files in this package are generated using [RosTeamWorkspace script setup-robot-description](https://rtw.b-robotized.com/master/use-cases/ros_packages/setup_robot_description_package.html). You can use the same script to generate initial files for other robots.

## General details about robot description packages

This description package follows, as much as possible, the recommendations from [ROS-Industrial Consortium about robot support packages](https://wiki.ros.org/Industrial/Tutorials/WorkingWithRosIndustrialRobotSupportPackages).
The general package structure is the following:

```
<manufacturer|robot_name>_description/             # Robot's description files
├── [CMakeLists.txt]                               # if ament_cmake is used (recommended)
├── package.xml
├── [setup.py]                                     # if amend_python is used
├── [setup.cfg]                                    # if amend_python is used
├── config/                                        # general YAML files for a robot
│   └── <robot_name>_<someting_specific>.yaml
├── launch/                                        # launch files related to testing robots' description
│   └── test_<robot_name>_description.launch.py
├── meshes/                                        # meshes used in <robot_name>_macro.urdf.xacro
│   ├── collision
│   │   └── <robot_name|robot_model>               # meshes are sorted by robot name or model
│   │       ├── <link_xy>.stl
│   │       └── ...
│   └── visual
│       └── <robot_name|robot_model>
│           ├── <link_xy>.dae
│           └── ...
├── rviz/                                          # rviz display configurations
│   └── <robot_name>_default.rviz
└── urdf/                                          # URDF file for the robot
    ├── common.xacro                               # Common XACRO definitions
    ├── <robot_name>.urdf.xacro                    # Main URDF for a robot - loads macro and other files
    └── <robot_name|robot_model>
        ├── <robot_name>_macro.xacro               # Macro file of the robot - can add prefix, define origin, etc.
        └── <robot_name>_macro.ros2_control.xacro  # URDF-part used to configure ros2_control

```

### Testing the validity of robot description

1. Go to the root of your workspace folder (there where `src`, `build`, `install` and `log` files are).
2. Install the package by calling `colcon build --symlink-install --packages-select alfa_robot_description`
3. (Re-)Source environment `source install/setup.bash`


> **NOTE:** If you use [RosTeamWorkspace (RTW)](https://rtw.b-robotized.com) than instead of the previous three steps, use `cb alfa_robot_description` command.

Now, launch description test:
```
ros2 launch alfa_robot_description view_alfa_robot.launch.xml
```
or
```
ros2 launch alfa_robot_description view_alfa_robot.launch.py
```

If there are no issues with the description, two windows are opened: `rviz2` and `Joint State Publisher`.
Rviz2 visualizes the robot's state and Joint state Publisher to changes joint values using sliders or generates random but valid configurations.
