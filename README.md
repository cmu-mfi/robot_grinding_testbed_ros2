# Robot Grinding Testbed ROS2
This repository contains any robot grinding testbed (rgt) specific ros2 packages. 

## 📂 Repository Structure

This repository is divided into multiple primary packages:

  * **`rgt_manager`**: Contains the rgt_manager, the main node managing transforms, robot_descriptions and tool changes. This package also contains scripts needed for controling and calibrating the rgt.
  * **`rgt_interface`**: Contains services used by the rgt_manager to interact with client scripts.
  * **`rgt_examples`**: Contains example scripts to showcase how to interact with the robots and the rgt_manager

## ⚙️ Prerequisites and Dependencies

This repository has been tested on:

  * **OS:** Ubuntu 24.04 LTS
  * **ROS 2:** Jazzy

### Dependencies
These repositories must be present for this project to work correctly:
  * **[ur_ros2](https://github.com/cmu-mfi/ur_ros2)** - ros2 interface for UR robots
  * **[ynx_ros2](https://github.com/cmu-mfi/ynx_ros2)** - ros2 interface for Yaskawa YNX robots
  * **[robot_manager_ros2](https://github.com/cmu-mfi/robot_manager_ros2)** - interfaces for the ur_robot_manager and ynx_robot_manager

### Install pkg dependencies using rosdep
Clone this repository into the `src` directory of your ros2 workspace.

```bash
cd <your-ros2-workspace>
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## ⚙️ Configuration
To configure the rgt_manager, modify the configuration in `rgt_manager/config/config.yaml`

## 🚀 Usage

### 1\. Setting up ur_ros2 and ynx_ros2

Follow the instructions in the mentioned repositories to set up the robots.

After the initial setup, both robots have to be launched with their individual bringup launch files (Check the specific repositories for more in-depth explanations)
```bash
ros2 launch ur_bringup bringup.launch.py ns:=ur20
```
```bash
ros2 launch ynx_bringup bringup.launch.py ns:=nex10
```

### 2\. Running the rgt manager

To start the rgt_manager, simply start it from the terminal:

```bash
ros2 run rgt_manager rgt_manager
```

### 3\. Interact with the systems

The `rgt_examples` package includes several Python nodes designed to demonstrate different ways to interact with the rgt. 

Below is a breakdown of the available example scripts:

| Script Name | Description |
| :--- | :--- |
| `tool_change_example` | Changes the tools of both nex10 and ur20 |

**Example:**
```bash
ros2 run rgt_examples tool_change_example
```
