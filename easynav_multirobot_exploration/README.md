# easynav_multirobot_exploration

[![ROS 2: kilted](https://img.shields.io/badge/ROS%202-kilted-blue)](#)

## Overview
This is the core orchestration package for the Multi-Robot Exploration system based on the `easynav` framework. While the root repository contains the general simulation environments and external plugins, **this package** is responsible for launching the system, managing the namespaces, configuring the navigation stack, and executing the exploration Behavior Trees.

> **Note:** For building instructions, dependencies, and full system tutorials, please refer to the [Main Repository README](../../README.md).

---

## Launch Files & Arguments

This package provides highly parameterized launch files to support multi-robot scalability. All nodes are pushed into their respective namespaces to avoid topic collisions.

### 1. `exploration.launch.py`
Initializes the Behavior Tree executor (`explorer` node) that drives the autonomous exploration logic.
* **`namespace`** (default: `''`): Namespace for the node and its topics.
* **`params_file`** (default: `config/explorer.params.yaml`): Absolute path to the parameter file.
* **`bt_xml_file`** (default: `behavior_trees/explore.xml`): Absolute path to the Behavior Tree XML file.

### 2. `slam_namespaced.launch.py`
Launches the SLAM toolbox isolated within a specific robot's namespace, using ROS 2 Lifecycle management to ensure proper startup sequencing.
* **`namespace`** (default: `''`): Namespace for the node and its topics.
* **`use_sim_time`** (default: `true`): Use simulation/Gazebo clock.
* **`autostart`** (default: `true`): Automatically startup the slamtoolbox (ignored if `use_lifecycle_manager` is true).
* **`use_lifecycle_manager`** (default: `false`): Enable bond connection during node activation.
* **`slam_params_file`** (default: `config/slam_namespaced.params.yaml`): Full path to the ROS 2 parameters file to use for the slam_toolbox node.

### 3. `multirobot_sim.launch.py`
Spawns the Gazebo simulation environment and the robot models.
* **`gui`** (default: `true`): Set to `false` to run Gazebo headless.
* **`world`** (default: `hospital`): Selects the simulation world. Options: `hospital`, `maze`.
* **`size`** (default: `big`): Modifies the scale/version of the selected world. Options: `small`, `big`.

### 4. `rviz_namespaced.launch.py`
Opens an RViz2 instance pre-configured to listen to a specific robot's TF tree and topics.
* **`namespace`** (required): The namespace to visualize.
* **`use_sim_time`** (default: `true`): Time context.

---

## Behavior Trees

The autonomous exploration is driven by BTs defined in the `behavior_trees/` directory. The current implementation focuses on a reactive closest-frontier approach.

### Core BT Nodes Provided:
* **`GetExplorationData` (Action):** Fetches current robot pose, frontier points, and map data from the TF tree and costmap via `NavState`.
* **`ChooseFrontierGoal` (Action):** Evaluates available frontiers and selects the optimal next goal (currently using the closest-point policy).
* **`GoToPose` (Action):** Interfaces with the `easynav` GoalManagerClient to send navigation goals and monitor feedback.
* **`IsExplored` (Condition):** Evaluates if the environment is fully mapped. Currently returns SUCCESS when frontier received is empty.

### BT structure:
<img width="1053" height="519" alt="imagen" src="https://github.com/user-attachments/assets/a001ce1f-c6ae-4a86-ba1d-2138bcc6af19" />

---

## Param Files

### 1. `navigation_costmap.params.yaml`
This is the core configuration for the `easynav_system`. It defines how the costmaps are generated and which plugins are active.
* **Key Components:**
  * **`MultiplexorMapsManager`**: Configures how maps from different robots are merged.
  * **`FrontierMapsManager`**: Defines the thresholds for obstacle detection and the clustering parameters for frontier extraction.
  * **Layered Costmaps**: Sets the inflation radius and cost scaling factors.

### 2. `explorer.params.yaml`
Parameters specifically for the Behavior Tree execution node (`explorer`).
* **`bt_xml_file`**: Default path to the Behavior Tree logic.
* **Goal Selection**: Thresholds for the "closest-frontier" policy.
* **IsExplored Model**: Path and confidence thresholds for the CNN-based exploration completion classifier.

### 3. `slam_namespaced.params.yaml`
Custom parameters for the `slam_toolbox` (Sync mode).
* **Frame IDs**: Configured to use relative frames (e.g., `base_link` instead of `/base_link`) to allow the `namespace` launch argument to prepend the correct robot ID.
* **Scan Topic**: Set to `scan_raw` to match the namespaced laser output.

---

### How to modify parameters
You can override any of these files during launch by passing the `params_file` or `slam_params_file` arguments:

```bash
ros2 launch easynav_multirobot_exploration exploration.launch.py \
  params_file:=/path/to/your/custom_explorer.params.yaml
```

> **Note:** When adding new parameters, ensure they are nested under the correct node name and namespace to be correctly parsed by the ROS 2 parameter server.
> ```yaml
> <namespace>/<yournode>:
>   ros__parameters:

## C++ Nodes & Executables

* **`explorer`**: The main executable spun up by `exploration.launch.py`. It registers all custom BT nodes, loads the XML tree from the `behavior_trees/` directory, and ticks the tree at a fixed rate, passing the injected parameters directly into the BehaviorTree blackboard.

---

## License
GPL-3.0-only