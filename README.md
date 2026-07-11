# MultiRobot Exploration
TFG John Alejandro Gutiérrez: Exploración Multi-Robot en Easynav.

## Videos
<table>
  <tr>
    <td align="center">
      <a href="https://urjc-my.sharepoint.com/:v:/g/personal/ja_gutierrezc_2022_alumnos_urjc_es/IQCxaLYnQm0ZSqLQYqUkaaP3ASkiDDFre1P6z-qKhXuTxvo?nav=eyJyZWZlcnJhbEluZm8iOnsicmVmZXJyYWxBcHAiOiJPbmVEcml2ZUZvckJ1c2luZXNzIiwicmVmZXJyYWxBcHBQbGF0Zm9ybSI6IldlYiIsInJlZmVycmFsTW9kZSI6InZpZXciLCJyZWZlcnJhbFZpZXciOiJNeUZpbGVzTGlua0NvcHkifX0&e=0qsbej">
        <img width="100%" alt="Captura Small" src="https://github.com/user-attachments/assets/b324a9ae-46ed-415e-a68f-0150465b4076"/><br>
        Small, 1 Robot
      </a>
    </td>
    <td align="center">
      <a href="https://urjc-my.sharepoint.com/:v:/g/personal/ja_gutierrezc_2022_alumnos_urjc_es/IQDSYOCRp6mJR4edTc-ZPHPEAecRmuNX-7NbBs8hipGAGRQ?nav=eyJyZWZlcnJhbEluZm8iOnsicmVmZXJyYWxBcHAiOiJPbmVEcml2ZUZvckJ1c2luZXNzIiwicmVmZXJyYWxBcHBQbGF0Zm9ybSI6IldlYiIsInJlZmVycmFsTW9kZSI6InZpZXciLCJyZWZlcnJhbFZpZXciOiJNeUZpbGVzTGlua0NvcHkifX0&e=hWuuam">
        <img width="100%" alt="Captura Medium" src="https://github.com/user-attachments/assets/d1ae683f-0d0b-46dd-a3cb-9e1ed329fe31"/><br>
        Medium, 2 Robots
      </a>
    </td>
    <td align="center">
      <a href="https://urjc-my.sharepoint.com/:v:/g/personal/ja_gutierrezc_2022_alumnos_urjc_es/IQAUgRZshyYXSqD55WNJ-8jWAfzMDe3oZOsFuilRGg22bkY?nav=eyJyZWZlcnJhbEluZm8iOnsicmVmZXJyYWxBcHAiOiJPbmVEcml2ZUZvckJ1c2luZXNzIiwicmVmZXJyYWxBcHBQbGF0Zm9ybSI6IldlYiIsInJlZmVycmFsTW9kZSI6InZpZXciLCJyZWZlcnJhbFZpZXciOiJNeUZpbGVzTGlua0NvcHkifX0&e=pqhLrW">
        <img width="100%" alt="Captura Big" src="https://github.com/user-attachments/assets/b76c3fb5-d36f-40e1-a60d-408ac37b2c64"/><br>
        Big, 3 Robots
      </a>
    </td>
  </tr>
</table>

## Prerequisites

This package has been developed and strictly tested on **Ubuntu 24.04** with **ROS 2 Kilted**.


### Installation & Build Instructions

**1. Create a workspace and clone the repository:**
```bash
mkdir -p ~/easynav_mr_ws/src
cd ~/easynav_mr_ws/src
git clone https://github.com/jagutic/easynav_multirobot_exploration easynav_multirobot_exploration
```

**2. Install dependencies automatically using `rosdep`:**
Make sure you have sourced your ROS 2 installation first.
```bash
cd ~/easynav_mr_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

**3. Build the packages:**
```bash
# Slam toolbox
colcon build --packages-select slam_toolbox --cmake-args   -DCMAKE_MODULE_PATH=~/qt_bridge   -DFORCE_QT6=ON   -DCMAKE_BUILD_TYPE=Release

# Skip navmap pkgs, not used
colcon build --packages-skip easynav_navmap_maps_manager easynav_navmap_planner easynav_navmap_localizer
```

**4. Source the workspace:**
```bash
source install/setup.bash
```


## EasyNav MultiRobot Exploration Tutorial
### Start the simulator

```bash
ros2 launch easynav_multirobot_exploration multirobot_sim.launch.py
```
Use ```gui:=false``` to start gazebo headless.
Use ```world:=hospital``` or ```world:=maze``` to use different simulations. 
Use ```size:=small``` or ```size:=big``` to use simulations on different size. 


### Launch SLAM robot mapping and localization
Use ```use_sim_time:=true``` or ```use_sim_time:=false``` to alternate between simulation and real robots. 
By default is set to 'true' (simulation).

```bash
# Terminal 1 (robot r1)
ros2 launch easynav_multirobot_exploration slam_namespaced.launch.py namespace:=r1

# Terminal 2 (robot r2)
ros2 launch easynav_multirobot_exploration slam_namespaced.launch.py namespace:=r2
```

### Start EasyNav for each robot to be able to navigate
Use ```use_sim_time:=true``` or ```use_sim_time:=false``` to alternate between simulation and real robots. 
By default is set to 'true' (simulation).

```bash
# Terminal 1 (robot r1)
ros2 run easynav_system system_main \
--ros-args \
--params-file ./src/easynav_multirobot_exploration/easynav_multirobot_exploration/config/navigation_costmap.params.yaml \
-r __ns:=/r1 \
-r /tf:=tf -r /tf_static:=tf_static

# Terminal 2 (robot r2)
ros2 run easynav_system system_main \
--ros-args \
--params-file ./src/easynav_multirobot_exploration/easynav_multirobot_exploration/config/navigation_costmap.params.yaml \
-r __ns:=/r2 \
-r /tf:=tf -r /tf_static:=tf_static
```

### Launch Exploration behaviour for each robot
Use ```use_sim_time:=true``` or ```use_sim_time:=false``` to alternate between simulation and real robots. 
By default is set to 'true' (simulation).

```bash
# Terminal 1 (robot r1)
ros2 launch easynav_multirobot_exploration exploration.launch.py namespace:=r1

# Terminal 2 (robot r2)
ros2 launch easynav_multirobot_exploration exploration.launch.py namespace:=r2
```

### Launch RViz instances for visualization
Use ```use_sim_time:=true``` or ```use_sim_time:=false``` to alternate between simulation and real robots. 
By default is set to 'true' (simulation).

```bash
# Terminal 1 (robot r1)
ros2 launch easynav_multirobot_exploration rviz_namespaced.launch.py namespace:=r1

# Terminal 2 (robot r2)
ros2 launch easynav_multirobot_exploration rviz_namespaced.launch.py namespace:=r2
```


### Tips & Gotchas
- **Namespaces everywhere**: ensure all relative topics in your parameters (e.g., ``scan_raw``)
are resolved under each robot namespace (e.g., ``/r1/scan_raw``).
- **Do not over-remap**: only remap ``/tf`` and ``/tf_static`` to relative topic names if each
robot maintains its own TF tree.
- **Frames**: with ``tf_prefix`` set, refer to frames as ``r1/base_link``, ``r1/odom``, etc.
- **Discovery**: if you have *separate* networks or want isolation, consider different
``ROS_DOMAIN_ID`` per fleet. Otherwise, keep the same domain to allow shared visualization.


## System Architecture

### EasyNav plugins
- **Lazy Localizer**: A lightweight localization plugin designed to **reuse SLAM published TFs**. Ensures that different localization sources do not collide or create jumps in the transform tree during the exploration process, where both SLAM and Navigation nned to be executed at the same time.

- **Multiplexor MapsManager**: Works **over** Costmap MapsManager. Subscribes to each robot’s local map topic from the configured namespaces, then uses **OpenCV** to merge multiple `OccupancyGrid` costmaps into a single shared global map. It anchors the merged world around a fixed robot namespace, handles each robot’s relative pose offsets, and writes the final result into `map.base` so the rest of the `easynav` stack can consume a unified base map. Also publishes the fused `OccupancyGrid` for RViz debugging and shared visualization across the fleet.

- **Frontier MapsManager**: Works **over** Costmap MapsManager. Using dynamic map saved at `navstate` (blackboard), extract frontier between free space and the rest of the map using **OpenCV** morphological filters. Uses **DBSCAN clustering** to group frontier points. Publishes them for globla use and saves them in `navstate` for debbuging.

### Exploration Behaviour
The autonomous decision-making flow is orchestrated using a **reactive Behavior Tree** (powered by BehaviorTree.CPP). Instead of pre-calculating a global exploration path, the robot continuously adapts to the environment in real-time.

- **The Exploration Loop:** The system cyclically fetches the latest frontier points and robot poses from the blackboard, applies a configurable frontier-scoring policy (`NEAREST` or `BEST_COST`) to select the optimal target, and dispatches the goal to the `easynav` navigation stack via a `PoseWithCost` message. A cost-hysteresis filter (`ProcessGoal`) prevents oscillations in goal selection. The tree reacts instantly to frontier changes: if a better goal appears mid-navigation, it re-routes the robot without waiting for the current goal to complete.

>*Note: For a detailed technical breakdown of the specific BT action and condition nodes (e.g., `ChooseFrontierGoal`, `ProcessGoal`, `GoToPose`), please refer to the [easynav_multirobot_exploration package README](easynav_multirobot_exploration/README.md).*

### Custom Interfaces
The package `exploration_interfaces` defines the ROS 2 message types used internally by the exploration system:

| Message | Fields | Description |
|---|---|---|
| `PoseWithCost` | `geometry_msgs/Pose pose`, `float64 cost` | Bundles a navigation target pose with its selection cost, used to pass goals between `ChooseFrontierGoal` and `ProcessGoal`. |

### Simulation Environments
Collection of Gazebo environments used for multi-robot validation:
- **Hospital**: The largest and most complex environment for stress-testing path planning.
- **Maze**: Designed for testing wall-following and frontier logic in tight spaces.
- **Maze (Small version)**: Fast-iteration environment for quick tests.

### TO-DO
- Goals blacklist.

- Reactive goals.
 
- Reinforcement Learning for exploration behaviour.

- Automatic multiplexor map coordination.
 
- Common map global topic?
