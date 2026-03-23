# MultiRobot Exploration
TFG John Alejandro Gutiérrez: Exploración Multi-Robot en Easynav.

## Dependencies & Prerequisites

This package has been developed and strictly tested on **Ubuntu 24.04** with **ROS 2 Kilted**.
To successfully build and run the multi-robot exploration nodes, you need the following system and ROS 2 dependencies:

* **ROS 2 Base:** `rclcpp`, `nav_msgs`, `geometry_msgs`, `visualization_msgs`
* **TF2 Ecosystem:** `tf2_ros`, `tf2_geometry_msgs` (for global and local frame transformations).
* **BehaviorTree.CPP (v4):** Used for the decision-making and execution flow of the exploration nodes.
* **OpenCV (cv2):** Core requirement for the image-processing morphological algorithms used in frontier detection and maps multiplexor.


## Installation & Build Instructions

**1. Create a workspace and clone the repository:**
```bash
mkdir -p ~/easynav_mre_ws/src
cd ~/easynav_mre_ws/src
git clone https://github.com/jagutic/easynav_multirobot_exploration easynav_multirobot_exploration
```

**2. Install dependencies automatically using `rosdep`:**
Make sure you have sourced your ROS 2 installation first.
```bash
cd ~/multi_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
*(Manual fallback if rosdep fails: `sudo apt install libopencv-dev ros-humble-behaviortree-cpp`)*

**3. Build the packages:**
```bash
colcon build --symlink-install
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

- **Multiplexor MapsManager**: Works **under** Costmap MapsManager. Receive all maps to mux through the topic passed as parameter. Uses **OpenCV** to handle the merging of multiple `OccupancyGrid` into a global map. This allows robots to share their knowledge of the world. Set this new muxed map as static map for all easynav system.

- **Frontier MapsManager**: Works **over** Costmap MapsManager. Using dynamic map saved at navstate (blackboard), extract frontier between free space and the rest of the map using **OpenCV** morphological filters. Erase noise for better use. Publishes frontier points for behaviours to use it, and saves it in navstate (blackboard) for debugging.

### Exploration Behaviour
The autonomous decision-making flow is orchestrated using a **reactive Behavior Tree** (powered by BehaviorTree.CPP). Instead of pre-calculating a global exploration path, the robot continuously adapts to the environment in real-time.

- **The Exploration Loop:** The system cyclically fetches the latest frontier points and dynamic maps from the blackboard, applies a **closest-frontier** policy to select the optimal target, and dispatches the goal to the `easynav` navigation stack. If a goal fails or is reached, the tree reacts instantly to select the next best frontier.

- **Mission Completion:** To determine when to stop, the behavior relies on the **IsExploredModel**, a custom-trained **CNN** (Convolutional Neural Network) that visually classifies the global map to decide if the area is fully explored, triggering a clean mission termination.

>*Note: For a detailed technical breakdown of the specific BT action and condition nodes (e.g., `ChooseFrontierGoal`, `GoToPose`), please refer to the [easynav_multirobot_exploration package README](easynav_multirobot_exploration/README.md).*

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