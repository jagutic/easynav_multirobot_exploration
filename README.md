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

**3. Build the package:**
```bash
colcon build --symlink-install --packages-select easynav_multirobot_exploration
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

### Launch Exploration for each robot
Use ```use_sim_time:=true``` or ```use_sim_time:=false``` to alternate between simulation and real robots. 
By default is set to 'true' (simulation).

```bash
# Terminal 1 (robot r1)
ros2 launch easynav_multirobot_exploration exploration.launch.py namespace:=r1

# Terminal 2 (robot r2)
ros2 launch easynav_multirobot_exploration exploration.launch.py namespace:=r2
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

### Behavior Tree
Decision-making flow orchestration using **BehaviorTree.CPP**. Visualization and real-time monitoring are fully supported via **Groot2** for debugging the exploration states.

- **MuxMaps (Map Merger)**
A specialized node that handles the synchronization and merging of multiple `OccupancyGrid` streams into a unified global frame using affine transformations. This allows robots to share map data in a common coordinate system.

- **DetectFrontier (OpenCV-based)**
High-performance frontier extraction leveraging OpenCV morphological filters. It uses operations like **Hit-or-Miss** and **Dilate** to accurately identify clean boundaries between free and unknown space, filtering out sensor noise.

- **ChooseFrontierGoal**


## Other Components

### **Simulation Environments**
Collection of Gazebo environments used for multi-robot validation:
- **Hospital**: The largest and most complex environment for stress-testing path planning.
* **Maze (Big version)**: Designed for testing wall-following and frontier logic in tight spaces.
* **Maze (Small version)**: Fast-iteration environment for quick logic verification.

###  **LazyLocalizer EasyNav Plugin**
- A **lightweight localization plugin** designed to bridge SLAM-published TFs. It ensures that different localization sources do not collide or create jumps in the transform tree during the exploration process.
