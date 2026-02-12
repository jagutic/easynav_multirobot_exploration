# MultiRobot Exploration
TFG John Alejandro Gutiérrez: Exploración Multi-Robot en Easynav.

## EasyNav MultiRobot Exploration Tutorial
### Start the simulator if needed

```bash
ros2 launch easynav_multirobot_exploration multirobot_exploration.launch.py
```
Use ```gui:=false``` to start gazebo headless.
Use ```world:=hospital``` or ```world:=maze``` to use different simulations. 
Use ```size:=small``` or ```size:=big``` to use simulations on different size. 


### Launch SLAM for each robot
Use ```use_sim_time:=true``` or ```use_sim_time:=false``` to alternate between simulation and real robots. 
By default is set to 'true' (simulation).

```bash
# Terminal 1 (robot r1)
ros2 launch multirobot_exploration slam_exploration_namespaced.launch.py namespace:=r1

# Terminal 2 (robot r2)
ros2 launch multirobot_exploration slam_exploration_namespaced.launch.py namespace:=r2
```


### Start EasyNav for each robot in a separate terminal
Use ```use_sim_time:=true``` or ```use_sim_time:=false``` to alternate between simulation and real robots. 
By default is set to 'true' (simulation).

```bash
# Terminal 1 (robot r1)
ros2 run easynav_system system_main \
--ros-args \
--params-file ./src/easynav_multirobot_exploration/multirobot_exploration/config/navigation_costmap.params.yaml \
-r __ns:=/r1 \
-r /tf:=tf -r /tf_static:=tf_static

# Terminal 2 (robot r2)
ros2 run easynav_system system_main \
--ros-args \
--params-file ./src/easynav_multirobot_exploration/multirobot_exploration/config/navigation_costmap.params.yaml \
-r __ns:=/r2 \
-r /tf:=tf -r /tf_static:=tf_static
```


### Launch RViz instances, one per robot
Use ```use_sim_time:=true``` or ```use_sim_time:=false``` to alternate between simulation and real robots. 
By default is set to 'true' (simulation).

```bash
ros2 launch multirobot_exploration rviz_namespaced.launch.py namespace:=r1

ros2 launch multirobot_exploration rviz_namespaced.launch.py namespace:=r2
```

### Tips & Gotchas
- **Namespaces everywhere**: ensure all relative topics in your parameters (e.g., ``scan_raw``)
are resolved under each robot namespace (e.g., ``/r1/scan_raw``).
- **Do not over-remap**: only remap ``/tf`` and ``/tf_static`` to relative topic names if each
robot maintains its own TF tree.
- **Frames**: with ``tf_prefix`` set, refer to frames as ``r1/base_link``, ``r1/odom``, etc.
- **Discovery**: if you have *separate* networks or want isolation, consider different
``ROS_DOMAIN_ID`` per fleet. Otherwise, keep the same domain to allow shared visualization.
