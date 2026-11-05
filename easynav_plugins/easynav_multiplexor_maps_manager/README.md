# easynav_multiplexor_maps_manager

[![ROS 2: kilted](https://img.shields.io/badge/ROS%202-kilted-blue)](#)

## Description
Merges each configured robot's `OccupancyGrid` into a shared map using its configured initial pose relative to a fixed robot namespace. The result is stored in NavState for the navigation stack and published for RViz.

## Authors and Maintainers
- **Authors:** 
- **Maintainers:**

## Supported ROS 2 Distributions
| Distribution | Status |
|---|---|
| kilted | ![kilted](https://img.shields.io/badge/kilted-supported-brightgreen) |

## Plugin (pluginlib)
- **Plugin Name:** `easynav_multiplexor_maps_manager/MultiplexorMapsManager`
- **Type:** `easynav::MultiplexorMapsManager`
- **Base Class:** `easynav::MapsManagerBase`
- **Library:** `easynav_multiplexor_maps_manager`
- **Description:** Combines and manages multiple incoming costmaps or occupancy grids into a single state.

---


## Parameters

### Plugin Parameters (namespace: `/<node_fqn>/<plugin_name>/...`)
| Name | Type | Default | Description |
|---|---|---:|---|
| `robot_namespaces` | `string[]` | *Required* | List of robot namespaces (identifiers) whose maps will be merged. |
| `fixed_map_ns` | `string` | *Required* | The namespace of the main robot. Its map will be used as the central anchor (origin) for the global merged map. |
| `<namespace>.x` | `double` | `0.0` | Initial relative X position of the specific robot with respect to the world. |
| `<namespace>.y` | `double` | `0.0` | Initial relative Y position of the specific robot with respect to the world. |
| `<namespace>.Y` | `double` | `0.0` | Initial relative Yaw (theta) orientation of the specific robot with respect to the world. |
| `<namespace>.topic` | `string` | *Required* | Map topic name for this robot, without the namespace prefix (for example, `local_map`). |

---


**Example Configuration**

```yaml
maps_manager_node:
  ros__parameters:
    map_types: [multiplexor, costmap]
    multiplexor:
      plugin: easynav_multiplexor_maps_manager/MultiplexorMapsManager
      robot_namespaces: ["r1", "r2"]
      fixed_map_ns: "r1"
      r1:
        x: 0.0
        y: 0.0
        Y: 0.0
        topic: "map"  # Subscribes to /r1/map
      r2:
        x: 2.5
        y: 1.0
        Y: 1.57
        topic: "map"  # Subscribes to /r2/map
    
    costmap:
     # costmap params
```

---

---

## Interfaces

### Subscriptions and Publications
| Direction | Topic | Type | Purpose | QoS |
|---|---|---|---|---|
| Subscription | `/<namespace>/<topic>` | `nav_msgs/msg/OccupancyGrid` | Subscribes to each configured robot's map topic. | `depth=1, volatile, best_effort` |
| Publisher | `<node_fqn>/<plugin_name>/map` | `nav_msgs/msg/OccupancyGrid` | Publishes the final merged global map for visualization (e.g., RViz). | `depth=1, transient_local, reliable` |


## NavState Keys (Blackboard)
| Key | Type | Access | Notes |
|---|---|---|---|
| `map.base` | `Costmap2D` | **Write** | The final multiplexed map used by the rest of the navigation stack. |
| `map_time` | `rclcpp::Time` | **Read** | Timestamp used to stamp the outgoing merged `OccupancyGrid` message. |

---

## TF Frames
| Role | Transform | Notes |
|---|---|---|
| Publishes | Global map -> fixed robot map; map -> robot | Establishes the fixed map anchor and broadcasts the current robot transform used by the shared map. |

---

## License
GPL-3.0-only
