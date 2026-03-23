# easynav_multiplexor_maps_manager

[![ROS 2: kilted](https://img.shields.io/badge/ROS%202-kilted-blue)](#)

## Description


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
| `<namespace>.topic` | `string` | *Required* | The specific map topic name (without the namespace prefix) to subscribe to for this robot. |

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
| Subscription | `/<namespace>/<topic>` | `nav_msgs/msg/OccupancyGrid` | Dynamically subscribes to the map topic of each robot defined in `robot_namespaces`. | `depth=1, transient_local, reliable` |
| Publisher | `<node_fqn>/<plugin_name>/map` | `nav_msgs/msg/OccupancyGrid` | Publishes the final merged global map for visualization (e.g., RViz). | `depth=1, transient_local, reliable` |


## NavState Keys (Blackboard)
| Key | Type | Access | Notes |
|---|---|---|---|
| `map.static` | `Costmap2D` | **Write** | The final multiplexed map is written here so the rest of the navigation stack can use it. |
| `map.static.update` | `bool` | **Write** | Flag set to `true` whenever a new multiplexed map has been successfully generated. |
| `map_time` | `rclcpp::Time` | **Read** | Timestamp used to stamp the outgoing merged `OccupancyGrid` message. |

---

## TF Frames
| Role | Transform | Notes |
|---|---|---|
| Publishes | — | This manager does not broadcast TF; costmaps use their internal `frame_id`. |

---

## License
GPL-3.0-only
