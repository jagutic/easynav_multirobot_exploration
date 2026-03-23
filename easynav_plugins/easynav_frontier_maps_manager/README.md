# easynav_frontier_maps_manager

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
- **Plugin Name:** `easynav_frontier_maps_manager/FrontierMapsManager`
- **Type:** `easynav::FrontierMapsManager`
- **Base Class:** `easynav::MapsManagerBase`
- **Library:** `easynav_frontier_maps_manager`
- **Description:** Dynamically processes costmaps to find contiguous unknown regions accessible by the robot.

---


## Parameters

### Plugin Parameters (namespace: `/<node_fqn>/<plugin_name>/...`)
| Name | Type | Default | Description |
|---|---|---:|---|
| `obstacle_threshold` | `int` | `FREE_SPACE (0)` | Maximum costmap value to be considered as traversable free space during the flood-fill reachability check. |
| `proximity_radius` | `double` | `0.0` | Radius (in meters) around the robot. Frontiers within this distance are discarded to prevent the robot from targeting its immediate footprint. |
| `clustering` | `bool` | `false` | If true, applies morphological dilation and connected components to group frontier pixels into unified centroids. |
| `dbscan_eps_px` | `int` | `0` | (Clustering only) Pixel radius for the structuring element used to group neighboring frontier points. |
| `dbscan_min_points` | `int` | `0` | (Clustering only) Minimum area (in pixels) required for a cluster to be considered a valid frontier. Filters out noise. |

---

**Example Configuration**

```yaml
maps_manager_node:
  ros__parameters:
    map_types: [costmap, frontier]
    costmap:
      # costmap params
    frontier:
      plugin: easynav_frontier_maps_manager/FrontierMapsManager
      obstacle_threshold: 10
      proximity_radius: 0.5
      clustering: true
      dbscan_eps_px: 3
      dbscan_min_points: 15
```


---

## Interfaces

### Subscriptions and Publications
| Direction | Topic | Type | Purpose | QoS |
|---|---|---|---|---|
| Publisher | `<node_fqn>/<plugin>/points` | `visualization_msgs/msg/Marker` | Publishes visual markers of the detected frontiers for RViz debugging (points or spheres depending on clustering mode). | `depth=10, transient_local, reliable` |


## NavState Keys (Blackboard)
| Key | Type | Access | Notes |
|---|---|---|---|
| `robot_pose` | `nav_msgs::msg::Odometry` | **Read** | Used as the seed point for the flood-fill reachability algorithm and proximity filtering. |
| `map.dynamic` | `Costmap2D` | **Read** | The dynamic costmap containing `FREE_SPACE`, `LETHAL_OBSTACLE`, and `NO_INFORMATION` values. |
| `frontier` | `std::vector<geometry_msgs::msg::Point>` | **Write** | The list of extracted frontier points/centroids exposed to the rest of the `easynav` system. |

---

## TF Frames
| Role | Transform | Notes |
|---|---|---|
| Publishes | — | This manager does not broadcast TF; costmaps use their internal `frame_id`. |

---

## License
GPL-3.0-only
