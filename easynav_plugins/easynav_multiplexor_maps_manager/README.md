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
- **Description:** 

---

## Parameters

### Plugin Parameters (namespace: `/<node_fqn>/easynav_multiplexor_maps_manager/MultiplexorMapsManager/...`)
| Name | Type | Default | Description |
|---|---|---:|---|
| `<plugin>.package` | `string` | `""` | Package name used to resolve relative map paths via `ament_index`. |
| `<plugin>.map_path_file` | `string` | `""` | Relative path (inside the package) to a ROS 1–style YAML map (with `image`, `resolution`, `origin`, etc.). |
| `<plugin>.filters` | `string[]` | `[]` | List of filter identifiers to be instantiated (see section below). |
| `<plugin>.<filter>.plugin` | `string` | `""` | Type of filter plugin (e.g., `easynav_multiplexor_maps_manager/InflationFilter`). |

---




**Example Configuration**

```yaml
maps_manager_node:
  ros__parameters:
    map_types: [costmap]
    costmap:
      plugin: easynav_multiplexor_maps_manager/MultiplexorMapsManager
      package: my_maps_pkg
      map_path_file: maps/warehouse.yaml
      filters: [inflation, obstacles]
      inflation:
        plugin: easynav_multiplexor_maps_manager/InflationFilter
        inflation_radius: 0.3         # default in code
        cost_scaling_factor: 3.0      # default in code
      obstacles:
        plugin: easynav_multiplexor_maps_manager/ObstacleFilter
```

---

## Interfaces (Topics and Services)

### Subscriptions and Publications
| Direction | Topic | Type | Purpose | QoS |
|---|---|---|---|---|
| Subscription | `<node_fqn>/<plugin>/incoming_map` | `nav_msgs/msg/OccupancyGrid` | Input occupancy map used to update the dynamic map. | `depth=1, transient_local, reliable` |
| Publisher | `<node_fqn>/<plugin>/map` | `nav_msgs/msg/OccupancyGrid` | Publishes the static costmap. | `depth=1` |
| Publisher | `<node_fqn>/<plugin>/dynamic_map` | `nav_msgs/msg/OccupancyGrid` | Publishes the dynamic (live) costmap. | `depth=100` |

### Services
| Direction | Service | Type | Purpose |
|---|---|---|---|
| Service Server | `<node_fqn>/<plugin>/savemap` | `std_srvs/srv/Trigger` | Saves the current costmap(s) to disk. |

---

## NavState Keys
| Key | Type | Access | Notes |
|---|---|---|---|
| `map.static` | `Costmap2D` | **Write** | Static map loaded from YAML. |
| `map.dynamic` | `Costmap2D` | **Write** | Dynamic map after applying filters. |
| `map.dynamic.filtered` | `Costmap2D` | **Read** | Previously filtered map used as input if available. |

---

## TF Frames
| Role | Transform | Notes |
|---|---|---|
| Publishes | — | This manager does not broadcast TF; costmaps use their internal `frame_id`. |

---

## License
GPL-3.0-only
