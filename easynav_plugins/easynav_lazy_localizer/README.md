# easynav_lazy_localizer

[![ROS 2: kilted](https://img.shields.io/badge/ROS%202-kilted-blue)](#)

## Description
Uses the existing map-to-robot TF to update `robot_pose` in NavState. It does not compute localization or publish a competing transform, allowing SLAM or another localization source to remain authoritative.

## Authors and Maintainers
- **Authors:** 
- **Maintainers:** 

## Supported ROS 2 Distributions
| Distribution | Status |
|---|---|
| kilted | ![kilted](https://img.shields.io/badge/kilted-supported-brightgreen) |

## Plugin (pluginlib)
- **Plugin Name:** `easynav_lazy_localizer/LazyLocalizer`
- **Type:** `easynav::LazyLocalizer`
- **Base Class:** `easynav::LocalizerMethodBase`
- **Library:** `easynav_lazy_localizer`
- **Description:** Lazy localizer, only copies existing tf.


## TF Frames
| Role | Transform | Notes |
|---|---|---|
| Requires | Configured map frame -> configured robot frame | Another node (for example SLAM) must publish this transform; common robot frames include `base_link` and `base_footprint`. |


## License
GPL-3.0-only
