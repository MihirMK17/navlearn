# navlearn_assets

Campaign environments: the bookstore world with its vendored models, and the occupancy
maps for every environment beyond `small_house`. Worlds and their maps version together
because they are provenance-linked — each `*_geom` map is generated from a specific
world's collision geometry at the robot's LiDAR height (0.1538 m) by
`ros2 run navlearn_analysis world_to_map`, and the paper's localization numbers rest on
that chain.

## Integration contract

An ament environment hook prepends this package's `worlds/` and `models/` onto
`GZ_SIM_RESOURCE_PATH` at source time. `bumperbot_description`'s `gazebo.launch.py`
searches that variable for `worlds/<name>.world` and appends it for `model://`
resolution — so `world_name:=bookstore` works without the robot stack knowing this
package exists.

## Contents

| dir | contents |
|---|---|
| `worlds/` | `bookstore.world` (aws-robomaker retail, vendored 2026-08-01) |
| `models/` | 34 `aws_robomaker_retail_*` model dirs |
| `maps/` | `bookstore`, `small_warehouse` (maps shipped with the AWS worlds — retained for provenance, **not used by any campaign**) and `bookstore_geom`, `small_warehouse_geom` (the generated maps campaigns run on; see each `PROVENANCE.md` and `maps/README.md` for why the shipped warehouse map was rejected) |

Upstream is aws-robomaker (MIT-0). Its repositories were archived 2026-07-21, so
vendoring is also the only way these assets stay reachable.

## Known vendored defect — frozen deliberately

`aws_robomaker_retail_Spotlight_01/model.sdf` references
`meshes/aws_Spotlight_01_{visual,collision}.DAE`, but the shipped files are named
`aws_spotlight_01_*.DAE` (lower case). On a case-sensitive filesystem the meshes never
load — six load-time errors, a cosmetic light fixture missing from the scene. Every
banked bookstore campaign ran with exactly this defect. It is left as-is on purpose:
fixing the case would add collision geometry to the LiDAR scene and silently break
comparability with the collected dataset. If a future campaign fixes it, that campaign
starts a new comparability era and says so.
