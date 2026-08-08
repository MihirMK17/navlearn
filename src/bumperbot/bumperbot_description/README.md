# bumperbot_description

The robot itself: URDF/xacro, meshes, the three course worlds
(`empty`, `small_house`, `small_warehouse`) and their models, plus the two nodes that
belong to the simulated robot's sensor path.

BumperBot originates from Antonio Brandi's *Self Driving and ROS 2* course material
(Apache-2.0); this package is the least-modified part of that lineage.

## The scan chain

```
gz sensor → ros_gz bridge → /scan_unfiltered → [scan_rate_governor]* → scan_sanitizer → /scan
```

- `scan_sanitizer` (here): clamps RPLidar-A1 ranges (0.12–12.0 m), maps invalid returns
  to `inf`. Always on — `/scan` is the sanitized stream, everywhere, always.
- `scan_rate_governor` (*optional, from `navlearn_benchmarks`*): composed by
  `bumperbot_bringup` only when a campaign starves the sensor; this package knows
  nothing about it.

## World resolution and models

`gazebo.launch.py` resolves `world_name` by searching this package's `worlds/` first,
then every `GZ_SIM_RESOURCE_PATH` entry for `worlds/<name>.world` — which is how asset
packages (e.g. `navlearn_assets`, the bookstore) contribute worlds without touching
this package. The launch *appends* to `GZ_SIM_RESOURCE_PATH` rather than overwriting
it; that append is load-bearing for the whole mechanism.

GPU note: rendering runs on the discrete GPU via PRIME offload (`gpu:=true` default).
The offload environment is set here because this launch spawns the `ign gazebo`
process.

## Simulation constraint worth knowing

Contact sensors must reference PRIMITIVE collisions (sphere/cylinder/box). `base_link`
carries a mesh collision; referencing a mesh in a contact sensor hangs Ignition
Fortress for ~6 minutes at spawn and breaks the map frame.
