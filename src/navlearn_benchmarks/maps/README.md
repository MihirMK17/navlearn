# Campaign occupancy maps

Environments for the three-map leg of the Paper 1 campaign. `small_house` is **not** here —
it predates the campaign, lives in `bumperbot_mapping/maps/small_house`, and the bringup
resolves it from there.

## Provenance

| map | upstream | licence | file |
|---|---|---|---|
| `small_warehouse` | [aws-robomaker-small-warehouse-world](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world) | MIT-0 | `maps/002/map.{pgm,yaml}` |
| `bookstore` | [aws-robomaker-bookstore-world](https://github.com/aws-robotics/aws-robomaker-bookstore-world) | MIT-0 | `maps/turtlebot3_waffle_pi/map.{pgm,yaml}` |

Both were fetched 2026-07-30. **AWS archived every aws-robomaker repository on 2026-07-21**,
so upstream will not change again and vendoring is the only way these stay reachable.

## Known limitation — these are not our robot's maps

Both shipped maps were built by SLAM on a **TurtleBot3 Waffle Pi**, whose LiDAR sits at a
different height from ours. In a warehouse that is not cosmetic: at one height a scan
returns shelf *legs*, at another it returns shelf *faces*, and the occupancy grid differs
accordingly.

That is acceptable for the **map-selection spread gate**, which asks whether a predictor
varies across an environment at all — a question about the building, not about our sensor.

It is **not** settled for campaign data collection. Re-running SLAM with our own robot so
all three maps share provenance is a `PROTOCOL.md` decision, not a default. Recording it
here so the choice is visible rather than inherited.

## Why these two environments

Chosen by a pre-registered criterion computed from the occupancy grid alone, before any
simulation: measured spread of the candidate predictors over free space
(`scripts/map_spread_gate.py`). No outcome data is involved, so the criterion is orthogonal
to the hypothesis being tested.

Measured 2026-07-30 at a common 0.05 m grid, 200 poses per map:

| map | entropy IQR (bits) | survivability IQR/median |
|---|---|---|
| small_house | 0.000 | 0.220 |
| small_warehouse | 0.000 | 0.314 |
| bookstore | 0.000 | 0.276 |

Entropy is degenerate on **all three** — repeated shelf rows and repeated bookstore aisles
are not globally ambiguous to a 12 m 360-degree scan, because enough global structure stays
in view even where local structure repeats. Survivability has usable spread everywhere.

So the three-map set earns its place by giving the survivability claim cross-environment
generalisation (residential, industrial, retail), **not** by providing an ambiguity
gradient. Spread is necessary, not sufficient: it shows the predictor *can* discriminate,
not that it *does* predict recovery. That is what the campaign's predictor leg tests.

## Reproducing the gate

```bash
python3 src/navlearn_benchmarks/scripts/map_spread_gate.py \
    --map small_house=src/bumperbot_mapping/maps/small_house/map.yaml \
    --map small_warehouse=src/navlearn_benchmarks/maps/small_warehouse/map.yaml \
    --map bookstore=src/navlearn_benchmarks/maps/bookstore/map.yaml \
    --out results/map_spread_gate
```

## Still outstanding

The Gazebo **worlds** are not vendored. `bumperbot_description/worlds` is read-only under
the scope rules, and the bringup resolves `world_name` against it, so placing them needs
either approval for that package or a world-path launch argument. `small_warehouse.world`
is already present there; `bookstore.world` is not.

## small_warehouse_geom — added 2026-08-05, and why the shipped map is not used

`small_warehouse/` is the map shipped with the AWS RoboMaker world. It is retained for
provenance and **is not used by any campaign cell.** Measured against the world's own
collision geometry sliced at this robot's LiDAR height (0.1538 m) by
`scripts/world_to_map.py`, it is not merely offset from the world frame — its occupied
extent spans 21.2 x 14.6 m where the world's geometry spans 14.0 x 20.9 m, a rotation of
roughly 90 degrees. Only 3.1% of its occupied cells lie within one cell of real geometry.
Ground-truth error measured against it would have been measured in the wrong frame.

`small_warehouse_geom/` replaces it: generated from the world's collision geometry — the
same geometry the simulator uses for ray intersection — sliced at 0.1538 m at 0.05 m/cell.
See its `PROVENANCE.md` for the exact invocation and counts.

The same tool was turned on `small_house`, whose map underpins legs 1-6 and the yaw leg:
median disagreement 0.000 m, 86.5% of occupied cells within one cell, 95.8% within
0.10 m. That map is geometrically faithful and needed no replacement, which is what makes
the two environments comparable despite being mapped by different means. Both validation
reports are in `results/maps/`.

Limitation, stated rather than buried: a geometric slice includes geometry no robot could
observe from inside the world, so it is a map of the world rather than of what is
observable. For isolating filter behaviour against a known world that is the stronger
choice, but it is not the object SLAM produces.
