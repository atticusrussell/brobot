# Maps

Maps live here as `<name>.pgm` + `<name>.yaml` and are loaded by
`ballbot_navigation/launch/navigation.launch.py` via the `map:=` argument.
`MAP_NAME` in that file sets the default.

| Map | Space | Captured | Resolution | Extent |
|---|---|---|---|---|
| `bluebrook` | Bluebrook house, first floor (single storey) | 2026-08-11 | 0.05 m/px | 338 × 293 px, 16.9 × 14.7 m |

## `bluebrook`

First map built on a correct TF chain. Captured with `slam_toolbox` in
`online_async` mode, driven by teleop.

Known defects:

- Fan-shaped free space in the upper right is the lidar seeing **through a glass
  door**. Pending a manual trim: paint the door line occupied and the space
  beyond it unknown. Note that this only stops the global planner routing
  through the door — the local costmap is built from live scans and the lidar
  will never see that glass, so it is not a collision guarantee. A
  `nav2_costmap_2d` keepout filter is the real fix.
- Room boundaries are closed and square, but coverage is not exhaustive; some
  edges are single-pass.
