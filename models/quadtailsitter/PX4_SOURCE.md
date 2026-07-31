# PX4 model provenance

The files in this directory were imported from
[`PX4/PX4-gazebo-models`](https://github.com/PX4/PX4-gazebo-models), model
`quadtailsitter`, on 2026-07-29.

- Upstream model: Swan K1 quad tailsitter, provided by Holybro.
- Upstream author metadata: Pernilla Wikström / Auterion.
- Upstream license: BSD 3-Clause (`PX4/PX4-gazebo-models`).
- Preserved: body and propeller meshes, mass, inertia, collision geometry,
  motor positions and directions, motor constants, and all
  `AdvancedLiftDrag` coefficients.
- Port changes: the unused PX4 Gazebo airspeed include was removed because
  ArduPlane's SITL airspeed backend derives its sample from the external JSON
  flight-dynamics state; the IMU sensor pose is rotated into ArduPilot's FRD
  aircraft axes because `ArduPilotPlugin` consumes IMU samples directly.
  Deprecated `motorNumber` tags were renamed to `actuator_number`, explicit
  motor namespaces were added, and unused sensor `gz_frame_id` extensions were
  removed to keep the port warning-free on Gazebo Harmonic. These changes do
  not alter physical geometry or coefficients.

The ArduPilot bridge is deliberately kept in the sibling
`quadtailsitter_with_ardupilot` wrapper so the imported physical model remains
easy to compare with upstream.
