# bumperbot_controller

Drive-level control for the differential base: the `ros2_control` spawners, a simple
and a noisy velocity controller, `twist_relay`, and joystick teleop (via `twist_mux`).
The benchmark stack measures against `/bumperbot_controller/cmd_vel`, which originates
here. Course-lineage code (Antonio Brandi's BumperBot base, Apache-2.0).
