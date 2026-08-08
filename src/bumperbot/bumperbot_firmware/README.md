# bumperbot_firmware

Hardware interface for the physical robot: the serial `hardware_interface` plugin,
Arduino-side transmit/receive helpers, and the MPU6050 IMU driver. Simulation never
loads this package; `real_robot.launch.py` does. Course-lineage code.
