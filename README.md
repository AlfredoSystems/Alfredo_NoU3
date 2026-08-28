# Alfredo_NoU3

<img src="https://github.com/AlfredoSystems/alfredosystems.github.io/blob/master/images/nou3-1.jpg" width="400px">

Library for the Alfredo NoU3. Supports motors and servos, has helper methods for different drivetrain types.

## Sensor fusion

IMU orientation (`NoU3.roll`, `NoU3.pitch`, `NoU3.yaw`, and `NoU3.getQuaternion()`) comes from the [VQF](https://github.com/dlaidig/vqf) sensor fusion algorithm (Laidig & Seel, *Information Fusion* 2023), vendored unmodified in `src/vqf/` (MIT License, © Daniel Laidig). Gyroscope bias is calibrated automatically whenever the robot sits still for about 2 seconds and keeps being tracked during motion, so no calibration steps are required. Optional accuracy tune-ups live in the `CalibrateAccel` and `CalibrateGyroScale` examples.

## Where to get started
Go to the the [NoU3 Documentation](https://alfredo-nou3.readthedocs.io/) for tutorials, hardware docs, and API Reference.

## How to build the docs
build the docs locally before pushing.
1) install sphinx
1) run **make html** in **docs** directory
1) run a webserver out of **docs/build/html** by running **python -m http.server 1234**
1) open **localhost:1234**
