# relic-recovery [![Build Status](https://travis-ci.org/acmerobotics/relic-recovery.svg?branch=master)](https://travis-ci.org/acmerobotics/relic-recovery)
Code used by ACME Robotics (FTC Team #8367) during the 2017-2018 season, Relic Recovery.

## Overview

### `RobotLib` Module

* `com.acmerobotics.library.cameraoverlay` — Contains the server for a simple DS camera overlay stream. It takes bitmaps from OpenCV/Vuforia, encodes them into JPEGs, and sends them to the client running on the DS (similar to MJPEG).
  * `CameraStreamServer` automatically creates a TCP socket that listens for connections from the corresponding client running on the DS. Once connected, `send()` can be used to send bitmaps to be displayed on the DS side. Alternatively, `getTracker()` returns a tracker that can be seamlessly integrated with the vision model described below.
  * The DS overlay app is located in `doc/FtcCameraOverlay.apk`. It uses an overlay and an accessibility service, complying with the rule against modifying the DS app.

**TODO**: image?

* `com.acmerobotics.library.dashboard` — The backend for a websocket-based dashboard. It delivers telemetry and information about the robot's pose and current path for visualization. Additionally, it uses reflection to facilitate configuration variables that can be set while an OpMode is running. More information (including usage) can be found in `dashboard/README.md`.
  * `RobotDashboard.getInstance()` can be used to get an instance of the dashboard. Telemetry messages can be sent through `sendTelemetryPacket()` (including the field overlay) or alternatively through `getTelemetry()` (note that the `Telemetry` instance returned only implements the most commonly-used methods of that interface). Numeric telemetry values are automatically available to graph over time.
  * Configuration variables can be set up by annotating classes that contain configurable fields with `@Config`. Then `static`, non-`final` variables of common types will be automatically added as configuration variables. Whenever the values of these variables are changed in the dashboard, the changes will be automatically applied (note that changes made through the dashboard _do not_ persist between app restarts).
  * Telemetry messages are designed to be flexible. To add additional fields, either modify `TelemetryPacket` or subclass it, and they will be sent to the frontend (see `dashboard/README.md` for advice on modifying the frontend).

**TODO**: image?

* `com.acmerobotics.library.hardware` — Contains a few standard drivers for sensors used this season and in the past. Most notably, the Sharp IR sensor response has been linearized to provide actual distance. In addition, there is code to bypass the read window for REV hub I2C devices to cut down on unnecessary reads (REV I2C command duration is _linear_ in the number of registers).
  * `LynxOptimizedI2cFactory.createLynxI2cDeviceSynch()` can be used to create `I2cDeviceSynch` instances for REV without the read window to improve performance.

* `com.acmerobotics.library.localization` — Immutable localization primitives.

* `com.acmerobotics.library.motion` — Contains generic, jerk-limited profile generation code and PID(F) controllers. Although primarily used for drivetrain motion, the motion profiling code can be used for any mechanism. The PID controller implementation is fairly standard and includes extensions for velocity and acceleration feedforward.
  * `MotionProfileGenerator.generateProfile()` uses the provided start state, goal state, and constraints to generate a 1D, jerk-limited motion profile. The `MotionProfile` instance can then be used to find the `MotionState` at any point in time (all profiles are continuous).

* `com.acmerobotics.library.path` — Provides basic path constructs and trajectory generation based on the aforementioned 1D motion profiling system. The path primitives include point turns and pure translations as well as cubic and quintic Hermite splines. These primitives can be arbitrarily combined into fluid, continuous trajectories.

* `com.acmerobotics.library.vision` — Contains a unified interface for switching between Vuforia-based and OpenCV-based camera views, including a generic overlay system that even allows drawing on Vuforia camera views. Also defines a tracker system used for managing different detectors.

* `com.acmerobotics.library.util` — Supporting utility classes and data structures.

* `com.acmerobotics.relicrecovery.configuration` — OpMode configuration activity and supporting types for various game elements.

**TODO**: image?

* `com.acmerobotics.relicrecovery.vision` — OpenCV-based trackers (detectors/pipelines) for various field elements.

### `TeamCode` Module
* `com.acmerobotics.relicrecovery.localization` — Various localization methods used by the drivetrain (`MecanumDrive`).

* `com.acmerobotics.relicrecovery.opmodes` — Contains all OpModes and a few additional classes to aid in their creation.

* `com.acmerobotics.relicrecovery.subsystems` — Contains subsystem classes that abstract low-level hardware interactions. The `Robot` class handles updating the myriad subsystems.

### `VisionTest` Module
Standalone app for testing trackers.