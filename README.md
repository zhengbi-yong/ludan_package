# Ludan Package

The Ludan package bundles the embedded firmware that drives the Ludan right arm
DM-series actuators together with a ROS 1 workspace that exposes those
actuators, the associated IMU, and example robot descriptions to higher-level
applications.  This repository is intended to be checked out as a whole so that
low-level motor control and high-level planning remain in sync.

## Repository layout

| Path | Description |
| --- | --- |
| `DM-motor-right-arm/` | STM32H7 firmware project that controls the DM actuators over CAN and exposes state via USB CDC. |
| `ludan_ws_ros1/` | ROS 1 (catkin) workspace containing hardware interfaces, controllers, drivers, and example applications. |
| `README.md` | Project overview (this file). |

### Firmware highlights (`DM-motor-right-arm/`)
* Generated from STM32CubeMX (`CtrlBoard-H7_IMU.ioc`) for an STM32H7 MCU with
  FreeRTOS support and peripherals for ADC, FDCAN1/2/3, SPI BMI088 IMU, UART,
  USB CDC, PWM timers, and DMA.【F:DM-motor-right-arm/Core/Src/main.c†L1-L74】
* Application code is organised into BSP (`User/Bsp`), controller logic
  (`User/Controller`), and FreeRTOS tasks (`User/APP`) that each drive specific
  DM-series actuators (e.g. `chassisR_task` for the right leg CAN bus and
  `body_task` for the waist joint).【F:DM-motor-right-arm/User/APP/chassisR_task.c†L1-L117】【F:DM-motor-right-arm/User/APP/body_task.c†L1-L92】
* Uses manufacturer MIT-mode commands (`mit_ctrl*`) and supports saving motor
  zero offsets, slope following for smooth setpoints, and IMU fusion via the
  BMI088 and Mahony filters.

### ROS workspace highlights (`ludan_ws_ros1/`)
* Built for ROS 1 (tested with Noetic) with packages for hardware abstraction,
  controller plugins, serial communication with the firmware, MoveIt
  configurations, and sensor drivers.
* `dmbot_serial`: serial bridge that reads actuator state from the embedded
  controller and writes MIT-style commands for up to 14 DM motors.  Motor types,
  command limits, and binary protocol framing are encoded in
  `robot_connect.h/cpp`.【F:ludan_ws_ros1/src/dmbot_serial/include/dmbot_serial/robot_connect.h†L1-L152】【F:ludan_ws_ros1/src/dmbot_serial/src/robot_connect.cpp†L5-L121】
* `legged_hw`: ROS control hardware interface that publishes joint states,
  exposes a hybrid joint interface, and runs a high-rate control loop with a
  configurable realtime priority.【F:ludan_ws_ros1/src/legged_hw/src/LeggedHW.cpp†L1-L46】【F:ludan_ws_ros1/src/legged_hw/src/LeggedHWLoop.cpp†L1-L94】
* `simple_hybrid_joint_controller`: ROS-control plugin that streams hybrid
  commands (position/velocity/Kp/Kd/feed-forward torque) and provides several
  convenience topics (`command_same`, `command_pos_all`, `command_matrix`,
  `command_one`, `command_moveJ`).【F:ludan_ws_ros1/src/simple_hybrid_joint_controller/src/AllJointsHybridController.cpp†L1-L139】
* `right_arm_hw`: translates `FollowJointTrajectory` actions from MoveIt into
  hybrid joint commands for the right arm joints, implementing linear
  interpolation before publishing to the `command_moveJ` topic.【F:ludan_ws_ros1/src/right_arm_hw/src/ftj_to_moveJ_bridge.cpp†L1-L142】
* Additional packages deliver URDFs (`legged_examples`, `wanren_arm`), GUI
  scripts (`motor_control_gui4a.py`), test nodes (`test_led`), and a Yesense IMU
  driver (`yesense_ros`).

## Getting started

### Prerequisites
* STM32CubeIDE or Keil MDK for building and flashing the firmware project.
* ROS 1 (recommended: Ubuntu 20.04 + ROS Noetic) with `catkin_tools` or
  `catkin_make`, `rosserial_python`, `serial`, and `ros-control` related
  dependencies.
* Access to the Ludan right arm hardware with DM actuators connected to the
  STM32 controller board and exposed to the host PC via a USB serial port.

### Building and flashing the firmware
1. Open `DM-motor-right-arm/CtrlBoard-H7_IMU.ioc` in STM32CubeIDE to regenerate
   code or inspect peripheral assignments.
2. Build the project with your preferred toolchain (CubeIDE or Keil MDK project
   under `DM-motor-right-arm/MDK-ARM`).
3. Flash the resulting binary to the STM32H7 controller.
4. On first boot, ensure each motor is enabled and calibrated using the provided
   FreeRTOS tasks (`ChassisR_task`, `Body_task`); zero offsets can be stored by
   setting the `a`/`c` flags in the application code if required.

### Building the ROS workspace
```bash
cd ludan_package/ludan_ws_ros1
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

### Running the system
1. Connect the controller via USB; the default serial port name is
   `/dev/mcu_rightarm` with baud `921600` (configurable via ROS parameters).
2. Start the serial bridge and hardware interface, for example:
   ```bash
   roslaunch dmbot_serial test_motor.launch
   roslaunch right_arm_hw bringup.launch
   ```
3. Load the hybrid controller and MoveIt pipeline:
   ```bash
   roslaunch simple_hybrid_joint_controller bringup_real.launch
   roslaunch right_arm_moveit_config moveit_rviz.launch
   ```
4. To send a scripted MoveIt trajectory, use the provided bridge node
   (`ftj_to_moveJ_bridge`) that converts `FollowJointTrajectory` goals into
   hybrid commands.

### Development tips
* The hybrid controller topics accept `std_msgs/Float64MultiArray` commands. Use
  `rostopic pub` or custom scripts to stream setpoints.  The `command_matrix`
  topic expects `N×5` values ordered as `[pos, vel, kp, kd, ff]` per joint.
* `dmbot_serial` republishes joint states on `/joint_states`; remap or record
  them for logging and use in MoveIt.
* The firmware publishes IMU data via USB CDC; integrate with ROS using
  `yesense_ros` to visualise orientation in RViz (`run_with_rviz.sh`).

## Known limitations
* **Hard-coded motor configuration.** Motor IDs, CAN bus assignments, and even
  DM model types are compiled into the firmware tasks and the ROS serial bridge,
  limiting reuse for different hardware layouts without recompilation.【F:DM-motor-right-arm/User/APP/chassisR_task.c†L41-L91】【F:ludan_ws_ros1/src/dmbot_serial/include/dmbot_serial/robot_connect.h†L14-L109】
* **Minimal fault handling.** CAN and serial communication routines perform
  little validation beyond checksum checks; failures such as timeouts, motor
  disconnects, or IMU faults are not surfaced to ROS diagnostics or handled by
  the firmware state machine.【F:ludan_ws_ros1/src/dmbot_serial/src/robot_connect.cpp†L65-L121】【F:DM-motor-right-arm/User/APP/body_task.c†L53-L92】
* **Tight coupling of tasks.** FreeRTOS tasks share global state (`chassis_move`,
  `robot_body`) and rely on flags set in other tasks without guarding, making it
  hard to reason about concurrency or extend behaviour.【F:DM-motor-right-arm/User/APP/body_task.c†L20-L83】【F:DM-motor-right-arm/User/APP/chassisR_task.c†L21-L117】
* **No automated testing or CI.** The repository lacks unit tests, hardware
  simulations, or continuous integration pipelines for both firmware and ROS
  components, so regressions are difficult to detect early.
* **Limited documentation of message contracts.** Custom binary protocol fields
  and ROS topic semantics are implied by the code and not formally specified,
  which increases the onboarding cost for new contributors.

## Future improvements
* Externalise actuator metadata (IDs, limits, bus routing) into configuration
  files or ROS parameters to avoid recompiling for hardware changes and enable
  per-joint safety limits.
* Add robust error handling and diagnostics: monitor CAN bus status, detect
  checksum failures, publish `/diagnostics` messages, and expose firmware
  heartbeat topics to detect drops.
* Refactor the firmware tasks into modular components with explicit state
  machines and thread-safe accessors; consider migrating to a HAL abstraction or
  code generation for CAN frames.
* Implement integration tests or simulations (e.g. Gazebo with the provided
  URDFs) that exercise the ROS control stack without hardware, coupled with CI
  workflows.
* Document the binary protocol between the STM32 and ROS driver and provide a
  reference decoder/encoder library to ease porting to ROS 2 or other hosts.
* Explore porting the ROS workspace to ROS 2 and leveraging `ros2_control` for
  better long-term support.

## License
See individual package licenses; third-party dependencies retain their original
licenses as noted in the respective subdirectories.
