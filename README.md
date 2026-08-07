# hex_ros_demo_chassis_impedance

## What does this package do

This package is an **impedance control demo** for the Maver X4 and Trigger A
chassis that works in **both ROS 1 and ROS 2**.

The node first drives the steering joints to a stable configuration, records the current base pose as the equilibrium, then switches into SE(2) impedance control: at every control cycle it reads `/chs_state`, limits the planar pose error toward the equilibrium, maps the body-frame error to a twist command, and publishes a `VEL` control command on `/chs_ctrl`. The robot driver (or the [`hex_ros_sim_maver_x4`](../hex_ros_sim_maver_x4) simulator) tracks that twist through chassis inverse kinematics, so the base compliantly returns toward the equilibrium pose when disturbed.

A keyboard interface (see [`hex_ros_teleop_keyboard`](../hex_ros_teleop_keyboard)) is used for runtime control:

* press **`q`** to stop the demo and settle the chassis back to the stable joint configuration.

Data recording is left to ROS's built-in bag tools (`ros2 bag record` / `rosbag record`).

## Maintainer

[Dong Zhaorui](https://github.com/IBNBlank)

## Prerequisites

Ensure the following software is installed:

* **ROS**: Refer to the [ROS Installation guide](http://wiki.ros.org/ROS/Installation)
* **hex_ros_msgs**: provides the robot/teleop message definitions.
* **hex_ros_urdf_maver_x4** or **hex_ros_urdf_trigger_a**: provides the model
  used by the selected launch file.
* A state/control source for the chassis, e.g. **hex_ros_sim_maver_x4** or
  **hex_ros_sim_trigger_a**.
* A keyboard source, e.g. **hex_ros_teleop_keyboard**.

### Verified Platforms

* [x] **x64**
* [ ] **Jetson Orin Nano**
* [x] **Jetson Orin NX**
* [ ] **Jetson AGX Orin**
* [ ] **Horizon RDK X5**
* [ ] **Rockchip RK3588**

## Public APIs

### Published Topics

| Topic       | Msg Type                                   | Description            |
| ----------- | ------------------------------------------ | ---------------------- |
| `/chs_ctrl` | `hex_ros_msgs/HexRosRoboChsCtrlStamped`    | Chassis control command. |

### Subscribed Topics

| Topic                    | Msg Type                                        | Description              |
| ------------------------ | ----------------------------------------------- | ------------------------ |
| `/chs_state`             | `hex_ros_msgs/HexRosRoboChsStateStamped`        | Current chassis state.   |
| `/teleop_keyboard_state` | `hex_ros_msgs/HexRosTeleopKeyboardStateStamped` | Keyboard key states.     |

### Parameters

| Name                 | Data Type        | Description                                              |
| -------------------- | ---------------- | -------------------------------------------------------- |
| `rate_ros`           | `double`         | Impedance control work loop rate [hz].                  |
| `rate_teleop`        | `double`         | Keyboard monitor rate [hz].                             |
| `model_urdf`         | `string`         | Path to the URDF (set by launch).                       |
| `model_frame_id`     | `string`         | Frame id of the robot base.                             |
| `chs_stable_pos`     | `vector<double>` | Stable joint position (init/exit) [rad].                |
| `chs_stable_vel`     | `vector<double>` | Stable joint velocity targets [rad/s].                  |
| `chs_kp` / `chs_kd`  | `vector<double>` | MIT gains used while moving to the stable configuration.|
| `chs_type`           | `string`         | Chassis type: `maver_x4` or `trigger_a`.                |
| `chs_impedance_kp` / `kd` | `vector<double>` | Isotropic impedance gains `[pos, yaw]`.            |
| `chs_pos_threshold`  | `double`         | Max XY error step applied per cycle [m].                |
| `chs_yaw_threshold`  | `double`         | Max yaw error step applied per cycle [rad].             |
| `chs_vel_kd`         | `vector<double>` | Joint `kd` used by VEL mode motor tracking.             |
| `arrive_threshold`   | `double`         | Max yaw joint error [rad] to consider settled.          |

## Getting Started

1. Install necessary dependencies:

   ```shell
   pip3 install 'hex-util-msg>=0.1.0a0'
   pip3 install 'hex-util-ros>=0.0.1a0'
   ```

2. Create a workspace and navigate to the `src` directory:

   ```shell
   mkdir -p catkin_ws/src
   cd catkin_ws/src
   ```

3. Clone the repository:

   ```shell
   git clone https://github.com/hexfellow/hex_ros_demo_chassis_impedance.git
   ```

4. Navigate back and build the workspace:

   For ROS 1:

   ```shell
   cd ../
   catkin_make
   ```

   For ROS 2:

   ```shell
   cd ../
   colcon build
   ```

5. Source the `setup.bash` file:

   For ROS 1:

   ```shell
   source devel/setup.bash --extend
   ```

   For ROS 2:

   ```shell
   source install/setup.bash --extend
   ```

### Usage

One-shot bringup (sim + keyboard + impedance):

```shell
ros2 launch hex_ros_demo_chassis_impedance sim_impedance_maver_x4.launch.py
```

For Trigger A:

```shell
ros2 launch hex_ros_demo_chassis_impedance sim_impedance_trigger_a.launch.py
```

Or start the pieces separately:

1. Start a chassis state/control source (e.g. the simulator) and the keyboard node:

   For ROS 2:

   ```shell
   ros2 launch hex_ros_sim_maver_x4 sim_maver_x4.launch.py
   ros2 launch hex_ros_teleop_keyboard teleop_keyboard.launch.py
   ```

2. Launch the `chassis_impedance` node:

   For ROS 1:

   ```shell
   roslaunch hex_ros_demo_chassis_impedance chassis_impedance_maver_x4.launch
   ```

   For ROS 2:

   ```shell
   ros2 launch hex_ros_demo_chassis_impedance chassis_impedance_maver_x4.launch.py
```

Replace the `maver_x4` suffix with `trigger_a` to use the Trigger A launch
files. Use the matching `real_impedance_<type>` launch file for a real robot.

3. The chassis settles to the stable joint pose, records the equilibrium, then enters impedance control. Push the base in the MuJoCo viewer to feel the restoring behavior. Press `q` to exit. To record data, use ROS's bag tools, e.g. `ros2 bag record -a`.
