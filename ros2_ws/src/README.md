# ROS 2 integration for Franka Emika research robots

This workspace is based on the upstream Bosch Research repository:
[boschresearch/franka_ros2](https://github.com/boschresearch/franka_ros2).

This repository contains the ROS 2 packages needed to bring up and control a
Franka robot, publish robot state, load the robot description, and work with
MoveIt for motion planning and visualization.

The workspace includes packages for:
- robot bringup and hardware integration
- gripper support
- robot description and semantic description
- MoveIt configuration
- example controllers
- robot state broadcasting and ROS 2 interfaces
- Gazebo simulation bringup with a separate gravity-compensated Panda sim backend


## Quick Start

The commands below assume the repository is checked out in:

```bash
~/franka_ros2_ws/src/franka_ros2
```

The setup described here was used successfully with:
- Ubuntu 22.04
- ROS 2 Humble
- a dedicated workspace at `~/franka_ros2_ws`
- a Franka robot reachable over Ethernet at `172.16.0.2`

With this setup, the Panda can be launched successfully with MoveIt under ROS 2.

## libfranka Setup

This workspace is intended to use a manually installed `libfranka` from source.
It was not set up against the ROS apt package `ros-humble-libfranka`.

This is important because the Franka ROS 2 packages use the system
installation discovered by CMake during configuration. If a conflicting binary
package is installed, the workspace can link against the wrong `libfranka`
version.

Important:
- use a manually installed `libfranka` from source
- use a version compatible with the checked-out `franka_ros2` stack
- avoid blindly using the newest available `libfranka` release
- this repository targets an older ROS 2 Humble-era `libfranka` API
- the repository changelog states that this code requires `libfranka >= 0.10.0`

### Manual installation from source

Typical installation procedure:

```bash
git clone --recursive https://github.com/frankaemika/libfranka.git
cd libfranka
git checkout <compatible-libfranka-version>
git submodule update --init --recursive

mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . -j"$(nproc)"
sudo cmake --install .
```

### Checks for conflicting installations

We explicitly checked that no conflicting ROS binary package was installed:

```bash
dpkg -l | grep -E 'franka|libfranka'
apt list --installed 2>/dev/null | grep -E 'franka|libfranka'
apt-cache policy ros-humble-libfranka
```

The desired result is:
- `ros-humble-libfranka` is not installed
- only a candidate package from the ROS repository may be shown

This confirms that the workspace should resolve the manually installed
`libfranka`.

### Optional verification

Useful checks after installation:

```bash
ldconfig -p | grep libfranka
pkg-config --modversion libfranka
```

These help verify that the system can see the installed `libfranka` and which
version is being resolved.


## Real-time kernel installation

A **PREEMPT_RT / real-time Linux kernel** was installed on the control PC.
This is important for reliable low-latency communication and controller
execution when operating the Panda from ROS 2.

### Why this matters

For Franka operation, especially when running controllers through the ROS 2
control stack, a real-time capable kernel is strongly recommended. Without it,
control execution can become less deterministic and may lead to timing issues.

### Installation

On Ubuntu 22.04, the real-time kernel was installed via the Ubuntu Pro
real-time kernel package flow:

```bash
sudo apt update
sudo apt install ubuntu-pro-client
sudo pro enable realtime-kernel --variant=preempt
sudo reboot
```

After reboot, the active kernel was checked with:

```bash
uname -r
cat /sys/kernel/realtime
```

A value of `1` in `/sys/kernel/realtime` indicates that the system is running a
real-time kernel.

### User permissions and limits

The RT kernel alone was not sufficient. The user account running the ROS 2
stack also needed real-time scheduling and memory locking limits configured.
Without these limits, the Franka control stack may not be allowed to request
the scheduler priority and locked memory needed for stable low-latency control.

Run the following as your normal user, using `sudo` where shown:

```bash
sudo groupadd -f realtime
sudo usermod -aG realtime "$USER"

sudo tee /etc/security/limits.d/99-realtime.conf >/dev/null <<'EOF'
@realtime soft rtprio 99
@realtime hard rtprio 99
@realtime soft priority 99
@realtime hard priority 99
@realtime soft memlock unlimited
@realtime hard memlock unlimited
EOF

sudo reboot
```

After reboot, open a fresh terminal and verify:
 
```bash
id
ulimit -r
ulimit -l
cat /sys/kernel/realtime 2>/dev/null || echo 0
```

What you want to see:
- `id` includes `realtime`
- `ulimit -r` is `99` or at least greater than `0`
- `cat /sys/kernel/realtime` prints `1`


### Practical note

The PC also kept a non-real-time kernel available as fallback. This is useful
because some hardware drivers, for example Wi-Fi, may behave differently or
stop working under the RT kernel. In practice:
- use the RT kernel for robot operation and controller work
- keep the generic kernel available for troubleshooting or normal desktop usage
- if needed, select the desired kernel from GRUB during boot

To check which kernels are installed:

```bash
dpkg -l | grep linux-image
```

To boot a different installed kernel temporarily, use `Advanced options for
Ubuntu` in the GRUB menu during startup and select the desired kernel there.

## Build

Change into the workspace root first:

```bash
cd ~/franka_ros2_ws
```

Install dependencies first:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -y
```


```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Environment Setup

Before running the stack:

```bash
source /opt/ros/humble/setup.bash
source ~/franka_ros2_ws/install/setup.bash
```

For convenience, these sourcing commands can be added to `~/.bashrc`.

## Bringup

Launch the main Franka bringup stack:

```bash
ros2 launch franka_bringup franka.launch.py robot_ip:=172.16.0.2
```

## Keyboard Servo Teleoperation

Launch MoveIt, MoveIt Servo, and keyboard teleoperation for the real Panda:

```bash
ros2 launch franka_moveit_config moveit_servo.launch.py robot_ip:=172.16.0.2 use_fake_hardware:=false launch_keyboard_teleop:=true keyboard_speed:=0.02 keyboard_turn:=0.10
```

This command connects to the real robot, starts the existing MoveIt/control
stack, starts MoveIt Servo, and opens `teleop_twist_keyboard` in a separate
terminal. Use conservative speeds for initial hardware tests and keep the robot
workspace clear.

## Controller Servo Teleoperation

Launch MoveIt, MoveIt Servo, and controller teleoperation for the real Panda:

```bash
ros2 launch franka_moveit_config moveit_servo.launch.py robot_ip:=172.16.0.2 use_fake_hardware:=false launch_controller_teleop:=false
```

This starts `game_controller_node` together with `teleop_twist_joy` and remaps
the stamped Twist output to `/servo_node/delta_twist_cmds`. The default preset is
`franka_moveit_config/config/ps5_servo_teleop.config.yaml`; switch to the Xbox
preset with:

```bash
ros2 launch franka_moveit_config moveit_servo.launch.py robot_ip:=172.16.0.2 use_fake_hardware:=false launch_controller_teleop:=true controller_config_file:=$(ros2 pkg prefix franka_moveit_config)/share/franka_moveit_config/config/xbox_servo_teleop.config.yaml
```

The controller teleop path publishes continuously while the enable button is
held, which avoids the keyboard autorepeat gaps that make Servo feel choppy.
The default preset maps the left stick to x/y translation, the right stick to
z translation and yaw, uses `game_controller_node` for standardized controller
indices, and requires L1 to be held while commanding motion.

Current PS5 preset assignment:

- L1, button 9: hold-to-enable
- R1, button 10: turbo speed while held
- left stick horizontal, axis 0: y translation
- left stick vertical, axis 1: x translation
- right stick horizontal, axis 2: yaw
- right stick vertical, axis 3: z translation
- Cross/A, button 0 while L1 is held: close/grasp the gripper
- Circle/B, button 1: stop the gripper
- Square/X, button 2 while L1 is held: open the gripper
- Triangle/Y, button 3 while L1 is held: home the gripper
- D-Pad Up directional arrow while L1 is held: go to ready position

For full roll/pitch/yaw control on a PS5 controller, enable the dedicated
6-DOF mapper:

```bash
ros2 launch franka_moveit_config moveit_servo.launch.py robot_ip:=172.16.0.2 use_fake_hardware:=false launch_controller_teleop:=true use_ps5_6dof_teleop:=true
```

In 6-DOF mode, L1 still enables motion and R1 still enables turbo. Without L2,
the sticks keep the existing x/y/z/yaw behavior. While L2 is held, translation
is suppressed and the sticks command rotations: left stick horizontal is roll,
left stick vertical is pitch, and right stick horizontal is yaw. The same face
button gripper commands are available in both controller modes.

The gripper button mapping and action parameters live in:

```bash
franka_moveit_config/config/gamepad_gripper_teleop.config.yaml
```

Gripper gamepad teleop starts automatically when `launch_controller_teleop` is
true. To disable only the gripper buttons, add:

```bash
launch_gripper_teleop:=false
```

## Simulation Controller Servo Teleoperation

Launch Gazebo, MoveIt, MoveIt Servo, and controller teleoperation together:

```bash
ros2 launch panda_gazebo_bringup gazebo_moveit_servo.launch.py launch_controller_teleop:=true
```

The simulation path mirrors the real-robot controller setup and uses
`game_controller_node` plus `teleop_twist_joy` to publish stamped Twist commands to
`/servo_node/delta_twist_cmds`. The default preset is
`panda_gazebo_bringup/config/ps5_servo_teleop.config.yaml`; switch to the Xbox
preset with:

```bash
ros2 launch panda_gazebo_bringup gazebo_moveit_servo.launch.py launch_controller_teleop:=true controller_config_file:=$(ros2 pkg prefix panda_gazebo_bringup)/share/panda_gazebo_bringup/config/xbox_servo_teleop.config.yaml
```

The default simulation preset uses slightly faster limits than the real-robot
preset, but keeps the same left stick x/y, right stick z/yaw, hold-to-enable
interaction.

The simulation launch supports the same PS5 6-DOF mapper:

```bash
ros2 launch panda_gazebo_bringup gazebo_moveit_servo.launch.py launch_controller_teleop:=true use_ps5_6dof_teleop:=true
```

## Simulation

The workspace also includes a separate Panda simulation stack in
`panda_gazebo_bringup` and `panda_gazebo_hardware`. This keeps the Gazebo
setup separate from the real robot bringup and control packages.

Launch Gazebo Sim with the Panda, the simulated `ros2_control` stack, and the
effort-based `panda_arm_controller`:

```bash
ros2 launch panda_gazebo_bringup gazebo.launch.py
```

This launch starts the robot in modern Gazebo through the custom
gravity-compensated simulation hardware plugin, publishes the robot
description and joint states, spawns the `joint_state_broadcaster` and
`panda_arm_controller`, and by default sends a one-shot home trajectory so the
robot starts in the expected Panda configuration.

Launch MoveIt and RViz for the simulated Panda:

```bash
ros2 launch panda_gazebo_bringup moveit.launch.py
```

This launch starts `move_group` and RViz against the simulated Panda model and
controller setup. It is intended to be used together with
`panda_gazebo_bringup/gazebo.launch.py` and reuses the Franka MoveIt
configuration while talking to the simulated arm controller instead of the real
robot.

Launch Gazebo, MoveIt, and RViz together in one command:

```bash
ros2 launch panda_gazebo_bringup gazebo_moveit.launch.py
```

This combined launch starts the full simulation workflow in one step:
Gazebo Sim, the gravity-compensated Panda simulation backend, the
`JointTrajectoryController`, MoveIt, and RViz. When using this combined launch,
do not start `panda_gazebo_bringup/moveit.launch.py` separately in parallel.

## Checks

Useful commands to verify the robot state and controller setup:

```bash
ros2 topic echo /franka/joint_states
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

## Examples

Example controller launch:

```bash
ros2 launch franka_bringup move_to_start_example_controller.launch.py robot_ip:=172.16.0.2
```

MoveIt launch:

```bash
ros2 launch franka_moveit_config moveit.launch.py robot_ip:=172.16.0.2
```

This launch was used successfully with the Panda running under MoveIt on ROS 2.

## License

All packages of `franka_ros2` are licensed under the [Apache 2.0 license][apache-2.0].

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html

[fci-docs]: https://frankaemika.github.io/docs
