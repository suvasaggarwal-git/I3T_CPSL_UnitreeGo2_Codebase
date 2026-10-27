# Setup process and ROS code for using Unitree GO2 EDU quadruped with essential ROS functionality. Functionalities include pointcloud and laserscan acquisition, SLAM, Nav2 navigation, camera data, etc.
### This guide assumes you have a GO2 or other robot equipped with an Intel Realsense D435i and a Livox LiDar
### and that both of these components have been mounted on the robot following the official docs.
### It also assumes the robot is running ROS2 Foxy, but this shouldn't cause many issues.

## Go2 Expansion Board Networking Setup

This section outlines how to get the expansion board component of the Go2 dog (an NVIDIA Orin board) connected to an external Wi-Fi network and how to access the ROS topics of the Go2's internal board on the expansion board. This is a separate process than connecting the Go2 to Wi-Fi as outlined in the official Unitree docs. Before starting this section, you will need:
1. A monitor, keyboard, and mouse
2. A BrosTrend AC1L USB Wi-Fi adapter (or equivalent Ubuntu-compatible Wi-Fi adapter)
3. A Type-C USB hub with an Ethernet port 
4. An HDMI cable that can be plugged into the aforementioned USB hub
5. An Ubuntu host PC with Ethernet port or equivalent

### Step 1: 
Follow the quick start section of the Unitree docs as found in the [Unitree Docs](https://support.unitree.com/home/en/developer/Quick_start) and the app binding section. **Ensure you connect to the dog through the app through the Wi-Fi mode**. Once connected through the app, record the Go2's IP (Device/Data/Automatic Machine Inspection) as it will be needed later.

It's also a good idea to go ahead and do the [Go2 ROS2 setup](https://support.unitree.com/home/en/developer/ROS2_service) that matches with your distro.

### Step 2:
Plug the USB hub into the Type-C port on the back of the Go2 expansion board. Plug the HDMI into the USB hub, and into the monitor. Connect the keyboard and mouse to the USB hub. The monitor should display the Ubuntu GUI. Login to the expansion board (default password is 123).

Go to the network settings. **Disable the PCI Ethernet connection**. This will allow the board to connect to the Internet. Plug the Ethernet cable into the USB hub, and then into an Internet-connected USB port. Check if the expansion board can connect to the Internet, e.g. by `ping google.com`.

### Step 3:
Download the BrosTrend AC1L adapter Linux drivers (or the equivalent for your Wi-Fi adapter) by running

```
sudo sh -c 'wget linux.brostrend.com/install -O /tmp/install && sh /tmp/install'
```

Follow the instructions in the terminal. Ensure all drivers are installed correctly.

### Step 4:
Once the drivers have been installed, go back to network settings, and connect to your Wi-Fi network of choice. Unplug the Ethernet cable from the USB hub, and ensure you can access the Internet on the Wi-Fi connection. Record the IP of the expansion board on the Wi-Fi so you can acces it later. If there are no problems, **re-enable the PCI Ethernet connection**; this will resume the communication between the expansion board and the Go2's internal board. 

In network settings, open the wired connection settings and change the DHCP settings IPv4 address to `192.168.123.99` and make sure the mask is `255.255.255.0`. This will allow you to access the topics published by the Go2's internal board on the expansion board.


### Step 5:
Run the commands `echo 'net.core.rmem_max=52428800' | sudo tee -a /etc/sysctl.conf` and `sudo sysctl -p`. This will increase the expansion board's maximum socket buffer size. To utilize this with ROS, go to `~/cyclonedds_ws/cyclonedds.xml` and add the following lines if they are not there already:
```
<Internal>
    <MinimumSocketReceiveBufferSize>50MB</MinimumSocketReceiveBufferSize>
</Internal>
```
This will allow the expansion board to handle more data across its Ethernet bus.

Finally, increase the power level of the expansion board to 25W or higher. 


## Go2 Expansion Board Intel RealSense Setup
The Go2's expansion board does not come preconfigured to use the D435i depth camera with Python bindings. Since the board uses an ARM architecture, the bindings can't be installed directly through pip either, and must be built from source. Steps 1-4 outlined below are taken directly from [this official process from a RealSense Git issue](https://github.com/IntelRealSense/librealsense/issues/6964). Steps 5-6 were added by me. The process should be nearly identical for any ARM system, and should be even simpler if your target platform does not have an ARM architecture.

### Step 1:
The first step in this process is to download the source code of librealsense from the Releases page as a zip file. The source code zip file can always be found in the "Assets" list at the bottom of the information listing for each SDK version on the Releases page.

https://github.com/IntelRealSense/librealsense/releases/

### Step 2: 
After downloading the zip file, its contents should be extracted so that you have a librealsense folder.At this point you can use the RSUSB installation method to install librealsense without dependence on Linux versions or kernel versions and without the need for patching. This makes this installation method particularly suited to Arm devices such as Jetson.

The backend installation method requires an internet connection. The steps are:

Go to the librealsense root directory. Create a folder within it called build and then go to this new folder using this instruction: `mkdir build && cd build`

### Step 3:
Now that you are in the build directory, run the CMake build instruction below to install librealsense and the Python bindings over the internet connection:
`cmake ../ -DFORCE_RSUSB_BACKEND=ON -DBUILD_PYTHON_BINDINGS:bool=true -DPYTHON_EXECUTABLE=...`

The above statement is a basic one that should test whether the build is likely to succeed or not. **(Replace the dots with the path to your Python exec)**

### Step 4:
If it does succeed then you can try a more advanced build, which builds the example programs and includes optimizations such as building with CUDA support for faster alignment processing on devices such as Jetson that are equipped with an Nvidia graphics GPU.

`cmake ../-DFORCE_RSUSB_BACKEND=ON -DBUILD_PYTHON_BINDINGS:bool=true -DPYTHON_EXECUTABLE=... -DCMAKE_BUILD_TYPE=release -DBUILD_EXAMPLES=false -DBUILD_GRAPHICAL_EXAMPLES=false -DBUILD_WITH_CUDA:bool=true`

(The original issue has this command running with examples and graphical examples on. This can cause issues with the following step, so they have been disabled here.)

### Step 5:
Run `make -j$(nproc)`

### Step 6: 
If everything builds correctly, run `find . -name "pyrealsense2*.so"` within the build directory to file the .so filepath. Once you have this path, find the site packages location with `python3 -m site --user-site`. Then, use `cp` to copy the .so to the site packages location.

## Livox Setup
Most of this process is taken care of by mounting the Livox according to the official docs from Unitree or whatever the target platform is. The interfacing of the Livox with the robot should be taken care of by the packages in this repo. However, you may need to change the `ip` parameter within the `src/CPSL_ROS_livox_ros_driver2/config/MID360_config.json` file to the IP of your Livox device (IP can be found on a sticker on the physical device). If that sticker is not present, run `sudo arp-scan --interface=eth0 --localnet`, it can also help determine if the Livox is connected to the dog.

## Go2 Internal IP Configuration
In order for the Go2 to receive commands from packages like Nav2, `cmd_vel` messages must be translated and sent to the Go2's internal board which handles all of its movement. As such, the `CmdVelTranslator.py` node within the `dog_utilities` needs a parameter `internal_board_ip` passed to it either when calling the main launch file, or when running the node itself. This should be the ethernet IP.

## ROS2 2D SLAM Installation

Install the required ROS 2 Foxy packages:

```bash
sudo apt install ros-foxy-pointcloud-to-laserscan
sudo apt install ros-foxy-slam-toolbox
```

## Troubleshooting

### Tsinghua Server Issues

If you run into problems with the Tsinghua mirror server during installation, try upgrading first:

```bash
sudo apt upgrade
```

If that doesn't resolve the issue, switch the ROS source list from the Tsinghua mirror to Ubuntu's native ROS package server:

```bash
sudo sed -i 's|http://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu/|http://packages.ros.org/ros2/ubuntu/|g' /etc/apt/sources.list.d/ros-fish.list
```

This replaces the Tsinghua mirror with the official `packages.ros.org` source, which is useful when the Tsinghua mirror is expired or out of date.

## ROS 3D Slam Installation

### 1. RTAB-Map standalone library (dependency of the ROS wrapper)
```bash
sudo apt install ros-foxy-rtabmap          # or build librtabmap from source
```

### 2. clone the branch you linked into your workspace src/
```bash
cd ~/I3T_CPSL_UnitreeGo2_Codebase/src
git clone --branch foxy-devel https://github.com/introlab/rtabmap_ros.git
```

### 3. resolve deps and build
```bash
cd ~/I3T_CPSL_UnitreeGo2_Codebase
rosdep install --from-paths src --ignore-src -r -y
```

If there is a build issue, run:
```bash
sudo bash -c 'for f in /usr/lib/aarch64-linux-gnu/libopencv_*.so.4.2.0;
  do base="${f%.so.4.2.0}.so"; if [ ! -e "$base" ]; then ln -s "$f" "$base";
  fi; done'
```

## SportBridge Setup for Nav2 Commands
```bash
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git ~/unitree_sdk2_python
cd ~/unitree_sdk2_python
pip install -e .
```

## Installing Ultralytics YOLO (JetPack 5.1.2)

Native installation of Ultralytics YOLO with TensorRT export support on a JetPack 5.1.2 Jetson device.

### 1. Install the Ultralytics Package

Update the package list, install pip, and install `ultralytics` with export dependencies:

```bash
sudo apt update
sudo apt install python3-pip -y
pip install -U pip
pip install ultralytics[export]
sudo reboot
```

### 2. Install PyTorch and Torchvision

The pip-installed Torch/Torchvision aren't compatible with the Jetson's ARM64 architecture, so replace them with the JetPack 5.1.2 pre-built wheels (`torch 2.1.0` + `torchvision 0.16.2`):

```bash
pip uninstall torch torchvision
pip install https://github.com/ultralytics/assets/releases/download/v0.0.0/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl
pip install https://github.com/ultralytics/assets/releases/download/v0.0.0/torchvision-0.16.2+c6f3977-cp38-cp38-linux_aarch64.whl
```

### 3. Install onnxruntime-gpu

PyPI has no `aarch64` build for the Jetson, so install `onnxruntime-gpu 1.17.0` (Python 3.8) manually:

```bash
wget https://nvidia.box.com/shared/static/zostg6agm00fb6t5uisw51qi6kpcuwzd.whl -O onnxruntime_gpu-1.17.0-cp38-cp38-linux_aarch64.whl
pip install onnxruntime_gpu-1.17.0-cp38-cp38-linux_aarch64.whl
```

> **Note:** Installing `onnxruntime-gpu` bumps NumPy to the latest version, which causes issues. Reinstall the pinned version afterward:
> ```bash
> pip install numpy==1.23.5
> ```

## Using this repo
1. Clone this repo. Build and source the workspace
1. Open 3 terminal windows and run the following
    * `ros2 launch go2_launcher dog.launch.py internal_board_ip:=YOUR_IP_HERE collect_realsense:=true or false` OR `dog_launch collect_realsense:=true`
    * `ros2 launch cpsl_ros2_sensors_bringup ugv_sensor_bringup.launch.py` OR `sensors_launch`
    * `ros2 launch cpsl_nav slam.launch.py scan_topic:=/livox/scan_best_effort` OR `slam2d_launch` if using 2D SLAM OR `ros2 launch cpsl_nav slam3d.launch.py rviz:=true` OR `slam3d_launch rviz:=true`for 3D SLAM
1. For getting a functional transform tree and mapping, that's all you need. If you want to issue Nav2 commands, open another terminal and run
    * `ros2 launch cpsl_nav nav2_go2.launch.py` OR `nav2`

## 3D SLAM with Livox and RealSense

After updating this workspace on the robot, rebuild and source it:

```bash
colcon build --symlink-install --packages-select cpsl_nav intel_realsense_functions dog_utilities go2_launcher
source install/setup.bash
```

Run these in separate terminals (source the workspace in each):

```bash
ros2 launch go2_launcher dog.launch.py collect_realsense:=true
ros2 launch cpsl_ros2_sensors_bringup ugv_sensor_bringup.launch.py
ros2 launch cpsl_nav slam3d.launch.py rviz:=true
```

Default 3D mapping uses Livox geometry and RealSense visual features. LiDAR
map points are not automatically colored with the camera image. RTAB-Map owns
`map -> odom`; the Go2 pose bridge owns `odom -> base_link`. The separate 2D
SLAM node generates `/slam_toolbox/map` by default, with its TF publication
disabled. Use `enable_2d_slam:=false` to skip that extra mapping workload. Do not run
the standalone 2D SLAM launch at the same time as 3D SLAM.

Tune `src/cpsl_nav/config/slam3d.ini` for RTAB-Map range, map resolution,
outlier filtering and ICP registration. The LiDAR map range is 10 m, cells
are 5 cm, and isolated points need five neighbors within 10 cm. Increase
`Grid/RangeMax` for longer-range LiDAR maps after checking the short-range map.
Neighbor filtering can remove thin or sparsely sampled objects. Launch selects
`Grid/Sensor=0` for LiDAR or `1` with `rgbd_only:=true`; use RGB-D-only mode to
compare camera-derived geometry. It still uses Go2 pose TF for odometry.
An alternate core config can be passed with `config:=/absolute/path/slam3d.ini`.

Tune `src/intel_realsense_functions/config/slam_camera.yaml` for camera depth
range and spatial smoothing. Temporal filtering and hole filling start disabled
to avoid smearing and inferred depths while walking. Restart the camera node
after editing. An alternate camera config can be passed to `dog.launch.py`
with `realsense_config:=/absolute/path/slam_camera.yaml`. The older `config.yaml`
in that package contains a server URL and does not configure camera filtering.

Camera global-time support is enabled when available, and frame acquisition
timestamps are converted into ROS time before processing. A warning indicates
fallback to receipt time if synchronized camera timestamps are unavailable.
The Go2 pose bridge preserves source timestamps within 0.5 s of its ROS clock,
falling back to receipt time with a warning when they are farther off (including
zero timestamps). Receipt time does not compensate for sensor/network latency. Synchronize the robot and host clocks, and
verify Livox timestamps use the same time base. The sensor synchronization
tolerance can be changed with `approx_sync_max_interval:=0.05` (seconds).

For optional ICP odometry, restart the Go2 launch with
`publish_odom_tf:=false` and start 3D SLAM with `use_icp_odometry:=true`.
ICP then owns `odom -> base_link` and supplies the odometry messages consumed
by RTAB-Map. Do not leave Go2 odometry TF enabled in this mode. This is scan
matching, not a LiDAR-IMU deskewing pipeline.

Both `slam_3d_config.rviz` (normal launch / tmux script) and
`slam_3d_yolo.rviz` (`start_slam3d.sh`) expose these independent checkboxes
in RViz's **Displays** panel:

| Display | Data | Starts visible |
| --- | --- | --- |
| LiDAR Map (ground-obstacle colors) | `/cloud_map`, existing map colors | Yes |
| 2D SLAM Map (slam_toolbox) | `/slam_toolbox/map`, LiDAR occupancy grid | No |
| Camera RGB Map (accumulated) | `/mapData`, RGB-D keyframes at optimized poses | No |
| Camera RGB Cloud (live) | `/realsense/depth/color/points`, current aligned RGB + depth | No |

Uncheck the LiDAR map and check the desired display to compare views, or check
multiple displays to overlay them. The camera maps cover only valid camera
depth within 0.3-3.5 m and the camera's field of view. The accumulated RGB map
uses RTAB-Map's `MapCloud` plugin with **Cloud from scan: false** and
**Color Transformer: RGB8**. This reconstructs camera geometry with actual
image colors; it does not project camera colors onto the Livox points.
If enabled after mapping has already started, request earlier keyframes after
checking the display:

```bash
ros2 service call /publish_map rtabmap_msgs/srv/PublishMap "{global_map: true, optimized: true, graph_only: false}"
```

The bundled Foxy plugin's **Download map** button is unimplemented; use the
service above instead. Topics are at the ROS root with the default launch
namespace; prefix them appropriately if running a namespaced robot.
The plugin package must be built/installed on the machine running RViz:
`rtabmap_rviz_plugins` (binary package: `ros-foxy-rtabmap-rviz-plugins`).

Physical mounting transforms remain in
`src/dog_utilities/urdf/go2_with_realsense.urdf`; measure them on your robot
before changing them.

Mapping still starts fresh by default (`reset_map:=true`). Use
`reset_map:=false` to resume a map. `localization:=true` always preserves the
database, even when the reset argument is left at its default.

## Launch scripts and arm control

Can also run shell script for functional tf tree, mapping, and rviz (Nav2 is separate):
`./run_dog_2d.sh true` for 2D SLAM
`./run_dog_3d.sh true` for 3D SLAM

Kill the session by running `kill_dog`.

# D1 Arm Usage Guide

This section describes how to control the CPSL D1 Arm, including how to start the arm driver node, publish movement commands, and read current servo angles.

---

# D1 Arm Driver

This package provides the driver and commands for operating the D1 Arm using ROS 2.

## Starting the Arm Node

To spin up the node that drives the arm, run the following command:

`ros2 run cpsl_d1_arm_driver d1_arm_node` OR `start_d1`

## Sending Commands

You can send commands to the arm by publishing a string to the `/d1_arm/command` topic. Swap any of the supported command strings into the payload.

**Example:**
`ros2 topic pub --once /d1_arm/command std_msgs/msg/String "{data: 'grasp_3_1'}"` OR `move_d1 grasp_3_1`

### Available Commands

#### Left/Right Movements
* `'left_1'`
* `'left_2'`
* `'right_1'`
* `'right_2'`

#### Grasp / Tool Movements
* `'grasp_1_1'`
* `'grasp_1_2'`
* `'grasp_2_1'`
* `'grasp_2_2'`
* `'grasp_3_1'`
* `'grasp_3_2'`
* `'grasp_4_1'`
* `'grasp_4_2'`

#### Home
* `'zero'`

## Getting Current Servo Angles

To retrieve the current joint angles of the arm's servos, run the following executable:

`./build/CPSL_D1_Arm/get_arm_joint_angle` OR `get_d1_angles`
