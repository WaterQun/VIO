# VIO (Visual Inertial Odometry)


This package is a bridge from PX4 to the Realsense T265 camera, which provides odometry data.

## Dependencies
* ROS Noetic (Ubuntu 20.04) or ROS Melodic (Ubuntu 18.04): [http://wiki.ros.org/ROS/Installation](http://wiki.ros.org/ROS/Installation)
* librealsense: [https://github.com/IntelRealSense/librealsense](https://github.com/IntelRealSense/librealsense)
* MAVROS: [https://dev.px4.io/en/ros/mavros_installation.html](https://dev.px4.io/en/ros/mavros_installation.html).


## Installation
These steps contain the installation process, software dependencies and building instructions.

1. This is a ROS package, it assumes you have either ROS Noetic (Ubuntu 20.04) or ROS Melodic (Ubuntu 18.04) installed, instructions can be found [here](http://wiki.ros.org/ROS/Installation). This is an example of installing ROS Noetic: 

   ```bash
   gpg --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
   gpg --export C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 | sudo tee /usr/share/keyrings/ros.gpg > /dev/null
   sudo sh -c 'echo "deb [signed-by=/usr/share/keyrings/ros.gpg] https://mirrors.ustc.edu.cn/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   sudo apt update
   sudo apt install ros-noetic-desktop-full
   echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

2. Install librealsense (version 2.50.0) from [here](https://github.com/IntelRealSense/librealsense/tree/v2.50.0). It is the last version that supports ROS1. You can building librealsense SDK like this: 

   ```bash
   sudo apt-get install git libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev build-essential
   git clone -b v2.50.0 https://github.com/IntelRealSense/librealsense.git
   cd librealsense
   ./scripts/setup_udev_rules.sh
   mkdir build && cd build
   cmake ..
   make -j4
   sudo make install
   ```

3. Install all realsense ROS packages from [here](https://github.com/IntelRealSense/realsense-ros/tree/2.3.2).

   ```bash
   sudo apt install ros-noetic-realsense2-*
   ```

4. Install MAVROS (version 0.29.0 or above) from [here](https://dev.px4.io/en/ros/mavros_installation.html).

   ```bash
   sudo apt-get install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs
   wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
   sudo bash ./install_geographiclib_datasets.sh
   ```

5. Install catkin and create your catkin workspace directory.

   ```bash
   sudo apt install python3-catkin-tools
   mkdir -p ~/vio_ws/src
   ```

6. Clone this repository in your catkin workspace.

   ```bash
   cd ~/vio_ws/src
   git clone https://github.com/waterqun/VIO.git
   ```

7. Build the package:

   ```bash
   cd ~/vio_ws
   catkin build px4_realsense_bridge
   ```

8. Run the ROS node:

   ```bash
   source ~/vio_ws/devel/setup.bash
   roslaunch px4_realsense_bridge bridge_mavros.launch
   ```

  > **Note** This launch file starts the mavros node as well. Mavros needs to be running but if it is started elsewhere the basic launch file *bridge.launch* can be used. The file *bridge_mavros_sitl.launch* is only for using this node in combination with the PX4 SITL toolchain (Simulation). Instructions on how to setup the PX4 SITL toolchain can be found [here](http://dev.px4.io/en/setup/dev_env_linux_ubuntu.html#common-dependencies). 

