Getting the fixed-wing HIL simulation framework to run
======================================================
You'll need to install and configure a few things until the framework is running correctly. Here's a description of the steps on Linux Ubuntu 14.04, running on a Lenovo computer.
Besides ROS and the RotorS simulator, you will need a ground control station (QGroundControl), as well as the proper hardware (Pixhawk with correct firmware). These instructions assume you are familiar with Git (documentation can be found on https://git-scm.com/doc).



RotorS
=======

RotorS is a MAV gazebo simulator.
It provides some multirotor models such as the [AscTec Hummingbird](http://www.asctec.de/en/uav-uas-drone-products/asctec-hummingbird/), the [AscTec Pelican](http://www.asctec.de/en/uav-uas-drone-products/asctec-pelican/), or the [AscTec Firefly](http://www.asctec.de/en/uav-uas-drone-products/asctec-firefly/), but the simulator is not limited for the use with these multicopters.

There are simulated sensors coming with the simulator such as an IMU, a generic odometry sensor, and the [VI-Sensor](http://wiki.ros.org/vi_sensor), which can be mounted on the multirotor.

This package also contains some example controllers, basic worlds, a joystick interface, and example launch files.

Below we provide the instructions necessary for getting started. See RotorS' wiki for more instructions and examples (https://github.com/ethz-asl/rotors_simulator/wiki).

If you are using this simulator within the research for your publication, please cite:
```bibtex
@Inbook{Furrer2016,
author="Furrer, Fadri
and Burri, Michael
and Achtelik, Markus
and Siegwart, Roland",
editor="Koubaa, Anis",
chapter="RotorS---A Modular Gazebo MAV Simulator Framework",
title="Robot Operating System (ROS): The Complete Reference (Volume 1)",
year="2016",
publisher="Springer International Publishing",
address="Cham",
pages="595--625",
isbn="978-3-319-26054-9",
doi="10.1007/978-3-319-26054-9_23",
url="http://dx.doi.org/10.1007/978-3-319-26054-9_23"
}
```
Installation Instructions
-------------------------

 1. Install and initialize ROS indigo desktop full, additional ROS packages, catkin-tools, and wstool:

 ```
 sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
 wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
 sudo apt-get update
 sudo apt-get install ros-indigo-desktop-full ros-indigo-joy ros-indigo-octomap-ros python-wstool python-catkin-tools
 sudo rosdep init
 rosdep update
 source /opt/ros/indigo/setup.bash
 ```
 2. If you don't have ROS workspace yet you can do so by

 ```
 mkdir -p ~/catkin_ws/src
 cd ~/catkin_ws/src
 catkin_init_workspace  # initialize your catkin workspace
 wstool init
 ```
 > **Note** for setups with multiple workspaces please refer to the official documentation at http://docs.ros.org/independent/api/rosinstall/html/ by replacing `rosws` by `wstool`.
 3. Get the simulator and additional dependencies

 ```
 cd ~/catkin_ws/src
 git clone git@github.com:ethz-asl/rotors_simulator.git
 git clone git@github.com:ethz-asl/mav_comm.git
 ```
  > **Note**: if you want to use `wstool` you can replace the above commands with
    ```
    wstool set --git local_repo_name git@github.com:organization/repo_name.git
    ```

 **Note**: if you want to build and use the `gazebo_mavlink_interface` plugin (which you will have to do for HIL simulation with a Pixhawk) you have to get MAVROS as an additional dependency from their website (https://github.com/mavlink/mavros). Follow the installation instructions provided there and build all of its packages prior to building the rest of your workspace.
 You can install MAVROS directly without going to the website by typing the following in a terminal:
 ```
 sudo apt-get install ros-indigo-mavros ros-indigo-mavros-extras
 ```

 4. ROS Indigo hosts the 2.x version of Gazebo. This simulation works using at least the 5.x version of Gazebo (more recent versions are less stable). The OSRF repository provides -gazebo5- versions of ROS/Indigo gazebo wrappers (gazebo5_ros_pkgs) which are built on top of the gazebo5 package. To use Gazebo 5.x with ROS Indigo:
 ```
 sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
 wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
 sudo apt-get update
 sudo apt-get install ros-indigo-gazebo5-ros-pkgs
 ```

 5. Build your workspace

 ```
 catkin_make
 ```
 > **Note**: don't forget to switch to the branch you wish (for example feature/fixed_wing_sim) before building, using
   ```
   git checkout feature/fixed_wing_sim
   ```

 Alternatively, with `python_catkin_tools` (therefore you need `python_catkin_tools`):
 ```
 cd ~/catkin_ws/
 catkin init  # If you haven't done this before.
 catkin build
 ```


 6. Add sourcing to your `.bashrc` file

 ```
 echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
 source ~/.bashrc
 ```

 7. Keyboard and joystick usage should already be working (good way to check whether everything has been installed properly). Install QGroundControl and set up your Pixhawk properly before using the HIL simulation.

Fixed-wing Aircraft Usage
-------------------------

#### Usage with a keyboard
Launch the simulator with a fixed-wing model, in our case, the Techpod model in the Uetliberg world:

```
roslaunch rotors_gazebo fixed_wing_with_keyboard_teleop.launch uav_name:=techpod world_name:=uetliberg
```
Unpause the physics (play button at the bottom of the gazebo window), and click on the terminal again. You can now pilot using w,a,d and the keyboard arrows.

#### Usage with a joystick

Connect a USB joystick to your computer and launch the simulator with a fixed-wing model, in our case, the Techpod model in the Yosemite world, which are the default parameters.

```
roslaunch rotors_gazebo fixed_wing_with_joy.launch uav_name:=techpod world_name:=yosemite
```

Depending on the type of the joystick and the personal preference for operation, you can assign the axis number using the `axis_<roll/pitch/thrust>_` parameter and the axis direction using the `axis_direction_<roll/pitch/thrust>` parameter.

#### Hardware-in-the-loop usage (with PX4)

 1. To run the hardware-in-the-loop (HIL) simulation you have to have gotten MAVROS as an additional dependency (step 3 of the installation instructions).

 2. Launch the simulator with a fixed-wing model and the HIL interface node.

 ```
 roslaunch rotors_gazebo fixed_wing_hil.launch
 ```

 3. Connect the PX4 autopilot to your computer (first SERIAL, then TELEM 1, as described in point 4 of the Pixhawk setup process) and launch an instance of MAVROS to relay messages to/from the hardware.

 ```
 roslaunch mavros px4.launch fcu_url:=<PX4_telem_address>:921600 gcs_url:=udp://127.0.0.1:14555@127.0.0.1:14560
 ```

 Where 'PX4_telem_address' is the device port on which the PX4 is connected (for example, '/dev/ttyUSB1') and the baud rate of 921600 is used for HIL communication. If the MAVROS node is operating properly, the RotorS GUI should receive a heartbeat message from the PX4 and some functionality should become enabled. The gcs_url sets up a UDP bridge to enable the telemetry data to be used both by mavros and a ground control station - e.g., QGC (14555 is the target host, and 14560 the listening port).

 4. If wished, open QGC and connect to PX4 via UDP link.

 5. Click the 'Enable HIL' button in the GUI.

 6. Once the 'HIL' mode has switched from OFF to ON, restart the PX4 by clicking the 'Reboot Autopilot' button in the GUI to restart the state estimator in HIL mode. Wait until the PX4 reboots and comes back online.

 7. Click the 'Arm' button in the GUI to arm the motors.

 8. At this point, the aircraft will move in accordance with the HIL_CONTROLS messages coming from the autopilot. It can be operated in a manual mode via a remote control communicating directly with the PX4.

  > **Note** If for some reason the commands are not passed on to the simulation, try loading, setting and writing the correct parameters for the aircraft using QGC.

QGroundControl
==============

Install QGroundControl to be able to monitor parameters while running the simulation.

Installation Instructions
-------------------------

 1. You will first need the source code of the ASL version of the program:

 ```
 git clone https://github.com/ethz-asl/fw_qgc.git
 git submodule init
 git submodule update
 ```

 2. Then, install Qt 5.5.1 to build the source code. Download the unified installer from https://www.qt.io/download-open-source/, make it executable using
 ```
 chmod +x qt-unified-linux-x64-2.0.3-1-online.run
 ```

 Double-click on it and follow the installation instructions. Make sure to select the correct version (5.5). Additionally, you will need the SDL1.2 library, which can be downloaded with:

 ```
 sudo apt-get install libsdl1.2-dev
 ```

 3. With QT Creator, open the project qgroundcontrol.pro located in fw_qgc, and make sure to select QT 5.5.1 GCC 64bit before clicking 'Configure project'. Before building and running, switch to 'Release' mode (and not 'Debug').
 Once it is finished, a new folder should have appeared, named 'build-qgroundcontrol-Desktop_Qt_5_5_1_GCC_64bit-Release' or something similar. In the 'release' subfolder, double-click qgroundcontrol to open it. You can lock it to the launcher for more convenience in the future (if no icon present, you can set it manually by updating the path to it in ~/.local/share/applications/qgroundcontrol.desktop).

  > **Note** If something does not work, try reinstalling Qt but this time running the installer with root privileges. To do so, after having made the installed executable, execute it with
    ```
    sudo ./qt-unified-linux-x64-2.0.3-1-online.run
    ```

Configuration
-------------
  1. When opening QGC for the first time, go to File->Settings->MAVLink and uncheck 'Only accept MAVs with the same protocol version'.

  2. Set up the necessary communication link. Go to File->Manage Communication Links, and click 'Add'. We are going to connect to the telemetry data used by MAVROS using its built-in UDP bridge as explained in the RotorS Fixed-wing Aircraft Usage section, point 3 of the HIL usage subsection. Name the link 'PX4 UDP', and set its type to 'UDP'. Listening Host is 14560, and Target Host is 127.0.0.1:14555.

  3. If you are connected to the Pixhawk (when the simulation and mavros are running), you can set the onboard parameters by going to the 'Analyze' menu and loading the correct .params file, clicking 'Set' and the 'Write'. This will save the parameter values into the Pixhawk.

Pixhawk
=======
The Developer's guide for the PX4 can be found at dev.io.px4. It provides a lot of information concerning the possible uses of the autopilot.


Setup process
-------------

  1. Write needed booting files onto the SD card: 'etc' folder containing 'rc.txt' as well as a 'telem_config' subfolder with 'telem0.txt' and 'telem1.txt' files; 'dataman' file; and 'params' file. Various log files and folders will appear when using the Pixhawk later on. They can be used for debugging.

  2. Get the correct version of the firmware from the ASL GitHub:
 ```
 git clone https://github.com/ethz-asl/fw_px4.git
 git submodule init
 git submodule update
 ```

  3. Flash the correct version of the firmware onto the Pixhawk. First, prepare the toolchain installation following the steps given on the DevGuide (Pixhawk is a NuttX based hardware):
 ```
 http://dev.px4.io/starting-installing-linux.html
 ```
 > **Note** Installing Ninja is not a bad idea.

 The actual flashing process is similar to that described in
 ```
 http://dev.px4.io/starting-building.html
 ```
 The difference is that we are using our own firmware, not the standard PX4 Firmware. The relevant directory is therefore 'fw_px4' instead of 'Firmware', and the firmware version is 'px4fmu-v2_asl'.

  > **Note** If 'make' does not yield the expected results (i.e., if 'make px4fmu-v2_asl' does not produce a successful run) and errors arise (ENOTSUP in my case), you might need to install GCC 4.9 manually as indicated in the toolchain installation, restart your computer and try flashing again.

  4. Connect the Pixhawk. The program 'screen' allows one to view the Pixhawk console (NuttShell) in the terminal, and to see the boot process, etc. It is very useful to monitor what happens with the Pixhawk. To install it, type the following in a terminal:
  ```
  sudo apt-get install screen
  ```
  If you are powering the Pixhawk by USB, first plug the SERIAL cable into your computer, open the link to the console in a terminal by typing
  ```
  screen <PX4_serial_address> 57600 8N1
  ```
  Where 'PX4_serial_address' is the device port on which the serial cable of the device is connected (for example, /dev/ttyUSB0) and the baud rate of 57600 is used. Then plug in the microUSB. You should see the booting process on the console. Then plug in TELEM1, and your Pixhawk is ready to use.
