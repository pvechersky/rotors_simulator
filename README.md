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

 **Note**: if you want to build and use the `gazebo_mavlink_interface` plugin (which you will have to do for HIL simulation with a Pixhawk) you have to get MAVROS as an additional dependency from their website (https://github.com/mavlink/mavros). Follow the installation instructions provided there and build all of its packages with 'catkin build' prior to building the rest of your workspace.
 Make sure to install it from source, and to get the latest source during installation (binary/prior version may have bugs).

 4. ROS Indigo hosts the 2.x version of Gazebo. This simulation works using at least the 5.x version of Gazebo (more recent versions are less stable). The OSRF repository provides -gazebo5- versions of ROS/Indigo gazebo wrappers (gazebo5_ros_pkgs) which are built on top of the gazebo5 package. To use Gazebo 5.x with ROS Indigo:
 ```
 sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
 wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
 sudo apt-get update
 sudo apt-get install ros-indigo-gazebo5-ros-pkgs
 ```

 5. Build your workspace with `python_catkin_tools` (therefore you need `python_catkin_tools`):
 ```
 cd ~/catkin_ws/
 catkin init  # If you haven't done this before.
 catkin build
 ```


 > **Note**: don't forget to switch the rotors_simulator package to the branch you wish (for example feature/fixed_wing_sim) before building, using
   ```
   cd ~/catkin_ws/src/rotors_simulator
   git checkout feature/fixed_wing_sim
   ```

 6. Add sourcing to your `.bashrc` file

 ```
 echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
 source ~/.bashrc
 ```

 7. Keyboard and joystick usage should already be working (good way to check whether everything has been installed properly). Install QGroundControl and set up your Pixhawk properly before using the HIL simulation.

 8.
  > **Note** If you shall ever need to reinstall everything again, make sure to uninstall gazebo 5 first (as a new ROS installation will not be compatible with gazebo 5), and to reinstall SDF libraries :
  ```
  sudo apt-get remove gazebo*
  sudo apt-get install libsdformat1
  ```
  You can then follow the installation instructions from the beginning again.

Fixed-wing Aircraft Usage
-------------------------

#### Usage with a keyboard
Launch the simulator with a fixed-wing model, in our case, the Techpod model in the Uetliberg world for example:

```
roslaunch rotors_gazebo fixed_wing_keyboard_teleop.launch uav_name:=techpod world_name:=uetliberg
```
Unpause the physics (play button at the bottom of the gazebo window), and click on the terminal again. You can now pilot using w,a,d and the keyboard arrows.
 > **Note** You can also load your own worlds if wished. The aircraft spawn position should be adjusted as needed in the launch file.

#### Usage with a joystick

Connect a USB joystick to your computer and launch the simulator with a fixed-wing model, in our case, the Techpod model in the Yosemite world, which are the default parameters.

```
roslaunch rotors_gazebo fixed_wing_with_joy.launch uav_name:=techpod world_name:=yosemite
```

Depending on the type of the joystick and the personal preference for operation, you can assign the axis number using the `axis_<roll/pitch/thrust>_` parameter and the axis direction using the `axis_direction_<roll/pitch/thrust>` parameter.

#### Hardware-in-the-loop usage (with PX4)

 1. To run the hardware-in-the-loop (HIL) simulation you have to have gotten MAVROS as an additional dependency (step 3 of the installation instructions), configured your Pixhawk autopilot and installed QGroundControl.

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

Extended Wind Plugin
-------------------------

The existing wind plugin in RotorS has been extended to allow the use of a custom, static wind field for a given world. This is done by enabling the wind plugin macro in the aircraft base file (in our case `techpod_base.xacro`, which is found in `rotors_simulator/rotors_description/urdf`), and feeding it the correct parameters and files. The wind grid and values are generated using a suitable model, and saved in a text file read by the plugin.

#### Grid specifications

The grid used to define the wind field must be equidistant in x, respectively y-direction. The points in z-direction are distributed with upwards linearly increasing distance (spacing factors can be tuned as desired).

#### Wind field text file format

The text file contains information about the grid geometry as well as the wind values at each grid point. The data needed in the text file consists of:

  1. Smallest x-coordinate of the grid `min_x`, of type integer, in [m].

  2. Smallest y-coordinate of the grid `min_y`, of type integer, in [m].

  3. Number of grid points in x-direction `n_x`, of type integer.

  4. Number of grid points in y-direction `n_y`, of type integer.

  5. Resolution in x-direction `res_x`, of type float, in [m].

  6. Resolution in y-direction `res_y`, of type float, in [m].

  7. Spacing factors in z-direction stored in `vertical_spacing_factors`, an (n_z)-dimensional array of float values between 0 for the lowest and 1 for the highest point.
  8. The altitude of each grid point contained in the lower x-y plane, stored in `bottom_z`, an (n_x*n_y)-dimensional array of float values, in [m].
  > **Note**: Element [0] is the grid corner point with the lowest x and y-coordinates, and the array is filled counting up in x-direction first, then in y-direction (such that a point with indices i,j corresponds to the (i+j*n_x)th element of the array).

  9. Similarly, the altitude of each grid point contained in the upper x-y plane, stored in `top_z`, an (n_x*n_y)-dimensional array of float values, in [m].

  10. The speed of the wind in x-direction for each grid point, stored in `u`, an (n_x*n_y*n_z)-dimensional array of float values, in [m/s].
  > **Note**: Element [0] is the grid corner point with the lowest x,y and z-coordinates, and the array is filled counting up in x-direction first, then in y-direction and finally in z-direction (such that a point with indices i,j,k corresponds to the (i + j*n_x + k*n_x*n_y)th element of the array).

  11. Similarly, the speed of the wind in y-direction stored in the array `v`, and in z-direction stored in `w`.

The order in which the data is saved in the text file is not relevant, but the format must comply with the following requirements:

  1. In the first line of the file, the name of one of the 12 needed data followed directly by a semicolon (e.g., `vertical_spacing_factors:`)

  2. In the following line, the corresponding data. If multiple values are needed, they must be separated by a space (e.g., `0.0 0.025641 0.051282 0.076923 ...`)

  3. The rest of the data follows the same format.

In the case of the hemicylindrical world, the beginning of the text file describing a divergent-free wind field looks as follows:

```
min_x:
-2000.0
min_y:
-2000.0
n_x:
201
n_y:
2
...
```

For clarity and convenience, the wind field text file is placed in the same folder as the world model.

#### Wind Plugin Macro

Here is an example of the plugin macro to be added in the base file, containing numerous necessary user-defined variables:

```
<xacro:wind_plugin_macro
  namespace="${namespace}"
  xyz_offset="0 0 0"
  wind_direction="1 0 0"
  wind_force_mean="0.0"
  wind_force_variance="0.0"
  wind_speed_mean="5.0"
  wind_speed_variance="0.0"
  wind_gust_direction="0 1 0"
  wind_gust_duration="0.0"
  wind_gust_start="0.0"
  wind_gust_force_mean="0.0"
  wind_gust_force_variance="0.0"
  custom_static_wind_field="true"
  custom_wind_field_path="$(find rotors_gazebo)/models/hemicyl/custom_wind_field.txt">
</xacro:wind_plugin_macro>
```
All parameters needed in the macro as well as their units are specified in the `component_snippets.xacro` file.
The four relevant values for the use of the extended plugin are `wind_direction` and `wind_speed_mean`, which specify the default constant wind field when the aircraft is flying outside of the custom wind field region, the boolean `custom_static_wind_field` which, when set to `true`, enables the extended functionality, as well as the string `custom_wind_field_path` which describes the path (from `~/.ros`) to the text file specifying the grid and wind field.

#### Functioning
In brief, the plugin works in distinct steps:

  1. Load the plugin and read the text file once, saving the data.

  2. During update event:

    2.1. Locate the aircraft and see if it is flying within the specified wind field bounds.

    2.2. If so, identify the grid points at the vertices of the enclosing cell and extract their wind values. If not, set the wind velocity to the default, user-defined value.

    2.3. Interpolate linearly in z,y and x-directions to find the wind velocity at the aircraft position.

    2.4. Publish the wind velocity in a wind speed message.



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

 2. Then, install Qt 5.5.1 to build the source code. Download the unified installer from https://www.qt.io/download-open-source/ and make it executable using :
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

  2. Set up the necessary communication link. Go to File->Manage Communication Links, and click 'Add'. We are going to connect to the telemetry data used by MAVROS using its built-in UDP bridge as explained in the RotorS Fixed-wing Aircraft Usage section, point 3 of the HIL usage subsection. Name the link (e.g, 'PX4 UDP'), and set its type to 'UDP'. Listening Host is 14560, and Target Host is 127.0.0.1:14555.

  3. If you are connected to the Pixhawk (when the simulation and mavros are running), you can set the onboard parameters by going to the 'Analyze' menu and loading the correct .params file, clicking 'Set' and then 'Write (ROM)'. This will save the parameter values into the Pixhawk.

Pixhawk
=======
The Developer's guide for the PX4 can be found at dev.px4.io. It provides a lot of information concerning the possible uses of the autopilot.


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
