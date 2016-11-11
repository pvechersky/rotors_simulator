RotorS
===============

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
 $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
 $ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
 $ sudo apt-get update
 $ sudo apt-get install ros-indigo-desktop-full ros-indigo-joy ros-indigo-octomap-ros python-wstool python-catkin-tools
 $ sudo rosdep init
 $ rosdep update
 $ source /opt/ros/indigo/setup.bash
 ```
 2. If you don't have ROS workspace yet you can do so by

 ```
 $ mkdir -p ~/catkin_ws/src
 $ cd ~/catkin_ws/src
 $ catkin_init_workspace  # initialize your catkin workspace
 $ wstool init
 ```
 > **Note** for setups with multiple workspaces please refer to the official documentation at http://docs.ros.org/independent/api/rosinstall/html/ by replacing `rosws` by `wstool`.
 3. Get the simulator and additional dependencies

 ```
 $ cd ~/catkin_ws/src
 $ git clone git@github.com:ethz-asl/rotors_simulator.git
 $ git clone git@github.com:ethz-asl/mav_comm.git
 ```
  > **Note** if you want to use `wstool` you can replace the above commands with
    ```
    wstool set --git local_repo_name git@github.com:organization/repo_name.git
    ```
  > **Note** if you want to build and use the `gazebo_mavlink_interface` plugin you have to get MAVROS as an additional dependency from link below. Follow the installation instructions provided there and build all of its packages prior to building the rest of your workspace. 
    ```
    https://github.com/mavlink/mavros
    ```
 4. Build your workspace with `python_catkin_tools` (therefore you need `python_catkin_tools`)

   ```
   $ cd ~/catkin_ws/
   $ catkin init  # If you haven't done this before.
   $ catkin build
   ```

 5. Add sourcing to your `.bashrc` file

   ```
   $ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   $ source ~/.bashrc
   ```

Basic Usage
-----------

Launch the simulator with a hex-rotor helicopter model, in our case, the AscTec Firefly in a basic world.

```
$ roslaunch rotors_gazebo mav_empty_world.launch mav_name:=firefly world_name:=basic
```

> **Note** The first run of gazebo might take considerably long, as it will download some models from an online database.

The simulator starts by default in paused mode. To start it you can either
 - use the Gazebo GUI and press the play button
 - or you can send the following service call.

   ```
   $ rosservice call gazebo/unpause_physics
   ```

There are some basic launch files where you can load the different multicopters with additional sensors. They can all be found in `~/catkin_ws/src/rotors_simulator/rotors_gazebo/launch`.

The `world_name` argument looks for a .world file with a corresponding name in `~/catkin_ws/src/rotors_simulator/rotors_gazebo/worlds`. By default, all launch files, with the exception of those that have the world name explicitly included in the file name, use the empty world described in `basic.world`.

### Getting the multicopter to fly

To let the multicopter fly you need to generate thrust with the rotors, this is achieved by sending commands to the multicopter, which make the rotors spin.
There are currently a few ways to send commands to the multicopter, we will show one of them here.
The rest is documented [here](../../wiki) in our Wiki.
We will here also show how to write a stabilizing controller and how you can control the multicopter with a joystick.

#### Send direct motor commands

We will for now just send some constant motor velocities to the multicopter.

```
$ rostopic pub /firefly/command/motor_speed mav_msgs/Actuators '{angular_velocities: [100, 100, 100, 100, 100, 100]}'
```

> **Note** The size of the `motor_speed` array should be equal to the number of motors you have in your model of choice (e.g. 6 in the Firefly model).

You should see (if you unpaused the simulator and you have a multicopter in it), that the rotors start spinning. The thrust generated by these motor velocities is not enough though to let the multicopter take off.
> You can play with the numbers and will realize that the Firefly will take off with motor speeds of about 545 on each rotor. The multicopter is unstable though, since there is no controller running, if you just set the motor speeds.


#### Let the helicopter hover with ground truth odometry

You can let the helicopter hover with ground truth odometry (perfect state estimation), by launching:

```
$ roslaunch rotors_gazebo mav_hovering_example.launch mav_name:=firefly world_name:=basic
```

#### Create an attitude controller

**TODO(ff):** `Write something here.`

#### Usage with a joystick

**TODO(ff):** `Write something here.`

Fixed-wing Aircraft Usage
-------------------------

#### Usage with a joystick

Connect a USB joystick to your computer and launch the simulator with a fixed-wing model, in our case, the Techpod model in the Yosemite world, which are the default parameters.

```
$ roslaunch rotors_gazebo fixed_wing_with_joy.launch uav_name:=techpod world_name:=yosemite
```

Depending on the type of the joystick and the personal preference for operation, you can assign the axis number using the "axis_<roll/pitch/thrust>_" parameter and the axis direction using the "axis_direction_<roll/pitch/thrust>" parameter.

#### Hardware-in-the-loop usage (with PX4)

1. To run the hardware-in-the-loop (HIL) simulation you have to get MAVROS as an additional dependency from link below. Follow the installation instructions provided there and build all of its packages prior to building the rest of your workspace.

```
https://github.com/mavlink/mavros
```

2. Launch the simulator with a fixed-wing model and the HIL interface node.

```
$ roslaunch rotors_gazebo fixed_wing_hil.launch
```

3. Connect the PX4 autopilot to your computer and launch an instance of MAVROS to relay messages to/from the hardware.

```
$ roslaunch mavros px4.launch fcu_url:=<PX4_address>:921600
```

Where 'PX4_address' is the device port on which the PX4 is connected (for example, '/dev/ttyUSB1') and the baud rate of 921600 is used for HIL communication. If the MAVROS node is operating properly, the RotorS GUI should receive a heartbeat message from the PX4 and some functionality should become enabled.

4. Click the 'Enable HIL' button in the GUI.

5. Once the 'HIL' mode has switched from OFF to ON, restart the PX4 by clicking the 'Reboot Autopilot' button in the GUI. Wait until the PX4 reboots and comes back online.

6. Click the 'Arm' button in the GUI to arm the motors.

7. At this point, the aircraft will move in accordance with the HIL_CONTROLS messages coming from the autopilot. It can be operated in a manual mode via a remote control communicating directly with the PX4.

