<img src=https://dreamvu.com/wp-content/uploads/2020/07/logo_footer_trans-1-1.png alt="DreamVU">
# ros2_dreamvu_pal_description

This unoffical package creates a ros2 node which publishes the shape and coordinate frames of [the DreamVU PAL camera](https://dreamvu.com/pal-usb/), based on a description described in a xacro file.

An official ros1-package is available on request from [the DreamVU company](https://support.dreamvu.com/portal/en/home), which is a package based on the Catkin build system and has a description purely based on urdf (while xarco can be used to <a href=http://wiki.ros.org/urdf/Tutorials/Using%20Xacro%20to%20Clean%20Up%20a%20URDF%20File>to clean urdf files</a> and ROS2 has the additional feature that it can read in a Xarco file directly).

This package here is based on the Colcon build system, which allows to build the package for the combination of Ubuntu 20.04 (Focal Fossa) with ROS2 Foxy Fitxroy.
  
This package allows you use the visualize the PAL camera with rviz2 of ROS2. It will publish the ros2 topics '/dreamvu/pal/robot_description' and '/dreamvu/pal/joint_states', which correspond to a number of 'static_tf' messages. Note, that all the 'static_tf' are coordinate transformations relative to the camera_center, which is defined (configuarble thanks to ROS2 parameters) on a height of 6cm above the base_link. The base_link defines the coordinate system of a robot, which can moves around through the map. The coordinate frames inside the sensor are only defined when the transformation between the base_link and map are known. If the sensor is not mounted on a robot, a separate static_transform publisher for this base_link to map transformation is provided in the launch-directory.

<img src=images/Rviz2.png"
     alt="DreamVu pal description in Rviz2"
     style="float: left; margin-right: 10px;" width=300/>

 ## Known issues

* The published messages require a chain of coordinate transformations from the pal_camera_center to the robot's base_link to the map to allow rviz2 to visualize the description (and the images and point cloud).
* The images should be published relative to optical frame, with the z-axis in the horizontal plane (instead of the current z-axis pointing upwards). The naming of the coordinates frames of this node and the ones in <a href=https://github.com/physar/ros2_pal_camera_node>ros2_pal_camera_node</a> should be consistent.
* The software is currently tested on one computer, more tests are on the way.
                                             
### Prerequisites

* [Ubuntu 20.04 (Focal Fossa)](https://releases.ubuntu.com/focal/)

* [ROS2 Foxy Fitxroy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

If the ros-foxy repository is not already on your package list, add it

```bash
sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
```
This ros-node is using the following ros-packages:

```bash
$ sudo apt install ros-foxy-ros-base
$ sudo apt install ros-foxy-rviz2
$ sudo apt install ros-foxy-xacro
$ sudo apt install ros-foxy-robot-state-publisher
```
* [Colcon build system](https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html)
```bash
$ sudo apt install python3-rosdep2
$ sudo apt install python3-colcon-common-extensions
```
## Build the package

To install the **ros2 pal_camera_description**, clone the package from github and build it:

```bash
$ cd ~/ros2_ws/src/ #use your current ros2 workspace folder
$ git clone https://github.com/physar/ros2_pal_camera_description.git
$ cd ~/ros2_ws/
$ sudo apt-get update
$ rosdep update
$ rosdep install --from-paths src --ignore-src -r -y
$ colcon build --packages-select dreamvu_pal_camera_description --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
$ source ~/ros2_ws/install/local_setup.bash
```
If you want to install this package permanently ot your shell,

```bash
$ echo source $(pwd)/install/local_setup.bash >> ~/.bashrc
$ source ~/.bashrc
```
# Starting the PAL camera description

To start the **ros2 pal_camera_description**, open a terminal and start it with the command.

```bash
$ source /opt/ros/foxy/setup.bash
$ source $~/ros2_ws/install/local_setup.bash
$ ros2 launch dreamvu_pal_camera_description pal_camera_description.launch.py cam_pos_z:=0.06
```

# Inspect the published description

```bash
$ ros2 topic list | grep dreamvu
```


This should give the following result:

```bash
/dreamvu/pal/joint_states
/dreamvu/pal/robot_description
```

The complete set of published data can be seen by the following command:
```bash
ros2 run rviz2 rviz2
```

Add in this view a TF (and activate the frame pal_camera_center) and a RobotModel (for the topic ```/dreamvu/pal/robot_description```).

# camera description and rviz2 combined

This package also contains a launch script with launches both the description and rviz2, with the views of rviz2 are already provided.

```bash
$ source /opt/ros/foxy/setup.bash
$ source $~/ros2_ws/install/local_setup.bash
$ ros2 launch dreamvu_pal_camera_description display+pal_camera.launch.py
```

 Troubleshooting

* If your PAL camera is description is not visible in rviz2, open the RobotModel and look which property is marked red to indicate an error.

* If the transforms indicate an error, start a static_transform_publisher from the map to the base_link by the command: 

```bash
$ source /opt/ros/foxy/setup.bash
$ source $~/ros2_ws/install/local_setup.bash
$ ros2 launch dreamvu_pal_camera_description transfer_base_link.launch.py
```

* rviz2 is dropping messages. This means that sensor data (images) are published with from a coordinate system which is not initiated (yet). Probably the chain is broken for the coordinate transformation from the map to the base_link. The camera images are published from the coordinate system 'pal_camera_center'. See for the solution the state_transform_publisher above.







                                             
                                             

                                           
