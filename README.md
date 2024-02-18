# F1Tenth Gazebo Simulator

A Gazebo-based simulation created for the F1Tenth Platform focused on perception-based autonomous driving.


## Installation
This project is intended to be built on a system running Ubuntu 20.04.

This project relies on ROS Noetic, Gazebo 11, and OpenCV 4.

Instructions to install ROS Noetic can be found [here](http://wiki.ros.org/noetic/Installation/Ubuntu).

Instructions to install Gazebo 11 can be found [here](https://classic.gazebosim.org/tutorials?tut=install_ubuntu).

Instructions to install OpenCV 4 can be found [here](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html).

This simulator is a series of ROS packages that can be built using catkin and the catkin_workspace command.

Tutorials to set up a catkin workspace and build a ros package can be found [here](https://wiki.ros.org/catkin/Tutorials).  

Once ROS, Gazebo, and OpenCV are set up, and the ROS packages are built in a catkin workspace, the last step is to add the car models to the gazebo model path, so that the vehicles can be quickly added from the side bar.

To do this simply open the bash.rc file in the user's home directory, and append the following line t
```bash
export GAZEBO_MODEL_PATH=[Path to Catkin Workspace]/src/new_f1tenth_simulator
```
where the path to the catkin workspace is the location under which the ROS packages were installed.

## Running the Simulator
To run the simulator, simply open a terminal and type in the following command:

```bash
roslaunch f1tenth_gazebo simulator.launch
```

This command should launch a seperate gazebo window with the simulator. 

When closing the program, it is recommended that you close the gazebo window before terminating the simulator. It is necessary to close both the Gazebo window, and terminate the simulator through the command line with CTR+C. Failure to do so may cause issues with sensor libraries when launching the simulator again through the terminal until the device is reset.

## Simulator Configurations
### Changing the Map
To change the track with which the cars will drive on, navigate to new_f1tenth_simulator/f1tenth_gazebo/simulator.launch and modify the track number parameter. This parameter will change the track of the f1tenth_simulator to the track described by the road_config.h file in the road description package. The track is actually added to the world through the f1tenth_track_model package and its track_plugin.cc file. If additional items or models are needed for the track, they can be added to the f1tenth_track_model package.

### Autonomous vehicle control options
By default, all cars in the simulator except for the base f1tenth_model car have automatic lane following options added by default. To disable this feature or switch the autonomous driving mode, simply open new_f1tenth_simulator/f1tenth_gazebo/simulator.launch  and remove or comment out the autonomous driving control nodes named sample_lane_follower_node.

### Enabling / disabling semantic segmentation output
By default, the semantic segmentation camera is only added to the base f1tenth model. To add the semantic segmentation camera to the other vehicles, simply edit new_f1tenth_simulator/f1tenth_gazebo/simulator.launch and add the commented nodes under the semantic segmentation section to the launch file. 

Due to gazebo having no innate support for semantic segmentation, we cannot use the semantic segmentation camera under the normal conditions of the simulator. Without modifying the models of the objects in the simulator, the image will come out as a black image. To avoid this issue, we simply modify all the textures of the vehicle and road such that they only contain one color. Using the RGB values of these textures, we can obtain the segmentation masks for each object. To condense the models to a single color, simply uncomment the material line in each model's urdf file. 
This should look something like
```
<!--<material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse><specular>0 0 0 0</specular><emissive>1 1 1 1</emissive></material>-->

``` 
Changing the emissive value to some r g b a combination will set a color that the simulator will identify. These colors are classified in the f1tenth_semantic_segmentation.cpp file in the f1tenth_semantic_segmentation folder.

### Adding additional cars to the simulator
Once the simulator has been launched, navigate to the Insert tap in the top left. Assuming the gazebo model path has been properly set in the installation instructions, there should be a drop-down menu for [Path to Catkin Workspace]/src/new_f1tenth_simulator. Simply left click on the car that you would like to add, and left click again in the main window to place the car into the simulation. 

Only one car of each type can be placed in the simulator at a time. This includes the f1tenth model included in the simulator by default.


## Common Issues
