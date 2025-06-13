# Global Planner: Robot Global Path Planning Solution  

## Project Overview  
The `global_planner` project is an open-source initiative focused on robot path planning, developed and simulated based on the open-source robot platform **TurtleBot3**. As a widely adopted open-source robot platform, TurtleBot3 is renowned for its high customizability and extensibility. The project's use of TurtleBot3 fully complies with relevant open-source license agreements.  


## Project Branch Description  
| Branch Name       | Function Description                                                                 |  
|-------------------|--------------------------------------------------------------------------------------|  
| **algorithm**     | Algorithm prototype branch storing initial implementations and test codes for various path planning algorithms, serving as a testing ground for algorithm development and verification. |  
| **main**          | Main branch containing optimized and tested planner plugins, which can be directly integrated into the ROS environment to provide efficient global path planning capabilities for TurtleBot3. |  


## Environment Configuration  

### System Requirements  
| Configuration       | Recommended Settings                                                                 |  
|---------------------|--------------------------------------------------------------------------------------|  
| Operating System    | Ubuntu 18.04 (ROS Melodic) / Ubuntu 20.04 (ROS Noetic); verified only on Ubuntu 20.04. |  
| ROS Version         | Corresponding to the Ubuntu version (Melodic for 18.04, Noetic for 20.04).            |  


### Installation Steps  
#### 1. Install ROS  
Follow the official ROS documentation to install ROS based on your Ubuntu version:  
- [ROS Melodic Installation Guide](http://wiki.ros.org/melodic/Installation/Ubuntu)  
- [ROS Noetic Installation Guide](http://wiki.ros.org/noetic/Installation/Ubuntu)  

#### 2. Install TurtleBot3-related Packages  
Execute the command in the terminal:  
```bash  
sudo apt-get install ros-$ROS_DISTRO-turtlebot3*  
```  
Here, `$ROS_DISTRO` will be automatically replaced with `melodic` or `noetic` based on your ROS version.  

#### 3. Clone the Project  
Clone the project into the `src` directory of your ROS workspace:  
```bash  
cd ~/catkin_ws/src  
git clone https://github.com/scnscnscn/global_planner.git  
```  

#### 4. Compile the Project  
Return to the root directory of your ROS workspace and compile:  
```bash  
cd ~/catkin_ws  
catkin_make  
```  

#### 5. Environment Variable Setup  
Set the TurtleBot3 model type each time you open a new terminal. For example, for the TurtleBot3 Burger model:  
```bash  
export TURTLEBOT3_MODEL=waffle  # Replace "waffle" with "burger" or "waffle_pi" as needed  
# Note: "burger" is the version without a camera.  
```  


## Usage Instructions  

### Run the Simulation  
Execute the following commands sequentially in the terminal:  
0. Choose the global planner by editing `move_base.launch` in `turtlebot3_navigation`:  
```xml  
<param name="base_global_planner" value="Replace with your global planner name" />  
```  

1. Start the Gazebo simulation environment:  
```bash  
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch  
```  

2. Start the navigation function:  
```bash  
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml  
```  
Here, `$HOME/map.yaml` is the path to your map file; modify it according to your needs.  

## Open-source License  
This project is licensed under the [MIT License](https://opensource.org/licenses/MIT). For any issues or improvement suggestions, please raise them on the project's [GitHub Issues](https://github.com/scnscnscn/global_planner/issues) page.  


## Contribution  
We welcome all forms of contributions, including code submissions, issue feedback, and documentation improvements. To participate in project development:  
1. Fork the project to your GitHub account.  
2. Create a new branch for development.  
3. Submit your code and initiate a Pull Request.  


Thank you for your support and participation!
