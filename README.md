# Real Time Gesture Recognition using ROS2
A gesture recognition system implemented with ROS 2.

## Getting started with the project
Here are a few details to help you get started on this project quickly.

### Prerequisites
The hardware and software requirements are listed below.

#### Hardware
* A PC (preferably with a Ubuntu 20.04 LTS installation)
* Orbbec Astra 3D camera

#### Software

* ROS 2 Foxy installed on a Ubuntu Linux Focal Fossa (20.04) 64-bit / x86 / 64-bit / ARM or any other ROS 2 supported machine. You may find more about the installation process from [ROS 2 foxy installation documentation.](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Binary.html)
* Set up and configure the ROS 2 as required. [Read More](https://docs.ros.org/en/foxy/Tutorials/Configuring-ROS2-Environment.html)
* Create a workspace for starting to create and organise projects with ROS 2. [Read More](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html)
* Install Astra drivers for OpenNI 2 for Linux. Installation and Download link can be [found here.](https://astra-wiki.readthedocs.io/en/latest/downloadDriver.html#linux)
* Python 3.8
* Open3D Python library ```pip install open3d```
* OpenNI Python library ```pip install openni```
* Tensorflow ```pip install tensorflow```
* Numpy ```pip install numpy```

## Running the project

To start with the project,

Clone the repo into your ROS2 workspace eg: ```~/dev_ws/src/```

```(bash)
git clone https://github.com/beslintony/Real-Time-Gesture-Recognition-using-ROS2
```
Note: Considering you have set up the workspace as above, ie, ```~/dev_ws/``` as your workspace, add the following alias in your ```~/.bashrc``` file. 

```alias fsod="cd ~/dev_ws && . install/setup.bash"```

Build the packages

```(bash)
fsod
colcon build
```

After completing initial build, run the depth_pub ROS2 node

```(bash)
fsod
ros2 run hand_gestures depth_pub
```

Open a new terminal window and run the pfh_publisher_node ROS 2 node

```(bash)
fsod
ros2 run pfh_publisher pfh_publisher_node 
```

Open yet another terminal window and run all the 8 nodes from pfh_subscriber package using the ROS 2 launch command

```(bash)
fsod
ros2 launch pfh_subscriber pfh_subscribers.launch.py
```

Run the pfh_all_lstm_sub

```(bash)
fsod
ros2 run pfh_subscriber pfh_all_lstm_sub 
```

## Hints

In the ```hand_gestures``` package, ```dist``` is hardcoded to the OpenNI driver for Astra device.

```(python)
dist = "/home/fdai6135/AstraOpenNI2Drivers/OpenNI-Linux-x64-2.3.0.66/Redist"
````

You could map the value to your own ```Redist``` folder of the OpenNI driver.

##
In the ```pfh_subscriber``` package, lstm models are moved to the `lib/python3.8/site-packages` at the compile time. In the file```pfh_subscriber/setup.py``` you will the following snippet:

```(python)
(os.path.join('lib/python3.8/site-packages', package_name, "savedModel"), glob('savedModel/new/*.h5')),
````

You should consider changing the path, if the python version you are using is different.

