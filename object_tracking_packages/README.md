# Object Tracking ROS Packages

## Description
This repository contains two ROS Noetic packages designed for real-time 3D object tracking:

- `object_tracking`: Utilizes YOLOv8 ByteTrack and data from an Intel RealSense camera to track objects in real-time and publish their coordinates.
- `object_tracking_msgs`: Contains the necessary custom message types for the `object_tracking` package, separated for convenience.

## Installation

### Prerequisites
- ROS Noetic
- Intel RealSense Camera Software
- Python 3


### Dependencies
Ensure you have the following Python packages installed:

```bash
pip install pyrealsense2
pip install ultralytics==8.0.202
pip install supervision==0.16.0
```

### Building the Packages
To build the packages, clone this repository into your ROS workspace and compile it using `catkin build`:

```bash
cd ~/catkin_ws/src
git clone https://github.com/Liquiduck/object_tracking object_tracking_packages
cd ~/catkin_ws
catkin build
source devel/setup.bash
```
## Usage
After installation, you can run the object tracking nodes using ROS. Ensure your Intel RealSense Camera is connected and recognized by the system.

To start the object tracking, run in seperate terminals:
```bash
roscore
```
```bash
roslaunch object_tracking object_tracking.launch
```

## Features
- Real-time 3D tracking of objects.
- Utilizes state-of-the-art YOLOv8 ByteTrack algorithm.
- Custom ROS messages for easy integration with other ROS packages.

## Contributing
Contributions to improve `object_tracking` and `object_tracking_msgs` are welcome. Please submit a pull request or open an issue to discuss proposed changes.

## License
This project is licensed under the [MIT License](LICENSE). Please see the `LICENSE` file for more details.

## Contact
For questions or support, please open an issue in the repository, and a maintainer will assist you.

## Acknowledgments
- Ultralytics for YOLOv8.
- The developers of ByteTrack.
- Intel RealSense team for camera interfaces


  
