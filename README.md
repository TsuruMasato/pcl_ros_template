# edge_detector_ros

ROS node to extract environmental edge in 3D Point Cloud.

## How to compile

1. `$ cd your_catkin_ws/src`

2. `$ git clone git@github.com:Osaka-University-Harada-Laboratory/edge_detector_ros.git`

3. `$ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release`  (This magic option accelerates C++ running speed)

4. `$ catkin build`

Then, please load new environment variables:

5. `$ source devel/setup.bash`

## How to run

#### Type A : single node

`$ rosrun edge_detector_ros edge_detector_node`

#### Type B : start with AzureKinect ros driver

`$ roslaunch edge_detector_ros edge_detector_with_AzureKinect.launch`
