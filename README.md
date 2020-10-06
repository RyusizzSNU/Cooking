# Project Background

## Copyright

# Setup

Download the 18.04 LTS desktop image from [Ubuntu Install]()

## NVIDIA driver and anaconda setup

This is case of teemo notebook

```
sudo apt-get install nvidia-driver-435
```

## ROS Melodic install

## Realsense Install

 ```
 export ROS_VER=melodic
 sudo apt-get install ros-$ROS_VER-realsense2-camera
 ```
 
 ## Anaconda setup
 
 Download the anaconda setup file and and execute it.
 Create conda environment with python2.
 
 ```
 conda create -n dope2 python=2.7
 ```
 
 Install dependencies
 
 ```
 pip install pyyaml numpy==1.8.1 scipy==1.4.1 opencv_python==3.
 ```
 
 ## Catkin Build
 
 Check that conda environment is deactivated.
 
 # Learning
 
 ## DOPE
 
 ```
 python train_no_crop.py --data ~/data/HandAI_object_data/Data_salt_plate --outf train_salt_plate --batchsize 64 --epochs 60 --imagesize 512 --gpuids 0 1 2 3
 ```
 
 # Run
 
 ## DOPE
 
Activate conda environment.

```
conda activate dope2
```

Launch realsense camera input and dope inference nodes.
 
```
roslaunch realsense2_camera rs_camera.launch
roslaunch dope dope.launch config:=/home/tidy/catkin_ws/src/dope/config/my_config_cooking.yaml`
``` 

To visualize the dope, see the rviz or image_view

```
rosrun image_view image_view image:=/dope/rgb_points
```

