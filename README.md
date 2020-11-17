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
 conda create -n dope python=3.8.3
 ```
 
 Install dependencies
 
 ```
 pip install pyyaml pyrr==0.10.3 numpy==1.18.1 scipy==1.4.1 opencv_python==3.4.9.33 pillow==7.1.2
 ```
 
 Install pytorch
 
 ```
 pip install torch==1.5.0+cu101 torchvision==0.6.0+cu101 -f https://download.pytorch.org/whl/torch_stable.html
 ```
 
 ## Catkin Build
 
 Check that conda environment is deactivated.
 
 # Learning
 
 ## DOPE
 
 ```
 python train_no_crop.py --data ~/data/HandAI_object_data/Data_salt_plate --outf train_salt_plate --batchsize 64 --epochs 60 --imagesize 512 --gpuids 0 1 2 3
 python train.py --data ~/data/HandAI_object_data/Data_salt_plate --outf ~/dope/wegiths/HandAI/salt_plate --batchsize 64 epochs 60 --gpuids 0 1 2 3 --object salt_plate
 ```
 
 # Run
 
 ## DOPE
 
Activate conda environment.

```
conda activate dope2
```

Download weight files

```
scp -P 22 gold@147.46.219.171:/home/gold/dope/weights/[file_name] ./catkin_ws/src/dope/weights/[file_name]
```

Launch realsense camera input and dope inference nodes.
 
```
roslaunch realsense2_camera rs_camera.launch
roslaunch dope dope.launch config:=/home/tidy/catkin_ws/src/dope/config/my_config_cooking.yaml`
```

Launch dual realsense camera input
(left)
```
roslaunch realsense2_camera rs_camera.launch camera:=cam_L serial_no:=843112073636 filters:=spatial,temporal
```
(right)
```
roslaunch realsense2_camera rs_camera.launch camera:=cam_R serial_no:=021222071327 filters:=spatial,temporal
```
(dope)
```
roslaunch dope dope_double.launch config:=/home/tidy/Cooking/catkin_ws/src/dope/config/config_pose_cooking_ver4.yaml
```

To visualize the dope, see the rviz or image_view

```
rosrun image_view image_view image:=/dope/rgb_points
```

Turn on/off the object model on ROS (using command line)

```
rosparam set /dope/activities/[object_name] true/false
```

Turn on/off the object model on ROS (using python code)

```
rospy.set_param('/dope/activities/[object_name]', True or False)
```

### Inference using files

```
python infer.py --weight_path ~/dope/weights/train_ver3/paddle_handle/ --data_path ~/data/HandAI_ver3/synthetic_paddle_handle --infer_save_path ~/dope/infer/paddle_handle
```
