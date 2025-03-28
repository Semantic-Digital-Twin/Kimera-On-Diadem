# Kimera Semantics on custom dataset
You may either use the docker image or clone this github repo and build the docker image locally. \
Please download our datasets from our Google Drive [here](https://drive.google.com/file/d/1Jddcrfw3Ei-o7FJ3xWGHyEFxxTvcLdn2/view?usp=sharing).

### A) Pull From Docker Hub
```bash
[sudo] docker pull aurunima/kimera_vio_sem:v2.1
```
Change the path to point to the folder where your datasets are stored on your local system and run the docker container
```bash
#!/bin/bash

docker run -it \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/home/<user>/<datasets_folder>/:/datasets/" \
    --name kimera_vio_sem aurunima/kimera_vio_sem:v2.1
```
You may save this script and execute it the same way you execute the .bash script below.

### B) Clone repo and build locally
```bash
https://github.com/Semantic-Digital-Twin/Kimera-On-Diadem.git
```
Build the docker container
```bash
sudo docker build [--no-cache] -t kimera_vio_sem_ros [-f <path-to-dockerfile>] .
```
Change the path  ```vi scripts/kimera_vio_sem_docker.bash``` to point to the folder where your datasets are stored on your local system and run the docker container
```bash
sudo ./scripts/kimera_vio_sem_docker.bash
```
Once you are in the container, download the kimera_semantics demo bag
```bash
mkdir /datasets/kimera && cd /datasets/kimera
gdown 1SG8cfJ6JEfY2PGXcxDPAMYzCcGBEh4Qq
```


## Running Kimera VIO Semantics with ROS in docker container
You might need to ensure that auto-initialise is changed from ```0``` to ```1``` in _/catkin_ws/src/Kimera-VIO/params/Euroc/BackendParams.yaml_

Sourcing added to bashrc file in the docker container **(You don't need to do this)**
```bash
echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
```
If you need to source, just use
```bash
source ~/.bashrc
```

To list all the ROS packages that are installed
```bash
rospack list-names
```

### Run Kimera VIO Semantics
Copy the demo rosbag to kimera_semantics package
```bash
cp /datasets/kimera/kimera_semantics_demo.bag /catkin_ws/src/kimera_semantics_ros/rosbag/
```
You will need 3 terminals with ```cd /catkin_ws``` in all of them \
**Terminal 1:**
```bash
roscore
```
**Terminal 2:**
```bash
rviz -d $(rospack find kimera_semantics_ros)/rviz/kimera_semantics_gt.rviz
```
**Terminal 3:**
```bash
roslaunch kimera_semantics_ros kimera_semantics.launch play_bag:=true
```

### Run Kimera VIO on EuRoC dataset
Ensure you have the corresponding .bag file downloaded in your ```/datasets``` folder. \
You may download the datasets from [MIT Spark's EuRoC Google Drive](https://drive.google.com/drive/folders/1_kwqHojvBusHxilcclqXh6haxelhJW0O) using ```gdown```
```bash
gdown <download ID>
```
Example
```bash
mkdir /datasets/EuRoC && cd /datasets/EuRoC
gdown http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_01_easy/V1_01_easy.zip
unzip -q V1_01_easy.zip [-d /path/to/dir]
```

Before you run Kimera on EuRoC, you need to yamelize the dataset you are using
```bash
cd Kimera-VIO/
bash ./scripts/euroc/yamelize.bash -p "/datasets/EuRoC/V1_01_easy"
```

You will need 4 terminals with ```cd /catkin_ws``` in all of them \
**Terminal 1:**
```bash
roscore
```
**Terminal 2:**
```bash
rosbag play /datasets/EuRoC/V1_01_easy.bag
```
Hit spacebar to pause until roslaunch OR use this
```bash
rosbag play --clock --pause /datasets/EuRoC/V1_01_easy.bag --rate 3
```
**Terminal 3:**
```bash
rviz -d $(rospack find kimera_vio_ros)/rviz/kimera_vio_euroc.rviz
```
**Terminal 4:**
```bash
roslaunch kimera_vio_ros kimera_vio_ros_euroc.launch online:=true
```
If not playing rosbag and just using one from storage, use
```bash
roslaunch kimera_vio_ros kimera_vio_ros_euroc.launch online:=false rosbag_path:="/datasets/EuRoC/V1_01_easy.bag"
```

### Run Kimera LCD module
You need to tweak the parameters for your use case

#### RPGO
```bash
vi /catkin_ws/src/Kimera-RPGO/include/KimeraRPGO/SolverParams.h
```
RPGO debug (you could add in your debug statements here)
```bash
vi /catkin_ws/src/Kimera-RPGO/Kimera-RPGO/src/RobustSolver.cpp
```
#### VIO
```bash
vi /catkin_ws/src/Kimera-VIO/src/loopclosure/LoopClosureDetector.cpp
```
Similarly modify 
```
/catkin_ws
└── src
    └── Kimera-VIO
        └── params
            └── RealSenseIR
                ├── LeftCameraParams.yaml
                ├── RightCameraParams.yaml
                ├── ImuParams.yaml
```
for your use case


# Notes
Kimera-VIO gives twisted odom; conflicts with the original odom

### Docker Peek
To start a stopped container,
```bash
sudo docker start -i <container-name>
OR
sudo docker start -i <container-id>
```
To open a terminal in the container
```bash
sudo docker exec -it <container-name> /bin/bash
```

### Install semantics if not in container
In the container, run the following commands
```bash
catkin init
catkin config --extend /opt/ros/noetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
```
Clone repo 
```bash
cd ~/catkin_ws/src
git clone git@github.com:MIT-SPARK/Kimera-Semantics.git
wstool merge Kimera-Semantics/install/kimera_semantics_ssh.rosinstall
```
Download and update all dependencies
```bash
wstool update
```
Compile code
```bash
catkin build kimera_semantics_ros
```
Refresh workspace
```bash
source ~/catkin_ws/devel/setup.bash
```
OR
```bash
source ~/.bashrc
```

# Acknowledgements
We have used [MIT SPARK's](https://github.com/MIT-SPARK) [Kimera](https://github.com/MIT-SPARK/Kimera) for our custom dataset. \
The focus of this repo is [Kimera Semantics](https://github.com/MIT-SPARK/Kimera-Semantics)

