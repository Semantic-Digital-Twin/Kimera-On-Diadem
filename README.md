# Kimera Semantics on custom dataset
You may either use the docker image or clone this github repo and build the docker image locally

### A) Pull From Docker Hub
```bash
[sudo] docker pull aurunima/kimera_vio_sem:v2.1
```

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

## Running Kimera VIO Semantics with ROS in docker container
You might need to ensure that auto-initialise is changed from ```0``` to ```1``` in _/catkin_ws/src/Kimera-VIO/params/Euroc/BackendParams.yaml_

Sourcing added to bashrc file In the docker container **(You don't need to do this)**
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

# Notes
Kimera-VIO gives twisted odom; conflicts with the original odom

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
