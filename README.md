# Kimera-On-Diadem

Build the docker container \
$ sudo docker build -t kimera_vio_ros [-f <path-to-dockerfile>] . \

Run the docker conatiner \
$ sudo ./scripts/docker/kimera_vio_docker.bash \

To start a stopped container, \
$ sudo docker start -i  <container-name> \

To open a terminal in the container \
$ sudo docker exec -it <conatiner-name> /bin/bash \

## Kimera VIO with ROS in docker container \
In the docker container \
$ echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc \
$ echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc \
$ source ~/.bashrc \

To list all the ROS packages that are installed \
$ rospack list-names \

You will need 4 terminals with cd /catkin_ws and source ~/.bashrc in all of them \
Terminal 1: $ roscore \

Terminal 2: $ rosbag play /home/user/datasets/EuRoC/V1_01_easy.bag \
	    Hit spacebar to pause until roslaunch	Or use this \
	    $ rosbag play --clock --pause /home/user/datasets/EuRoC/V1_01_easy.bag --rate 3 \

Terminal 3: $ rviz -d $(rospack find kimera_vio_ros)/rviz/kimera_vio_euroc.rviz \

Terminal 4: $ roslaunch kimera_vio_ros kimera_vio_ros_euroc.launch online:=true \
            If not playing rosbag and just using one from storage, use \
            $ roslaunch kimera_vio_ros kimera_vio_ros_euroc.launch online:=false rosbag_path:="/home/user/datasets/EuRoC/V1_01_easy.bag" \

You might need to change auto-initialise from 0 to 1 in /catkin_ws/src/Kimera-VIO/params/Euroc/BackendParams.yaml \


### Install semantics \
In the container, run the follwoing commands \
catkin init \
catkin config --extend /opt/ros/melodic # Change `melodic` to your ROS distro \
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release \

\# Clone repo \
cd ~/catkin_ws/src \
git clone git@github.com:MIT-SPARK/Kimera-Semantics.git \
wstool merge Kimera-Semantics/install/kimera_semantics_ssh.rosinstall \
\# Download and update all dependencies \
wstool update \
\# Compile code \
catkin build kimera_semantics_ros \
\# Refresh workspace \
source ~/catkin_ws/devel/setup.bash
