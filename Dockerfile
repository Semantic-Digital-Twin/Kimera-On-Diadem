FROM ros:noetic-ros-base
ENV DEBIAN_FRONTEND=noninteractive

ENV DIRPATH=/root/
WORKDIR $DIRPATH

RUN apt-get update && apt-get install -y --no-install-recommends apt-utils
RUN apt-get update && \
      apt-get install -y \
      build-essential \
      software-properties-common \
      tzdata \
      cmake \
      gfortran \
      git vim tmux trash-cli wget curl virtualenv \
      libatlas-base-dev \
      libboost-all-dev \
      libeigen3-dev \
      libgflags-dev \
      libgoogle-glog-dev \
      libmetis-dev \
      libtbb-dev \
      pkg-config \
      protobuf-compiler \
      autoconf \
      unzip \
      libjpeg-dev \
      libpng-dev \
      libtiff-dev \
      libvtk7-dev \
      libgtk-3-dev \
      xvfb \
      libparmetis-dev \
      python3 \
      python3-dev \
      python3-pip \
      python3-wstool \
      python3-catkin-tools \
      python3-tk && \
      apt-get clean && \
      rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y libopencv*
RUN apt-get update && apt-get install -y \
      ros-noetic-catkin \
      ros-noetic-cmake-modules \
      ros-noetic-cv-bridge \
      ros-noetic-image-pipeline \
      ros-noetic-geometry \
      ros-noetic-rviz \
      ros-noetic-rqt* \
      ros-noetic-image-geometry \
      ros-noetic-pcl-ros \
      ros-noetic-gtsam

RUN mkdir -p /catkin_ws/src/
RUN git clone https://github.com/MIT-SPARK/Kimera-VIO-ROS.git /catkin_ws/src/Kimera-VIO-ROS
RUN cd /catkin_ws/src && \
    git clone https://github.com/MIT-SPARK/Kimera-Semantics.git

RUN cd /catkin_ws/src/Kimera-Semantics && git checkout develop
# Manually merged the rosinstall files and copied them to the image
COPY install/kimera_vio_sem_ros_https.rosinstall /catkin_ws/src/Kimera-VIO-ROS/install/
COPY launch/ARTPark_test1dot0.launch /catkin_ws/src/Kimera-Semantics/kimera_semantics_ros/launch/

RUN cd /catkin_ws/src/ && wstool init \
    && wstool merge -y Kimera-VIO-ROS/install/kimera_vio_sem_ros_https.rosinstall \
    && wstool update

ENV ROS_WS=/catkin_ws/
WORKDIR $ROS_WS

RUN . /opt/ros/noetic/setup.sh && catkin build

# Compile code
RUN . /opt/ros/noetic/setup.sh && cd /catkin_ws/ \
    && catkin config --extend /opt/ros/noetic \
    && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release \    
    && catkin build kimera_semantics_ros

# Remember to change autoInitialize from 0 to 1 in Kimera-VIO/params/Euroc/BackendParams.yaml and change paths from /camera to /camera/camera in launch files and build again

RUN catkin build

RUN echo '. /opt/ros/noetic/setup.bash' >> ~/.bashrc \
    && echo '. /catkin_ws/devel/setup.bash' >> ~/.bashrc
