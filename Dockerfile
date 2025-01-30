FROM ros:noetic-ros-base
ENV DEBIAN_FRONTEND=noninteractive

ENV DIRPATH /root/
WORKDIR $DIRPATH

RUN apt-get update && apt-get install -y --no-install-recommends apt-utils
RUN apt-get update && \
      apt-get install -y \
      build-essential \
      cmake \
      gfortran \
      git \
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
      ros-noetic-cmake-modules \
      ros-noetic-cv-bridge \
      ros-noetic-image-pipeline \
      ros-noetic-geometry \
      ros-noetic-rviz \
      ros-noetic-image-geometry \
      ros-noetic-pcl-ros

RUN mkdir -p /catkin_ws/src/
RUN git clone https://github.com/MIT-SPARK/Kimera-VIO-ROS.git /catkin_ws/src/Kimera-VIO-ROS
# Clone Semantics repo
# RUN cd /catkin_ws/src && \
#     git clone https://github.com/MIT-SPARK/Kimera-Semantics.git

# Need to merge the rosinstall file o	f Kimera-Semantics and that of Kimera-VIO-ROS
# RUN cd /catkin_ws/src/Kimera-Semantics && git checkout develop

COPY install/kimera_vio_sem_ros_https.rosinstall /catkin_ws/src/Kimera-VIO-ROS/install/
RUN cd /catkin_ws/src/ && wstool init \
    && wstool merge Kimera-VIO-ROS/install/kimera_vio_sem_ros_https.rosinstall \
    && wstool update

RUN cd /catkin_ws/src/gtsam && git checkout 4.2
RUN . /opt/ros/noetic/setup.sh && cd /catkin_ws/ && catkin init && \
      catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
      catkin build gtsam --cmake-args -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
      -DGTSAM_BUILD_TESTS=OFF \
      -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
      -DCMAKE_BUILD_TYPE=Release \
      -DGTSAM_BUILD_UNSTABLE=ON \
      -DGTSAM_POSE3_EXPMAP=ON \
      -DGTSAM_ROT3_EXPMAP=ON \
      -DGTSAM_TANGENT_PREINTEGRATION=OFF \
      -DGTSAM_USE_SYSTEM_EIGEN=ON \
      -DGTSAM_USE_SYSTEM_METIS=ON

# Compile code
# RUN cd /catkin_ws/src/Kimera-Semantics && git checkout develop
# RUN . /opt/ros/noetic/setup.sh && cd /catkin_ws/ \
#     && catkin config --extend /opt/ros/noetic \
#     && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release \    
#     && catkin build kimera_semantics_ros

RUN cd /catkin_ws/ && catkin build

# Add workspace to bashrc.
# RUN echo '. /opt/ros/noetic/setup.bash' >> ~/.bashrc \
#     && echo '. /catkin_ws/devel/setup.bash' >> ~/.bashrc
