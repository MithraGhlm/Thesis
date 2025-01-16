# command to build Dockerfile
# docker build -t <image name>

FROM osrf/ros:humble-desktop-full-jammy

RUN apt-get update
RUN mkdir -p /ros2_ws

# installing git and Gazebo
RUN apt-get install -y git lsb-release wget gnupg curl
RUN sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg 
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update
RUN apt-get install -y vim
RUN apt-get install -y ros-humble-ros-gz
RUN apt-get install -y ros-humble-xacro
RUN apt-get install -y ros-humble-joint-state-publisher-gui
RUN apt-get install -y ros-humble-controller-manager
RUN apt-get install -y ros-humble-navigation2
RUN apt-get install -y ros-humble-nav2-bringup
RUN apt-get install -y ros-humble-twist-mux
RUN apt-get install -y ros-humble-turtlebot3*
RUN apt-get install -y ros-humble-controller-manager
RUN apt-get install -y ros-humble-ros2-control
RUN apt-get install -y ros-humble-ros2-controllers
RUN apt-get install -y ros-humble-gazebo-ros2-control
RUN apt-get install -y ros-humble-slam-toolbox
RUN apt-get install -y ros-humble-rmw-cyclonedds-cpp
RUN apt-get install -y ros-humble-urdf-tutorial
RUN apt-get install -y bluez
RUN apt-get install -y joystick jstest-gtk evtest
RUN apt-get install -y ros-humble-urg-c
RUN apt-get install -y ros-humble-laser-proc
# PCL is build from source. But it can be also added here to be installed like other packages.

RUN echo "Done!"

