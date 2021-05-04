# This is an auto generated Dockerfile for ros:ros1-bridge
# generated from docker_images_ros2/ros1_bridge/create_ros_ros1_bridge_image.Dockerfile.em
FROM ros:foxy-ros-base-focal

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

ENV ROS1_DISTRO noetic
ENV ROS2_DISTRO foxy

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-comm=1.15.9-1* \
    ros-noetic-roscpp-tutorials=0.10.2-1* \
    ros-noetic-rospy-tutorials=0.10.2-1* \
    && rm -rf /var/lib/apt/lists/*

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-ros1-bridge=0.9.6-1* \
    ros-foxy-demo-nodes-cpp=0.9.3-1* \
    ros-foxy-demo-nodes-py=0.9.3-1* \
    && rm -rf /var/lib/apt/lists/*

# setup entrypoint
COPY ./ros_entrypoint.sh /

# install wget for getting ubxtool.py from git repo
RUN  apt-get update \
  && apt-get install -y wget --no-install-recommends \
  && rm -rf /var/lib/apt/lists/*

# install latest python3 interpreter and install the required packages
RUN apt-get update \
  && apt-get install -y python3-pip python3-dev \
  && cd /usr/local/bin \
  && ln -s /usr/bin/python3 python \
  && pip3 --no-cache-dir install --upgrade pip \
  && pip3 --no-cache-dir install gps pynmea2 pyserial
  && rm -rf /var/lib/apt/lists/*

# setup gnss
WORKDIR ~/
RUN mkdir gnss && cd gnss && \
    wget https://github.com/KoKoKotlin/Robotik-TeSe/blob/master/woche2/Sensoren/gnss/ubxtool.py \        # get the ubxsetup script
 && wget https://github.com/KoKoKotlin/Robotik-TeSe/blob/master/woche2/Sensoren/gnss/read_serial.py      # get the python script for reading the gnss receiver
 && python3 ubxtool.py -P 18 -v 2 -f /dev/ttyACM0 -p RESET                                               # reset the gnss receiver to default config