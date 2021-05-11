# Usage:
# 1. Install docker with apt (sudo apt-get install docker.io) and start the docker deamon (sudo systemctl start docker)
# 2. Add yourself to the docker group so that you don't need sudo (sudo groupadd docker && sudo usermod -aG docker $USER)
# 3. chmod +x ros_entrypoint.sh && chmod +x setup_gnss.sh
# 4. docker build . in the dir of the Dockerfile
# 5. docker images => look up the image id of the newly built image
# 6. docker run --device=/dev/ttyACM0 -it --entrypoint=/bin/bash {image_id} (make sure that the gnss receiver is connected to the pc)
# => now you should have a console in the docker container and can execute the setup_gnss.py and read_serial.py
# if you get a permission denied for the device file run the following command on the host: sudo chmod -R 777 /dev/ttyACM0
# To exit the container CTRL+C + CTRL+D
# For deleting all images and running containers: docker system prune -a
# Example for copying from docker container: docker cp <container-id>:/path/in/container /path/in/host

# This is an auto generated Dockerfile for ros:ros1-bridge
# generated from docker_images_ros2/ros1_bridge/create_ros_ros1_bridge_image.Dockerfile.em
FROM ros:foxy-ros-base-focal

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

ENV ROS_DISTRO noetic
ENV ROS2_DISTRO foxy

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-comm \
    && rm -rf /var/lib/apt/lists/*

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-ros1-bridge \
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
  && pip3 --no-cache-dir install gps pynmea2 pyserial \
  && rm -rf /var/lib/apt/lists/*

# setup gnss
WORKDIR home
RUN mkdir gnss
WORKDIR /home/gnss
RUN wget https://raw.githubusercontent.com/KoKoKotlin/Robotik-TeSe/master/woche2/Sensoren/gnss/ubxtool.py \        
 && wget https://raw.githubusercontent.com/KoKoKotlin/Robotik-TeSe/master/woche2/Sensoren/gnss/read_serial.py

COPY ./setup_gnss.sh .
