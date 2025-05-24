# FROM ros:humble-ros-base-jammy
# Użycie obrazu kompatybilnego z ARM64
FROM arm64v8/ros:humble-ros-base-jammy


# Source the ROS 2 setup script
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Install requirements
RUN apt update && apt install -y \ 
    git \ 
    sudo \ 
    python3-pip \ 
    python3-dev \ 
    python3-matplotlib \ 
    python3-lxml \ 
    python3-pil \ 
    python3-setuptools \ 
    python3-future \
    ros-humble-tf-transformations \
    ros-humble-camera-ros

RUN apt-get install -y ros-humble-cv-bridge

RUN pip install \
    empy==3.3.4 \
    numpy==1.26.4 \
    lark==1.2.2 \
    MAVProxy==1.8.71 \
    dronekit \
    haversine \
    opencv-python==4.7.0\
    transforms3d \
    colorama \
    tabulate \
    requests


# Fix dronekit
RUN sed -i 's/collections.MutableMapping/collections.abc.MutableMapping/g' /usr/local/lib/python3.10/dist-packages/dronekit/__init__.py

# Ardupilot
RUN apt-get install -y git
WORKDIR /tools
RUN apt-get install -y sudo


# RUN useradd -m -s /bin/bash ardupilot && \
#     chown -R ardupilot:ardupilot /ardupilot_ws
RUN useradd -m -s /bin/bash ardupilot && \
    echo "ardupilot ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers && \
    chown -R ardupilot:ardupilot /tools

USER ardupilot
ENV USER=ardupilot

WORKDIR /tools/ardupilot

RUN git clone --recurse-submodules --depth=1 https://github.com/ArduPilot/ardupilot.git

WORKDIR /tools/ardupilot
RUN /tools/ardupilot/ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh -y
RUN . ~/.profile && \
    git config --global --add safe.directory /tools/ardupilot

RUN echo "source /root/ardupilot/.profile" >> /home/ardupilot/.bashrc

USER root
WORKDIR /root

CMD ["/bin/bash"]
