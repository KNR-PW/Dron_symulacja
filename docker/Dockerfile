FROM cyberbotics/webots:R2025a-ubuntu22.04

#Set environment variables for enabling GPU acceleration
ENV __NV_PRIME_RENDER_OFFLOAD=1
ENV __GLX_VENDOR_LIBRARY_NAME=nvidia
ENV __VK_LAYER_NV_optimus=NVIDIA_only
ENV LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

# Install ROS2 Humble
RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
ENV LANG=en_US.UTF-8

# Install necessary dependencies
RUN apt-get update 
RUN apt-get install -y curl 
RUN apt-get install -y software-properties-common

# Add the ROS 2 apt repository
RUN add-apt-repository universe
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list

# Install ROS 2 Humble desktop
RUN apt-get update
RUN apt-get install -y ros-humble-desktop
RUN apt install -y python3-colcon-common-extensions

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
    ros-humble-webots-ros2-driver \
    ros-humble-tf-transformations

RUN pip install \
    empy==3.3.4 \
    # numpy==1.26.4 \
    numpy==1.23 \
    lark==1.2.2 \
    MAVProxy==1.8.71 \
    dronekit \
    haversine \
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
RUN git clone --recurse-submodules --depth=1 https://github.com/ArduPilot/ardupilot.git && \
    cd ardupilot && \
    ./Tools/environment_install/install-prereqs-ubuntu.sh -y && \
    . ~/.profile \
    git config --global --add safe.directory /tools/ardupilot

USER root
WORKDIR /root

CMD ["/bin/bash"]
