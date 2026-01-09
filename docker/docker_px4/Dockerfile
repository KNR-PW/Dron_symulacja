FROM ubuntu:24.04

# setup ros2:jazzy
RUN apt update && apt install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

RUN apt-get install -y software-properties-common
RUN add-apt-repository universe

RUN apt update && apt install -y curl && \
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb

RUN apt-get update && apt-get install -y ros-dev-tools
RUN apt-get update -y && apt-get upgrade -y --fix-missing

RUN apt-get install -y ros-jazzy-desktop
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

RUN apt-get update && apt-get install -y \ 
    git \ 
    sudo \ 
    python3-pip \ 
    python3-dev \ 
    python3-matplotlib \ 
    python3-lxml \ 
    python3-pil \ 
    python3-setuptools \ 
    python3-future

#px4 config

RUN apt-get update && apt-get install -y --no-install-recommends \
    git zip unzip tar \
    python3-jinja2 python3-numpy python3-yaml \
    ninja-build exiftool \
    && rm -rf /var/lib/apt/lists/*

# === NVIDIA GPU wsparcie ===
# Wymagane: uruchamianie kontenera z:
#   docker run --gpus all ...
# Pakiety dla OpenGL itp.
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1-mesa-dev libglu1-mesa-dev freeglut3-dev \
    && rm -rf /var/lib/apt/lists/*


# pobranie potrzebnych bibliotek 

RUN pip3 install \
    dronekit \
    haversine \
    paho-mqtt \
    --break-system-packages

# naprawianie biblioteki dronekit

RUN sed -i 's/collections.MutableMapping/collections.abc.MutableMapping/g' /usr/local/lib/python3.12/dist-packages/dronekit/__init__.py
RUN sed -i '45 s/imp/importlib/1' /usr/lib/python3/dist-packages/past/builtins/misc.py

# pobieranie ros_gz_bridge

RUN apt-get update && apt-get install -y ros-jazzy-ros-gz-bridge

USER root
WORKDIR /root
CMD ["/bin/bash"]
