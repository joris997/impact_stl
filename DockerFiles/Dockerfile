FROM nvidia/opengl:1.2-glvnd-runtime-ubuntu22.04

ENV user=px4space
ENV home=/home/$user
ENV ROS_DISTRO=humble

WORKDIR $home

RUN apt-get update -y && \
    DEBIAN_FRONTEND=noninteractive apt-get upgrade -y && \
    DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC apt-get install -y \
    bash \
    sudo \
    wget \
    curl \
    tzdata \
    python3-pip \
    lsb-release \
    gnupg \
    git \
    mesa-utils \
    libgl1-mesa-dri \
    libegl1-mesa \
    libgles2-mesa \
    libglx-mesa0 \
    libglx0 \
    nvidia-utils-460 \
    libnvidia-gl-470 && \
    ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    dpkg-reconfigure --frontend noninteractive tzdata

## Create non root user and give it sudo permissions
RUN useradd -m -s /bin/bash px4space  \
    && usermod -aG sudo px4space \
    && echo 'px4space ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers \
    && chown -R px4space:px4space $home

## Switch to non root user
## Install ROS2
RUN  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && sudo  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && sudo apt-get update -y \
    && sudo apt-get upgrade -y \
    && sudo apt-get install -y ros-humble-desktop python3-rosdep


RUN echo "source /opt/ros/humble/setup.bash" >> $home/.bashrc
RUN ["/bin/bash", "-c", "source $home/.bashrc"]

## Install Gazebo
# RUN pip3 install vcstool \
#     && pip3 install -U colcon-common-extensions \
#     && pip3 show vcstool | grep Location \
#     && pip3 show colcon-common-extensions | grep Location \
#     && sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2-latest.list' \
#     && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
#     && sudo apt-get update -y \
#     && sudo apt-get install -y python3-vcstool python3-colcon-common-extensions \
#     && mkdir -p $home/Gazebo/src \
#     && cd $home/Gazebo/src \
#     && curl -O https://raw.githubusercontent.com/gazebo-tooling/gazebodistro/master/collection-garden.yaml \
#     && sed -i "s|gz-sim7|pr-spacecraft-thrusters|" collection-garden.yaml \
#     && sed -i 's|https://github.com/gazebosim/gz-sim|https://github.com/DISCOWER/gz-sim.git|' collection-garden.yaml \
#     && vcs import < collection-garden.yaml \
#     && curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
#     && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
#     && sudo apt-get update \
#     && sudo apt-get install -y \
#     $(sort -u $(find . -iname 'packages-'$(lsb_release -cs)'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/gz\|sdf/d' | tr '\n' ' ') && \
#     cd $home/Gazebo && \
#     colcon build --merge-install --parallel-workers 1 && \
#     echo "source $home/Gazebo/install/setup.bash" >> $home/.bashrc
RUN apt-get update \
    && apt-get install -y lsb-release curl gnupg \
    && curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get install -y gz-garden


## Install micro-ROS
RUN  rosdep fix-permissions &&  rosdep init &&  rosdep update && \
    apt-get install -y ros-${ROS_DISTRO}-ament-cmake && \
    apt-get install -y python3-colcon-common-extensions && \
    mkdir -p $home/microros_ws/ && cd $home/microros_ws/ && \
    git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y && \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build && \
    . install/setup.sh && \
    ros2 run micro_ros_setup create_firmware_ws.sh host && \
    ros2 run micro_ros_setup build_firmware.sh && \
    . install/setup.sh && \
    ros2 run micro_ros_setup create_agent_ws.sh && \
    ros2 run micro_ros_setup build_agent.sh && \
    . install/local_setup.sh && \
    echo "source $home/microros_ws/install/local_setup.bash" >> $home/.bashrc


RUN pip install aqtinstall
ENV PATH="$home/.local/bin:$PATH"
# Install qt version 6.6.3
RUN aqt install-qt --outputdir $home/qt linux desktop 6.6.3 gcc_64 -m qtcharts qtconnectivity qtlocation qtmultimedia qtpositioning qtsensors qtserialport qtspeech qtshadertools qt5compat qtquick3d

ENV PATH=$home/qt/6.6.3/gcc_64/bin:$PATH
ENV LD_LIBRARY_PATH=$home/qt/6.6.3/gcc_64/lib:$LD_LIBRARY_PATH
ENV QT_PLUGIN_PATH=$home/qt/6.6.3/gcc_64/lib/pkgconfig:$PKG_CONFIG_PATH



### PX4-Space-Systems
RUN mkdir -p $home/PX4 && cd $home/PX4 && \
    git clone https://github.com/DISCOWER/PX4-Space-Systems.git --recursive && \
    echo "export PX4_SPACE_SYSTEMS_DIR='$home/PX4/PX4-Space-Systems'" >> $home/.bashrc && \
    cd $home/PX4/PX4-Space-Systems && chmod +x Tools/setup/ubuntu.sh && ./Tools/setup/ubuntu.sh -y && \
    pip3 install -r Tools/setup/requirements.txt && \
    make px4_sitl


# install plotjuggler as real-time plotting tool and rosbag conversion
RUN apt-get update -y && apt-get install -y ros-humble-plotjuggler-ros
# gazebo-ros-bridge
RUN sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - && \
    apt-get update && apt install -y ros-humble-ros-gzgarden
# install texlive for plotting
# RUN apt-get install -y texlive-full
# RUN curl -sSL https://install.python-poetry.org | python3 -
# planning dependencies
RUN sudo apt-get install -y ffmpeg


USER px4space

RUN sudo chown -R px4space:px4space $home
ENV PATH=$home/.local/bin:$PATH
RUN mkdir -p $home/QGroundControl && \
    cd $home/QGroundControl && \
    git clone --recursive --depth=1 https://github.com/DISCOWER/qgroundcontrol.git && \
    sudo bash ./qgroundcontrol/tools/setup/install-dependencies-debian.sh && \
    cd qgroundcontrol && \
    cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug && \
    cmake --build build --config Debug && \
    pwd && \
    cp ./build/QGroundControl $home/startQGC

### PX4 messages compile
RUN mkdir -p $home/PX4/ros2_ws/src && \
    cd $home/PX4/ros2_ws/src && \
    git clone --recursive --depth=1 https://github.com/DISCOWER/px4_msgs.git && \
    cd $home/PX4/ros2_ws && \
    . /opt/ros/humble/setup.sh && colcon build --symlink-install && \
    echo "source $home/PX4/ros2_ws/install/setup.bash" >> $home/.bashrc


WORKDIR $home

### PX4 simulation software compile
RUN mkdir -p $home/PX4 && \
    pip3 install --user symforce

RUN mkdir -p $home/space_ws/replans

### Specific impact_stl dependencies
RUN mkdir -p $home/space_ws/src && \
    cd $home/space_ws/src && \
    git clone https://github.com/joris997/my_msgs.git && \
    git clone https://github.com/joris997/px4-offboard.git
    
# downgrade numpy for compatibility reasons
RUN pip install --upgrade numpy==1.24.0 && pip install casadi && pip install rosbags


### Set runtime directory environment variable
ENV XDG_RUNTIME_DIR=/tmp/runtime-root