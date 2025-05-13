# Ubuntu 24.04 
FROM ubuntu:noble

ENV LANG C.UTF-8
ENV LC_AL C.UTF-8
ENV ROS2_DISTRO=jazzy
ARG DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV TZ=Europe/Zagreb
ARG HOME=/root

# Tools useful during development
RUN apt-get update \
 && apt install -y \
        build-essential \
        atop \
        clang \
        cmake \
        cppcheck \
        expect \
        gdb \
        git \
        gnutls-bin \
        libbluetooth-dev \
        libccd-dev \
        libcwiid-dev \
        libfcl-dev \
        libgoogle-glog-dev \
        libspnav-dev \
        libspdlog-dev \
        libusb-dev \
        libboost-thread-dev \
        libboost-system-dev \
        libboost-filesystem-dev \
        libboost-regex-dev \
        libboost-program-options-dev \
        libconsole-bridge-dev \
        libpoco-dev \
        libtinyxml2-dev \
        liblz4-dev \
        libbz2-dev \
        uuid-dev \
        liblog4cxx-dev \
        libgpgme-dev \
        libgtest-dev \
        python3-dbg \
        python3-empy \
        python3-setuptools \
        python3-pip \
        python3-venv \
        python3-nose \
        python3-pycryptodome \
        python3-defusedxml \
        python3-mock \
        python3-netifaces \
        python3-gnupg \
        python3-psutil \
        vim \
        nano \
        net-tools \
        tmux \
        tmuxinator \
        iputils-ping \
        xvfb \
        curl\
        jq \
        wget\
        ranger \
        htop \
        libgl1 \
        libglx-mesa0 \
        libgl1-mesa-dri \
        rfkill \
        sudo \
        usbutils \
        software-properties-common \
        console-setup \
 && add-apt-repository universe \
 && apt-get clean -qq

RUN apt-get update \
 && apt install -y \
        python3-flake8 \
        python3-flake8-blind-except \
        python3-flake8-builtins \
        python3-flake8-class-newline \
        python3-flake8-comprehensions \
        python3-flake8-deprecated \
        python3-flake8-docstrings \
        python3-flake8-import-order \
        python3-flake8-quotes \
        python3-pytest \
        python3-pytest-cov \
        python3-pytest-repeat \
        python3-pytest-rerunfailures


# Agent forwarding during docker build https://stackoverflow.com/questions/43418188/ssh-agent-forwarding-during-docker-build
# install ssh client
ENV DOCKER_BUILDKIT=1
RUN apt-get install -y openssh-client
ENV GIT_SSH_COMMAND="ssh -v"
USER root

RUN --mount=type=ssh id=default \
    mkdir -p ~/.ssh/ && \
    ssh-keyscan -H github.com >> ~/.ssh/known_hosts

# install ROS2 Jazzy
RUN apt update &&  apt install locales \
  && locale-gen en_US en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && export LANG=en_US.UTF-8 

# set up ros2 repo
RUN /bin/sh -c 'sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg' \
  && /bin/sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null'


RUN  apt update && apt install -y \
   python3-rosdep \
   ros-dev-tools  \
   libboost-program-options-dev \
   python-is-python3 \
   nlohmann-json3-dev

RUN apt-get update && apt-get upgrade -y && apt install ros-${ROS2_DISTRO}-desktop -y

RUN apt-get install -y python3-colcon-common-extensions

#Add mcap to bag things in ROS2
RUN VERSION="releases/mcap-cli/v0.0.53" && \
    RELEASE_URL=$(curl -s https://api.github.com/repos/foxglove/mcap/releases | jq -r --arg VERSION "$VERSION" '.[] | select(.tag_name == $VERSION) | .assets[0].browser_download_url') && \
    echo "Downloading release $VERSION from: $RELEASE_URL" && \
    curl -L -o /bin/mcap "$RELEASE_URL" && \
    cd /bin && chmod +x mcap


RUN echo "alias ros2_ws='source $HOME/ros2_ws/install/setup.bash'" >> $HOME/.bashrc
RUN echo "alias source_ros2='source /opt/ros/${ROS2_DISTRO}/setup.bash'" >> $HOME/.bashrc

RUN PYTHON_VERSION=$(python --version | cut -d " " -f2 | cut -d "." -f1,2) && \
    rm -rf /usr/lib/python${PYTHON_VERSION}/EXTERNALLY-MANAGED

#crazyflies setup
RUN apt-get update && apt install -y \
  octovis \
  octomap-tools \
  libusb-1.0-0-dev \
  libxcb-xinerama0 \
  libxcb-cursor0 \
  make gcc-arm-none-eabi \
  swig3.0 \
  && ln -s /usr/bin/swig3.0 /usr/bin/swig

RUN pip install --ignore-installed rowan transforms3d nicegui keyboard

RUN apt install -y ros-${ROS2_DISTRO}-tf-transformations \
                   ros-${ROS2_DISTRO}-nav2-map-server \
                   ros-${ROS2_DISTRO}-nav2-lifecycle-manager\
                   ros-${ROS2_DISTRO}-rosbridge-suite\
                   ros-${ROS2_DISTRO}-octomap-ros \
                   ros-${ROS2_DISTRO}-octomap-server \
                   ros-${ROS2_DISTRO}-rosbag2-storage-mcap \
                   ros-${ROS2_DISTRO}-octomap-msgs \
                   libeigen3-dev \
                   libompl-dev \
                   ompl-demos \
                   libboost-program-options-dev \
                   libboost-filesystem-dev \
                   libnlopt-cxx-dev \
                   libgoogle-glog-dev 


RUN mkdir -p $HOME/ros2_ws/src && cd $HOME/ros2_ws/src \
  && git clone https://github.com/IMRCLab/crazyswarm2 --recursive \
  && git clone --recurse-submodules https://github.com/IMRCLab/motion_capture_tracking \
  && cd motion_capture_tracking && git checkout feature_jazzy

RUN cd $HOME/ros2_ws/src \
    && git clone https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco.git \
    && git clone https://github.com/larics/icuas25_msgs.git


RUN echo "alias cd_icuas25_competition='cd /root/ros2_ws/src/icuas25_competition'" >> $HOME/.bashrc
RUN echo "export PATH='$HOME/.local/bin:$PATH'" >> $HOME/.bashrc

# setup ros2 environment variables
RUN echo "export ROS_DOMAIN_ID=$(shuf -i 1-101 -n 1)" >> $HOME/.bashrc


WORKDIR $HOME/ros2_ws

COPY Dockerfile $HOME/CrazySim/ros2_ws/src/icuas25_competition/
COPY icuas25_competition/scripts $HOME/CrazySim/ros2_ws/src/icuas25_competition/scripts
COPY icuas25_competition/src $HOME/CrazySim/ros2_ws/src/icuas25_competition/src
COPY icuas25_competition/startup $HOME/CrazySim/ros2_ws/src/icuas25_competition/startup
COPY icuas25_competition/include $HOME/CrazySim/ros2_ws/src/icuas25_competition/include
COPY icuas25_competition/launch $HOME/CrazySim/ros2_ws/src/icuas25_competition/launch
COPY icuas25_competition/CMakeLists.txt $HOME/CrazySim/ros2_ws/src/icuas25_competition/CMakeLists.txt
COPY icuas25_competition/package.xml $HOME/CrazySim/ros2_ws/src/icuas25_competition/package.xml

# run/add custom scripts
RUN bash -c "chmod +x $HOME/CrazySim/ros2_ws/src/icuas25_competition/scripts/edit.sh && $HOME/CrazySim/ros2_ws/src/icuas25_competition/scripts/edit.sh"
# RUN bash -c "chmod +x $HOME/CrazySim/ros2_ws/src/icuas25_competition/scripts/add_markers.py && $HOME/CrazySim/ros2_ws/src/icuas25_competition/scripts/add_markers .py"
RUN cat $HOME/CrazySim/ros2_ws/src/icuas25_competition/scripts/build.sh >> $HOME/.bashrc
RUN cat $HOME/CrazySim/ros2_ws/src/icuas25_competition/scripts/wait_for_start.sh >> $HOME/.bashrc
RUN echo "export PATH_GEN=/root/CrazySim/ros2_ws/src/icuas25_competition/scripts/py_sim/main.py" >> $HOME/.bashrc

RUN  apt install -y\
    libfontconfig1-dev \
    libfreetype-dev \
    libx11-dev \
    libx11-xcb-dev \
    libxcb-cursor-dev \
    libxcb-glx0-dev \
    libxcb-icccm4-dev \
    libxcb-image0-dev \
    libxcb-keysyms1-dev \
    libxcb-randr0-dev \
    libxcb-render-util0-dev \
    libxcb-shape0-dev \
    libxcb-shm0-dev \
    libxcb-sync-dev \
    libxcb-util-dev \
    libxcb-xfixes0-dev \
    libxcb-xinerama0-dev \
    libxcb-xkb-dev \
    libxcb1-dev \
    libxext-dev \
    libxfixes-dev \
    libxi-dev \
    libxkbcommon-dev \
    libxkbcommon-x11-dev \
    libxrender-dev

                 
RUN cd $HOME && git clone https://github.com/bitcraze/crazyflie-clients-python \
  && cd crazyflie-clients-python \
  && pip install --ignore-installed  -e .

RUN pip install setuptools==78.1.0

RUN cd $HOME && git clone --recursive https://github.com/bitcraze/crazyflie-firmware.git \
  && cd crazyflie-firmware\
  && make cf2_defconfig \
  && make -j$(nproc) \
  && make bindings_python \
  && cd build \
  && python3 setup.py install --user \
  && export PYTHONPATH=$HOME/crazyflie-firmware/build:$PYTHONPATH

RUN cd $HOME && git clone https://github.com/bitcraze/crazyflie-lib-python.git \
  && cd crazyflie-lib-python\
  && pip install -e .

RUN pip install --upgrade pyusb scipy numpy
RUN pip install numpy==1.26
RUN pip install open3d

# final build of ROS2 ws
RUN bash -c "source /opt/ros/${ROS2_DISTRO}/setup.bash;source $HOME/ros2_ws/install/setup.bash;colcon build --symlink-install --merge-install --cmake-args=-DCMAKE_EXPORT_COMPILE_COMMANDS=ON" && \
    echo "ros2_ws" >> $HOME/.bashrc && echo "source_ros2" >> $HOME/.bashrc

ENV USERNAME=root

# Add user and add it to sudo group
RUN sudo usermod -a -G plugdev $USERNAME
COPY to_copy/99-bitcraze.rules /etc/udev/rules.d/.

WORKDIR $HOME
COPY to_copy/aliases $HOME/.bash_aliases
COPY to_copy/nanorc $HOME/.nanorc
COPY to_copy/tmux $HOME/.tmux.conf
COPY to_copy/ranger $HOME/.config/ranger/rc.conf

USER root
WORKDIR $HOME
CMD ["/bin/bash"]
