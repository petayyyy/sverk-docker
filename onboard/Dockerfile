# Используем базовый образ Ubuntu 22.04
FROM ubuntu:jammy

### ROS HUMBLE
# Добавляем репозитории ROS 2
RUN apt-get update && apt-get install -y \
    curl \
    wget \
    gnupg \
    lsb-release \
    software-properties-common && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Устанавливаем ROS 2 Humble Desktop и системные пакеты
RUN apt-get update && apt-get install -y \
    ros-dev-tools \
    python3-pip \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

### PX4
# Устанавливаем зависимости для PX4
RUN apt-get update && apt-get install -y \
    # Базовые зависимости PX4
    python3 \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    python3-dev \
    ninja-build \
    exiftool \
    # Компиляторы и инструменты сборки
    gcc \
    g++ \
    gperf \
    ccache \
    make \
    cmake \
    # Python зависимости
    python3-yaml \
    python3-cerberus \
    python3-jinja2 \
    python3-argcomplete \
    python3-jsonschema \
    python3-empy \
    python3-toml \
    python3-packaging \
    python3-six \
    python3-numpy \
    # Авионика
    gcc-arm-none-eabi \
    binutils-arm-none-eabi \
    # Дополнительные зависимости
    libimage-exiftool-perl \
    file \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Создаем пользователя для PX4
RUN useradd -m -s /bin/bash user && \
    echo "user:password" | chpasswd && \
    usermod -aG sudo user && \
    echo "user ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

# Устанавливаем Python зависимости для PX4
RUN pip3 install --upgrade pip && \
    pip3 install empy==3.3.4 pyros-genmsg && \
    pip3 install "setuptools<71" && \
    pip3 install kconfiglib

# Переключаемся на пользователя для сборки PX4
USER user
WORKDIR /home/user

# Клонируем и собираем PX4 v1.16.0
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive && \
    cd PX4-Autopilot && \
    git checkout v1.16.0 && \
    export USER=user && \
    export HOME=/home/user && \ 
    bash ./Tools/setup/ubuntu.sh && \
    git submodule sync --recursive && \
    git submodule update --init --recursive && \
    make px4_sitl -j$(nproc)

# Собираем Micro XRCE-DDS Agent
RUN git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc)

# Создаем workspace и собираем px4 пакеты ROS 2
RUN mkdir -p /home/user/sverk_ws/src/ && \
    cd /home/user/sverk_ws/src/ && \
    git clone https://github.com/PX4/px4_msgs.git && \
    git clone https://github.com/PX4/px4_ros_com.git  

# Инициализируем rosdep
WORKDIR /home/user/sverk_ws
USER root
RUN rosdep init && rosdep update --rosdistro humble && rosdep install --from-paths src --ignore-src -y --rosdistro humble
USER user

RUN bash -c "source /opt/ros/humble/setup.sh && colcon build"

USER root
# Добавляем пути в .bashrc пользователя
RUN echo "source /opt/ros/humble/setup.bash" >> /home/user/.bashrc && \
    echo "source /home/user/sverk_ws/install/local_setup.bash" >> /home/user/.bashrc && \
    echo 'export PATH=$PATH:/home/user/Micro-XRCE-DDS-Agent/build' >> /home/user/.bashrc

# Фиксим права доступа ко всем файлам пользователя
RUN chown -R user:user /home/user/

USER user

ENTRYPOINT [""]