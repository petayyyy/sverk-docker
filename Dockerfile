# Используем базовый образ Ubuntu 22.04
FROM ubuntu:jammy

# Устанавливаем локаль и временную зону
RUN apt-get update && apt-get install -y locales tzdata && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    ln -fs /usr/share/zoneinfo/Europe/Moscow /etc/localtime && \
    dpkg-reconfigure --frontend noninteractive tzdata
ENV LANG=en_US.UTF-8
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

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
    ros-humble-desktop \
    ros-dev-tools \
    python3-pip \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Устанавливаем графическую среду Xfce, VNC и инструменты
RUN apt-get update && apt-get install -y \
    xfce4 \
    xfce4-goodies \
    x11vnc \
    xvfb \
    firefox \
    thunar \
    xfce4-terminal \
    net-tools \
    iputils-ping \
    build-essential \
    git \
    nano \
    vim \
    mesa-utils \
    imagemagick \
    desktop-file-utils \
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
RUN rosdep init && rosdep update --rosdistro humble
USER user

RUN bash -c "source /opt/ros/humble/setup.sh && colcon build"

### QGroundControl
# Устанавливаем зависимости для QGroundControl
USER root
WORKDIR /home/user

RUN apt-get update && apt-get install -y \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
    libfuse2 \
    libxcb-xinerama0 \
    libxkbcommon-x11-0 \
    libxcb-cursor-dev \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*
USER user

# Скачиваем и извлекаем QGroundControl
RUN wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage -O /home/user/QGroundControl.AppImage && \
    chmod +x /home/user/QGroundControl.AppImage && \
    ./QGroundControl.AppImage --appimage-extract && \
    mv squashfs-root qgroundcontrol && \
    rm QGroundControl.AppImage && \
    chown -R user:user /home/user/qgroundcontrol

# Скачиваем иконку для QGroundControl
RUN wget https://raw.githubusercontent.com/mavlink/qgroundcontrol/master/resources/icons/qgroundcontrol.ico -O /home/user/qgroundcontrol.ico && \
    convert /home/user/qgroundcontrol.ico -resize 256x256 /home/user/qgroundcontrol/qgroundcontrol.png && \
    chown user:user /home/user/qgroundcontrol.*

# Создаем ярлык для запуска НА РАБОЧЕМ СТОЛЕ
RUN mkdir -p /home/user/Desktop/ && \
    echo "[Desktop Entry]\n\
Version=1.0\n\
Name=QGroundControl\n\
Comment=Ground Control Station for Drones\n\
Exec=/home/user/qgroundcontrol/AppRun\n\
Icon=/home/user/qgroundcontrol/qgroundcontrol.png\n\
Terminal=false\n\
Type=Application\n\
Categories=Utility;Application;\n\
StartupWMClass=QGroundControl" > /home/user/Desktop/qgroundcontrol.desktop && \
    chmod +x /home/user/Desktop/qgroundcontrol.desktop && \
    chown user:user /home/user/Desktop/qgroundcontrol.desktop

# Создаем ярлык для меню приложений
RUN mkdir -p /home/user/.local/share/applications/ && \
    cp /home/user/Desktop/qgroundcontrol.desktop /home/user/.local/share/applications/ && \
    chown user:user /home/user/.local/share/applications/qgroundcontrol.desktop

### DISPLAY
# Возвращаемся к root для настройки сервисов
USER root
RUN apt-get update && apt-get remove -y xfce4-power-manager-plugins xfce4-power-manager

# Клонируем и настраиваем noVNC для доступа через браузер
RUN apt-get update && apt-get install -y \
    websockify \
    && git clone https://github.com/novnc/noVNC.git /opt/novnc \
    && git clone https://github.com/novnc/websockify /opt/novnc/utils/websockify \
    && rm -rf /var/lib/apt/lists/*

# Создаем папку для изображений и копируем обои
RUN mkdir -p /home/user/images/
COPY /images/photo_2025-11-06_16-22-49.jpg /home/user/images/wallpaper.jpg

# Настраиваем обои рабочего стола для Xfce
RUN mkdir -p /home/user/.config/xfce4/xfconf/xfce-perchannel-xml/ && \
    echo '<?xml version="1.0" encoding="UTF-8"?>\n\
    <channel name="xfce4-desktop" version="1.0">\n\
    <property name="backdrop" type="empty">\n\
        <property name="screen0" type="empty">\n\
        <property name="monitor0" type="empty">\n\
            <property name="image-path" type="string" value="/home/user/images/wallpaper.jpg"/>\n\
            <property name="image-style" type="int" value="5"/>\n\
            <property name="last-image" type="string" value="/home/user/images/wallpaper.jpg"/>\n\
            <property name="last-single-image" type="string" value="/home/user/images/wallpaper.jpg"/>\n\
        </property>\n\
        </property>\n\
    </property>\n\
    </channel>' > /home/user/.config/xfce4/xfconf/xfce-perchannel-xml/xfce4-desktop.xml && \
    chown -R user:user /home/user/.config

# Создаем скрипт для запуска VNC и noVNC
RUN echo '#!/bin/bash\n\
# Запускаем виртуальный дисплей\n\
Xvfb :99 -screen 0 1280x720x24 &\n\
export DISPLAY=:99\n\
# Ожидаем запуска X сервера\n\
sleep 2\n\
# Запускаем графическую среду Xfce\n\
startxfce4 &\n\
# Запускаем VNC-сервер\n\
x11vnc -display :99 -forever -shared -nopw -quiet -bg &\n\
# Запускаем noVNC для доступа через браузер\n\
/opt/novnc/utils/novnc_proxy --vnc localhost:5900 --listen 6080 &\n\
# Выводим информацию для подключения\n\
echo "====================================="\n\
echo "noVNC запущен! Для подключения откройте:"\n\
echo "http://localhost:6080/vnc.html"\n\
echo "====================================="\n\
# Источним ROS 2 для пользователя\n\
su - user -c "source /opt/ros/humble/setup.bash && source /home/user/sverk_ws/install/local_setup.bash"\n\
# Оставляем контейнер работающим\n\
tail -f /dev/null' > /start_vnc_novnc.sh && \
    chmod +x /start_vnc_novnc.sh

# Добавляем пути в .bashrc пользователя
RUN echo "source /opt/ros/humble/setup.bash" >> /home/user/.bashrc && \
    echo "source /home/user/sverk_ws/install/local_setup.bash" >> /home/user/.bashrc && \
    echo 'export PATH=$PATH:/home/user/Micro-XRCE-DDS-Agent/build' >> /home/user/.bashrc && \
    echo 'export DISPLAY=:99' >> /home/user/.bashrc

# Фиксим права доступа ко всем файлам пользователя
RUN chown -R user:user /home/user/

# Открываем порты для VNC и noVNC
EXPOSE 5900 6080

# Устанавливаем переменные окружения
ENV DISPLAY=:99

COPY scripts/edit_rcS.bash /home/user/edit_rcS.bash
RUN apt-get update && apt-get install -y dos2unix && \
    dos2unix /home/user/edit_rcS.bash && \
    chmod +x /home/user/edit_rcS.bash && \
    chown user:user /home/user/edit_rcS.bash

RUN mkdir -p /home/user/Desktop/ && \
    echo "[Desktop Entry]\n\
Version=1.0\n\
Name=Remote QGroundControl\n\
Comment=Configure Remote QGroundControl connection\n\
Exec=xfce4-terminal --hold -e '/home/user/edit_rcS.bash'\n\
Icon=/home/user/qgroundcontrol/qgroundcontrol.png\n\
Terminal=false\n\
Type=Application\n\
Categories=Utility;Application;\n\
StartupWMClass=RemoteQGroundControl" > /home/user/Desktop/remote_qgroundcontrol.desktop && \
    chmod +x /home/user/Desktop/remote_qgroundcontrol.desktop && \
    chown user:user /home/user/Desktop/remote_qgroundcontrol.desktop

USER user

# Точка входа - запускаем скрипт VNC
ENTRYPOINT ["/start_vnc_novnc.sh"]