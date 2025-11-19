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

# Добавляем репозитории ROS 2 и Gazebo Harmonic
RUN apt-get update && apt-get install -y \
    curl \
    wget \
    gnupg \
    lsb-release \
    software-properties-common && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Устанавливаем ROS 2 Humble Desktop, Gazebo Harmonic и системные пакеты
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
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
    && rm -rf /var/lib/apt/lists/*

# Клонируем и настраиваем noVNC для доступа через браузер
RUN apt-get update && apt-get install -y \
    websockify \
    python3-numpy \
    && git clone https://github.com/novnc/noVNC.git /opt/novnc \
    && git clone https://github.com/novnc/websockify /opt/novnc/utils/websockify \
    && rm -rf /var/lib/apt/lists/*

# Инициализируем rosdep
RUN rosdep init && rosdep update --rosdistro humble

# Создаем скрипт для запуска VNC и noVNC
RUN echo '#!/bin/bash\n\
# Запускаем виртуальный дисплей\n\
Xvfb :99 -screen 0 1280x720x24 &\n\
export DISPLAY=:99\n\
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
# Источним ROS 2\n\
source /opt/ros/humble/setup.bash\n\
# Оставляем контейнер работающим\n\
exec "$@"' > /start_vnc_novnc.sh && \
    chmod +x /start_vnc_novnc.sh

# Настраиваем точку входа для ROS 2
RUN echo '#!/bin/bash\n\
source /opt/ros/humble/setup.bash\n\
exec "$@"' > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

# Открываем порты для VNC и noVNC
EXPOSE 5900 6080

# Устанавливаем переменные окружения
ENV DISPLAY=:99

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/start_vnc_novnc.sh", "bash"]