# Docker образ симулятора Sverk
## Характеристики системы
Ros 2 Humble  
Px4 v1.16.0  
Micro-XRCE-DDS-Agent v2.4.3  
QgroundControl  
Ubuntu 22.04  
___
## Основные команды
Сборка контейнера
```bash
docker build -t sverk-docker:v1.1 .
```
Запуск контейнера
```bash
docker run -it --rm -d -p 6080:6080 -p 5900:5900 --security-opt seccomp=unconfined --shm-size=2g --name sverk-docker-container sverk-docker:v1.0
```
Запуск контейнера тестовая вариация
```bash
docker run -it --rm -d -p 6080:6080 -p 5900:5900 --security-opt seccomp=unconfined --shm-size=2g  -m 4g --memory-swap=4g  --cpus=2.0 --gpus all --name sverk-docker-container sverk-docker:v1.0
```
Запуск симулятора
```bash
make px4_sitl gz_x500
```
Запуск симулятора с камерой 
```bash
make px4_sitl gz_x500_mono_cam_down
```
Запуск симулятора с камерой и aruco
```bash
make px4_sitl gz_x500_mono_cam_down_aruco
```
Запуск MicroXRCEAgent
```bash
MicroXRCEAgent udp4 -p 8888
```
Проверка работы датчиков
```bash
ros2 launch px4_ros_com sensor_combined_listener.launch.py
```
Запуск offboard_control
```bash
ros2 run px4_ros_com offboard_control
```
Запуск скрипта для подключения QgroundControl с хоста
```bash
./home/user/edit_rcS.bash
```  
### Ссылка для подключения к сборке - http://localhost:6080/vnc.html  

## Отладка
```bash
sudo apt install ros-humble-ros-gz
```