Запуск контейнера
```bash
docker run -it --rm -d -p 6080:6080 -p 5900:5900 --shm-size=2g --name sverk-docker-container sverk-docker:v1.0
```
Запуск симулятора
```bash
make px4_sitl gz_x500
```
Запуск MicroXRCEAgent
```bash
MicroXRCEAgent udp4 -p 8888
```
Запуск MicroXRCEAgent
```bash
ros2 launch px4_ros_com sensor_combined_listener.launch.py
```
Запуск MicroXRCEAgent
```bash
ros2 run px4_ros_com offboard_control
```