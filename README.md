## 每次运行之前：
- `internet`:`sudo dhclient ens33`
- `192.168.58.2`

- 依次运行

```Python
roscore

sudo chmod 666 /dev/ttyUSB0
rosrun force_msg force_msg_node

cd ~/catkin_ws/src/fr3_move/scripts/
/usr/local/bin/python3.10 ./main.py
```
- 或者直接运行脚本, 密码: 123

```
~/catkin_ws/test.sh