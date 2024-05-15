## 每次运行之前：

- 连接`usb`到虚拟机：`sillicon cp2102`
- 依次运行

```Python
roscore

sudo chmod 666 /dev/ttyUSB0
rosrun force_msg force_msg_node

cd ~catkin_ws/src/fr3_move/scripts/
/usr/local/bin/python3.10 ./main.py
```