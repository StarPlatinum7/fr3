#!/usr/bin/env /usr/local/bin/python3.10
# -*- coding: utf-8 -*-

#import rospy
#from std_msgs.msg import UInt8MultiArray,Float32MultiArray

import time
import numpy as np
import pandas as pd
from motion import motion
import os

if __name__ == "__main__":
        distance = 20   # 距离(mm)
        speed = 0.15      # 速度(m/s)
        force = 0.1    # 力(N)

        match = motion()
        match.catcher(False)
        # 循环20次1
        x = input("请输入实验次数：")
        for i in range(int(x)):
                print("第"+str(i+1)+"次实验开始")
                # match.catch_knife()
                
                match.move_to_begin()
                match.scrape()
                #match.get_state()
                match.drop_knife()
                # 保存数据
                
                
                match.clean()
        print("实验结束，感谢使用，请注意善后工作")
                # rospy.spin()

        match.Con_SOCKET.close()
        match.Con_SOCKETINFO.close()