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

                #experiment begin
                match.catch_knife()
                match.move_to_begin(i,force)
                match.scrape(distance,2/speed)
                #match.get_state()
                match.drop_knife()

                # 保存数据
                now = time.strftime('%m%d-%H%M%S', time.localtime())
                print("实验结束，正在保存数据...请注意放置火柴！！！！")
                state = input("请输入实验状态: 1.无现象，成功刮取 2.  3.  4. 5. 6.其他现象 7.实验失败不记录数据 0.exit \n")
                if state == '7':
                        print("实验失败，数据不保存")
                        continue
                elif state == '6':
                        reason = input("请输入现象描述：")
                        path = '/home/sunny/data/force_data/d'+str(distance)+'-v'+str(speed)+'-f'+str(force)
                        if not os.path.exists(path):
                                os.makedirs(path)
                        tablename = path+'/'+str(state)+'-'+reason+'-force_table_' + now + '.csv'
                        table = pd.concat([match.time_table,match.force_table,match.speed_table],axis=1)
                        table.to_csv(tablename)
                        print("力传感器数据已保存至",tablename)
                        print("第"+str(i+1)+"次实验结束")
                elif state == '0':
                        print("实验结束")
                        break
                else:
                        path = '/home/sunny/data/force_data/d'+str(distance)+'-v'+str(speed)+'-f'+str(force)
                        if not os.path.exists(path):
                                os.makedirs(path)
                        tablename = path+'/'+str(state)+'-force_table_' + now + '.csv'
                        table = pd.concat([match.time_table,match.force_table,match.speed_table],axis=1)
                        table.to_csv(tablename)
                        print("力传感器数据已保存至",tablename)
                        print("第"+str(i+1)+"次实验结束")
                
                
                match.clean()
        print("实验结束，感谢使用，请注意善后工作")
                # rospy.spin()

        match.Con_SOCKET.close()
        match.Con_SOCKETINFO.close()