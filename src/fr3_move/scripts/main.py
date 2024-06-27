#!/usr/bin/env /usr/local/bin/python3.10
# -*- coding: utf-8 -*-

#import rospy
#from std_msgs.msg import UInt8MultiArray,Float32MultiArray

import time
import numpy as np
import pandas as pd
from motion import motion
import os
import re

def read_last_line_efficient(filename):
    with open(filename, 'r', encoding='gbk', errors='replace') as file:
        tail_lines = ['', '']  # 用于存储最后两行的数组
        for line in file:
            tail_lines[0], tail_lines[1] = tail_lines[1], line.strip()
        return tail_lines[0]  # 返回倒数第二行


if __name__ == "__main__":
        distance = 50   # 距离(mm)
        speed = 0.15      # 速度(m/s)
        force = 0.1    # 力(N)

        action = motion()
        #match.catcher(False)
        action.catch_knife()
        # 循环20次1
        x = input("请输入实验次数：")
        for i in range(int(x)):
                print("第"+str(i+1)+"次实验开始")

                #experiment begin
                #action.catch_knife()

                action.move_to_begin(i,force)
                action.scrape(distance,2/speed)

                action.get_state()

                #action.drop_knife()

                # 保存数据
                now = time.strftime('%m%d-%H%M%S', time.localtime())
                with open('/media/sunny/disk/LOG.TXT', 'r', encoding='gbk', errors='replace') as file:
                        tail_lines = ['', '']  # 用于存储最后两行的数组
                        for line in file:
                                tail_lines[0], tail_lines[1] = tail_lines[1], line.strip()
                temp_data=[s for s in re.split(r'[ ,\x00]+', tail_lines[0]) if s]
        
                print("实验结束，正在保存数据...请注意放置！！！！")
                state = input("请输入实验状态: 1.无现象，成功刮取 2.  3.  4. 5. 6.其他现象 7.实验失败不记录数据 0.exit \n")
                if state == '7':
                        print("实验失败，数据不保存")
                        continue
                elif state == '6':
                        reason = input("请输入现象描述：")
                        path = '/home/sunny/data/force_data/d'+str(distance)+'-v'+str(speed)+'-f'+str(force)
                        if not os.path.exists(path):
                                os.makedirs(path)
                        tablename = path+'/'+str(state)+'-'+reason+'-force_table_' +'-T'+temp_data[2]+'-H'+temp_data[3]+'-'+now + '.csv'
                        table = pd.concat([action.time_table,action.force_table,action.speed_table,action.TempHimudity_table],axis=1)
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
                        tablename = path+'/'+str(state)+'-force_table_'+'-T'+temp_data[2]+'-H'+temp_data[3]+'-'+now + '.csv'
                        table = pd.concat([action.time_table,action.force_table,action.speed_table,action.TempHimudity_table],axis=1)
                        table.to_csv(tablename)
                        print("力传感器数据已保存至",tablename)
                        print("第"+str(i+1)+"次实验结束")
                
                
                action.clean()
        print("实验结束，感谢使用，请注意善后工作")
                # rospy.spin()

        action.Con_SOCKET.close()
        action.Con_SOCKETINFO.close()