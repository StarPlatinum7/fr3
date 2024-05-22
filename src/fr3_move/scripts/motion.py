#!/usr/bin/env /usr/local/bin/python3.10
# -*- coding: utf-8 -*-

#fr3    pythonsdk2.0
import rospy
import Robot
import sys
from std_msgs.msg import UInt8MultiArray,Float32MultiArray
import time
import numpy as np
import pandas as pd
import socket

class motion:


    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('motion', anonymous=True)

        rospy.Subscriber('/force_msg', Float32MultiArray, self.force_callback)
        
        # 初始化变量
        self.robot = Robot.RPC('192.168.58.2')
        self.force_data = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.speed = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        self.force_data_init = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.timeinit = time.time()
        self.is_recording = False
        self.force_table = pd.DataFrame(columns=['Fx','Fy','Fz','Mx','My','Mz'])
        self.speed_table = pd.DataFrame(columns=['Speed x','Speed y','Speed z','OriSpeed x','OriSpeed y','OriSpeed z','线性速度','姿态速度'])
        self.time_table = pd.DataFrame(columns=['time'])

        ip = '192.168.58.2'
        self.Con_SOCKET = socket.socket()            
        error = self.Con_SOCKET.connect_ex((ip, 8080))
        if error == 0:
            print("机器人socket连接成功")
        else:
            print("机器人socket连接失败,退出程序")
            self.Con_SOCKET.close()
            sys.exit()

        self.Con_SOCKETINFO = socket.socket()
        errorinfo = self.Con_SOCKETINFO.connect_ex(('192.168.58.2', 8083))
        if errorinfo == 0:
            print("机器人socketinfo 8083端口连接成功")
        else:
            print("机器人socketinfo 8083端口连接失败,退出程序")
            self.Con_SOCKETINFO.close()
            sys.exit()

        rospy.Subscriber('/force_msg', Float32MultiArray, self.force_callback)
        #自检力传感器
        print("正在检查力传感器连接...")
        rospy.sleep(0.5)
        if self.force_data == [0.0,0.0,0.0,0.0,0.0,0.0]:
            print("无法获取力传感器数据，程序退出")
            print("请检查力传感器连接,是否开启力传感器节点：rosrun force_msg force_msg_node ")
            #退出程序
            sys.exit()
        else:
            print("力传感器连接成功，程序正常运行")

    #控制夹爪开合,true是开,false是合
    def catcher(self,open = True):
        if open == False:
            self.robot.SetToolDO(0,1,0,0)    
            self.robot.SetToolDO(1,0,0,0)
        else:
            self.robot.SetToolDO(0,0,0,0)
            self.robot.SetToolDO(1,1,0,0)


        
    # 力传感器回调函数，记录力传感器数据
    def force_callback(self,force_msg):
        timenow = [0.0]
        timenow[0] = time.time()-self.timeinit
        self.force_data = force_msg.data
        
        force_pddata = pd.Series(self.force_data,index=['Fx','Fy','Fz','Mx','My','Mz'])
        if self.is_recording == True:
            force_data_table = force_pddata.to_frame().T
            self.force_table = pd.concat([
                self.force_table.dropna(axis=1, how='all'),
                force_data_table.dropna(axis=1, how='all')
            ], axis=0, ignore_index=True)

            speed_pddata = pd.Series(self.speed, index=['Speed x', 'Speed y', 'Speed z', 'OriSpeed x', 'OriSpeed y', 'OriSpeed z', '线性速度', '姿态速度'])
            speed_data_table = speed_pddata.to_frame().T
            self.speed_table = pd.concat([
                self.speed_table.dropna(axis=1, how='all'),
                speed_data_table.dropna(axis=1, how='all')
            ], axis=0, ignore_index=True)

            time_pddata = pd.Series(timenow, index=['time'])
            time_data_table = time_pddata.to_frame().T
            self.time_table = pd.concat([
                self.time_table.dropna(axis=1, how='all'),
                time_data_table.dropna(axis=1, how='all')
            ], axis=0, ignore_index=True)   

    def dotmove(self,desc_pos,speed):  
        # #2.3. 笛卡尔空间直线运动
        # ret = self.robot.MoveL(joint_angle,position,0,0,speed,180.0,100.0,-1.0,eP1,0,0,dP1)
        tool = 0 #工具坐标系编号
        user = 0 #工件坐标系编号
        ret = self.robot.MoveL(desc_pos, tool, user,vel=speed)   #笛卡尔空间直线运动
    
    def xyzmove(self,direction,distance,speed):
        rospy.sleep(1)
        begin_pos = list(self.robot.GetActualTCPPose(1)[1])

        final_pos = begin_pos.copy()
        if direction=='x':
            final_pos[0] += distance
        elif direction=='y':
            final_pos[1] += distance
        elif direction=='z':
            final_pos[2] += distance
        elif direction=='rx':
            final_pos[3] +=distance
        elif direction=='ry':
            final_pos[4] +=distance
        elif direction=='rz':
            final_pos[5] +=distance
        self.dotmove(final_pos,speed)
        rospy.sleep(1)


    def get_state(self):
        #反馈当前状态

        #6.3. 获取当前关节位置(角度)
        JointPos = self.robot.GetActualJointPosDegree(1)
        #6.10. 获取当前工具位姿
        CartesianPose = self.robot.GetActualTCPPose(1)
        print("JointPos:",JointPos)
        print("CartesianPose:",CartesianPose)

    # 抓取刮刀
    def catch_knife(self):
        # 打开夹爪
        self.catcher(True)
        rospy.sleep(4)

        #闭合夹爪
        self.catcher(False)
        rospy.sleep(1)


    def move_to_begin(self,i,force):
        #begin
        J1=[110.7085678100586, -344.7763671875, 292.5274047851562, -178.5812835693359, 0.40011101961135864, 136.7396697998047]
        #top 14
        J2=J1.copy()
        J2[2]+=40

        self.dotmove(J2,20)
        rospy.sleep(1)

        #move to the begin position
        self.xyzmove(direction='z',distance=-40,speed=20)
        
        #通过距离改变刮刀的力
        # if i <= 20:
        #     self.xyzmove(direction='z',distance=-1*25*i,speed=5)
        # else:
        #     self.xyzmove(direction='z',distance=(25*(i-10)),speed=5)
        # self.xyzmove(direction='z',distance=-85,speed=10)

        # rospy.sleep(3)

        #记录开始时力传感器数据
        self.force_data_init = self.force_data
        # 开始记录数据
        self.is_recording = True
        self.timeinit = time.time()
        # 微调距离
        # while abs(self.force_data[2] - self.force_data_init[2]) <= force:
        #     RV = self.Con_SOCKETINFO.recv(1024)
        #     if (int.from_bytes(RV[234:237], byteorder="little") == 1):
        #         self.xyzmove(direction='z',distance=-1,speed=30)
        
        print("force adjust success")
        rospy.sleep(1)
       
        


    def scrape(self,distance,speed):

        #scrape 30
        self.xyzmove(direction='y',distance=distance,speed=int(speed))
        self.is_recording = False
        rospy.sleep(1)
      
        
        #after scrape top 40
        self.xyzmove(direction='z',distance=80,speed=20)
       

        
    def drop_knife(self):
        
        drop_pos=[286.0150756835937, -276.2381286621093, 400.8870849609375, -178.0904541015625, 3.782169580459594, 166.8185272216797]
        #move to the drop dot
        self.dotmove(drop_pos,30)
        rospy.sleep(1)
        self.catcher(True)

    def back_to_begin(self):
        begin_pos=[135.5006866455078, -440.7455139160156, 354.004180908203, -161.8565673828125, 18.43313598632812, 135.1242523193359]
        self.dotmove(begin_pos,30)

    # 清除数据
    def clean(self):
        self.force_table = self.force_table.drop(self.force_table.index,inplace=True)
        self.speed_table = self.speed_table.drop(self.speed_table.index,inplace=True)
        self.time_table = self.time_table.drop(self.time_table.index,inplace=True)