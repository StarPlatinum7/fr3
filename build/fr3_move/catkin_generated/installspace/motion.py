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

class match:


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
        # 自检力传感器
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
            self.force_table = pd.concat([self.force_table,force_data_table],axis=0,ignore_index=True)

            speed_pddata = pd.Series(self.speed,index=['Speed x','Speed y','Speed z','OriSpeed x','OriSpeed y','OriSpeed z','线性速度','姿态速度'])
            speed_data_table = speed_pddata.to_frame().T
            self.speed_table = pd.concat([self.speed_table,speed_data_table],axis=0,ignore_index=True)

            time_pddata = pd.Series(timenow,index=['time'])
            time_data_table = time_pddata.to_frame().T
            self.time_table = pd.concat([self.time_table,time_data_table],axis=0,ignore_index=True)

    def xyzmove(self,direction,distance,speed = 0.1):
        if direction == 'x' and distance > 0:
            n_pos = [0.2,0.0,0.0,0.0,0.0,0.0]   
        elif direction == 'y' and distance > 0:
            n_pos = [0.0,0.2,0.0,0.0,0.0,0.0]
        elif direction == 'z' and distance > 0:
            n_pos = [0.0,0.0,0.2,0.0,0.0,0.0]
        elif direction == 'x' and distance < 0:
            n_pos = [-0.2,0.0,0.0,0.0,0.0,0.0]
        elif direction == 'y' and distance < 0:
            n_pos = [0.0,-0.2,0.0,0.0,0.0,0.0]
        elif direction == 'z' and distance < 0:
            n_pos = [0.0,0.0,-0.2,0.0,0.0,0.0]
        elif direction == 'xh' and distance > 0:
            n_pos = [2.0,0.0,0.0,0.0,0.0,0.0]
        elif direction == 'zf' and distance < 0:
            n_pos = [0.0,0.0,-0.05,0.0,0.0,0.0]  
        
        gain = [1.0,1.0,1.0,0.0,0.0,0.0]   # 位姿增量比例系数，仅在增量运动下生效，范围[0~1]；
        t = 0.001 # 指令周期，单位[s]，[0.001~0.016]；      
        success = False    
        for i in range(abs(distance)):    
            #2.10. 笛卡尔空间伺服模式运动
            self.robot.ServoCart(1, n_pos, gain, 0.0, 0.0, t, 0.0, 0.0)
            # ServoCartdata = 'ServoCart(1,'+str(n_pos[0])+','+str(n_pos[1])+','+str(n_pos[2])+','+str(n_pos[3])+','+str(n_pos[4])+','+str(n_pos[5])+','+str(gain[0])+','+str(gain[1])+','+str(gain[2])+','+str(gain[3])+','+str(gain[4])+','+str(gain[5])+','+str(0.0)+','+str(0.0)+','+str(t)+','+str(0.0)+','+str(0.0)+')'
            # ServoCart = '/f/bIII52III341III'+str(len(ServoCartdata))+'III'+ServoCartdata+'III/b/f'

            # r = self.Con_SOCKET.send(ServoCart.encode('UTF-8'))
            # RCV = self.Con_SOCKET.recv(1024)
            # waitms = 'WaitMs('+str(int(100))+')'
            # wait = '/f/bIII52III304III'+str(len(waitms))+'III'+waitms+'III/b/f'
            # self.Con_SOCKET.send(wait.encode('UTF-8'))
            # RCV = self.Con_SOCKET.recv(1024)
            # print(RCV)
            #6.9. 获取TCP反馈速度
            self.speed = self.robot.GetActualTCPSpeed()
            #6.7. 获取TCP反馈合速度
            self.CompositeSpeed = self.robot.GetActualTCPCompositeSpeed()
            self.speed = self.speed[1:7] + self.CompositeSpeed[1:3]

            self.robot.WaitMs(speed)
        return success

    def dotmove(self,joint_angle,position,speed = 100.0):  
        #利用sdk的方式控制机械臂运动
        eP1=[0.000,0.000,0.000,0.000]
        dP1=[0.000,0.000,0.000,0.000,0.000,0.000]
        #2.3. 笛卡尔空间直线运动
        ret = self.robot.MoveL(joint_angle,position,0,0,speed,180.0,100.0,-1.0,eP1,0,0,dP1)

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
        rospy.sleep(1)
        # catch_pos on the top 10cm
        J1=[56.56022780012376, -85.33809850711634, 54.43127320544554, -59.7358205058787, -91.87333210860149, -75.5430799427599]
        P1=[-100.4511566162109, -331.2756042480468, 443.5514831542968, 178.3462677001953, -1.08969461917877, -137.8912506103515]
        self.dotmove(J1,P1,100.0)
        J2 = [55.88647219214109, -92.18812848081683, 87.91303275835396, -87.2127359220297, -91.41081857209159, -76.92496422493812]
        P2 = [-104.4608993530273, -331.6921081542968, 340.4016723632812, 178.962661743164, -1.768772959709167, -137.1906585693359]
        # 下降10cm
        # self.xyzmove(direction='z',distance=-500,speed=5)
        self.dotmove(J2,P2,50.0)
        self.xyzmove(direction='z',distance=-60,speed=20)
        self.xyzmove(direction='y',distance=5,speed=15)

        # 等待动作是否完成
        rev = int.from_bytes(self.Con_SOCKETINFO.recv(1024)[234:237], byteorder="little")
        while (1):
            rev = int.from_bytes(self.Con_SOCKETINFO.recv(1024)[234:237], byteorder="little") 
            if (rev == 1):
                break
            else:
                continue
        
        # 闭合夹爪
        self.robot.WaitMs(2000)
        rospy.sleep(2)
        self.catcher(False)

        rospy.sleep(3)
        # 上升10cm
        # self.xyzmove(direction='z',distance=500,speed=15)
        self.dotmove(J1,P1,50.0)

        rev = int.from_bytes(self.Con_SOCKETINFO.recv(1024)[234:237], byteorder="little")
        while (1):
            rev = int.from_bytes(self.Con_SOCKETINFO.recv(1024)[234:237], byteorder="little") 
            if (rev == 1):
                break
            else:
                continue
    # 清除数据
    def clean(self):
        self.force_table = self.force_table.drop(self.force_table.index,inplace=True)
        self.speed_table = self.speed_table.drop(self.speed_table.index,inplace=True)
        self.time_table = self.time_table.drop(self.time_table.index,inplace=True)