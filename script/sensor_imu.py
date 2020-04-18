#!/usr/bin/python
# coding=gbk
# Copyright 2020 Wechange Tech.
# Developer: FuZhi, Liu (liu.fuzhi@wechangetech.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os
import rospy
import tf
import time
import sys
import math
import serial
import string
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
import ctypes
import struct


#class queue is design for uart receive data cache
class queue:
    def __init__(self, capacity = 1024*4):
        self.capacity = capacity
        self.size = 0
        self.front = 0
        self.rear = 0
        self.array = [0]*capacity
 
    def is_empty(self):
        return 0 == self.size
 
    def is_full(self):
        return self.size == self.capacity
 
    def enqueue(self, element):
        if self.is_full():
            raise Exception('queue is full')
        self.array[self.rear] = element
        self.size += 1
        self.rear = (self.rear + 1) % self.capacity
 
    def dequeue(self):
        if self.is_empty():
            raise Exception('queue is empty')
        self.size -= 1
        self.front = (self.front + 1) % self.capacity
 
    def get_front(self):
        return self.array[self.front]
    
    def get_front_second(self):
        return self.array[((self.front + 1) % self.capacity)]

    def get_queue_length(self):
        return (self.rear - self.front + self.capacity) % self.capacity

    def show_queue(self):
        for i in range(self.capacity):
            print self.array[i],
        print(' ')
#class Sensor:
class Sensor:
    def __init__(self):
        #Get params
        self.G = 9.8 #EARTH_GRAVITY
        self.device_port = rospy.get_param('~port','/dev/ttyACM0')
        self.baudrate = int(rospy.get_param('~baudrate','115200'))
        self.imuId = rospy.get_param('~imu_id','imu')
        self.imu_topic = rospy.get_param('~imu_topic','imu')
        self.imu_freq = float(rospy.get_param('~imu_freq','100'))
        self.magId = rospy.get_param('~mag_id','mag')
        self.mag_topic = rospy.get_param('~mag_topic','mag')
        self.mag_freq = float(rospy.get_param('~mag_freq','70'))
        self.gravity = rospy.get_param('~gravity','true')
        #define param
        self.current_time = rospy.Time.now()
        self.previous_time = self.current_time
        self.serialIDLE_flag = 0
        self.ImuErrFlag = False
        self.Circleloop = queue(capacity = 1024*8)
        self.Gyro = [0,0,0]
        self.Accel = [0,0,0]
        self.Quat = [0,0,0,0]
        self.Mag = [0,0,0]
        self.firmware_version = [0,0,0]
        self.hardware_version = [0,0,0]
        self.last_cmd_vel_time = rospy.Time.now()
        self.last_ackermann_cmd_time = rospy.Time.now()
        # Serial Communication
        try:
            self.serial = serial.Serial(self.device_port,self.baudrate,timeout=10)
            rospy.loginfo("Opening Sensor")
            try:
                if self.serial.in_waiting:
                    self.serial.readall()
            except:
                rospy.loginfo("Opening Sensor Try Faild")
                pass
        except:
            rospy.logerr("Can not open Serial"+self.device_port)
            self.serial.close
            sys.exit(0)
        rospy.loginfo("Sensor Open Succeed")
        #if move base type is ackermann car like robot and use ackermann msg ,sud ackermann topic,else sub cmd_vel topic
        self.imu_pub = rospy.Publisher(self.imu_topic,Imu,queue_size=10)
        self.mag_pub = rospy.Publisher(self.mag_topic,MagneticField,queue_size=10)
        self.timer_communication = rospy.Timer(rospy.Duration(1.0/1000),self.timerCommunicationCB)
        self.timer_imu = rospy.Timer(rospy.Duration(1.0/self.imu_freq),self.timerIMUCB) 
        self.timer_mag = rospy.Timer(rospy.Duration(1.0/self.mag_freq),self.timerMagCB) 

        self.getVersion()
        #imu initialization need about 0.2s,during initialization,move base system is blocked
        #so need this gap
        self.getSN()
        time.sleep(0.1)
    #CRC-8 Calculate
    def crc_1byte(self,data):
        crc_1byte = 0
        for i in range(0,8):
            if((crc_1byte^data)&0x01):
                crc_1byte^=0x18
                crc_1byte>>=1
                crc_1byte|=0x80
            else:
                crc_1byte>>=1
            data>>=1
        return crc_1byte
    def crc_byte(self,data,length):
        ret = 0
        for i in range(length):
            ret = self.crc_1byte(ret^data[i])
        return ret               
    #Communication Timer callback to handle receive data
    #depend on communication protocol
    def timerCommunicationCB(self,event):
        length = self.serial.in_waiting
        if length:
            reading = self.serial.read_all()
            if len(reading)!=0:
                for i in range(0,len(reading)):
                    data = (int(reading[i].encode('hex'),16)) 
                    try:
                        self.Circleloop.enqueue(data)
                    except:
                        rospy.logerr("Circleloop.enqueue Faild")
        else:
            pass
        if self.Circleloop.is_empty()==False:
            if self.Circleloop.is_empty()==False:
                data = self.Circleloop.get_front()
            else:
                pass
            if data == 0x5a:
                length = self.Circleloop.get_front_second()
                if length > 1 :
                    if self.Circleloop.get_front_second() <= self.Circleloop.get_queue_length():
                        databuf = []
                        for i in range(length):
                            databuf.append(self.Circleloop.get_front())
                            self.Circleloop.dequeue()
                        
                        if (databuf[length-1]) == self.crc_byte(databuf,length-1):
                            pass
                        else:
                            pass
                        #parse receive data                     
                        if(databuf[3] == 0xf2):
                            self.hardware_version[0] = databuf[4]
                            self.hardware_version[1] = databuf[5]
                            self.hardware_version[2] = databuf[6]
                            self.firmware_version[0] = databuf[7]
                            self.firmware_version[1] = databuf[8]
                            self.firmware_version[2] = databuf[9]
                            version_string = "Sensor Hardware Ver %d.%d.%d,Firmware Ver %d.%d.%d"\
                                %(self.hardware_version[0],self.hardware_version[1],self.hardware_version[2],\
                                self.firmware_version[0],self.firmware_version[1],self.firmware_version[2])
                            rospy.loginfo(version_string)
                        elif(databuf[3] == 0x18 or databuf[3] == 0x1c):
                            for i in range(3):
                                x = databuf[i*4+4:i*4+8]
                                self.Gyro[i] = struct.unpack('<f', struct.pack('4B', *x))[0]
                            for i in range(3):
                                x = databuf[i*4+16:i*4+20]
                                self.Accel[i] = struct.unpack('<f', struct.pack('4B', *x))[0]
                            for i in range(4):
                                x = databuf[i*4+28:i*4+32]
                                self.Quat[i] = struct.unpack('<f', struct.pack('4B', *x))[0]
                        elif (databuf[3] == 0x1A):
                            for i in range(3):
                                x = databuf[i*4+4:i*4+8]
                                self.Mag[i] = struct.unpack('<f', struct.pack('4B', *x))[0]
                        elif(databuf[3] == 0xf4):
                            sn_string = "Sensor SN:"
                            for i in range(4,16):
                                sn_string = "%s%02x"%(sn_string,databuf[i])
                            rospy.loginfo(sn_string)                            
                        else:
                            self.timer_imu.shutdown()
                            self.timer_communication.shutdown()
                            rospy.logerr("Invalid Index %d"%databuf[3])
                            pass
                else:
                    pass
            else:
                self.Circleloop.dequeue()
        else:
            # rospy.loginfo("Circle is Empty")
            pass
    #get move base hardware & firmware version    
    def getVersion(self):
        #Get version info
        output = chr(0x5a) + chr(0x06) + chr(0x01) + chr(0xf1) + chr(0x00) + chr(0xd7) #0xd7 is CRC-8 value
        while(self.serialIDLE_flag):
            time.sleep(0.01)
        self.serialIDLE_flag = 1
        try:
            while self.serial.out_waiting:
                pass
            self.serial.write(output)
        except:
            rospy.logerr("Get Version Command Send Faild")
        self.serialIDLE_flag = 0 
    #get move base SN
    def getSN(self):
        #Get version info
        output = chr(0x5a) + chr(0x06) + chr(0x01) + chr(0xf3) + chr(0x00) + chr(0x46) #0x46 is CRC-8 value
        while(self.serialIDLE_flag):
            time.sleep(0.01)
        self.serialIDLE_flag = 1
        try:
            while self.serial.out_waiting:
                pass
            self.serial.write(output)
        except:
            rospy.logerr("Get SN Command Send Faild")
        self.serialIDLE_flag = 0     
    #IMU Timer callback function to get raw imu info
    def timerIMUCB(self,event):
        if self.gravity:
            output = chr(0x5a) + chr(0x06) + chr(0x01) + chr(0x17) + chr(0x00) + chr(0xff) #0x33 is CRC-8 value
        else:
            output = chr(0x5a) + chr(0x06) + chr(0x01) + chr(0x1b) + chr(0x00) + chr(0xff) #0x33 is CRC-8 value
        while(self.serialIDLE_flag):
            time.sleep(0.01)
        self.serialIDLE_flag = 3
        try:
            while self.serial.out_waiting:
                pass
            self.serial.write(output)
        except:
            rospy.logerr("Imu Command Send Faild")

        self.serialIDLE_flag = 0
        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.imuId

        msg.angular_velocity.x = self.Gyro[0]
        msg.angular_velocity.y = self.Gyro[1]
        msg.angular_velocity.z = self.Gyro[2]

        msg.linear_acceleration.x = self.Accel[0]*self.G
        msg.linear_acceleration.y = self.Accel[1]*self.G
        msg.linear_acceleration.z = self.Accel[2]*self.G

        msg.orientation.w = self.Quat[0]
        msg.orientation.x = self.Quat[1]
        msg.orientation.y = self.Quat[2]
        msg.orientation.z = self.Quat[3]
        self.imu_pub.publish(msg)  
    #IMU Timer callback function to get raw imu info
    def timerMagCB(self,event):
        output = chr(0x5a) + chr(0x06) + chr(0x01) + chr(0x19) + chr(0x00) + chr(0xff) #0x33 is CRC-8 value
        while(self.serialIDLE_flag):
            time.sleep(0.01)
        self.serialIDLE_flag = 3
        try:
            while self.serial.out_waiting:
                pass
            self.serial.write(output)
        except:
            rospy.logerr("Mag Command Send Faild")

        self.serialIDLE_flag = 0
        msg = MagneticField()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.magId

        msg.magnetic_field.x = self.Mag[0]
        msg.magnetic_field.y = self.Mag[1]
        msg.magnetic_field.z = self.Mag[2]
        self.mag_pub.publish(msg)
#main function
if __name__=="__main__":
    try:
        rospy.init_node('sensor_imu',anonymous=True)
        bc = Sensor()
        rospy.spin()
    except KeyboardInterrupt:
        bc.serial.close
        print("Shutting down")
