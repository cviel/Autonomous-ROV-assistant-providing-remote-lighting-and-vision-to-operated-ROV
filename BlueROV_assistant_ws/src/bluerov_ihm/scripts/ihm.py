#! /usr/bin/env python3


# Copyright 2022, Samuel PROUTEN, Hugo SABATIER, Enzo ESSONO, Martin GOUNABOU, Christophe VIEL
# 
# Redistribution and use in source and binary forms, with or without modification, are permitted # provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import cv2
import numpy as np
import pyautogui
import rospy
from mavros_msgs.msg import State, OverrideRCIn
from sensor_msgs.msg import Joy, BatteryState, FluidPressure
from std_msgs.msg import Float64, String, Bool
from std_srvs.srv import Trigger, TriggerResponse

from bluerov_tracking.msg import Detection


class IHM:

    def __init__(self):
        # Robot params
        self.armed = False
        self.robot_mode = "Disarmed"
        self.boost = False
        self.tracking_buoy = False
        self.tracking_with_heading_mode = False
        self.tracking_with_distance_mode = False
        
        self.nb_channels = rospy.get_param(rospy.get_name() + "/nb_channels", 18)
        self.commands = [1500] * self.nb_channels
        if self.nb_channels > 8:     # if mavros version can control light
            self.commands[8] = 1000  # pwm light
        
                
        self.heading_offset = 90.0
        
        self.button = [0] * 8
        self.axes = [0] * 8

        self.pressure = 0
        self.battery_state = 0.
        self.min_battery_state = 15
        
        self.record = False
        
        self.heading = 0
        self.heading_target = -1
        self.depth = 0
        self.depth_target = -1
        self.distance = -1
        self.distance_target = -1
        self.detected = False
        self.detected_x = 0
        self.detected_y = 0
        self.first_detection = 0
        
        # IHM params
        self.font = cv2.FONT_HERSHEY_DUPLEX
        self.big_font = cv2.FONT_HERSHEY_TRIPLEX
        self.font_scale = 0.75
        self.font_thickness = 1
        self.eps = 30
        self.big_eps = 40

        self.red = (60, 76, 231)
        self.green = (113, 204, 46)
        self.purple = (182, 89, 155)
        self.grey = (160, 160, 160)
        self.blue = (219, 152, 52)
        self.white = (255, 255, 255)
        self.yellow = (15, 196, 241)
        self.orange = (34, 126, 230)

        # Create IHM window
        cv2.namedWindow("Telemetry")
        self.screen_size = pyautogui.size()
        self.ihm_size = (900, 500, 3)
        cv2.moveWindow("Telemetry", self.screen_size[0] - self.ihm_size[1], 0)
        self.ihm_img = np.zeros(self.ihm_size, np.uint8)
        self.x, self.y = 20, 30

        # Joysticks params
        self.R = int(self.ihm_size[1] / 8)
        self.r = int(self.R / 2)

        rospy.init_node("ihm_node")

        self.listener()

        self.rate = rospy.Rate(10)

    def callback_armed(self, msg):
        self.armed = msg.armed

    def callback_mode(self, msg):
        self.robot_mode = msg.data

    def callback_compass(self, msg):
        self.heading = msg.data

    def callback_target_heading(self, msg):
        self.heading_target = msg.data

    def callback_joy(self, msg):
        self.button = msg.buttons
        self.axes = msg.axes

    def callback_press(self, msg):
        self.pressure = msg.fluid_pressure / 100000

    def callback_depth(self, msg):
        self.depth = -msg.data

    def callback_target_depth(self, msg):
        self.depth_target = msg.data

    def callback_detection(self, msg):
        self.detected = msg.detected
        self.detected_x = msg.x
        self.detected_y = msg.y

    def callback_battery_state(self, msg):
        self.battery_state = round(msg.voltage, 2)

    def callback_target_distance(self, msg):
        self.distance_target = round(msg.data, 2)

    def callback_distance(self, msg):
        self.distance = round(msg.data, 2)

    def callback_tracking_buoy(self, msg):
        self.tracking_buoy = msg.data

    def callback_tracking_with_heading_mode(self, msg):
        self.tracking_with_heading_mode = msg.data

    def callback_tracking_with_distance_mode(self, msg):
        self.tracking_with_distance_mode = msg.data

    def callback_heading_offset(self, msg):
        self.heading_offset = msg.data
        
    def callback_override(self, msg):
        self.commands = msg.channels

    def callback_record2(self, msg):
        self.record = msg.data


    def listener(self):
        rospy.Subscriber("/mavros/state", State, self.callback_armed)

        rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, self.callback_compass)

        rospy.Subscriber("/joy", Joy, self.callback_joy)

        rospy.Subscriber("/mavros/imu/static_pressure", FluidPressure, self.callback_press)

        rospy.Subscriber("/mavros/global_position/rel_alt", Float64, self.callback_depth)

        rospy.Subscriber("cam_detection", Detection, self.callback_detection)

        rospy.Subscriber("robot_mode", String, self.callback_mode)

        rospy.Subscriber("target_depth", Float64, self.callback_target_depth)

        rospy.Subscriber("target_heading", Float64, self.callback_target_heading)

        rospy.Subscriber("/mavros/battery", BatteryState, self.callback_battery_state)

        rospy.Subscriber("target_distance", Float64, self.callback_target_distance)

        rospy.Subscriber("distance", Float64, self.callback_distance)
        
        rospy.Subscriber("tracking_buoy", Bool, self.callback_tracking_buoy)
        
        rospy.Subscriber("tracking_with_heading_mode", Bool, self.callback_tracking_with_heading_mode)
        
        rospy.Subscriber("tracking_with_distance_mode", Bool, self.callback_tracking_with_distance_mode)       

        rospy.Subscriber("heading_offset", Float64, self.callback_heading_offset)  
        
        rospy.Subscriber('/mavros/rc/override', OverrideRCIn, self.callback_override) 
        
        rospy.Subscriber('record_cam2', Bool, self.callback_record2) 
        

    def display(self, text, color, font=None, dx=0):
        if font is None:
            font = self.font
        cv2.putText(self.ihm_img, text, (self.x + dx, self.y), font, self.font_scale, color, self.font_thickness)

    def display_if(self, text1, color1, condition, text2, color2, font1=None, font2=None, dx=0):
        self.display(text1, color1, font1, dx=dx) if condition else self.display(text2, color2, font2, dx=dx)

    def space(self, eps):
        self.y += eps

    def small_space(self):
        self.space(self.eps)

    def big_space(self):
        self.space(self.big_eps)

    def display_joysticks(self):
        cv2.circle(self.ihm_img, (int(2 * self.R), self.y), self.R, self.grey, 5)
        cv2.circle(self.ihm_img, (int(6 * self.R), self.y), self.R, self.grey, 5)
        cv2.circle(self.ihm_img, (int(2 * self.R - self.axes[0] * self.R), int(self.y - self.axes[1] * self.R)), self.r,
                   self.blue, -1)
        cv2.circle(self.ihm_img, (int(6 * self.R - self.axes[3] * self.R), int(self.y - self.axes[4] * self.R)), self.r,
                   self.blue, -1)

    def start(self):
        while not rospy.is_shutdown():
            # Reinitialisation
            self.ihm_img = np.zeros(self.ihm_size, np.uint8)
            self.x, self.y = 20, 30
            if (self.detected == 1)&(self.first_detection == 0):
                self.first_detection = 1
    
            # ---- ROBOT STATE ----
            self.display("---- ROBOT STATE ----", self.white, self.big_font)
            self.small_space()

            # Armed
            self.display_if("Armed", self.red, self.armed, "Disarmed", self.grey)
            self.small_space()

            # Mode : (stabilisation heading et depth, tracking)
            self.display("Mode : " + self.robot_mode, self.blue)
            self.small_space()

            # Boost
            self.display_if("BOST ON", self.purple, self.axes[5] < 0, "Boost", self.grey)
            self.small_space()

            # Battery State
            self.display_if("Battery State : " + str(self.battery_state) + "V", self.green,
                            self.battery_state > self.min_battery_state, "Battery State",
                            self.red)
            self.small_space()
                           
            # Light level
            if self.nb_channels > 8: 
                val_light = (self.commands[8]-1000)/10 + 10
            else:
                val_light = 0
                                
            self.display_if("Light level : " + str(val_light) + "%", self.green,
                            100 > val_light, "Light level : " + str(val_light) + "%",
                            self.red)
            self.small_space()                                            
            
            # Recording
            self.display_if("Recording: ON", self.red, self.record, "Recording: OFF", self.grey)
            
            self.big_space()

            # ---- TELEMETRY ----
            self.display("---- TELEMETRY ----", self.white, self.big_font)
            self.small_space()

            # Pressure
            self.display("Pressure : " + str(round(self.pressure, 4)), self.blue)
            self.small_space()

            # Depth
            self.display("Depth : " + str(self.depth) + "m", self.blue)
            # self.display_if(str(self.depth_target), self.green, self.robot_mode not in ["Manual", "Disarmed"], "-", self.green, dx=300)
            self.small_space()

            # Heading
            self.display("Heading : " + str(self.heading) + "deg", self.blue)
            self.display_if("Target: "+str(int(self.heading_target))+ "deg", self.green, self.robot_mode not in ["Manual", "Disarmed"], "-", self.green, dx=300)
            self.small_space()

            # Distance
            self.display("Distance from target : " + str(self.distance), self.blue)
            # self.display_if(str(self.distance_target), self.green, self.robot_mode in ["Fully Autonomous", "Tracking + Distance"], "-", self.green, dx=300)
            self.big_space()

            # ---- VISION ----
            self.display("---- VISION ----", self.white, self.big_font)
            self.small_space()

            # Detected
            self.display_if("Detected", self.green, self.detected, "Detected", self.grey)
            self.small_space()

            # Target position
            self.display_if("Target position : (" + str(self.detected_x) + "," + str(self.detected_y) + ")", self.blue, self.detected, "Target position : -", self.green)
            self.big_space()

            is_assistant = rospy.get_param("/main/is_assistant", default=True)

            if True: # is_assistant: # self.is_assistant:
                # --- Tracking buoy ----
                self.display("---- ROV assistant ----", self.white, self.big_font)
                self.small_space()
                # Detected
                self.display_if("Tracking of the buoy", self.green, self.tracking_buoy, "No tracking of the buoy", self.grey)
                self.small_space()
                if self.tracking_buoy: # only possible is tracking activated
                    self.display_if("Heading offset: " + str(self.heading_offset) + "deg", self.green, self.tracking_with_heading_mode, "No heading tracked (" + str(self.heading_offset) + "deg)", self.grey)
                    self.small_space()
                    self.display_if("Desired distance :"+ str(self.distance_target), self.green, self.tracking_with_distance_mode, "No distance tracked ("+ str(self.distance_target) + ")", self.grey)
                    self.small_space()
                else:
                    if (self.first_detection == 1)&(self.detected == 0):
                        self.display_if("Target lost", self.red, self.detected, "Target lost", self.red)
                        self.small_space()
                    else:
                        self.small_space()
                        self.small_space()


            # ---- MANUEL COMMAND ----
            self.display("---- COMMAND ----", self.white, self.big_font)
            self.small_space()

            self.big_space()
            self.small_space()
            
            
            self.display_joysticks()

            cv2.imshow("Telemetry", self.ihm_img)
            cv2.waitKey(1)
            self.rate.sleep()


if __name__ == '__main__':

    ihm = IHM()
    ihm.start()
    
