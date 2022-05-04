#!/usr/bin/env python3


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


import time

import os

import parameters as param
#import parameters_ROV1 as param
from scipy.spatial.transform import Rotation 


import mavros
import numpy as np
import rospy
from bluerov_tracking.msg import Detection
from mavros import command as mavros_command
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, String, Bool
from std_srvs.srv import Trigger


 
def sawtooth(x):
    return (x + np.pi) % (2 * np.pi) - np.pi


def clip(val, min_val, max_val):
    if val <= min_val:
        return min_val
    elif val >= max_val:
        return max_val
    return val



class ROV:

    def __init__(self):
        rospy.on_shutdown(self.shutdown_function)

        self.pwm_light = 1000
        self.record = False
        
        # Heading
        self.heading = 0.0
        self.heading_local_target = 0.0
        self.gain_heading_tracking = 90
        self.angular_velocity_z = 0.0
        self.heading_reference = 0.0
        self.heading_offset = 180.0
        self.heading_global_target = 0.0
        self.heading_last = 0.0

        # Depth
        self.depth = 0.0
        self.depth_target = 1.5
        self.depth_last = 0.0

        # Distance
        self.distance = 0.0
        self.distance_target = 0.0
        self.distance_last = 0.0

        # Robot modes
        self.depth_control_mode = False
        self.heading_control_mode = False
        self.armed = False
        self.boost = False
        self.tracking_mode = False
        self.tracking_with_heading_mode = False
        self.tracking_with_distance_mode = False
        self.robot_mode = "Disarmed"

        # Detection
        self.detected = False
        self.detected_x = 0.0
        self.detected_y = 0.0
        self.detected_w = 0.0
        self.detected_h = 0.0
        
        self.test = 0.0
        self.timer = 0.0 
        self.timer_max = 2 # in second. Time during which the ROV continues to search for its target after having lost it before stopping. 
        

        # IMU
        self.Phi = 0.0
        self.Theta = 0.0
        self.Psy = 0.0
        
        # Buttons and Axes
        self.frame_id = 0
        self.frame_id_last = 0
        self.letter_to_indice = {"A": 0, "B": 1, "X": 2, "Y": 3, "LH": 4,
                                 "RH": 5, "Back": 6, "Start": 7, "?": 8, "L3": 9, "R3": 10}
        self.button_values = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # A, B, X, Y, LH , RH , back, start, ?, L3, R3
        self.axes = [0, 0, 0, 0, 0, 0, 0, 0]

        # ROS
        rospy.init_node('control_node')
        self.rate = rospy.Rate(10)
        self.command_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
        self.robot_mode_pub = rospy.Publisher('robot_mode', String, queue_size=10)
        self.target_depth_pub = rospy.Publisher('target_depth', Float64, queue_size=10)
        self.target_heading_pub = rospy.Publisher('target_heading', Float64, queue_size=10)
        self.distance_pub = rospy.Publisher('distance', Float64, queue_size=10)
        self.target_distance_pub = rospy.Publisher('target_distance', Float64, queue_size=10)
        self.record_srv = rospy.ServiceProxy('record_cam', Trigger)
        self.record_srv2 = rospy.Publisher('record_cam2', Bool, queue_size=10)
        
        self.heading_target_msg = Float64()
        self.depth_target_msg = Float64()
        self.distance_msg = Float64()
        self.target_distance_msg = Float64()

        self.is_assistant = rospy.get_param(rospy.get_name() + "/is_assistant", default=False)

        self.listener()  # Subscribes to the messages

        self.nb_channels = rospy.get_param(rospy.get_name() + "/nb_channels", 18)
        self.commands = [1500] * self.nb_channels
        if self.nb_channels > 8:     # if the MAVROS version can control the channels 8, i.e. the light
            self.commands[8] = 1000  # light pwm : 1000 = off, 2000 = max luminosity
        self.val_pwm = 100

        
        self.time_last = time.time()
        self.depth_integral = 0.0

        # MAVROS
        mavros.set_namespace()
        mavros_command.arming(False)
        print("Robot disarmed.")

        print('Ready to ARM. (start)')

    def button(self, letter):
        return self.button_values[self.letter_to_indice[letter]]

    def shutdown_function(self):
        print("CLEAN SHUTDOWN.")
        # Command to 0
        self.commands = [1500] * self.nb_channels
        if self.nb_channels > 8:
            self.commands[8] = 1000
        self.send_commands()
        print('All commands to 0')
        # Disarmed
        mavros_command.arming(False)
        print('Clean stop, robot disarmed.')

    def send_commands(self):
        for i in range(8):
            self.commands[i] = clip(self.commands[i], 1000, 2000)
        msg = OverrideRCIn()
        msg.channels = self.commands
        self.command_pub.publish(msg)
        
        msg = Bool()
        pub = rospy.Publisher("tracking_buoy", Bool, queue_size=10)  
        msg.data = self.tracking_mode;
        pub.publish(msg)
     
        pub = rospy.Publisher("tracking_with_heading_mode", Bool, queue_size=10)  
        msg.data = self.tracking_with_heading_mode;
        pub.publish(msg)

        pub = rospy.Publisher("tracking_with_distance_mode", Bool, queue_size=10)  
        msg.data = self.tracking_with_distance_mode;
        pub.publish(msg)

        pub = rospy.Publisher("record_cam2", Bool, queue_size=10)  
        msg.data = self.record
        pub.publish(msg)
                
        msg = Float64()
        pub = rospy.Publisher("heading_offset", Float64, queue_size=10)  
        msg.data = self.heading_offset
        pub.publish(msg)

   
        if self.tracking_with_heading_mode:
            target_heading = self.heading_global_target + param.compass_offset
        else:
            target_heading = self.heading_local_target
        pub = rospy.Publisher("target_heading", Float64, queue_size=10)  
        msg.data = target_heading
        pub.publish(msg)   
           
        
    def callback_heading(self, msg):
        self.heading = msg.data

    def callback_heading_reference(self, data):
        self.heading_reference = data.data
        self.heading_global_target = self.heading_reference + self.heading_offset

    def callback_imu(self, msg):
        self.angular_velocity_z = msg.angular_velocity.z
        W = msg.orientation.w
        X = msg.orientation.x
        Y = msg.orientation.y
        Z = msg.orientation.z
        orientq=(W, X, Y, Z)
        ### Conversion quaternions in rotation matrix
        self.Phi, self.Theta, self.Psy = Rotation.from_quat([orientq[1], orientq[2], orientq[3],   orientq[0]]).as_euler("xyz") # Roulis, Tangage, Lacet 
        
        
    def callback_joy(self, msg):
        self.button_values = msg.buttons
        self.axes = msg.axes
        self.frame_id = msg.header.seq

    def callback_depth(self, msg):
        self.depth = -msg.data

    def callback_detection(self, msg):
        self.detected = msg.detected
        if self.detected:
            self.detected_x = msg.x
            self.detected_y = msg.y
            self.detected_w = msg.w
            self.detected_h = msg.h
            self.distance = -np.log10(max(0.001, self.detected_h))
            self.timer = time.time()
            self.test = 1
        else:  
            if (time.time() > self.timer+self.timer_max): ## If ROV looses the target, it continues to move in the same direction during timer_max second to try to catch it again. After timer_max second, the ROV stops. 
                self.detected_x = 0
                self.detected_y = 0           
                if self.test == 1:
                    self.tracking_mode = False
                    self.tracking_with_distance_mode = False
                    self.tracking_with_heading_mode = False
                    self.depth_target = self.depth
                    self.heading_local_target = self.heading
                    for i in range(8):
                        self.commands[i] = 1500
                    self.test = 0


                


    def listener(self):
        rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, self.callback_heading)
        rospy.Subscriber("/joy", Joy, self.callback_joy)
        rospy.Subscriber("/mavros/global_position/rel_alt", Float64, self.callback_depth)
        rospy.Subscriber("cam_detection", Detection, self.callback_detection)
        rospy.Subscriber("/mavros/imu/data", Imu, self.callback_imu)

        if self.is_assistant:
            rospy.Subscriber("rov_origin_cap", Float64, self.callback_heading_reference)
            print("\nI am assistant.\n")

    def run(self):
        while not rospy.is_shutdown():  # Infinite loop
            if self.frame_id_last != self.frame_id:  # Check if new commands are given with joystick
                self.frame_id_last = self.frame_id

                if self.button("Start") == 1:  # Start button
                    self.armed = not self.armed
                    mavros_command.arming(self.armed)
                    self.commands = [1500] * self.nb_channels
                    if self.nb_channels > 8:
                        self.commands[8] = 1000
                    self.tracking_mode = False
                    self.tracking_with_distance_mode = False
                    self.tracking_with_heading_mode = False
                    self.depth_control_mode = False
                    self.heading_control_mode = False
                    self.boost = False
                    self.robot_mode = "Armed" if self.armed else "Disarmed"
                    print(self.robot_mode)

                if self.button("Y") != 0:  # button Y : record
                    print('Y pressed -> Record cam')
                    self.record = not self.record
                    success = self.record_srv()
                    if success:
                        print("Record service success")

                if self.axes[5] < 0 and not self.boost:  # button RT: toggle slow/fast 
                    self.boost = True
                    print("Fast mode ON")  # keep the boost ON while the button is press
                    
                elif self.axes[5] >= 0 and self.boost:
                    self.boost = False
                    print("Fast mode OFF")

                if self.boost:
                    self.val_pwm = 500
                else:
                    self.val_pwm = 100
                    
                                    
                if param.Inverse_joystick:
                    # move left/right
                    if self.axes[0] != 0:  # joy left, left/right
                        self.commands[5] = int(-self.val_pwm * self.axes[0] + 1500)
                    else:
                        self.commands[5] = 1500

                    # rotation left/right
                    if self.axes[3] != 0:  # joy right, left/right
                        self.commands[3] = int(-self.val_pwm / 2 * self.axes[3] + 1500)
                        self.rotation_on = True
                    else:
                        self.commands[3] = 1500
                        self.rotation_on = False

                    # move forward/backward
                    if self.axes[1] != 0:  # joy left, up/down
                        self.commands[4] = int(self.val_pwm * self.axes[1] + 1500)
                    else:
                        self.commands[4] = 1500

                    # move up/down
                    if self.axes[4] != 0:  # joy right, up/down
                        self.commands[2] = int(self.val_pwm * self.axes[4] + 1500)
                        self.move_up = True
                    else:
                        self.commands[2] = 1500
                        self.move_up = False
                                           
                else:               
                    # move left/right
                    if self.axes[3] != 0:  # joy right left/right
                        self.commands[5] = int(-self.val_pwm * self.axes[3] + 1500)
                    else:
                        self.commands[5] = 1500

                    # rotation left/right
                    if self.axes[0] != 0:  # joy left left/right
                        self.commands[3] = int(-self.val_pwm / 2 * self.axes[0] + 1500)
                        self.rotation_on = True
                    else:
                        self.commands[3] = 1500
                        self.rotation_on = False

                    # move forward/backward
                    if self.axes[4] != 0:  # joy right up/down
                        self.commands[4] = int(self.val_pwm * self.axes[4] + 1500)
                    else:
                        self.commands[4] = 1500

                    # move up/down
                    if self.axes[1] != 0:  # joy left up/down
                        self.commands[2] = int(self.val_pwm * self.axes[1] + 1500)
                        self.move_up = True
                    else:
                        self.commands[2] = 1500
                        self.move_up = False


                # camera inclination
                if self.axes[7] != 0:  # fleche haut/bas
                    self.commands[7] = int(100 * self.axes[7] + 1500)

                else:
                    self.commands[7] = 1500

                if self.button('A') != 0:  # button A : tracking mode
                    self.tracking_mode = not self.tracking_mode
                    # Entrer tracking only if target is detected
                    if self.tracking_mode and not self.detected:
                        self.tracking_mode = False
                    if not self.tracking_mode:
                        self.tracking_with_heading_mode = False
                        self.tracking_with_distance_mode = False
                    print("Tracking :", self.tracking_mode)

                if self.button('B') != 0:  # button B : Choose cap with tracking
                    if self.tracking_mode and not self.tracking_with_heading_mode:
                        self.tracking_with_heading_mode = True
                    elif self.tracking_with_heading_mode:
                        self.tracking_with_heading_mode = False
                    elif not self.tracking_mode:
                        self.tracking_with_heading_mode = False


                if self.button("X") != 0:  # button X : activate distance control
                    if self.tracking_mode and not self.tracking_with_distance_mode:
                        self.tracking_with_distance_mode = True
                        self.distance_target = self.distance
                    elif self.tracking_with_distance_mode:
                        self.tracking_with_distance_mode = False
                    elif not self.tracking_mode:
                        self.tracking_with_distance_mode = False

                # light control
                if self.button("Back") != 0:  # button Back : control light intensity

                    self.pwm_light = self.pwm_light+100
                    if (self.pwm_light > 1900):
                        self.pwm_light = 1000
  

                if self.nb_channels > 8: # if the MAVROS version can control the light
                    self.commands[8] = self.pwm_light


                # if self.tracking_with_heading_mode:
                if self.axes[6] != 0:  # fleche gauche/droite
                    self.heading_offset = (self.heading_offset - 5.0 * self.axes[6]) % 360
                    self.heading_global_target = self.heading_reference + self.heading_offset
                    print("Heading offset = ", self.heading_offset)

                # if self.tracking_with_distance_mode:
                distance_offset = 0.05
                if self.button("LH") != 0:
                    self.distance_target += distance_offset
                    # self.distance_target = clip(self.distance_target, 0, 1)
                    print("Distance target = ", self.distance_target)
                if self.button("RH") != 0:
                    self.distance_target -= distance_offset
                    # self.distance_target = clip(self.distance_target, 0, 1)
                    print("Distance target = ", self.distance_target)

                # depth and heading stabilisation when tracking_mode not activate
                if not self.tracking_mode:
                    # if joystick rotation released and stabilisation heading off => stabilisation heading on
                    if self.rotation_on == False and not self.heading_control_mode: 
                        self.heading_control_mode = True
                        self.heading_local_target = self.heading  # current heading becomes target heading
                        print('Stabilisation CAP ON at', self.heading_local_target)
                    #  else if joystick rotation used and stabilisation heading on => stabilisation heading off
                    elif self.rotation_on == True and self.heading_control_mode: 
                        self.heading_control_mode = False
                        print('Stabilisation CAP OFF')

                    if self.move_up == False and not self.depth_control_mode: 
                        # if joystick depth released and stabilisation heading off => stabilisation depth on
                        self.depth_control_mode = True
                        self.depth_target = self.depth  # current depth becomes target depth
                        print('Stabilisation depth ON at', self.depth_target)
                    #  else if joystick depth used and stabilisation heading on => stabilisation depth off
                    elif self.move_up == True and self.depth_control_mode: 
                        self.depth_control_mode = False
                        print('Stabilisation depth OFF')

            
            if self.tracking_mode:
                self.depth_control_mode = True
                self.heading_control_mode = True

                # Control using detected positions in image
                x0 = self.detected_x*np.cos(self.Phi) + 0*self.detected_y*np.sin(self.Phi)
                y0 = self.detected_y*np.cos(self.Phi) - 0*self.detected_x*np.sin(self.Phi)
                self.heading_local_target = self.heading + self.gain_heading_tracking *x0
                self.depth_target = self.depth - y0

                # Control globally
                if self.tracking_with_heading_mode:
                    global_heading_error = sawtooth((self.heading_global_target + param.compass_offset - self.heading) * np.pi / 180)
                    k_y,k_yh = param.k_y, param.k_yh 
                    self.commands[5] = int(-k_y * np.tanh(k_yh*global_heading_error) + 1500)

                if self.tracking_with_distance_mode:
                    self.target_distance_msg.data = self.distance_target
                    distance_error = self.distance_target - self.distance
                    distance_d = (self.distance - self.distance_last) / (time.time() - self.time_last)
                    kt_p, kt_d, kt_h = param.kt_p, param.kt_d, param.kt_h

                    self.commands[4] = -int(kt_p * np.tanh(kt_h*distance_error) + kt_d * distance_d  ) + 1500

            if (self.detected_w > 0.20)|(self.detected_h>0.20): # if buoy too close, move slower
                k_stab = 0.5
            else:
                k_stab = 1

            
            # stabilisation depth
            if self.depth_control_mode:
                self.depth_target_msg.data = self.depth_target
                kd_p, kd_d, kd_h, kd_i = param.kd_p, param.kd_d, param.kd_h, param.kd_i
                e = self.depth - self.depth_target
                e_d = (self.depth - self.depth_last) / (time.time() - self.time_last)
                e_i = (self.depth -self.depth_target + self.depth_last - self.depth_target) 
                # self.commands[2] = int(kd_p * np.tanh(kd_h*e) + kd_d * e_d) + 1500
                self.commands[2] = int(k_stab*(kd_p * np.tanh(kd_h*e) + kd_d * e_d + kd_i*e_i ) ) + 1500

            # stabilisation heading
            if self.heading_control_mode:
                self.heading_target_msg.data = self.heading_local_target  # Update cap_d message for IHM
                e = sawtooth((self.heading_local_target - self.heading) * np.pi / 180)
                e_i = sawtooth((self.heading_local_target - self.heading) * np.pi / 180) + sawtooth((self.heading_local_target - self.heading_last )* np.pi / 180)
                kc_p, kc_d, kc_h, kc_i =  param.kc_p, param.kc_d, param.kc_h, param.kc_i  

                self.commands[3] = int(k_stab*(kc_p * np.tanh(kc_h*e) + kc_d * self.angular_velocity_z + kc_i*e_i)) + 1500

            # Send mode
            if self.armed:
                if self.tracking_mode:
                    if self.tracking_with_heading_mode:
                        self.robot_mode = "Tracking + Heading"
                        if self.tracking_with_distance_mode:
                            self.robot_mode = "Fully Autonomous"
                    elif self.tracking_with_distance_mode:
                        self.robot_mode = "Tracking + Distance"
                    else:
                        self.robot_mode = "Tracking"
                else:
                    if self.depth_control_mode or self.heading_control_mode:
                        self.robot_mode = "Stabilisation"
                    else:
                        self.robot_mode = "Manual"

            self.heading_last = self.heading
            self.depth_last = self.depth
            self.distance_last = self.distance
            self.time_last = time.time()

            self.send_commands()

            mode_msg = String()
            mode_msg.data = self.robot_mode
            self.robot_mode_pub.publish(mode_msg)

            self.target_depth_pub.publish(self.depth_target_msg)
            self.target_heading_pub.publish(self.heading_target_msg)
            self.distance_pub.publish(self.distance)
            self.target_distance_pub.publish(self.target_distance_msg)





            self.rate.sleep()
            
            
            
            
