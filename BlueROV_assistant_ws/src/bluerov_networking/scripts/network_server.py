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


import socket

import rospy
from std_msgs.msg import Float64


class Server:

    def __init__(self):

        rospy.init_node('network_node')  # , anonymous=True, disable_signals=True)

        # node launch in the ROS of the master 
        rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, self.callback_compass)

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        rospy.on_shutdown(self.s.close)

        self.host = rospy.get_param(rospy.get_name() + "/host", '1.0.0.1')
        self.port = rospy.get_param(rospy.get_name() + "/port", 4002)

        self.s.bind((self.host, self.port))

        self.rate = rospy.Rate(10)

        self.origin_cap = 0.

        self.connected = False

    def callback_compass(self, msg):
        self.origin_cap = msg.data

    def run(self):

        while not rospy.is_shutdown():

            if self.connected:

                try:
                    conn.sendall(str(self.origin_cap).encode())
                except Exception as e:
                    print(e)
                    self.connected = False
                    conn.close()

            else:
                self.s.listen()
                conn, addr = self.s.accept()
                self.connected = True
                print("Connection:", self.connected)

            self.rate.sleep()


if __name__ == '__main__':
    main = Server()
    main.run()
