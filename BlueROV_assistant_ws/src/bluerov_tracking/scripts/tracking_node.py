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



import os.path

import cv2
from vidgear.gears import WriteGear


import gi
import imutils
import numpy as np

import datetime
import time

gi.require_version('Gst', '1.0')
from gi.repository import Gst

import rospy
from bluerov_tracking.msg import Detection
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Float64

heading = 0
def callback_compass(data):
    global heading
    heading = data.data

altitude = 0
def callback_alt(data):
    global altitude
    altitude = -data.data

distance = 0
def callback_distance(data):
    global distance
    distance = data.data
    
target_heading = 0
def callback_target_heading(data):
    global target_heading
    target_heading = data.data
    
target_depth = 0
def callback_target_alt(data):
    global target_depth
    target_depth = data.data
    
target_distance = 0
def callback_target_distance(data):
    global target_distance
    target_distance = data.data
        
                       	
def listener():
    rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, callback_compass)  
    rospy.Subscriber("/mavros/global_position/rel_alt", Float64, callback_alt)
    rospy.Subscriber("distance", Float64, callback_distance)
    rospy.Subscriber("target_heading", Float64, callback_target_heading)
    rospy.Subscriber("target_depth", Float64, callback_target_alt)
    rospy.Subscriber("target_distance", Float64, callback_target_distance)




class Video:
    """BlueRov video capture class constructor

    Attributes:
        port (int): Video UDP port
        video_codec (string): Source h264 parser
        video_decode (string): Transform YUV (12bits) to BGR (24bits)
        video_pipe (object): GStreamer top-level pipeline
        video_sink (object): Gstreamer sink element
        video_sink_conf (string): Sink configuration
        video_source (string): Udp source ip and port
    """

    def __init__(self, port=5600):
        """Summary

        Args:
            port (int, optional): UDP port
        """

        Gst.init(None)

        self.port = port
        self._frame = None

        # [Software component diagram](https://www.ardusub.com/software/components.html)
        # UDP video stream (:5600)
        self.video_source = 'udpsrc port={}'.format(self.port)
        # [Rasp raw image](http://picamera.readthedocs.io/en/release-0.7/recipes2.html#raw-image-capture-yuv-format)
        # Cam -> CSI-2 -> H264 Raw (YUV 4-4-4 (12bits) I420)
        self.video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
        # Python don't have nibble, convert YUV nibbles (4-4-4) to OpenCV standard BGR bytes (8-8-8)
        self.video_decode = \
            '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
        # Create a sink to get data
        self.video_sink_conf = \
            '! appsink emit-signals=true sync=false max-buffers=2 drop=true'

        self.video_pipe = None
        self.video_sink = None

        self.resolution = (640, 480)  

        self.run()

    def start_gst(self, config=None):
        """ Start gstreamer pipeline and sink
        Pipeline description list e.g:
            [
                'videotestsrc ! decodebin', \
                '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                '! appsink'
            ]

        Args:
            config (list, optional): Gstreamer pileline description list
        """

        if not config:
            config = \
                [
                    'videotestsrc ! decodebin',
                    '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                    '! appsink'
                ]

        command = ' '.join(config)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name('appsink0')

    @staticmethod
    def gst_to_opencv(sample):
        """Transform byte array into np array

        Args:
            sample (TYPE): Description

        Returns:
            TYPE: Description
        """
        buf = sample.get_buffer()
        caps = sample.get_caps()

        array = np.ndarray(
            (
                caps.get_structure(0).get_value('height'),
                caps.get_structure(0).get_value('width'),
                3
            ),
            buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)

        resolution = (
            caps.get_structure(0).get_value('width'),
            caps.get_structure(0).get_value('height')

        )

        return array, resolution

    def frame(self):
        """ Get Frame

        Returns:
            iterable: bool and image frame, cap.read() output
        """
        return self._frame

    def frame_available(self):
        """Check if frame is available

        Returns:
            bool: true if frame is available
        """
        return type(self._frame) != type(None)

    def run(self):
        """ Get frame to update _frame
        """

        self.start_gst(
            [
                self.video_source,
                self.video_codec,
                self.video_decode,
                self.video_sink_conf,

            ])

        self.video_sink.connect('new-sample', self.callback)

    def callback(self, sink):
        sample = sink.emit('pull-sample')
        new_frame, new_resolution = self.gst_to_opencv(sample)
        self._frame = new_frame

        self.resolution = new_resolution

        return Gst.FlowReturn.OK


class Tracker:

    def __init__(self):

        rospy.init_node("tracking_node")

        self.target_type = rospy.get_param(rospy.get_name() + "/target_type", "Orange Buoy")
        
        self.set_hsv_limits()

        self.coord_publisher = rospy.Publisher("cam_detection", Detection, queue_size=10)
        self.msg = Detection()

        self.record = False
        self.start_record = False
        rospy.Service("record_cam", Trigger, self.callback_record)
        self.filename0 = rospy.get_param(rospy.get_name() + '/video_file_path', os.path.join(os.path.curdir, 'test_video'))
        
        
        self.output_params = {"-vcodec":"libx264", "-crf": 0, "-preset": "fast"} # define (Codec,CRF,preset) FFmpeg tweak parameters for writer 

        self.resolution = cv2.VideoWriter_fourcc(*'DIVX');
        self.extension = '.avi'
#        self.resolution = cv2.VideoWriter_fourcc(*'MP4V');
#        self.extension = '.mp4'       
        
        filename = rospy.get_param(rospy.get_name() + '/video_file_path', os.path.join(os.path.curdir, 'test_video'))
        filename += 'video.avi'
        print(f"Video file :{filename}")

        # Create useful objects
        self.video = Video()
        self.fps = 50

        while not self.video.frame_available():  # wait for the first frame
            pass

        print("\n---------------- Video OK ------------\n")
        self.frame = self.video.frame()

        self.frame_size = (int(self.video.resolution[0]), int(self.video.resolution[1]))

        cv2.namedWindow("Camera")
        cv2.moveWindow("Camera", 0, 0)
        cv2.namedWindow("Mask")
        cv2.moveWindow("Mask", 0, self.frame_size[1] + 2 * 32)


        self.tracked = False
        self.bbox_detected = None

        self.freq_detection = 10  

        self.i = 1
        self.rate = rospy.Rate(self.fps)

    def set_hsv_limits(self):

        if self.target_type == "Red Lego":
            self.hsvL, self.hsvU = (168, 111, 53), (255, 255, 255)
            print("Target type is :", self.target_type)
        elif self.target_type == "Orange Buoy":
            self.hsvL, self.hsvU = (0, 95, 167), (40, 160, 255) #(0, 70, 120), (20, 255, 255)
            print("Target type is :", self.target_type)
        elif self.target_type == "Black ROV":
            self.hsvL, self.hsvU = (106, 55, 0), (213, 255, 43)
            print("Target type is :", self.target_type)
        else:
            print("No target type given, default is Orange Buoy.")
            self.hsvL, self.hsvU = (0, 70, 120), (20, 255, 255)

    def detect_pos_target(self, frame, hsvLower=(0, 70, 120), hsvUpper=(20, 255, 255), min_radius=10, min_proportion=0.4):

        # blur the frame and convert it to the HSV color space
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        # construct a mask for the color, then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, hsvLower, hsvUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        # find contours in the mask
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            # compute the coordinates of the minimum enclosing box
            # and make sure the box is not outside the image
            xmin, xmax = max(0, int(x - radius)), min(frame.shape[1], int(x + radius))
            ymin, ymax = max(0, int(y - radius)), min(frame.shape[0], int(y + radius))

            # compute the proportion of detected pixels in the box
            number_of_white_pix = np.sum(mask[ymin:ymax, xmin:xmax] == 255)
            square_proportion = number_of_white_pix / (2 * radius) ** 2

            # draw the circle around the target on the mask
            cv2.circle(mask, (int(x), int(y)), int(radius), (255, 255, 0), 2, 2)
            cv2.imshow("Mask", imutils.resize(mask, height=240))

            # if the density of detected pixels and the radius of the minimum
            # enclosing circle are greater than the minimum
            # return a box (xmin, ymin, w, h)
            if square_proportion > min_proportion and radius > min_radius:
                bbox_target = (int(x - radius), int(y - radius), int(2 * radius), int(2 * radius))
                return bbox_target
        
        return None

    def reliable_detection(self, min_proportion_of_frame_area=0.00):
        return self.bbox_detected[2]*self.bbox_detected[3] > self.frame_size[0]*self.frame_size[1] * min_proportion_of_frame_area


    def update_msg(self, bbox):
        self.msg.detected = self.tracked

        if bbox is not None:
            x = int(bbox[0] + bbox[2] / 2)
            y = int(bbox[1] + bbox[3] / 2)
            x, y = round(x / (self.frame_size[0] / 2) - 1, 2), round(-y / (self.frame_size[1] / 2) + 1, 2)
            w, h = round(bbox[2] / self.frame_size[0], 2), round(bbox[3] / self.frame_size[1], 2)
            self.msg.x = x
            self.msg.y = y
            self.msg.w = w
            self.msg.h = h


    def draw_bbox(self, img, bbox, color = (0, 0, 255)):
        if bbox is not None:
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(img, p1, p2, color, 2, 1)


    def callback_record(self, srv):
        self.record = not self.record
        return TriggerResponse(True, "Changed record mode." + str(self.record))


    def run(self):

        while not rospy.is_shutdown():

            if not self.video.frame_available():
                continue

            self.frame = self.video.frame()
            self.display_img = self.frame.copy()

            # additionnal information
            listener()
            font = cv2.FONT_HERSHEY_SIMPLEX
            text1 = "heading = " + str(heading) + " deg"
            text2 = "depth = " + str(altitude) + " m"
            taille_font = 0.5
            current_date_and_time = datetime.datetime.now()
            text_date = str(current_date_and_time)[0:19]    
                

            if self.record:
            
                if self.start_record == False:
                    current_date_and_time = datetime.datetime.now()
                    current_date_and_time_string = str(current_date_and_time)[0:19]  #[0:19] : keep only seconds
                    # create directory to stock data
                    path =  self.filename0  + current_date_and_time_string
                    os.mkdir(path)
                    # video name
                    filename = path + '/video' + current_date_and_time_string + self.extension
                    #self.writer = cv2.VideoWriter(filename, self.resolution, self.fps, self.frame_size)
                    self.writer = WriteGear(filename, compression_mode = True, logging = True, **self.output_params) #Define writer with output filename 'Output.mp4' 
                    self.time_last_frame = time.time()
                    
                    
                    # creation of others files
                    name_file_cap = path + "/heading_" + current_date_and_time_string + ".txt" 
                    name_file_target_cap = path + "/target_heading_" + current_date_and_time_string + ".txt" 
                    name_file_alt = path + "/depth_" + current_date_and_time_string + ".txt"   
                    name_file_target_alt = path + "/target_depth_" + current_date_and_time_string + ".txt" 
                    name_file_distance = path + "/distance_" + current_date_and_time_string + ".txt"   
                    name_file_target_distance = path + "/target_distance_" + current_date_and_time_string + ".txt"                   
                    name_file_time = path + "/time_" + current_date_and_time_string + ".txt"                     
                    self.file_cap = open(name_file_cap,'w');
                    self.file_target_cap = open(name_file_target_cap,'w');
                    self.file_alt = open(name_file_alt,'w');
                    self.file_target_alt = open(name_file_target_alt,'w');
                    self.file_distance = open(name_file_distance,'w');
                    self.file_target_distance = open(name_file_target_distance,'w');                    
                    self.file_time = open(name_file_time,'w');
                    
                              
                    self.start_record = True


                # add information 
                self.frame_record = self.frame.copy()      
                cv2.putText(self.frame_record, text1, (10,20), font, taille_font, (0, 255, 0), 1, cv2.LINE_AA)  
                cv2.putText(self.frame_record, text2, (10,40), font, taille_font, (0, 255, 0), 1, cv2.LINE_AA) 
                cv2.putText(self.frame_record, text_date, (400,20), font, taille_font, (0, 255, 0), 1, cv2.LINE_AA)    
                
                # Write the frame into the file 'filename.avi' 
                time_instant = time.time()
                number_frame0 = (time_instant - self.time_last_frame)*self.fps
                number_frame = int(number_frame0)  # time in s * number of frame by seconds
                
                if number_frame > 0: 
                    self.time_last_frame = time_instant + (number_frame0-number_frame)/self.fps 
                    
                    
                    #self.writer.write(self.frame_record)
                    for i in (range(number_frame)):  # record the correct number of frame
                        self.writer.write(self.frame_record)


                self.file_cap.write(str(heading)+' \n')
                self.file_target_cap.write(str(target_heading)+' \n')
                self.file_alt.write(str(altitude)+' \n' )
                self.file_target_alt.write(str(target_depth)+' \n' )
                self.file_distance.write(str(distance)+' \n' )
                self.file_target_distance.write(str(target_distance)+' \n' )
                self.file_time.write(str(current_date_and_time)+' \n' )                
                
                
                
            elif self.start_record == True:
                #self.writer.release()
                self.writer.close()
                self.file_cap.close()
                self.file_target_cap.close()                
                self.file_alt.close()
                self.file_target_alt.close()
                self.file_distance.close()
                self.file_target_distance.close()
                self.file_time.close()
                self.start_record = False
            

            # ------ Start or Restart
            if not self.tracked or self.i == 0:

                # Try to detect
                self.bbox_detected = self.detect_pos_target(self.frame, self.hsvL, self.hsvU, min_proportion=0.)
                if self.bbox_detected is not None and self.reliable_detection():  # If detection
                    self.tracker = cv2.TrackerCSRT_create()
                    self.tracker.init(self.frame, self.bbox_detected)  # Start tracker with bbox_detected
                    self.tracked = True

            else:
                # Update the tracker
                self.tracked, self.bbox_detected = self.tracker.update(self.frame)


            self.update_msg(self.bbox_detected)
            self.coord_publisher.publish(self.msg)

            self.i += 1
            self.i %= self.freq_detection

            
            self.draw_bbox(self.display_img, self.bbox_detected)
       
            
            cv2.imshow("Camera", self.display_img)
            cv2.waitKey(1)

            self.rate.sleep()

if __name__ == '__main__':

    tracker = Tracker()
    tracker.run()
