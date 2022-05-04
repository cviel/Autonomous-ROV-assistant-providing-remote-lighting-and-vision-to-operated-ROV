
Copyright 2022, Samuel PROUTEN, Hugo SABATIER, Enzo ESSONO, Martin GOUNABOU, Christophe VIEL

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.



# Description of the node

This node proposes a method to automatize the position and orientation of an ROV assistant such it provides a remote lighting and vision of a work area to a ROV worker, said "main ROV". The ROV assistant locates the ROV worker using HSV detection and vision tracking to detect an colored buoy (orange or red by default) placed on the top of the ROV worker. By controlling its orientation, the ROV assistant keeps the ROV worker inside the center of the camera field. A maintain of the distance between the two ROVs can be performed using forward translation to maintain the buoy's image at a constant size. Then, by keeping the ROV worker in the camera field and performing lateral translation, the ROV assistant places itself to a desired orientation with the ROV worker. More information on the method is provided in [link article].


An implementation for BlueROV is proposed here, based on the node MAVROS. For an optimal control, two computers (one for each ROV) linked via Ethernet are required. A tracking of a target is however possible using only one ROV and the colored buoy attached to the target to follow.


An umbilicals management is described in [link article] to keep the two tethers in different workspace and so avoid entanglement between them.

Four modes of automatization are proposed:
- Tracking colored buoy (button A) : the ROV changes its orientation and depth to follow the colored buoy to keep it at center of the camera. 
	
- Tracking colored buoy and maintain a distance (button A + X) : the ROV changes track the buoy and maintain a constant distant between it and the buoy by trying to keep the buoy at a fixed size on the camera image (more detail in [link article]. The distance offset can be change using LH, RH.
	
- Tracking colored buoy and a desired heading between the two ROVs (button A + B) : the ROV changes track the buoy and maintain a constant heading offset between DHEADING the two ROVs (example: an offset DHEADING = 180° induces the two ROVs stay face to face, DHEADING = 90° induces the ROV assistant stays on the left of the main ROV. Ethernet connexion between the two ROVs is required for this mode to obtain main ROV heading. Else, the main ROV heading is considered equal to zero, allowing to follow a buoy attached on an other target. The heading offset can be changed using arrows left/right.
	
- Tracking colored buoy + maintain a distance + desired heading between the two ROVs (button A + B + X): all previous modes simultaneously.



# BlueROV workspace


### Prerequisite


###### Material

- 1 or 2 BlueROVs
- 1 or 2 computers
- 1 or 2 GamePades
- 1 Ethernet cable
- 1 colored buoy to attach on the main ROV


###### ROS

This work has been developped and tested using __ROS *noetic*__.

Following package are required: 

- ros-noetic-mavros
- ros-noetic-joy

###### Python environment

Following modules are required:

- numpy
- opencv-python (version 4.2.0)
- pyautogui
- imutils
- opencv-contrib-python
- sockets
- vidgear

###### QGroundControl

Follow these instructions to install QGroundControl :
[cliquer ici.](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html)

###### Build ROS workspace

Build the workspace with catkin_make or catkin build in function of your configuration. Think to use devel/setup.bash.


### Start

- Start the BlueROV then connect it using the USB-Ethernet. Configure the IP adresse with __*192.168.2.1*__.

- In the nodes developped here, the two ROV are controlled using 2 computers:
    - Connect the 2 computers via Ethernet.
    - Configure IP adress of the connexion with __*1.0.0.2*__ for the ROV worker (or main ROV) and __*1.0.0.3*__ for the ROV assistant.
    

- Start QGroundControl and let's the initialisation finished (TO DO at each reboot of the Bluerov). Then, change the Mavlink system ID to __1__. Close QGroundControl.
  
- You can now use the ROS nodes of the workspace.


### Packages

###### bluerov_launch

This package groups the launch files. It is first recommand to test the file "track.launch" first to check if the ROV answers
correctly to the control and detects the desired color. If the ROV does not react to the control, open a new terminal and use 
the following command to change the Mavlink system ID of the ROV:
    - rosrun mavros mavparam set SYSID_MYGCS 1



- track.launch : Launch nodes required to control the ROV with the joystick and the tracking color. __Use this launch_file
  if only 1 ROV is used.__
  
- asistant_rov.launch : Use it for the ROV assistant. Launch nodes required to control the ROV, the tracking color, and the networking between the  main ROV and the ROV assistant. Note that this launch can work alone without the network with the main ROV: in absence of communication with the main ROV, the ROV assistant follows the tracking color and takes the main ROV heading equal to zero for its calculation.
    - argument __video_file_path__ (default =  "bluerov_launch/video" : path of the video file which will be saved by the ROV camera.
    - argument __target_type__ (default = "Orange") : Choice of the color to track. To default colors are already implement: "Orange" and "Red Lego" (see inside the launch file).
    - argument __IP_address__ (default = "1.0.0.2") : IP adress of the computer piloting the main ROV.


- main_rov.launch : Use it for the main ROV. Launch nodes required to control the ROV, the tracking color, and the networking between the main ROV and the ROV assistant. This launch requires the connexion between the two ROVs is required to work. 
    - argument __video_file_path__ (default =  "bluerov_launch/video" : path of the video file which will be saved by the ROV camera.
    - argument __target_type__ (default = "Orange") : Choice of the color to track. To default colors are already implement: "Orange" and "Red Lego" (see inside the launch file).
    - argument __IP_address__ (default = "1.0.0.2") : IP adress of the computer piloting the main ROV.

    
    
###### bluerov_control

This package groups the nodes dedicated to the ROV control. More information on the control strategy at [article link].


###### bluerov_tracking

This package groups the nodes dedicated to the tracking of the ROV assistant. More information on the tracking strategy at [article link].


###### bluerov_ihm

This package groups the nodes dedicated to the IHM (Interaction Human Machine) of the ROV.

###### bluerov_networking

This package groups the nodes dedicated to the communication between the two computer controlling the main ROV and the ROV assistant.


### Joystick command

![gampepad_command](https://user-images.githubusercontent.com/35916776/166648085-08dd9f8d-7096-4bfd-acc5-3aa6256ef66f.png)


- **Arm/disarm ROV** : start
- **Up/Down** : _joystick left_ up/down
- **Yaw** : _joystick left_ left/right
- **Forward/Backward** : _joystick right_ up/down
- **Left/Right** : _joystick right_ left/right
- **Boost** : RT
- **Tracking on/off** : A
- **Tracking + Distance on/off** : X
- **Tracking + Heading on/off** : B
- **Record on/off** : Y
- **Change heading offset** : arrows left/right
- **Change distance offset** : LH, RH
- **Tilt camera** : arrows up/down
- **Light control** : select 


Boost : increase the ROV velocity while the button is pressed. Affect only the displacement commanded in manual control

Light control : change the intensity of ROV's lights from 0% to 100% (only if the mavros version allows it).




###### Known problem with joy_node

On some computer, it can exist a virtual user js0 in the repertory /dev/input/.
To solve this problem:
    - open a terminal and start roscore.
    - open a new terminal and use the following command: rosparam set joy_node/dev "/dev/input/js1"




