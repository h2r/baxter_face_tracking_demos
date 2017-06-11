# baxter_face_tracking_demos
This demos was created Brown University's h2r Human to Robots Laboratory.

A collection of face tracking demos for baxter.  To run, you need to have Ein, which you can find here https://github.com/h2r/ein.

Go to "~/catkin_ws/src".  Run "git clone https://github.com/dhalper1/baxter_face_tracking_demos.git".

Go to "~/catkin_ws".  And run "catkin_make".  To use any of the demos you need to make sure the desired camera is available.  Run "rosrun baxter_tools camera_control.py -l".  This will list out your available cameras.  If the camera you wish to use is not in that list, find a camera you don't need (right_hand_camera/left_hand_camera/head_camera).  Let's say I want the head camera but it only listed the two hand cameras.  I have to close one of the hand cameras I don't need.  To do that run the command "rosrun baxter_tools camera_control.py -c [left/right]_hand_camera" (replace "[left/right]" with either the "left" or "right" depending on which camera you don't need at the moment).  Then you are free to run your demo.

Some cooler demos I suggest are the right_arm_face_follower.py, left_arm_face_follow.py, waves_for_people.py, or head_pans_to_face.py.  To run these type "rosrun baxter_face_tracking_demos [file name]".
