#!/usr/bin/env python
import sys
from pal_detection_msgs.msg import Detections2d
from actionlib import SimpleActionClient, GoalStatus
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
import time
# ROS imports
import rospy


def show_usage():
    """Show usage information giving the possible motions to use."""
    # Get the available motion names from param server
    param_names = rospy.get_param_names()
    motion_names = []
    for param_name in param_names:
        # If the parameter is like '/play_motion/motions/MOTION_NAME/joints'
        if "/play_motion/motions" in param_name and '/joints' in param_name:
            motion_name = param_name.replace('/play_motion/motions/', '')
            motion_name = motion_name.replace('/joints', '')
            motion_names.append(motion_name)

    rospy.loginfo("""Usage:

\trosrun run_motion run_motion_python_node.py MOTION_NAME"

\twhere MOTION_NAME must be one of the motions listed in: """ + str(motion_names))


def wait_for_valid_time(timeout):
    """Wait for a valid time (non-zero), this is important
    when using a simulated clock"""
    # Loop until:
    # * ros master shutdowns
    # * control+C is pressed (handled in is_shutdown())
    # * timeout is achieved
    # * time is valid
    start_time = time.time()
    while not rospy.is_shutdown():
        if not rospy.Time.now().is_zero():
            return
        if time.time() - start_time > timeout:
            rospy.logerr("Timed-out waiting for valid time.")
            exit(0)
        time.sleep(0.1)
    # If control+C is pressed the loop breaks, we can exit
    exit(0)


def get_status_string(status_code):
    return GoalStatus.to_string(status_code)

def callback(msg):
	
	if 'msg.detections[0].x' in locals():
		x = msg.detections[0].x
		y = msg.detections[0].y
				
		
		rospy.loginfo('X: {}, Y: {}'.format(x, y))
	
	#rospy.loginfo('Y: {}'.format(y))	
	#do something with closest

	if (msg.detections ):
		#flag =1
		client = SimpleActionClient('/play_motion', PlayMotionAction)

 		rospy.loginfo("Waiting for Action Server...")
    		client.wait_for_server()

    		goal = PlayMotionGoal()
    		goal.motion_name = "offer_hand"
    		goal.skip_planning = False
    		goal.priority = 0  # Optional

    		rospy.loginfo("Sending goal with motion: " + "offer_hand")
    		client.send_goal(goal)

    		rospy.loginfo("Waiting for result...")
    		action_ok = client.wait_for_result(rospy.Duration(30.0))

    		state = client.get_state()

    		if action_ok:
        		rospy.loginfo("Action finished succesfully with state: " + str(get_status_string(state)))
			
    		else:
        		rospy.logwarn("Action failed with state: " + str(get_status_string(state)))
		
	else:
		
		client = SimpleActionClient('/play_motion', PlayMotionAction)

 		rospy.loginfo("Waiting for Action Server...")
    		client.wait_for_server()

    		goal = PlayMotionGoal()
    		goal.motion_name = "home"
    		goal.skip_planning = False
    		goal.priority = 0  # Optional

    		rospy.loginfo("Sending goal with motion: " + "home")
    		client.send_goal(goal)

    		rospy.loginfo("Waiting for result...")
    		action_ok = client.wait_for_result(rospy.Duration(30.0))

    		state = client.get_state()

    		if action_ok:
        		rospy.loginfo("Action finished succesfully with state: " + str(get_status_string(state)))
			
    		else:
        		rospy.logwarn("Action failed with state: " + str(get_status_string(state)))

def main():

	rospy.init_node('person')
	#pub = rospy.Publisher('closest_person', PersonDistance, queue_size=10)
	#monitor = PersonMonitor (pub, people)
	
	rospy.Subscriber("/person_detector/detections", Detections2d, callback)
	rospy.spin()

if __name__ == '__main__':
	main()
