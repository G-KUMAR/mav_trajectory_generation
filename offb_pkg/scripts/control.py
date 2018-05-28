#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ActuatorControl
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import SetModeRequest
from mavros_msgs.srv import SetModeResponse
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandBoolRequest
from mavros_msgs.srv import CommandBoolResponse

current_state = State()

def state_cb(data):
	global current_state
	current_state = data
	# flag = data.armed
	# print(flag)
	# rospy.loginfo(current_state.connected)



def control():
	act_control = []
	act_control = np.load("hover.npy")

	state_sub = rospy.Subscriber("/mavros/state",State,state_cb,queue_size=10)

	rospy.wait_for_service('mavros/cmd/arming')
	arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

	rospy.wait_for_service('mavros/set_mode')
	set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

	acutator_control_pub = rospy.Publisher("/mavros/actuator_control",ActuatorControl,queue_size=10)

	local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local",PoseStamped,queue_size=10)

	rospy.init_node('control',anonymous=True)
	rate =  rospy.Rate(250.0)

	# print("*"*80)
	while not rospy.is_shutdown() and not current_state.connected:
		rate.sleep()
		rospy.loginfo(current_state.connected)
	# print("#"*80)

	# pose = PoseStamped()
	# pose.pose.position.x = 0
	# pose.pose.position.y = 0
	# pose.pose.position.z = 0.1

	# j = 100
	# while j > 0 and not rospy.is_shutdown():
	# 	local_pos_pub.publish(pose)
	# 	rate.sleep()
	# 	j -= 1

	offb_set_mode = SetModeRequest()
	offb_set_mode.custom_mode = "OFFBOARD"


	arm_cmd = CommandBoolRequest()
	arm_cmd.value = True

	last_request = rospy.Time.now()

	i = 0
	act = ActuatorControl()
	flag1 = False
	flag2 = False
		# rospy.loginfo("Outside")
	while not rospy.is_shutdown():
		if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_request > rospy.Duration(5.0)):
			offb_set_mode_response = set_mode_client(offb_set_mode)
			if offb_set_mode_response.mode_sent:
				rospy.loginfo("Offboard enabled")
				flag1 = True

			last_request = rospy.Time.now()
		else:
			if not current_state.armed and (rospy.Time.now() - last_request > rospy.Duration(5.0)):
				arm_cmd_response = arming_client(arm_cmd)
				if arm_cmd_response.success :
					rospy.loginfo("Vehicle armed")
					flag2 = True

				last_request = rospy.Time.now()


		# rospy.loginfo("Inside")
		if 1 == 1: 
		# if flag1 and flag2:
			rospy.loginfo("act")
			act.controls = act_control[i,:]
			i = (i+1)%(act_control.shape[0])
			acutator_control_pub.publish(act)
			# local_pos_pub.publish(pose)
			
		rate.sleep()





if __name__ == '__main__':
	try:
		# current_state = State()
		control()
	except rospy.ROSInterruptException:
		pass