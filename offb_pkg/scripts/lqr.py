#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import ActuatorControl
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import SetModeRequest
from mavros_msgs.srv import SetModeResponse
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandBoolRequest
from mavros_msgs.srv import CommandBoolResponse
from sensor_msgs.msg import Imu

from tf.transformation import *

import control as ctrl 


current_state = State()

imu_data = Imu()

k_p_yaw = 0.05
k_d_yaw = 0

k_p_pitch = 0.2
k_d_pitch = 0.045

k_p_roll = 0.2
k_d_roll = 0.045


local_velocity = TwistStamped()
def lv_cb(data):
	global local_velocity
	local_velocity = data



local_position = PoseStamped()
def lp_cb(data):
	global local_position
	local_position = data


def state_cb(data):
	global current_state
	current_state = data
	# flag = data.armed
	# print(flag)
	# rospy.loginfo(current_state.connected)


def imu_cb(data):
	global imu_data
	imu_data = data
 


def control():
	act_control = []
	act_control = np.load("hover.npy")

	sp = []
	sp = np.load("s_des.npy")
	print(sp.shape)

	state_sub = rospy.Subscriber("/mavros/state",State,state_cb,queue_size=10)

	rospy.wait_for_service('mavros/cmd/arming')
	arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

	rospy.wait_for_service('mavros/set_mode')
	set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

	acutator_control_pub = rospy.Publisher("/mavros/actuator_control",ActuatorControl,queue_size=10)

	local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local",PoseStamped,queue_size=10)

	mocap_pos_pub = rospy.Publisher("/mavros/mocap/pose",PoseStamped,queue_size=100)

	imu_sub = rospy.Subscriber("/mavros/imu/data",Imu,imu_cb, queue_size=100)

	local_pos_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, lp_cb, queue_size=100)
	local_vel_sub = rospy.Subscriber("/mavros/local_position/velocity", PoseStamped, lv_cb, queue_size=100)


	rospy.init_node('control',anonymous=True)
	rate =  rospy.Rate(10.0)

	# print("*"*80)
	while not rospy.is_shutdown() and not current_state.connected:
		rate.sleep()
		rospy.loginfo(current_state.connected)
	# print("#"*80)

	pose = PoseStamped()
	pose.pose.position.x = 0
	pose.pose.position.y = 0
	pose.pose.position.z = 3

	mocap_pose = PoseStamped()

	

	j = 100
	while j > 0 and not rospy.is_shutdown():
		local_pos_pub.publish(pose)
		rate.sleep()
		j -= 1

	offb_set_mode = SetModeRequest()
	offb_set_mode.custom_mode = "OFFBOARD"


	arm_cmd = CommandBoolRequest()
	arm_cmd.value = True

	last_request = rospy.Time.now()

	i = 0
	act = ActuatorControl()
	flag1 = False
	flag2 = False

	prev_imu_data = Imu()
	prev_time = rospy.Time.now()

	# prev_x = 0
	# prev_y = 0
	# prev_z = 0

	# prev_vx = 0
	# prev_vy = 0
	# prev_vz = 0

	row = 0
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
		if flag1:
			# rospy.loginfo("act")
			# act.controls = act_control[i,:]
			# i = (i+1)%(act_control.shape[0])
			# acutator_control_pub.publish(act)
			# local_pos_pub.publish(pose)
			# rospy.loginfo(imu_data.linear_acceleration.z)
			# curr_time = rospy.Time.now()
			# double dt = curr_time - prev_time
			# mocap_pose.pose.position.x = k_p_roll * imu_data.angular_velocity.x + k_d_roll * (imu_data.angular_velocity.x - prev_imu_data.angular_velocity.x) /(dt)
			# mocap_pose.pose.position.y = k_p_pitch * imu_data.angular_velocity.y + k_d_pitch * (imu_data.angular_velocity.y - prev_imu_data.angular_velocity.y)/(dt)
			# mocap_pose.pose.position.z = k_p_yaw * (imu_data.angular_velocity.z-3)	 + k_d_yaw * (imu_data.angular_velocity.z - prev_imu_data.angular_velocity.z)/(dt)
			# mocap_pose.pose.position.x = 0
			# mocap_pose.pose.position.y = 0.001
			# mocap_pose.pose.position.z = 0

			x = local_position.pose.position.x
			y = local_position.pose.position.y
			z = local_position.pose.position.z

			x_des = sp[row,0]
			y_des = sp[row,1]
			z_des = sp[row,2]

			vx = local_velocity.twist.linear.x
			vy = local_velocity.twist.linear.y
			vz = local_velocity.twist.linear.z	

			vx_des = sp[row,3]
			vy_des = sp[row,4]
			vz_des = sp[row,5]


			phi = imu_data.

			mocap_pose.pose.position.x = k_p_roll * ( y-y_des) + k_d_roll * ( vy-vy_des)
			mocap_pose.pose.position.y = k_p_pitch * (x_des - x) + k_d_pitch * (vx_des - vx)
			# mocap_pose.pose.position.z = k_p_yaw * (z - z_des) + k_d_yaw * (vz - vz_des)
			# mocap_pose.pose.position.y = 0
			mocap_pose.pose.position.z = 0





			# mocap_pose.pose.position.y = sp[row,0]
			# mocap_pose.pose.position.x = sp[row,1]
			# mocap_pose.pose.position.z = -sp[row,2]
			# mocap_pose.header.stamp = rospy.Time.now()

			pose.pose.position.z = 3

			# prev_imu_data = imu_data
			# prev_time = current_time

			row  = row+1
			# rospy.loginfo(mocap_pose.pose.position.x)
			# rospy.loginfo(mocap_pose.pose.position.y)
			# rospy.loginfo(mocap_pose.pose.position.z)
			# rospy.loginfo(row)

			local_pos_pub.publish(pose)
			mocap_pos_pub.publish(mocap_pose)


			
		rate.sleep()





if __name__ == '__main__':
	try:
		# current_state = State()
		control()
	except rospy.ROSInterruptException:
		pass