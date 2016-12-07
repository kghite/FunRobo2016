#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8MultiArray
from geometry_msgs.msg import Twist
import numpy as np

ARRAY_SIZE = 101
INPUTS = ['wpt', 'obst']

class Midbrain_Arbiter(object):
	def __init__(self):
		rospy.init_node('midbrain_arbiter')

		self.vel_array = np.zeros([len(INPUTS), ARRAY_SIZE])
		self.vel_array[:,49] = 1
		self.turn_array = np.zeros([len(INPUTS), ARRAY_SIZE])
		self.turn_array[:,49] = 1

		rospy.Subscriber('wpt/cmd_vel', Int8MultiArray, self.wpt_cmd_vel_cb)
		rospy.Subscriber('obst/cmd_vel', Int8MultiArray, self.obst_cmd_vel_cb)

		self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

	def wpt_cmd_vel_cb(self, msg):
		#print("TURN INPUT")
		#print(msg)
		self.update_array(msg.data, INPUTS.index('wpt'))

	def obst_cmd_vel_cb(self, msg):
		#print("VEL INPUT")
		#print(msg)
		self.update_array(msg.data, INPUTS.index('obst'))

	def update_array(self, data, row):
		print(data)
		data = np.asarray(data).reshape([2, ARRAY_SIZE])
		print("INPUT DATA")
		self.vel_array[row] = data[0]
		print(self.vel_array)
		self.turn_array[row] = data[1]
		print(self.turn_array)

	def run(self):
		vel_sum_array = np.zeros(ARRAY_SIZE)
		turn_sum_array = np.zeros(ARRAY_SIZE)
		#vel_sum_array = np.array([1., 2, 3, 4, 5, 6, 5, 5 ,5, 2])
		#turn_sum_array = np.array([0., 0, 10, 1, 2, 3, 3, 2, 1, 1])
		for i in range(len(INPUTS)):
			print("SUM ARRAYS")
			vel_sum_array += self.vel_array[i]
			turn_sum_array += self.turn_array[i]

		#print(vel_sum_array)
		#print(turn_sum_array)
		
		vel = vel_sum_array.argmax()
		#print(vel)
		turn = turn_sum_array.argmax()
		#print(turn)
		msg = Twist()
		msg.linear.x = float(2*vel)/float((ARRAY_SIZE-1))-1
		msg.angular.z = float(2*turn)/float((ARRAY_SIZE-1))*0.25-0.25 #Turn a maximum of 0.25 instead of 1
		print(msg)
		self.cmd_vel_pub.publish(msg)

if __name__ == '__main__':
	main = Midbrain_Arbiter()
	r = rospy.Rate(50)
	while not rospy.is_shutdown():
		main.run()
		r.sleep()
