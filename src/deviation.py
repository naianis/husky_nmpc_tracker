#!/usr/bin/env python

import rospy
import math
import numpy
import tf
import matplotlib.pyplot as plt
from move_husky import MoveHusky
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion, Point
from visual_path_husky.msg import SysStatus
from husky_nmpc_tracker.msg import SysStatus1, DeviationParams

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA, Bool, UInt8

class MoveToGoal(object):

	def __init__(self):
		self.pub_obj = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
		self.sub_obj = rospy.Subscriber('/odometry/filtered', Odometry, self.OdomCallback)

		# Topic containing current system status
		#self.safety_sub = rospy.Subscriber('/safety_controller', SysStatus, self.SafetyCallback)
		self.safety_sub_1 = rospy.Subscriber('/safety_controller', SysStatus1, self.SafetyCallback)

		# Only used to debug through RViz
		self.marker_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=1)

		# Topic used to sinilize the status of the circle performance
		# 0 for no circle being performed
		# 1 to performing circle before its middle
		#2 for at least half of the semicircle done
		#self.circle_control_publisher = rospy.Publisher('/circle_control', UInt8, queue_size=3)
		self.circle_side_publisher = rospy.Publisher('/circle_side_publisher', Bool, queue_size=3)
		self.circle_pub = rospy.Publisher('/deviation_params', DeviationParams, queue_size=3)

		self.safety_msg = SysStatus1()		# Holds the current system security info
		self.iz = 0						# Trajectory matrix counter
		self.flag_keeper = False
		self.circle_flag = UInt8()
		self.process_begin = False
		self.traj_x = numpy.zeros(33)
		self.traj_y = numpy.zeros(33)

		self.move_obj = MoveHusky()
		self.sin_signal = True
		self.deviationparams = DeviationParams()
		self.deviationparams.status = 0


	def show_marker_in_rviz(self, marker_publisher, vx, vy):

		pontos = Point()
		marker = Marker()

		marker.header.frame_id = "odom"
		marker.type = marker.SPHERE_LIST
		marker.action = marker.ADD

		# marker scale
		marker.scale.x = 0.1
		marker.scale.y = 0.1
		marker.scale.z = 0.1

		# marker color
		marker.color.a = 1.0
		marker.color.r = 1.0
		marker.color.g = 1.0
		marker.color.b = 0.0

		# marker orientaiton
		marker.pose.orientation.x = 0.0
		marker.pose.orientation.y = 0.0
		marker.pose.orientation.z = 0.0
		marker.pose.orientation.w = 1.0

		# marker position
		marker.pose.position.x = 0.0
		marker.pose.position.y = 0.0
		marker.pose.position.z = 0.0

		# marker line points
		marker.points = []
		k=0

		while k < len(vx):
			pontos.x = vx[k]
			pontos.y = vy[k]
			pontos.z = 0.0
			marker.points.append(Point(vx[k],vy[k],0.0))
			k = k+1

		#print(marker.points)
	    # Publish the Marker
		self.marker_publisher.publish(marker)


	def PointsInCircumference(self):
		# Gets initial position
		position = rospy.wait_for_message('/odometry/filtered', Odometry)
		quaternion = (position.pose.pose.orientation.x, position.pose.pose.orientation.y, position.pose.pose.orientation.z, position.pose.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)

		initial_position = []
		initial_position.append(position.pose.pose.position.x)
		initial_position.append(position.pose.pose.position.y)
		initial_position.append( euler[2])

		print (initial_position)

		# Finds the radius of the circumference which must be at least 1.5m
		r = self.safety_msg.obstacle_size/2 + self.safety_msg.distance_to_obstacle + 0.1		# + 1 m for safety
		print(self.safety_msg)
		#if r < 0.3:
		#	r = 0.3

		self.deviationparams.curvature = 1/r
		print("raio = ") + str(r)
		# Finds the linear coeficient theta
		theta = initial_position[2]

		if theta < 0:
			sin_signal = 1
			rospy.logerr("SENO UM")
			self.sin_signal = True #!!!
			self.deviationparams.side = True
		else:
			sin_signal = -1
			rospy.logerr("SENO MENOS UM")
			self.sin_signal = False #!!!
			self.deviationparams.side = False


		# Finds the initial angle value to calculate semi circumference
		thetai = theta - math.pi

		# Calculates the origin of the circumference
		origin = [r * math.cos(theta) + initial_position[0], r * math.sin(theta) + initial_position[1]]

		points = int(r*10)
		step = math.pi/points

		print("Points: ", points, "		Step: ", step)

		#x = numpy.zeros(points + 1)
		#y = numpy.zeros(points + 1)
		
		x = []
		y = []
		t= math.pi/4
		i=0

		# calculates the circle matrix points
		while t < math.pi:
			x.append(r * math.cos(t+thetai) + origin[0])
			y.append(r * math.sin(t+thetai) * sin_signal + origin[1])
			#x[i] = r * math.cos(t+thetai) + origin[0]
			#y[i] = r * math.sin(t+thetai) * sin_signal + origin[1]
			t = t + step
			i+=1
		'''	
		delete_points =  int(points*0.3)
		for i in range (delete_points):
			x = numpy.delete(x,i)
			y = numpy.delete(y,i)
		'''
		plt.plot(x,y,'ro')
		#plt.show()

		self.show_marker_in_rviz(self.marker_publisher, x, y)
		return x, y


	def SafetyCallback(self,status_msg):

		self.safety_msg = status_msg

		if status_msg.flag and not self.process_begin:
			self.traj_x, self.traj_y = self.PointsInCircumference()
			self.flag_keeper = status_msg.flag
			self.process_begin = True
			#print(self.safety_msg)
			
		self.deviationparams.status = 0
		if self.iz < ((len(self.traj_x) / 2) + 1) and self.process_begin:
			self.deviationparams.status = 1

		if self.iz > (len(self.traj_x) - 1) and self.process_begin:
			self.deviationparams.status = 2

		self.circle_pub.publish(self.deviationparams)


	def OdomCallback(self,robot_state):

		'''quaternion = (robot_state.pose.pose.orientation.x, robot_state.pose.pose.orientation.y, robot_state.pose.pose.orientation.z, robot_state.pose.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)

		print (euler[2])
		'''
		if self.safety_msg.flag == False and self.process_begin == False:
			return
		#initial_point e final_point devem ser parametros de acordo com a informacao recebida do lidar

		if not self.process_begin:
			return

		if self.safety_msg.found_line and self.deviationparams.status == 0: #!!!
			self.process_begin = False
			self.iz = 0
			return

		v_max = 5.0
		w_max = 0.5

		goal_x = self.traj_x[self.iz]
		goal_y = self.traj_y[self.iz]

		# transform from quaternion for yaw degrees
		quaternion = (robot_state.pose.pose.orientation.x, robot_state.pose.pose.orientation.y, robot_state.pose.pose.orientation.z, robot_state.pose.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		yaw = euler[2]

		dx = goal_x - robot_state.pose.pose.position.x
		dy = goal_y - robot_state.pose.pose.position.y

		# velocities coeficients
		kp = 0.1
		ka = 1

		# linear and angular errors
		p = math.sqrt( pow(dx,2) + pow(dy,2) )
		a = -yaw + math.atan2(dy,dx)

		# calculates proportional linear and angular velocities
		v_ref = kp * p + 0.1
		w_ref = ka * a

		#restricts the maximum velocities
		if v_ref > v_max:
			v_ref = v_max
		if v_ref < -v_max:
			v_ref = - v_max

		if w_ref > w_max:
			w_ref = w_max
		if w_ref < -w_max:
			w_ref = -w_max

		twist_obj = Twist()

		if self.iz == 2 and abs(a) > 0.15:
			v_ref = 0

		if self.iz >= len(self.traj_x) - 1 :
			twist_obj.linear.x = 0.0
			twist_obj.angular.z = 0.0
			print("Bigger iz")

		# publishes velocities
		twist_obj.linear.x = v_ref
		twist_obj.angular.z = w_ref
		self.move_obj.move_robot(twist_obj)

		if p < 0.2 :
			print("Got to goal ") + str(self.iz)
			self.iz = self.iz + 1


def main():
	rospy.init_node('circle_performer')
	goal_obj = MoveToGoal()

	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		rate.sleep


if __name__ == '__main__':
	main()
