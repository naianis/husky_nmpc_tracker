#!/usr/bin/env python

import rospy
import time
import cv2
import rosbag
import ctypes
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import ros_numpy
import numpy as np
from resource import *


class binary_search_struct():
	def __init__(self):
		self.pos_x = 0
		self.pos_y = 0
		self.left = 0
		self.right = 0
		self.flag = 0
		self.dad = 0
		self.dad2 = [0,0]
		self.depth = 0
		self.depth2 = [0,0]
		self.step = 0

#-----------------------------------------------------------------------------
class HeuristicBinary(object):

#-----------------------------------------------------------------------------
	def __init__(self):

		# Debug variables
		self.i = 0
		self.j = 0

		self.bridge_object = CvBridge()
		self.filtered_img = np.zeros((480,640))

		self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.rgb_process)
		self.pc_points_subscriber = rospy.Subscriber("/camera/depth/points", PointCloud2, self.pointcloud_cb)
		#self.pc_image_subscriber = rospy.Subscriber("/camera/depth/image_raw", Image, self.image_callback)
		self.time_publisher = rospy.Publisher('runtime_info', Float32, queue_size=5)

		depth_image = rospy.wait_for_message('/camera/depth/image_raw', Image)
		depth_image = self.bridge_object.imgmsg_to_cv2(depth_image, "32FC1")

		self.last_depth = rospy.wait_for_message('/camera/depth/points', PointCloud2)
		self.depth_1 = np.zeros(((640,480,3)))
		self.depth_2 = np.zeros(((640,480,3)))
		self.previousdepth = np.zeros(((5,640,480,3)))
		self.depth_counter = 0
		rospy.sleep(1)


		'''
		#loads a pointcloud bag cointaining a only the floor
		bag = rosbag.Bag('catkin_ws/src/husky_nmpc_tracker/src/floor.bag')
		for topic, msg, t in bag.read_messages(topics=['/camera/depth/points']):
			#print(msg)
			initial_raw_cloud = list(point_cloud2.read_points(msg, skip_nans=False, field_names = ("x", "y", "z")))
			initial_cloud = np.reshape(initial_raw_cloud,(self.pc_width,self.pc_height,self.pc_depth))
			print ("initial cloud middle:") + str(initial_cloud[640/2][480/2][2])
		bag.close()
		final_cloud = np.zeros(((self.pc_width,self.pc_height,self.pc_depth)))
		for i in range(self.pc_width):
			for j in range(self.pc_height):
				final_cloud[i][j][2] = abs(cloud[i][j][2] - initial_cloud[i][j][2])
		print ("final cloud middle:") + str(final_cloud[640/2][480/2][2])
		'''

#-----------------------------------------------------------------------------
	def pointVariance(self, cloud, x, y):
		point_list = [self.previousdepth[0][x][y][2],self.previousdepth[1][x][y][2], self.previousdepth[2][x][y][2], self.previousdepth[3][x][y][2], self.previousdepth[4][x][y][2]]
		var = np.var(point_list)
		print("Variance = ") + str(var)


#-----------------------------------------------------------------------------
	def busca2(self, cloud, cloud_struct, cv_image, last_pc):

		#pega o meio da lista
		cloud_struct.dad[0], cloud_struct.dad[1] = cloud_struct.dad[0]/2, cloud_struct.dad[1]/2
		cloud_struct.pos_x, cloud_struct.pos_y = cloud_struct.dad[0], cloud_struct.dad[1]
		#print("your dad is ") + str(cloud_struct.dad)
		#verifica se chegou na profundidade maxima
		if cloud_struct.depth2[0] > 5:
			print("No obstacle found")
			return

		#print("depth = ") + str(cloud_struct.depth2[0])

		count = [0,0]
		while count[1] < pow(2,cloud_struct.depth2[0]):

			while count[0] < pow(2,cloud_struct.depth2[0]):

				#print cloud_struct.pos_x, cloud_struct.pos_y
				if cloud[cloud_struct.pos_x][cloud_struct.pos_y][2] < 1.5 and cloud[cloud_struct.pos_x][cloud_struct.pos_y][2] > 0.2:
					#if abs(cloud[cloud_struct.pos_x][cloud_struct.pos_y][2] - last_pc[cloud_struct.pos_x][cloud_struct.pos_y][2]) < 0.1:
					if cv_image[cloud_struct.pos_y][cloud_struct.pos_x] != 0:
						self.pointVariance(cloud,cloud_struct.pos_x,cloud_struct.pos_y)
						print ("flag at ") + str(cloud_struct.pos_x) + (", ") + str(cloud_struct.pos_y)
						print("Value = ") + str(cloud[cloud_struct.pos_x][cloud_struct.pos_y][2])
						self.i = cloud_struct.pos_x
						self.j = cloud_struct.pos_y
						return
				count[0] += 1
				cloud_struct.pos_x = cloud_struct.pos_x + (2 * cloud_struct.dad[0])

			cloud_struct.pos_x = cloud_struct.dad[0]
			count[0] = 0
			count[1] += 1
			cloud_struct.pos_y = cloud_struct.pos_y + (2 * cloud_struct.dad[1])

		#adiciona profundidade.
		cloud_struct.depth2[0] += 1
		#cloud_struct.pos_x = cloud_struct.pos_x - (2 * cloud_struct.dad[0])
		cloud_struct.pos_y = cloud_struct.pos_y - (2 * cloud_struct.dad[1])
		#print("your final position is ") +str(cloud_struct.pos_x) + (", ") +str(cloud_struct.pos_y)
		self.busca2(cloud, cloud_struct,cv_image,last_pc)

#-----------------------------------------------------------------------------
	def busca_linear(self,cloud, cloud_struct, camera):
		for i in range(cloud_struct.pos_x - 1):
			count = 0
			for j in range(cloud_struct.pos_y -1):
				if cloud[i][j][2] < 1.5 and cloud[i][j][2] > 0.2:
					if camera[j][i] != 0:
						self.i = i
						self.j = j
						print("End of linear search. Position ") + str(i) + (", ") +str(j)+ ("   At value ") + str (cloud[i][j][2])
						print("Camera value = ") + str(camera[j][i])
						return

		print("No obstacles were found in linear search")
		return

#-----------------------------------------------------------------------------
	def one_dimensional_search(self, cloud, cloud_struct):

		#pega o meio da lista
		cloud_struct.dad = cloud_struct.dad/2
		cloud_struct.pos_x = cloud_struct.dad
		print("your dad is ") + str(cloud_struct.dad)
		#verifica se chegou na profundidade maxima
		if cloud_struct.depth > 7:
			print("No obstacle found")
			return

		print("depth = ") + str(cloud_struct.depth)

		count = 0
		while count < pow(2,cloud_struct.depth):
			print cloud_struct.pos_x
			if cloud[cloud_struct.pos_x][480/2][2] < 1.5:
				print ("flag at ") + str(cloud_struct.pos_x)
				print("Value = ") + str(cloud[cloud_struct.pos_x][480/2][2])
				return

			count += 1
			cloud_struct.pos_x = cloud_struct.pos_x + (2 * cloud_struct.dad)

		#adiciona profundidade.
		cloud_struct.depth += 1
		cloud_struct.pos_x = cloud_struct.pos_x - (2 * cloud_struct.dad)
		print("your final position is ") +str(cloud_struct.pos_x)
		self.one_dimensional_search(cloud, cloud_struct)

#-----------------------------------------------------------------------------
	def rgb_process(self, data):

		try:
			# We select bgr8 because its the OpneCV encoding by default
			cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
		except CvBridgeError as e:
			print(e)

		hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
		lower_yellow = np.array([10,100,30])
		upper_yellow = np.array([40,255,70])

		lower_gray = np.array([0,0,13])
		upper_gray = np.array([1,1,15])

		lower_roof = np.array([0,0,177])
		upper_roof = np.array([2,2,179])


		mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
		res = cv2.bitwise_and(cv_image,cv_image, mask= mask)

		kernel = np.ones((15,15),np.uint8)
		dilation = cv2.dilate(res,kernel,iterations = 1)

		floor_mask = cv2.inRange(hsv, lower_gray, upper_gray)
		floor_res = cv2.bitwise_and(cv_image,cv_image, mask= floor_mask)

		roof_mask = cv2.inRange(hsv, lower_roof, upper_roof)
		roof_res = cv2.bitwise_and(cv_image,cv_image, mask= roof_mask)

		final_res = floor_res + dilation + roof_res
		grayImage = cv2.cvtColor(final_res, cv2.COLOR_BGR2GRAY)
		blur = cv2.GaussianBlur(grayImage,(5,5),0)
		ret, th = cv2.threshold(blur,10,255,cv2.THRESH_BINARY_INV)

		cv2.circle(cv_image,(self.i, self.j), 10,(0,0,255),-1)

		#cv2.imshow("Original", cv_image)
		#cv2.imshow("Roof", roof_res)
		#cv2.imshow("Res", dilation)
		#cv2.imshow("Floor_Res", floor_res)
		#cv2.imshow("Final", final_res)
		#cv2.imshow("thresh", th)
		#cv2.waitKey(1)


		self.cam_img = cv_image
		self.filtered_img = th
		#print(th[0][475])

#-----------------------------------------------------------------------------
	def image_callback(self, data):

		try:
			# We select bgr8 because its the OpneCV encoding by default
			depth_image = self.bridge_object.imgmsg_to_cv2(data, "32FC1")
		except CvBridgeError as e:
			print(e)

		'''
		hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
		lower_yellow = np.array([10,100,30])
		upper_yellow = np.array([40,255,70])

		lower_gray = np.array([0,0,13])
		upper_gray = np.array([1,1,15])

		lower_roof = np.array([0,0,177])
		upper_roof = np.array([2,2,179])

		mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
		res = cv2.bitwise_and(cv_image,cv_image, mask= mask)

		kernel = np.ones((15,15),np.uint8)
		dilation = cv2.dilate(res,kernel,iterations = 1)

		floor_mask = cv2.inRange(hsv, lower_gray, upper_gray)
		floor_res = cv2.bitwise_and(cv_image,cv_image, mask= floor_mask)

		roof_mask = cv2.inRange(hsv, lower_roof, upper_roof)
		roof_res = cv2.bitwise_and(cv_image,cv_image, mask= roof_mask)

		final_res = floor_res + dilation + roof_res
		grayImage = cv2.cvtColor(final_res, cv2.COLOR_BGR2GRAY)
		blur = cv2.GaussianBlur(grayImage,(5,5),0)
		ret, th = cv2.threshold(blur,10,255,cv2.THRESH_BINARY_INV)

		cv2.circle(cv_image,(self.i, self.j), 10,(0,0,255),-1)
		'''
		#cv2.imshow("deoth", depth_image)
		#cv2.imshow("Roof", roof_res)
		#cv2.imshow("Res", dilation)
		#cv2.imshow("Floor_Res", floor_res)
		#cv2.imshow("Final", final_res)
		#cv2.imshow("thresh", th)
		#cv2.waitKey(1)


		#self.cam_img = cv_image
		#self.filtered_img = th
		#print(th[0][475])

#-----------------------------------------------------------------------------
	def pointcloud_cb(self,data):

		raw_cloud = list(point_cloud2.read_points(data, skip_nans=False, field_names = ("x", "y", "z")))
		cloud = np.reshape(raw_cloud,(data.width,data.height,3))
		teste_cloud_3 = np.reshape(raw_cloud,(data.height,data.width,3))

		raw_last = list(point_cloud2.read_points(self.last_depth, skip_nans=False, field_names = ("x", "y", "z")))
		last_pc_shape = np.reshape(raw_last,(data.height,data.width,3))
		last_pc = np.transpose(last_pc_shape, (1, 0, 2))

		#print ("check if middle is okay: ") + str(cloud[640/2][480/2][2])

		cloud_struct = binary_search_struct()
		cloud_struct.pos_x =  data.width
		cloud_struct.pos_y =  data.height
		cloud_struct.dad =  [data.width,data.height]

		'''print teste_cloud_3[0][0][2], teste_cloud_3[0][320][2], teste_cloud_3[0][638][2]
		print teste_cloud_3[100][0][2], teste_cloud_3[100][320][2], teste_cloud_3[100][638][2]
		print teste_cloud_3[240][0][2], teste_cloud_3[240][320][2], teste_cloud_3[240][638][2]
		print teste_cloud_3[450][0][2], teste_cloud_3[450][320][2], teste_cloud_3[450][638][2]
		'''

		final_cloud = np.transpose(teste_cloud_3, (1, 0, 2))
		
		if self.depth_counter > 4:
			self.depth_counter = 0
		
		print("preenchendo vetor na posicao") + str(self.depth_counter)
		self.previousdepth[self.depth_counter] = final_cloud
		self.depth_counter += 1

		'''
		print final_cloud[0][0][2], final_cloud[320][0][2], final_cloud[638][0][2]
		print final_cloud[0][100][2], final_cloud[320][100][2], final_cloud[638][100][2]
		print final_cloud[0][240][2], final_cloud[320][240][2], final_cloud[638][240][2]
		print final_cloud[0][450][2], final_cloud[320][450][2], final_cloud[638][450][2]

		print '--------'
		print final_cloud[0][0][0], final_cloud[320][0][0], final_cloud[638][0][0]
		print final_cloud[0][100][0], final_cloud[320][100][0], final_cloud[638][100][0]
		print final_cloud[0][240][0], final_cloud[320][240][0], final_cloud[638][240][0]
		print final_cloud[0][450][0], final_cloud[320][450][0], final_cloud[638][450][0]

		print '--------'
		print final_cloud[0][0][1], final_cloud[320][0][1], final_cloud[638][0][1]
		print final_cloud[0][100][1], final_cloud[320][100][1], final_cloud[638][100][1]
		print final_cloud[0][240][1], final_cloud[320][240][1], final_cloud[638][240][1]
		print final_cloud[0][450][1], final_cloud[320][450][1], final_cloud[638][450][1]
		'''
		new_img = self.cam_img
		#print new_img[0][0]
		
		for i in range(640):
			for j in range(480):
				if abs(final_cloud[i][j][2] - last_pc[i][j][2]) < 0.1:
					new_img[j][i] = (0,0,0)
		'''
		rviz_axis_test = self.cam_img
		for i in range(640):
			for j in range(480):
				if final_cloud[i][j][1] < 0.0:
					rviz_axis_test[j][i] = (200,10,10)
		'''
		#cv2.imshow("considered area", new_img)
		#cv2.imshow("axis_test", rviz_axis_test)
		#cv2.waitKey(1)
		#usage = getrusage(RUSAGE_SELF)
		#print(usage.ru_utime)

		'''
		print '--------'
		print cloud[1][1][0], cloud[320][1][0], cloud[635][1][0]
		print cloud[1][240][0], cloud[320][240][0], cloud[635][240][0]
		print cloud[1][400][0], cloud[320][400][0], cloud[635][400][0]
		print cloud[1][475][0], cloud[320][475][0], cloud[635][475][0]
		print '--------'
		print cloud[1][1][1], cloud[320][1][1], cloud[635][1][1]
		print cloud[1][240][1], cloud[320][240][1], cloud[635][240][1]
		print cloud[1][400][1], cloud[320][400][1], cloud[635][400][1]
		print cloud[1][475][1], cloud[320][475][1], cloud[635][475][1]
		print '--------'
		print cloud[1][1][2], cloud[320][1][2], cloud[635][1][2]
		print cloud[1][240][2], cloud[320][240][2], cloud[635][240][2]
		print cloud[1][400][2], cloud[320][400][2], cloud[635][400][2]
		print cloud[1][475][2], cloud[320][475][2], cloud[635][475][2]
		'''
		initial_time = time.time()

		#self.busca_linear(final_cloud, cloud_struct, self.filtered_img)
		self.busca2(final_cloud, cloud_struct, self.filtered_img, last_pc)

		#final_time = time.time()
		#elapsed_time = final_time - initial_time
		#print ("Elapsed time: ") + str(final_time - initial_time)
		#self.time_publisher.publish(elapsed_time)


		self.depth_1 = self.depth_2
		self.depth_2 = last_pc
		self.last_depth = data




def main():
	rospy.init_node('pointcloud_binary_search')
	HeuristicBinary()

	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		rate.sleep


if __name__ == '__main__':
	main()
