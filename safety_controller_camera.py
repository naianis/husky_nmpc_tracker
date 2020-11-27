#! /usr/bin/env python
import rospy
import math
import tf
import time
import cv2
import rosbag
import ctypes
import ros_numpy
import numpy as np
from sensor_msgs.msg import LaserScan
from visual_path_husky.msg import SysStatus
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int32, UInt8
from opencv101.msg import imageParams
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image

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


class SafetyController(object):

    def __init__(self):
        self.safety_publisher = rospy.Publisher('safety_controller', SysStatus, queue_size=5)
        #self.sub = rospy.Subscriber('/scan', LaserScan, self.scanCallback )
        self.circle_feedback = rospy.Subscriber('/circle_control', UInt8, self.circleCallback)
        self.camera_feedback = rospy.Subscriber('/camera_params', imageParams, self.cameraCb)
        #self.line_feedback = rospy.Subscriber('/line_control', Bool, self.lineCallback)

        self.scan_message = LaserScan()
        self.status = SysStatus()
        self.circle_feedback_message = UInt8()
        self.line_feedback_message = Bool()

        self.circle_feedback_message = 0


        self.bridge_object = CvBridge()
        self.filtered_img = np.zeros((480,640))

        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.rgb_process)
        self.pc_points_subscriber = rospy.Subscriber("/camera/depth/points", PointCloud2, self.pointcloud_cb)



    def busca2(self, cloud, cloud_struct, cv_image):
        self.status.flag = False
        #pega o meio da lista
        cloud_struct.dad[0], cloud_struct.dad[1] = cloud_struct.dad[0]/2, cloud_struct.dad[1]/2
        cloud_struct.pos_x, cloud_struct.pos_y = cloud_struct.dad[0], cloud_struct.dad[1]
        #print("your dad is ") + str(cloud_struct.dad)
        #verifica se chegou na profundidade maxima
        if cloud_struct.depth2[0] > 6:
            print("No obstacle found")
            return

        #print("depth = ") + str(cloud_struct.depth2[0])

        count = [0,0]
        while count[1] < pow(2,cloud_struct.depth2[0]):
            while count[0] < pow(2,cloud_struct.depth2[0]):

                #print cloud_struct.pos_x, cloud_struct.pos_y
                if cloud[cloud_struct.pos_x][cloud_struct.pos_y][2] < 1.5 and cloud[cloud_struct.pos_x][cloud_struct.pos_y][2] > 0.2:
                    if cv_image[cloud_struct.pos_y][cloud_struct.pos_x] != 0:
                        print ("flag at ") + str(cloud_struct.pos_x) + (", ") + str(cloud_struct.pos_y)
                        print("Value = ") + str(cloud[cloud_struct.pos_x][cloud_struct.pos_y][2])
                        self.i = cloud_struct.pos_x
                        self.j = cloud_struct.pos_y
                        self.getVirtualSize(cloud,cv_image, [cloud_struct.pos_x,cloud_struct.pos_y])
                        self.getPosition()
                        self.status.distance_to_obstacle = cloud[cloud_struct.pos_x][cloud_struct.pos_y][2]
                        
                        self.status.flag = True
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
        self.busca2(cloud, cloud_struct,cv_image)


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


    #calculates the estimated size of the obstacle
    def obstacleSize(self, scan_message, first_index):
        i = first_index

        while scan_message.ranges[i] < 5 and i < 720:
            i += 1

        last_index = i - 1
        angle = (last_index - first_index) * scan_message.angle_increment
        #cos law
        virtual_size = math.sqrt (pow(scan_message.ranges[first_index],2) + pow(scan_message.ranges[last_index],2) - 2 * scan_message.ranges[first_index] * scan_message.ranges[last_index] * math.cos(angle))
        self.status.obstacle_size = virtual_size
        return virtual_size


    #get position of the robot when an obstacle is found
    def getPosition(self):

        #print("getting position")
        position = rospy.wait_for_message('/odometry/filtered', Odometry)

        quaternion = (position.pose.pose.orientation.x, position.pose.pose.orientation.y, position.pose.pose.orientation.z, position.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)

        self.status.initial_position = []
        self.status.initial_position.append(position.pose.pose.position.x)
        self.status.initial_position.append(position.pose.pose.position.y)
        self.status.initial_position.append( euler[2])

        #print (self.status.initial_position)


    def getDistance(self, current_index, ranges):
        smaller_distance = ranges[current_index]
        i = current_index
        for i in range(len(ranges)):
            if ranges[i] < smaller_distance:
                smaller_distance = ranges[i]

        return smaller_distance


    def cameraCb(self, data):
        self.status.found_line = True
        if data.ponto2 == (0,0) and data.ponto3 == (0,0):
            self.status.found_line = False
        self.safety_publisher.publish(self.status)


    def circleCallback(self, circle_msg):
        self.circle_feedback_message = circle_msg.data

        if circle_msg.data == 1:
            self.status.flag = True
            self.status.complete_circle = False

        if circle_msg.data == 2:
            self.status.complete_circle = True
            self.status.flag = False
            rospy.logerr("ACABOU O CIRCULO")

 
    def getVirtualSize(self,cloud,cv_image,index):
        print '-------'
        print cloud[400][index[1]][2]

        for i in range(640):
            if cloud[i][index[1]][2] < 1.5 and cv_image[index[1]][i] != 0:
                initial_pixel = i
                print 'first value found at', i, ': ', cloud[i][index[1]][2]
                break

        last_pixel = 639
        for j in range(initial_pixel, 640):
            if cloud[j][index[1]][2] < 1.5  and cv_image[index[1]][j] == 0:
                last_pixel = j
                break

        print 'last PIXEL found at', last_pixel

        incremental_angle = 0.994838/640
        final_angle = (last_pixel - initial_pixel)*incremental_angle
        print 'final_angle', final_angle
        side_1 = math.sqrt(pow(cloud[initial_pixel][index[1]][0],2) + pow(cloud[initial_pixel][index[1]][2],2))
        side_2 = math.sqrt(pow(cloud[last_pixel][index[1]][0],2) + pow(cloud[last_pixel][index[1]][2],2))
        print 'sides 1 and 2: = ', side_1, ', ', side_2 
        virtual_size = math.sqrt (pow(side_1,2) + pow(side_2,2) - (2 * side_1 * side_2 * math.cos(final_angle)))
        print virtual_size
        print '---------'
        
        self.status.obstacle_size = virtual_size


    def scanCallback(self, scan_data):
        if len(scan_data.ranges) < 270: # those reading are somehow wrong
            return

        else:
            global flag_keeper
            flag_keeper = self.status.flag
            angle_max = math.pi/6

            #calculates the index of data of interest [-pi/6, pi/6]
            index = int (angle_max / scan_data.angle_increment)
            size = len(scan_data.ranges)
            self.status.flag = False

            # search obstacles between angles of interest
            for i in range(index*2):
                current_index = int(size/2 - index + i)
                if scan_data.ranges[current_index] < 0.7 and scan_data.ranges[current_index] > 0.1:
                    size = self.obstacleSize(scan_data, current_index)
                    self.status.distance_to_obstacle = self.getDistance(current_index,scan_data.ranges)
                    if(size > 0.1):
                        self.status.flag = True
                        self.getPosition(flag_keeper)
                        self.obstacleSize(scan_data, current_index)
                        #rospy.loginfo("Obstacle found")
                        break


            if self.circle_feedback_message == 1:
                #rospy.loginfo("Circle Flag activated")
                self.status.flag = True

            self.safety_publisher.publish(self.status)


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

        #cv2.circle(cv_image,(self.i, self.j), 10,(0,0,255),-1)

        #cv2.imshow("Original", cv_image)
        #cv2.imshow("Roof", roof_res)
        #cv2.imshow("Res", dilation)
        #cv2.imshow("Floor_Res", floor_res)
        #cv2.imshow("Final", final_res)
        cv2.imshow("thresh", th)
        cv2.waitKey(1)


        self.cam_img = cv_image
        self.filtered_img = th
        #print(th[0][475])


    def pointcloud_cb(self,data):

        raw_cloud = list(point_cloud2.read_points(data, skip_nans=False, field_names = ("x", "y", "z")))
        u_cloud = np.reshape(raw_cloud,(data.height,data.width,3))
        cloud = np.transpose(u_cloud, (1, 0, 2))

        #print ("check if middle is okay: ") + str(cloud[640/2][480/2][2])

        cloud_struct = binary_search_struct()
        cloud_struct.pos_x =  data.width
        cloud_struct.pos_y =  data.height
        cloud_struct.dad =  [data.width,data.height]

        initial_time = time.time()
        #self.busca_linear(cloud, cloud_struct, self.filtered_img)
        self.busca2(cloud, cloud_struct, self.filtered_img)
        final_time = time.time()
        #print ("Elapsed time: ") + str(final_time - initial_time)

        if self.circle_feedback_message == 1:
            rospy.loginfo("Circle Flag activated")
            self.status.flag = True

        self.safety_publisher.publish(self.status)
        
        


def main():

    rospy.init_node('safety_controller')

    control_obj = SafetyController()
    rospy.spin()

if __name__ == '__main__':
    main()
