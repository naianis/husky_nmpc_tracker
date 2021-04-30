#! /usr/bin/env python
import rospy
import math
import tf
import time
from sensor_msgs.msg import LaserScan
from visual_path_husky.msg import SysStatus
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int32, UInt8, Float32
from opencv101.msg import imageParams


class SafetyController(object):

    def __init__(self):
        self.safety_publisher = rospy.Publisher('safety_controller', SysStatus, queue_size=5)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.scanCallback )
        self.circle_feedback = rospy.Subscriber('/circle_control', UInt8, self.circleCallback)
        self.camera_feedback = rospy.Subscriber('/camera_params', imageParams, self.cameraCb)

        self.time_publisher = rospy.Publisher('runtime_info', Float32, queue_size=5)

        #self.line_feedback = rospy.Subscriber('/line_control', Bool, self.lineCallback)

        self.scan_message = LaserScan()
        self.status = SysStatus()
        self.circle_feedback_message = UInt8()
        self.line_feedback_message = Bool()

        self.circle_feedback_message = 0

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
    def getPosition(self, flag_keeper):

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


    def scanCallback(self, scan_data):
        if len(scan_data.ranges) < 270: # those reading are somehow wrong
            return

        else:
            initial_time = time.time()
            global flag_keeper
            flag_keeper = self.status.flag
            angle_max = math.pi/6

            #calculates the index of data of interest [-pi/6, pi/6]
            index = int (angle_max / scan_data.angle_increment)
            size = len(scan_data.ranges)
            self.status.flag = False
            
            # search obstacles between angles of interest
            for i in range(710):
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
            
            final_time = time.time()
            elapsed_time = final_time - initial_time
            self.time_publisher.publish(elapsed_time)

            if self.circle_feedback_message == 1:
                #rospy.loginfo("Circle Flag activated")
                self.status.flag = True

            self.safety_publisher.publish(self.status)



def main():

    rospy.init_node('safety_controller')

    control_obj = SafetyController()
    rospy.spin()

if __name__ == '__main__':
    main()
