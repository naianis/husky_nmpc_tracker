#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Polygon.h"
#include "sensor_msgs/PointCloud2.h"
#include<iostream>
#include <sstream>                  
#include <boost/foreach.hpp>
#include <unistd.h>
#include <thread>
#include <chrono>

using namespace std;

void switch_cb(const geometry_msgs::Polygon::ConstPtr& msg){
    cout<<"Message recieved: "<<endl;
    //ROS_INFO_STREAM(msg.points);

}



int main(int argc, char **argv)
{
    ros::init(argc,argv,"obstacle_avoidance");
    ros::NodeHandle n;

    //ros::Publisher detection_pub = n.advertise<geometry_msgs::Polygon>("obstacle_dimension",1000);

    ros::Subscriber switch_sub = n.subscribe("obstacle_dimension", 1000, switch_cb);
    
    geometry_msgs::Polygon fake_obstacle;
    fake_obstacle.points.resize(2);
    fake_obstacle.points[0].x = -0.209582;
    fake_obstacle.points[1].x = 0.293163;
    fake_obstacle.points[0].y = -0.487265;
    fake_obstacle.points[1].y = 0.406777;
    fake_obstacle.points[0].z = 1.03303;
    fake_obstacle.points[1].z = 1.48649;
  

    

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        //detection_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}