#include "ros/ros.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/foreach.hpp>


#include "std_msgs/String.h"
#include "geometry_msgs/Polygon.h"
#include "sensor_msgs/PointCloud2.h"

#include <pcl/common/common.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>



ros::Publisher reduced_cloud;


void pointcloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){

    pcl::PCLPointCloud2 pcl_pc2;

    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::PCLPointCloud2::Ptr ptr_pcl_pc2; (&pcl_pc2);
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
    pcl::copyPointCloud(pcl_pc2, *cloud);

    //std::cerr << "PointCloud before filtering: " << pcl_pc2.width * pcl_pc2.height 
    //    << " data points (" << pcl::getFieldsList (pcl_pc2) << ")." << std::endl;

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.02f, 0.02f, 0.02f);
    sor.filter (*cloud_filtered);

    //std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
    //    << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;
    
    reduced_cloud.publish(*cloud_filtered);

   
   
    }

int main (int argc, char** argv){
    ros::init(argc,argv,"pc_downsampler");
    ros::NodeHandle n;
    ros::Subscriber pointcloud_sub = n.subscribe("/camera/depth/points", 3, pointcloud_cb);

    reduced_cloud = n.advertise<sensor_msgs::PointCloud2>("reduced_cloud", 2);
  
    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return (0);
}