#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Polygon.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/UInt8.h"
#include <husky_nmpc_tracker/SysStatus1.h>
#include <husky_nmpc_tracker/DeviationParams.h>
#include <opencv101/imageParams.h>

#include <cmath>
#include <sstream>                  //pq alguns includes saocom <> e outros nao?
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/foreach.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/common/common.h>

#include <unistd.h>
#include <thread>
#include <chrono>

#include <stdio.h>
#include <time.h>
//TODO: colocar camera mais pra baixo 
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::Publisher detection_pub;
ros::Publisher status_pub;

husky_nmpc_tracker::SysStatus1 status;
std_msgs::UInt8 circle_feedback_message;
husky_nmpc_tracker::DeviationParams deviationparams;


void xyz_vizualization(const PointCloud::ConstPtr& cloud){
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "segmentation cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "segmentation cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    while (!viewer->wasStopped ())  {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}


void xyzrgb_visualization(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud){
/*    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptrCloud(&cloud);

    pcl::visualization::PCLVisualizer::Ptr myviewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    myviewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PCLVisualizer::Ptr rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr ptrCloud);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(ptrCloud);
    myviewer->addPointCloud<pcl::PointXYZRGB> (ptrCloud, rgb, "sample cloud");
    myviewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    myviewer->addCoordinateSystem (1.0);
    myviewer->initCameraParameters ();
    
    while (!myviewer->wasStopped ()){
        myviewer->spinOnce (100);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
*/
}


pcl::PointCloud<pcl::PointXYZ>::Ptr floor_filter(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud){
    float max_distance = 0.8;
    //printf ("Cloud: width = %d, height = %d\n", cloud->width, cloud->height);
    
    ros::WallTime start_, end_;
    start_ = ros::WallTime::now();

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> seg;

    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if(inliers->indices.size() == 0){
        PCL_ERROR("could not estimate a planar model");
        //return(&cloud);
    }

   // std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
   //                                   << coefficients->values[1] << " "
   //                                   << coefficients->values[2] << " " 
   //                                   << coefficients->values[3] << std::endl;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr segmentation_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    segmentation_cloud->width = 640;
    segmentation_cloud->height = 320;
    segmentation_cloud->points.resize(segmentation_cloud->width * segmentation_cloud->height);

    //std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
    
    pcl::ExtractIndices<pcl::PointXYZ> filter;
    filter.setInputCloud (cloud);
    filter.setIndices (inliers);
    // Extract the points in cloud_in referenced by indices_in as a separate point cloud:
    filter.filter (*segmentation_cloud);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr negative_segmentation_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    negative_segmentation_cloud->width = 640;
    negative_segmentation_cloud->height = 320;
    negative_segmentation_cloud->points.resize(negative_segmentation_cloud->width * negative_segmentation_cloud->height);

    filter.setNegative (true);
    filter.filter (*negative_segmentation_cloud);
    //filter.filter (cloud);
    //xyz_vizualization(negative_segmentation_cloud);
    
    //cout<<"Size before nan removal:"<< negative_segmentation_cloud->size();    
    //cout<<"Size AFTER nan removal:"<< negative_segmentation_cloud->size();

    //end_ = ros::WallTime::now();end_ = ros::WallTime::now();
    //double execution_time = (end_ - start_).toNSec() * 1e-6;
    //ROS_INFO_STREAM(execution_time);

    return(negative_segmentation_cloud);
    

    
    }


bool binary_search(const PointCloud::ConstPtr& cloud, int indice, int depth){
    int max_depth = 10;

    //cout<<"Indice = "<< indice << endl;
    //ROS_INFO_STREAM(cloud->points[indice].z);

    indice = indice/2;
    int pos = indice;

    //ROS_INFO_STREAM(cloud->size());
      
    if(depth>max_depth){
        //cout<<"no obstacle found. Mac depth reached."<<endl;
        status.distance_to_obstacle = 0.0;
        return(false);
    }
    
    int counter = 0;

    while(counter < pow(2,depth)){
        if(cloud->points[pos].z < 0.8){
            cout<<"OBSTACLE AT "<< pos <<endl;
            ROS_INFO_STREAM(cloud->points[pos].z);
            status.distance_to_obstacle = cloud->points[pos].z;
            return(true);
        }
        counter++;
        pos = pos + 2*indice;
        //cout<<"Position "<<pos<<endl;
    }

    depth++;
    binary_search(cloud, indice, depth);
    
}


geometry_msgs::Polygon get_size(const PointCloud::ConstPtr& cloud){
    ros::WallTime start_, end_;
    start_ = ros::WallTime::now();

    pcl::PointXYZ minPt, maxPt;

    float min_x, max_x = cloud->points[0].x;   

    for(int i=0; i< cloud->width * cloud->height; i++){
        if (cloud->points[i].x > max_x || (isnan(max_x)))
            max_x = cloud->points[i].x;
        
        if (cloud->points[i].x < min_x || (isnan(min_x)))
            min_x = cloud->points[i].x;

    }


    //cout<<"max x = "<< max_x << "min_x = " << min_x<<endl;
    end_ = ros::WallTime::now();end_ = ros::WallTime::now();
    double execution_time = (end_ - start_).toNSec() * 1e-6;
    //cout<<"TEMPO EXECUCAO MIN E MAX " << execution_time<<endl;
     
     status.obstacle_size = max_x - min_x;

    geometry_msgs::Polygon obstacle_dimensions;
    obstacle_dimensions.points.resize(2);

    return(obstacle_dimensions);

    
}


void circle_cb(const std_msgs::UInt8& data){
    circle_feedback_message.data = data.data;

        if (data.data == 1){
            status.flag = true;
            status.complete_circle = false;
        }
            
        if (data.data == 2){
            status.complete_circle = true;
            status.flag = false;
            cout<<"ACABOU O CIRCULO"<<endl;
        }
            

}

void deviation_cb(const husky_nmpc_tracker::DeviationParams& data){
        deviationparams = data;

        if (deviationparams.status == 1){
            status.flag = true;
            status.complete_circle = false;
        }
            
        if (deviationparams.status == 2){
            status.complete_circle = true;
            status.flag = false;
            cout<<"ACABOU O CIRCULO"<<endl;
        }       
}

void camera_cb(const opencv101::imageParams& data){
    status.found_line = true;
    if (data.ponto2[0] == 0 && data.ponto2[1] == 0)
        status.found_line = false;
        
    status_pub.publish(status);
}


void pointcloud_cb(boost::shared_ptr<sensor_msgs::PointCloud2 const> msg)
{    
    ros::WallTime start_, end_;
    start_ = ros::WallTime::now();
    //std::cerr << "PointCloud size: " << msg->width<< "x" << msg->height <<endl; 

    sensor_msgs::PointCloud2 Pc;

    if(msg == NULL){
        cout << "Invalid pointcloud!" << endl;
    }
    Pc = *msg;
    
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg (Pc, cloud);
    
    pcl::PointCloud<pcl::PointXYZ> xyz_cloud;
    pcl::copyPointCloud(cloud, xyz_cloud);
    //pcl::PointCloud<pcl::PointXYZ> *ptrXYZCloud(&xyz_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptrXYZCloud(new pcl::PointCloud<pcl::PointXYZ>);
    *ptrXYZCloud = xyz_cloud;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = floor_filter(ptrXYZCloud);
    
    //std::cerr << "PointCloud size to indice: " << filtered_cloud->width << "x" << filtered_cloud->height <<endl; 
    int indice = (filtered_cloud->width *filtered_cloud->height);
    
    status.flag = false;
    if(indice>1 && binary_search(filtered_cloud, indice, 0)){
        geometry_msgs::Polygon obstacle_dimensions = get_size(filtered_cloud);

        status.flag = true;
        detection_pub.publish(obstacle_dimensions);
    }
    status_pub.publish(status);
    end_ = ros::WallTime::now();end_ = ros::WallTime::now();
    double execution_time = (end_ - start_).toNSec() * 1e-6;
    
    //cout<<"TEMPO EXECUCAO " << execution_time<<endl;
    //ROS_INFO_STREAM(execution_time);    
    //xyz_vizualization(filtered_cloud);

//    BOOST_FOREACH (const pcl::PointXYZ& pt, cloud.points)
//        printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
    
    
}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"detection_switch_cpp");
    ros::NodeHandle n;

    ros::WallTime start_, end_, ref1, ref2, ref3, ref4;
    start_ = ros::WallTime::now();
	
    status_pub = n.advertise<husky_nmpc_tracker::SysStatus1>("safety_controller",10);
    detection_pub = n.advertise<geometry_msgs::Polygon>("obstacle_dimension",10);
    
    //xyz_vizualization(filtered_cloud);
    
    ros::Subscriber pointcloud_sub = n.subscribe("reduced_cloud", 3, pointcloud_cb);
    ros::Subscriber image_sub = n.subscribe("camera_params", 3, camera_cb);
    //ros::Subscriber circle_feedback = n.subscribe("circle_control",3, circle_cb);
    ros::Subscriber deviation_feedback = n.subscribe("deviation_params",3, deviation_cb);
    //ros::Subscriber pointcloud_sub = n.subscribe("/camera/depth/points", 3, pointcloud_cb);

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}