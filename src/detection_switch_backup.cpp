#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Polygon.h"
#include "sensor_msgs/PointCloud2.h"
#include <visual_path_husky/SysStatus.h>

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


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
ros::Publisher detection_pub;


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
    float max_distance = 1.5;
    printf ("Cloud: width = %d, height = %d\n", cloud->width, cloud->height);
    
    /*
    BOOST_FOREACH (const pcl::PointXYZ& pt, cloud->points){
        if(pt.z < max_distance){
            printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
        }
    */
    
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
    
    //xyz_vizualization(segmentation_cloud);

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

    cout<<"Indice = "<< indice << endl;
    ROS_INFO_STREAM(cloud->points[indice].z);

    indice = indice/2;
    int pos = indice;

    ROS_INFO_STREAM(cloud->size());
    
    
    if(depth>max_depth){
        cout<<"no obstacle found. Mac depth reached."<<endl;
        return(false);
    }
    
    int counter = 0;

    while(counter < pow(2,depth)){
        if(cloud->points[pos].z < 1.5){
            cout<<"OBSTACLE"<<endl;
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

    //float min_x, max_x = cloud->points[0].x;
    float min_x = 100000.0;
    float max_x = -10000.0;

    for(int i=0; i< cloud->width * cloud->height; i++){
        if (cloud->points[i].x > max_x)
            max_x = cloud->points[i].x;
        
        if (cloud->points[i].x < min_x )
            min_x = cloud->points[i].x;

    }

    cout<<"max x = "<< max_x << "min_x = " << min_x<<endl;
    
    

    end_ = ros::WallTime::now();end_ = ros::WallTime::now();
    double execution_time = (end_ - start_).toNSec() * 1e-6;
    
    //cout<<"TEMPO EXECUCAO MIN E MAX " << execution_time<<endl;

    geometry_msgs::Polygon obstacle_dimensions;
    obstacle_dimensions.points.resize(2);

    return(obstacle_dimensions);

    
}

void pointcloud_cb(boost::shared_ptr<sensor_msgs::PointCloud2 const> msg)
{    
    //cout<<"pc callback"<<endl;
    ros::WallTime start_, end_;
    start_ = ros::WallTime::now();
    std::cerr << "PointCloud size: " << msg->width<< "x" << msg->height <<endl; 

    sensor_msgs::PointCloud2 Pc;

    if(msg == NULL){
        cout << "Invalid pointcloud!" << endl;
        //return;
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
    
    int indice = (filtered_cloud->size());
    
    if(binary_search(filtered_cloud, indice, 0)){
        geometry_msgs::Polygon obstacle_dimensions = get_size(filtered_cloud);
        detection_pub.publish(obstacle_dimensions);
    }
    
    end_ = ros::WallTime::now();end_ = ros::WallTime::now();
    double execution_time = (end_ - start_).toNSec() * 1e-6;
    
    cout<<"TEMPO EXECUCAO " << execution_time<<endl;
    ROS_INFO_STREAM(execution_time);
    
    

//    BOOST_FOREACH (const pcl::PointXYZ& pt, cloud.points)
//        printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
    
    
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"detection_switch_cpp");
    ros::NodeHandle n;

    ros::WallTime start_, end_, ref1, ref2, ref3, ref4;
    start_ = ros::WallTime::now();
	
    visual_path_husky::SysStatus status;
    
    detection_pub = n.advertise<geometry_msgs::Polygon>("obstacle_dimension",10);
    /*
    boost::shared_ptr<sensor_msgs::PointCloud2 const> sharedPc;
    sensor_msgs::PointCloud2 Pc;
    //sharedPc = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/depth/points", n);
    sharedPc = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("reduced_cloud", n);
    
    start_ = ros::WallTime::now();

    if(sharedPc != NULL){
        Pc = *sharedPc;
        cout << "Valid pointcloud" << endl;
    }
    
    ref1 = ros::WallTime::now();
    double res1 = (ref1 - start_).toNSec() * 1e-6; 
    cout<<"Ref 1 = "<< res1<<endl;   

    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg (Pc, cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptrCloud(&cloud);
    
    pcl::PointCloud<pcl::PointXYZ> xyz_cloud;

    ref2 = ros::WallTime::now();
    double res2 = (ref2 - start_).toNSec() * 1e-6;
    cout<<"Ref 2 = "<< res2 <<endl; 

    pcl::copyPointCloud(cloud, xyz_cloud);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptrXYZCloud(&xyz_cloud);

    ref3 = ros::WallTime::now();
    double res3 = (ref3 - start_).toNSec() * 1e-6;
    cout<<"Ref 3 = "<< res3 <<endl; 

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = floor_filter(ptrXYZCloud);
    int indice = (filtered_cloud->size());
    
    ref4 = ros::WallTime::now();
    double res4 = (ref4 - start_).toNSec() * 1e-6;
    cout<<"Ref 4 = "<< res4 <<endl;

    if(binary_search(filtered_cloud, indice, 0)){
        geometry_msgs::Polygon obstacle_dimensions = get_size(filtered_cloud);
        detection_pub.publish(obstacle_dimensions);
    }
    //xyz_vizualization(filtered_cloud);
    //xyz_vizualization(xyz_cloud);

    end_ = ros::WallTime::now();end_ = ros::WallTime::now();
    double execution_time = (end_ - start_).toNSec() * 1e-6;
    
    cout<<"TEMPO EXECUCAO " << execution_time<<endl;
    ROS_INFO_STREAM(execution_time);
    //xyz_vizualization(filtered_cloud);
    */
    ros::Subscriber pointcloud_sub = n.subscribe("reduced_cloud", 3, pointcloud_cb);
    //ros::Subscriber pointcloud_sub = n.subscribe("/camera/depth/points", 3, pointcloud_cb);

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}