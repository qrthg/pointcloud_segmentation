// Related header
/// i.e. ROS specific 
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <visualization_msgs/MarkerArray.h>
// #include <tf2/btQuaternion.h>

// PCL specific includes
#define PCL_NO_PRECOMPILE
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>

#include <pcl/kdtree/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <iostream>

#define PointT pcl::PointXYZI
#define PointCloudT pcl::PointCloud<PointT>
#define PointCloudPtrT pcl::PointCloud<PointT>::Ptr 


void segmentCylinder(const PointCloudPtrT& cloud, 
                           PointCloudPtrT& cloud_cyl, 
                           PointCloudPtrT& cloud_other,
                           pcl::ModelCoefficients::Ptr& coefficients_cylinder )
{
    // Based on: http://pointclouds.org/documentation/tutorials/cylinder_segmentation.php
    
    // All the objects needed
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
    pcl::ExtractIndices<PointT> extract;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

    // Datasets  
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.025);
    seg.setRadiusLimits (0.05, 0.20);
    seg.setInputCloud (cloud);
    seg.setInputNormals (cloud_normals);

    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers_cylinder, *coefficients_cylinder);

    // Extract the cylinder and others pointclouds
    extract.setInputCloud (cloud);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);
    extract.filter (*cloud_cyl);
    extract.setNegative (true);
    extract.filter (*cloud_other);
  
}
visualization_msgs::MarkerArray publish_cylinder(const sensor_msgs::PointCloud2ConstPtr& pcMsg, 
                      const PointCloudPtrT& cloud_cylinder,
                      const pcl::ModelCoefficients::Ptr& coefficients_cylinder)
{
//     Eigen::Vector3f position=Eigen::Vector3f(coefficients_cylinder->values[0],coefficients_cylinder->values[1],coefficients_cylinder->values[2]);
//     Eigen::Vector3f orientation=Eigen::Vector3f(coefficients_cylinder->values[5], coefficients_cylinder->values[4], coefficients_cylinder->values[3]);
//     orientation.normalize();
    float radius=coefficients_cylinder->values[6];
    
    std::cerr << "Cylinder radius (m): " << radius << std::endl;
//             btQuaternion quart;
//             quart.setEulerZYX(orientation[0],orientation[1],orientation[2]);
    
    float max_z = - 1000.0f;
    float min_z =   1000.0f;
    float mean_x, mean_y, mean_z;
    mean_x = mean_y = mean_z = 0;
    
    // calc center of visible part of the cylinder
    BOOST_FOREACH (PointT& pt, cloud_cylinder->points) {
        mean_x += pt.x;
        mean_y += pt.y;
        mean_z += pt.z;
        if(pt.z > max_z)
            max_z = pt.z;
        if(pt.z < min_z)
            min_z = pt.z;
    }
    mean_x /= cloud_cylinder->points.size();
    mean_y /= cloud_cylinder->points.size();
    mean_z /= cloud_cylinder->points.size();
        
    Eigen::Vector3d pseudo_center(0, 0, 0);
    pseudo_center(0) = mean_x;
    pseudo_center(1) = mean_y;
    pseudo_center(2) = mean_z;
    
    Eigen::Vector3d central_axis(coefficients_cylinder->values[3], coefficients_cylinder->values[4], coefficients_cylinder->values[5]);

    // Project the pseduo_center onto the central_axis
    // https://en.wikipedia.org/wiki/Vector_projection
    Eigen::Vector3d point_on_central_axis(coefficients_cylinder->values[0], coefficients_cylinder->values[1], coefficients_cylinder->values[2]);
    Eigen::Vector3d pseudo_center_vector = pseudo_center - point_on_central_axis;
    Eigen::Vector3d pseudo_center_vector_projection = pseudo_center_vector.dot(central_axis) / central_axis.dot(central_axis) * central_axis;
    Eigen::Vector3d center = pseudo_center_vector_projection + point_on_central_axis;

    visualization_msgs::MarkerArray ma;
    visualization_msgs::Marker m;
    m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    m.action = visualization_msgs::Marker::ADD; 
    m.header.frame_id = pcMsg->header.frame_id;
    m.header.stamp = pcMsg->header.stamp;
    m.text = std::string("Diam ")+ std::to_string(radius*2.0f) + std::string(" (m)");
    m.color.r = 1.0;
    m.color.g = 1.0;
    m.color.b = 1.0;
    m.color.a = 1.0;
    m.scale.x = .10;
    m.scale.y = .10;
    m.scale.z = .10;
    m.pose.position.x = mean_x;
    m.pose.position.y = mean_y;
    m.pose.position.z = max_z+0.15; 
    m.id = 0;
    ma.markers.push_back (m);
    
    m.type = visualization_msgs::Marker::CYLINDER;
    m.action = visualization_msgs::Marker::ADD; 
    m.pose.position.x = center(0);
    m.pose.position.y = center(1);
    m.pose.position.z = center(2);/*
    m.pose.position.x = mean_x+radius;
    m.pose.position.y = mean_y;
    m.pose.position.z = mean_z;*/
    m.pose.orientation.x = coefficients_cylinder->values[3]; //quart.getX();
    m.pose.orientation.y = coefficients_cylinder->values[4]; //quart.getY();
    m.pose.orientation.z = coefficients_cylinder->values[5]; //quart.getZ(); 
    m.pose.orientation.w = 1; //quart.getW(); 
    m.scale.x = radius*2.0f;
    m.scale.y = radius*2.0f;
    m.scale.z = max_z-min_z;
    m.id = 1;
    ma.markers.push_back (m);
    return ma;
}

//////////////////////////////        
class PointcloudFilter
{
    public:
        PointcloudFilter() : nh_()
        {
            // Publishers
            pub1          = nh_.advertise<sensor_msgs::PointCloud2>  ("cloud_cyl_out", 1);
            pub2          = nh_.advertise<sensor_msgs::PointCloud2>  ("cloud_other_out", 1);
            cylinder_info = nh_.advertise<visualization_msgs::MarkerArray>("cyl_info", 1);
            
            // Subscribers
            m_pointCloudSub_   = new message_filters::Subscriber<sensor_msgs::PointCloud2> (nh_, "cloud_in", 1);
            m_pointCloudSub_->registerCallback(boost::bind(&PointcloudFilter::pointCloudMsgReceived, this,_1));
            
        };

    private:
        ros::NodeHandle nh_;
        message_filters::Subscriber<sensor_msgs::PointCloud2>* m_pointCloudSub_;
        
        ros::Publisher pub1, pub2, cylinder_info;
                         
        
        //  Callback called when pointclouds are available
        void pointCloudMsgReceived(const sensor_msgs::PointCloud2ConstPtr& pcMsg) 
        {                
            // Convert from message to pointcloud
            PointCloudPtrT cloud_in  (new PointCloudT); 
            pcl::fromROSMsg (*pcMsg, *cloud_in);   
            ros::Time stamp = pcMsg->header.stamp;
                                    
            // Init outputs
            PointCloudPtrT cloud_other (new PointCloudT);
            PointCloudPtrT cloud_cylinder (new PointCloudT);
            pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
            
            // Segment cylinder
            segmentCylinder(cloud_in, cloud_cylinder, cloud_other, coefficients_cylinder);
            
            // Publish the filtered clouds
            // Convert back to ros message
            cylinder_info.publish(publish_cylinder(pcMsg, cloud_cylinder, coefficients_cylinder));
    
            // cylinder
            sensor_msgs::PointCloud2 pcMsgOut;
            pcl::toROSMsg (*cloud_cylinder, pcMsgOut);
            pcMsgOut.header.frame_id = pcMsg->header.frame_id;
            pcMsgOut.header.stamp = stamp;
            pub1.publish (pcMsgOut);
            
            // Non cylinder
            pcl::toROSMsg (*cloud_other, pcMsgOut);
            pcMsgOut.header.frame_id = pcMsg->header.frame_id;
            pcMsgOut.header.stamp = stamp;
            pub2.publish (pcMsgOut);
        }
};

/////////////////////////////////////////////////////////////////////
// Main function, initiate package
int main(int argc, char **argv) {
     
    ros::init(argc, argv, "cyl_seg_node");
    PointcloudFilter pcf;

    ROS_INFO_STREAM("Pointcloud ground cylinder segmentation initiated...");

    // Go!
    ros::spin();
  
}
