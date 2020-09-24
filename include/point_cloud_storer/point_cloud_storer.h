#ifndef __POINT_CLOUD_STORER
#define __POINT_CLOUD_STORER

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/tf.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// PCL
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

// OMP
#include <omp.h>


class PointCloudStorer
{
public:
    typedef pcl::PointXYZI PointXYZI;
    typedef pcl::PointCloud<PointXYZI> CloudXYZI;
    typedef pcl::PointCloud<PointXYZI>::Ptr CloudXYZIPtr;

    PointCloudStorer(void);

    void process(void);
    void callback(const sensor_msgs::PointCloud2ConstPtr&, const nav_msgs::OdometryConstPtr&);

private:
    int STORE_NUM;
    std::string TARGET_FRAME;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;

    ros::Publisher stored_cloud_pub;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> sync_subs;
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub;
    message_filters::Synchronizer<sync_subs> sync;

    CloudXYZIPtr cloud;
    bool first_flag;
    std::list<int> cloud_size_list;

    nav_msgs::Odometry last_odom;
};

#endif// __POINT_CLOUD_STORER
