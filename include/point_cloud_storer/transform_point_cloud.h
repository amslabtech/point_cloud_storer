#ifndef __TRANSFORM_POINT_CLOUD
#define __TRANSFORM_POINT_CLOUD

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>


class TransformPointCloud
{
public:
    TransformPointCloud(void);

    void process(void);
    void callback(const sensor_msgs::PointCloud2ConstPtr&);

private:
    std::string TARGET_FRAME;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;

    ros::Publisher transformed_cloud_pub;
    ros::Subscriber cloud_sub;

    tf::TransformListener listener;
};

#endif// __TRANSFORM_POINT_CLOUD
