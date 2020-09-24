#include "point_cloud_storer/transform_point_cloud.h"

TransformPointCloud::TransformPointCloud(void)
:local_nh("~")
{
    local_nh.param("TARGET_FRAME", TARGET_FRAME, {"lidar_link"});

    transformed_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud/transformed", 1);
    cloud_sub = nh.subscribe("cloud", 1, &TransformPointCloud::callback, this);
}

void TransformPointCloud::callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    std::cout << "=== transorm point cloud ===" << std::endl;

    // transform cloud
    sensor_msgs::PointCloud pc_in;
    sensor_msgs::PointCloud pc_trans;
    sensor_msgs::PointCloud2 pc2_trans;

    sensor_msgs::convertPointCloud2ToPointCloud(*msg, pc_in);

    try{
        listener.waitForTransform(TARGET_FRAME, msg->header.frame_id, msg->header.stamp, ros::Duration(1.0));
        listener.transformPointCloud(TARGET_FRAME, msg->header.stamp, pc_in, msg->header.frame_id, pc_trans);
        sensor_msgs::convertPointCloudToPointCloud2(pc_trans, pc2_trans);
        transformed_cloud_pub.publish(pc2_trans);
    }catch (tf::TransformException& ex) {
        ROS_WARN("TF exception:\n%s", ex.what());
    }

}

void TransformPointCloud::process(void)
{
    ros::spin();
}
