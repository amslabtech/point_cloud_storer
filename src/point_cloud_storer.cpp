#include "point_cloud_storer/point_cloud_storer.h"

PointCloudStorer::PointCloudStorer(void)
:local_nh("~"), cloud_sub(nh, "cloud", 10), odom_sub(nh, "odom", 10)
, sync(sync_subs(10), cloud_sub, odom_sub)
{
    local_nh.param("STORE_NUM", STORE_NUM, {20});
    local_nh.param("TARGET_FRAME", TARGET_FRAME, {"lidar_link"});

    stored_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud/stored", 1);

    sync.registerCallback(boost::bind(&PointCloudStorer::callback, this, _1, _2));

    cloud = CloudXYZIPtr(new CloudXYZI);

    first_flag = true;

    std::cout << "STORE_NUM: " << STORE_NUM << std::endl;
}

void PointCloudStorer::callback(const sensor_msgs::PointCloud2ConstPtr& msg_cloud, const nav_msgs::OdometryConstPtr& msg_odom)
{
    std::cout << "=== point cloud storer ===" << std::endl;
    double start = ros::Time::now().toSec();

    nav_msgs::Odometry current_odom = *msg_odom;

    // transform cloud
    CloudXYZIPtr temp_cloud(new CloudXYZI);
    pcl::fromROSMsg(*msg_cloud, *temp_cloud);
    int cloud_size = temp_cloud->points.size();
    std::cout << "new cloud size: " <<  cloud_size << std::endl;

    if(!first_flag){
        tf::Quaternion current_pose, last_pose;
        quaternionMsgToTF(current_odom.pose.pose.orientation, current_pose);
        quaternionMsgToTF(last_odom.pose.pose.orientation, last_pose);
        tf::Quaternion relative_rotation = last_pose * current_pose.inverse();
        relative_rotation.normalize();
        Eigen::Quaternionf rotation(relative_rotation.w(), relative_rotation.x(), relative_rotation.y(), relative_rotation.z());
        tf::Quaternion q_global_move(
            last_odom.pose.pose.position.x - current_odom.pose.pose.position.x,
            last_odom.pose.pose.position.y - current_odom.pose.pose.position.y,
            last_odom.pose.pose.position.z - current_odom.pose.pose.position.z,
            0.0
        );
        tf::Quaternion q_local_move = last_pose.inverse() * q_global_move * last_pose;
        Eigen::Vector3f offset(q_local_move.x(), q_local_move.y(), q_local_move.z());

        pcl::transformPointCloud(*cloud, *cloud, offset, rotation);

        // store pointcloud
        *cloud += *temp_cloud;
        cloud_size_list.push_back(cloud_size);
        if(cloud_size_list.size() > STORE_NUM){
            cloud->points.erase(cloud->points.begin(), cloud->points.begin() + *(cloud_size_list.begin()));
            cloud_size_list.pop_front();
        }
        std::cout << "stored cloud size: " <<  cloud->points.size() << std::endl;
        std::cout << "stored clouds num: " <<  cloud_size_list.size() << std::endl;
        for(const auto& s : cloud_size_list){
            std::cout << s << ", ";
        }
        std::cout << std::endl;
    }else{
        *cloud = *temp_cloud;
        cloud_size_list.push_back(cloud_size);
        first_flag = false;
    }
    cloud->header = temp_cloud->header;
    last_odom = current_odom;

    // publish
    sensor_msgs::PointCloud2 publish_cloud;
    cloud->width = cloud->points.size();
    cloud->height = 1;
    pcl::toROSMsg(*cloud, publish_cloud);
    publish_cloud.header.stamp = current_odom.header.stamp;
    publish_cloud.header.frame_id = TARGET_FRAME;
    stored_cloud_pub.publish(publish_cloud);

    std::cout << "time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
}

void PointCloudStorer::process(void)
{
    ros::spin();
}
