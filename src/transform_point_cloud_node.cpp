#include "point_cloud_storer/transform_point_cloud.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "transform_point_cloud");
    TransformPointCloud transform_point_cloud;
    transform_point_cloud.process();
    return 0;
};
