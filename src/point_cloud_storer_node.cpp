#include "point_cloud_storer/point_cloud_storer.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_storer");
    PointCloudStorer point_cloud_storer;
    point_cloud_storer.process();
    return 0;
};
