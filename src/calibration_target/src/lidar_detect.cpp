#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include "detect_points.h"

ros::Publisher pub;

void callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *laser_ptr);
    DetectPoints dp(-20, 20, 1000.0, 6000.0, 123);
    dp.setLength(1150);
    dp.setWidth(900);
    dp.setPointCloud(laser_ptr);

    pcl::PointCloud<pcl::PointXYZ> corner;

    std::vector<Eigen::Vector3f> points;
    if(dp.getCornerPoints(points))
    {
        for(int i = 0; i < points.size(); i ++)
        {
            pcl::PointXYZ point;
            point.x = points[i](0);
            point.y = points[i](1);
            point.z = points[i](2);
            corner.push_back(point);
        }

        sensor_msgs::PointCloud2 newMsg;
        pcl::toROSMsg(corner, newMsg);
        newMsg.header = msg->header;
        pub.publish(newMsg);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_lidar_calibration");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne", 100, callback);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/corner", 100);
    
    ros::spin();
    return 0;
}

