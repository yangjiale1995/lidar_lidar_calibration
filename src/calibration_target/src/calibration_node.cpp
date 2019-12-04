#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <boost/foreach.hpp>
#include <iostream>

using namespace message_filters;

pcl::PointCloud<pcl::PointXYZ> laser1;
pcl::PointCloud<pcl::PointXYZ> laser2;

// pointcloud callback
void callback(const sensor_msgs::PointCloud2::ConstPtr &msg1, const sensor_msgs::PointCloud2::ConstPtr &msg2)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr laser_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg1, *laser_ptr);
    laser1 += *laser_ptr;
    pcl::fromROSMsg(*msg2, *laser_ptr);
    laser2 += *laser_ptr;

    if(laser1.points.size() >= 8 && laser2.points.size() >= 8)
    {
        pcl::registration::TransformationEstimationLM<pcl::PointXYZ, pcl::PointXYZ, float> lm;
        Eigen::Matrix4f guess;
        lm.estimateRigidTransformation(laser1, laser2, guess);
        std::cout << guess << std::endl;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "registration");
    ros::NodeHandle nh;
    
    message_filters::Subscriber<sensor_msgs::PointCloud2> laser0_sub(nh, "/corner1", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> laser1_sub(nh, "/corner2", 1);
    typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), laser0_sub, laser1_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();

    return 0;
}

