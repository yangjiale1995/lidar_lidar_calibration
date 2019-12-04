#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/PointIndices.h>
#include <pcl/common/angles.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/plane_clipper3D.h>
#include <pcl/filters/extract_indices.h>

#include <iostream>
#include "ndt_omp.h"

const int N_SCANS = 16;     //16线雷达
const int N_HORIZONS = 2000;    //水平分辨率0.18,一圈2000个点
const int groundScanID = 7;     //地面点主要分布在下8线
const int pointThresh = 100;    //地面点个数最小值

using namespace message_filters;


//标定类
class Calibration
{
    public:
        //构造函数 
        Calibration(ros::NodeHandle nh) 
        {
            //激光雷达topic,下雷达线数
            nh.param("top_lidar_topic", top_topic_, std::string("/top_topic"));
            nh.param("front_lidar_topic", front_topic_, std::string("/front_topic"));
            nh.param("front_scans", front_scans_, 16);

            ROS_INFO_STREAM("top_lidar_topic: " + top_topic_);
            ROS_INFO_STREAM("front_lidar_tpic: " + front_topic_);
            ROS_INFO_STREAM("front_scans: " + std::to_string(front_scans_));

            //时间同步器
            message_filters::Subscriber<sensor_msgs::PointCloud2> front_sub(nh, front_topic_, 1);
            message_filters::Subscriber<sensor_msgs::PointCloud2> top_sub(nh, top_topic_, 1);
            typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
            Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), front_sub, top_sub);
            sync.registerCallback(boost::bind(&Calibration::callback, this, _1, _2));
           
            //平面提取类参数设置
            segment_.setOptimizeCoefficients(true);
            segment_.setDistanceThreshold(0.1);
            segment_.setMaxIterations(5000);
            segment_.setProbability(0.8);
            segment_.setModelType(pcl::SACMODEL_PLANE);
            segment_.setMethodType(pcl::SAC_RANSAC);
        
            ros::spin();
        }

        //回调函数
        void callback(const sensor_msgs::PointCloud2::ConstPtr &front_msg, const sensor_msgs::PointCloud2::ConstPtr &top_msg)
        {
            std::cout << ".................." << std::endl;
            pcl::PointCloud<pcl::PointXYZI> front_laser, top_laser;
            pcl::fromROSMsg(*front_msg, front_laser);
            pcl::fromROSMsg(*top_msg, top_laser);
            
            //剔除NAN点
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(front_laser, front_laser, indices);
            pcl::removeNaNFromPointCloud(top_laser, top_laser, indices);

            //32线的激光雷达需要旋转90度确保z轴朝上
            Eigen::Matrix4f pre_transformation = Eigen::Matrix4f::Identity();
            if(front_scans_ == 32)
            {
                Eigen::AngleAxisf unit_y(pcl::deg2rad(90.0), Eigen::Vector3f::UnitY());
                pre_transformation.block<3,3>(0,0) = unit_y.matrix();
            }
            pcl::transformPointCloud(front_laser, front_laser, pre_transformation);
       
            //点云滤波
            pcl::PointCloud<pcl::PointXYZI> front_filtered, top_filtered;
            front_filtered = filter(front_laser);
            top_filtered = filter(top_laser);

            pcl::io::savePCDFile("front.pcd", front_filtered);
            pcl::io::savePCDFile("top.pcd", top_filtered);

            //地面检测
            Eigen::Vector4f front_ground = detectGroundFront(front_filtered); 
            Eigen::Vector4f top_ground = detectGroundTop(top_filtered);

            if(front_ground(3) == 0 || top_ground(3) == 0)
                return;

            Eigen::Vector3f front_ground_normal = front_ground.head(3);
            Eigen::Vector3f top_ground_normal = top_ground.head(3);

            //计算初始值
            float angle = acos(front_ground_normal.dot(top_ground_normal));
            Eigen::Vector3f rotation_axis = front_ground_normal.cross(top_ground_normal);
            rotation_axis.normalize();
            Eigen::AngleAxisf q(angle, rotation_axis);

            Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
            guess.block<3,3>(0,0) = q.matrix();
            guess(2,3) = front_ground(3) - top_ground(3);

            pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
            ndt.setResolution(1.0);
            ndt.setNumThreads(omp_get_num_procs() * 2);
            ndt.setStepSize(1.0);
            ndt.setTransformationEpsilon(1e-6);
            ndt.setMaximumIterations(100000);
            ndt.setNeighborhoodSearchMethod(pclomp::KDTREE);

            ndt.setInputSource(front_filtered.makeShared());
            ndt.setInputTarget(top_filtered.makeShared());

            ndt.align(front_filtered, guess);

            guess = ndt.getFinalTransformation();
            guess *= pre_transformation;
           
            pcl::io::savePCDFile("yang.pcd", front_filtered + top_filtered);

            if(ndt.hasConverged())
            {
                Eigen::Vector3f ypr = guess.block<3,3>(0,0).eulerAngles(2,1,0);
                std::cout << "x y z yaw pitch roll: " << guess(0,3) << '\t' << guess(1,3) << '\t' << guess(2,3) << '\t' << ypr(0) << '\t' << ypr(1) << '\t' << ypr(2) << std::endl;
            }

        }

    private:
        
        //点云过滤
        pcl::PointCloud<pcl::PointXYZI> filter(pcl::PointCloud<pcl::PointXYZI> laser)
        {
            pcl::CropBox<pcl::PointXYZI> crop;

            //80m开外的点不要
            crop.setMin(Eigen::Vector4f(-80.0, -80.0, -100.0, 1.0));
            crop.setMax(Eigen::Vector4f(80.0, 80.0, 100.0, 1.0));
            crop.setNegative(false);
            crop.setInputCloud(laser.makeShared());
            crop.filter(laser);

            //0.5m之内的点不要
            crop.setMin(Eigen::Vector4f(-0.5, -0.5, -100.0, 1.0));
            crop.setMax(Eigen::Vector4f(0.5, 0.5, 100.0, 1.0));
            crop.setNegative(true);
            crop.setInputCloud(laser.makeShared());
            crop.filter(laser);

            return laser;
        }
      
        //上雷达地面检测
        Eigen::Vector4f detectGroundTop(pcl::PointCloud<pcl::PointXYZI> laser)
        {
            pcl::PointCloud<pcl::PointXYZI> fullCloud;
            pcl::PointXYZI nanPoint;
            nanPoint.x = nanPoint.y = nanPoint.z = std::numeric_limits<float>::quiet_NaN();
            nanPoint.intensity = -1;
            fullCloud.resize(N_SCANS * N_HORIZONS);
            std::fill(fullCloud.begin(), fullCloud.end(), nanPoint);

            for(pcl::PointXYZI point : laser)
            {
                float angle = pcl::rad2deg(atan2(point.z, sqrt(pow(point.x, 2) + pow(point.y, 2))));
                int rowId = round((angle + N_SCANS - 1) / 2.0);
                if(rowId < 0 || rowId >= N_SCANS)
                    continue;

                angle = pcl::rad2deg(atan2(point.x, point.y)) + 180.0;
                int columnId = round(angle * N_HORIZONS / 360.0);
                if(columnId < 0 || columnId >= N_HORIZONS)
                    continue;
                
                fullCloud.points[columnId + rowId * N_HORIZONS] = point;
            }

            pcl::PointCloud<pcl::PointXYZI> filtered;
            for(int j = 0; j < N_HORIZONS; j ++)
            {
                for(int i = 0; i < groundScanID; i ++)
                {
                    int lowerId = j + i * N_HORIZONS;
                    int upperId = j + (i + 1) * N_HORIZONS;
                    if(fullCloud.points[lowerId].intensity == -1 || fullCloud.points[upperId].intensity == -1)
                        continue;
                    
                    float diffX = fullCloud.points[upperId].x - fullCloud.points[lowerId].x;
                    float diffY = fullCloud.points[upperId].y - fullCloud.points[lowerId].y;
                    float diffZ = fullCloud.points[upperId].z - fullCloud.points[lowerId].z;

                    float angle = pcl::rad2deg(atan2(diffZ, sqrt(pow(diffX, 2) + pow(diffY, 2))));

                    if(fabs(angle) < 10.0)
                    {
                        filtered.push_back(fullCloud.points[lowerId]);
                        filtered.push_back(fullCloud.points[upperId]);
                    }
                }
            }

            pcl::ModelCoefficients::Ptr coeffients(new pcl::ModelCoefficients());
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

            segment_.setInputCloud(filtered.makeShared());
            segment_.segment(*inliers, *coeffients);

            if(inliers->indices.size() < pointThresh)
                return Eigen::Vector4f::Zero();
            else 
                return Eigen::Vector4f(coeffients->values[0], coeffients->values[1], coeffients->values[2], coeffients->values[3]);

        }

        //下雷达地面检测
        Eigen::Vector4f detectGroundFront(pcl::PointCloud<pcl::PointXYZI> laser)
        {
            Eigen::Vector4f ground(0.0, 0.0, 1.0, 0.0);
            pcl::PlaneClipper3D<pcl::PointXYZI> clipper(ground);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
            clipper.clipPointCloud3D(laser, inliers->indices);

            pcl::ExtractIndices<pcl::PointXYZI> extract;
            extract.setInputCloud(laser.makeShared());
            extract.setIndices(inliers);
            extract.setNegative(true);
            extract.setInputCloud(laser.makeShared());
            extract.filter(laser);

            pcl::ModelCoefficients::Ptr coeffients(new pcl::ModelCoefficients());
            segment_.setInputCloud(laser.makeShared());
            segment_.segment(*inliers, *coeffients);

            if(inliers->indices.size() < pointThresh)
                return Eigen::Vector4f::Zero();
            else 
                return Eigen::Vector4f(coeffients->values[0], coeffients->values[1], coeffients->values[2], coeffients->values[3]);
        }

        std::string top_topic_;
        std::string front_topic_;
        int front_scans_;
        pcl::SACSegmentation<pcl::PointXYZI> segment_;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "calibration");
    ros::NodeHandle nh;

    Calibration calibration(nh);

    return 0;
}
