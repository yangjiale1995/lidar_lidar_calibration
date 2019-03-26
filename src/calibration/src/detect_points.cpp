#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/common/angles.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/PointIndices.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include "detect_points.h"

Eigen::Vector3f ground_norm(0, 0, 1);

//Y轴从大到小排序
bool compareVectorCol1(Eigen::Vector3f point1, Eigen::Vector3f point2)
{
    return (point1(1) > point2(1));
}

//z轴从大到小排序
bool compareVectorCol2(Eigen::Vector3f point1, Eigen::Vector3f point2)
{
    return (point1(2) > point2(2));
}


//构造函数
DetectPoints::DetectPoints()
{
}

DetectPoints::DetectPoints(float min_angle, float max_angle, float min_distance, float max_distance, float radius)
{
    pcl::PointCloud<pcl::PointXYZI> laser;
    laser_ptr_ = laser.makeShared();
    setFOV(min_angle, max_angle);
    setDistance(min_distance, max_distance);
    setRadius(radius);
}

//构造函数
DetectPoints::DetectPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr laser_ptr, float min_angle, float max_angle, float min_distance, float max_distance, float radius)
{
    setPointCloud(laser_ptr);
    setFOV(min_angle, max_angle);
    setDistance(min_distance, max_distance);
    setRadius(radius);
}


//析构函数
DetectPoints::~DetectPoints()
{
    laser_ptr_->clear();
}

//设置标定板长度
void DetectPoints::setLength(float length)
{
    if(length <= 0)
    {
        ROS_WARN_STREAM("length is negative");
    }
    else
    {
        length_ = length / 1000.0;
    }
}

//设置标定板宽度
void DetectPoints::setWidth(float width)
{
    if(width <= 0)
    {
        ROS_WARN_STREAM("width is negative");
    }
    else
    {
        width_ = width / 1000.0;
    }
}

//设置网格宽度
void DetectPoints::setRadius(float radius)
{
    if(radius < 0)
    {
        ROS_ERROR_STREAM("radius is negative");
    }
    else
    {
        radius_ = radius / 1000.0;
    }
}

//设置点云
void DetectPoints::setPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr laser_ptr)
{
    init();     //初始化

    //判断点云是否为空
    if(laser_ptr->points.empty())
    {
        ROS_WARN_STREAM("point cloud is empty");
    }
    else
    {
        //计算点云线号并按照线号排序
        std::vector<pcl::PointCloud<pcl::PointXYZI> > scan(N_SCANS);
        for(int i = 0; i < laser_ptr->points.size(); i ++)
        {
            pcl::PointXYZI point = laser_ptr->points[i];
            float angle_z = pcl::rad2deg(atan2(point.z, sqrt(pow(point.x, 2) + pow(point.y, 2))));
            int scan_id = round(angle_z);

            //if(scan_id == -15)
            //{
            //    continue;
            //}

            if(scan_id < 0)
            {
                scan_id += (N_SCANS - 1);
            }
            if(scan_id < 0 || scan_id > N_SCANS - 1)
            {
                continue;
            }
            point.intensity = scan_id;
            scan[scan_id].push_back(point);
        }

        laser_ptr->points.clear();
        for(int i = 0; i < N_SCANS; i ++)
        {
            (*laser_ptr) += scan[i];
        }

        laser_ptr_ = laser_ptr;
    }

}

//设置水平视角
void DetectPoints::setFOV(float min_angle, float max_angle)
{
    if(min_angle < pcl::rad2deg(-M_PI) || max_angle > pcl::rad2deg(M_PI) || max_angle <= min_angle)
    {
        ROS_ERROR_STREAM("set FOV failed");
    }
    else
    {
        min_angle_ = min_angle;
        max_angle_ = max_angle;
    }
}

//设置距离
void DetectPoints::setDistance(float min_distance, float max_distance)
{
    if(min_distance < 0 || max_distance < 0 || max_distance <= min_distance)
    {
        ROS_ERROR_STREAM("set distance error");
    }
    else
    {
        min_distance_ = min_distance / 1000.0;
        max_distance_ = max_distance / 1000.0;    
    }
}

//得到指定视角和距离的点云
bool DetectPoints::getObjectPointCloud()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr object_pointcloud(new pcl::PointCloud<pcl::PointXYZI>());
    for(int i = 0; i < laser_ptr_->points.size(); i ++)
    {
        pcl::PointXYZI point = laser_ptr_->points[i];
        float angle = pcl::rad2deg(atan2(point.y, point.x));
        float distance = sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
        if(angle > min_angle_ && angle < max_angle_ && distance > min_distance_ && distance < max_distance_)
        {
            object_pointcloud->push_back(point);
        }
    }
    laser_ptr_ = object_pointcloud;

    if(laser_ptr_->points.empty())
    {
        ROS_ERROR_STREAM("object pointcloud is empty");
        return false;
    }

    pcl::io::savePCDFile("object.pcd", *laser_ptr_);
    return true;
}


/*角点*/
bool DetectPoints::getFeaturePoints(std::vector<cv::Point3f> &features)
{
    if(radius_ == 0 || length_ == 0 || width_ == 0)
    {
        ROS_ERROR("radius = %f, length = %f, width = %f", radius_, length_, width_);
        return false;
    }
    if(laser_ptr_->points.empty())
    {
        ROS_ERROR_STREAM("laser_ptr is empty");
        return false;
    }

    if(!getObjectPointCloud())
    {
        return false;
    }

    if(!detectPlane())
    {
        return false;
    }

    if(!getBorder())
    {
        return false;
    }

    if(!detectLines())
    {
        return false;
    }


    std::vector<Eigen::Vector3f> features_eigen;
    Eigen::Vector3f center_point = std::accumulate(points_.begin(), points_.end(), Eigen::Vector3f(0.0, 0.0, 0.0)) / points_.size();
    
    for(int i = -1; i < 1; i ++)
    {
        for(int j = -1; j < 1; j ++)
        {
            Eigen::Vector3f feature_point = center_point + (0.5 + i) * radius_ * directions_[0] + (0.5 + j) * radius_ * directions_[1];
            features_eigen.push_back(feature_point);
        }
    }

    sort(features_eigen.begin(), features_eigen.end(), compareVectorCol1);
    sort(features_eigen.begin() + 1, features_eigen.begin() + 3, compareVectorCol2);


    for(int i = -2; i < 2; i += 3)
    {
        for(int j = -2; j < 2; j += 3)
        {
            Eigen::Vector3f feature_point = center_point + (0.5 + i) * radius_ * directions_[0] + (0.5 + j) * radius_ * directions_[1];
            features_eigen.push_back(feature_point);
        }
    }

    sort(features_eigen.begin() + 4, features_eigen.end(), compareVectorCol1);
    sort(features_eigen.begin() + 5, features_eigen.begin() + 7, compareVectorCol2);

    for(int i = 0; i < features_eigen.size(); i ++)
    {
        features.push_back(cv::Point3f(features_eigen[i](0) * 1000.0, features_eigen[i](1) * 1000.0, features_eigen[i](2) * 1000.0));
    }

    return true;
}

//初始化
void DetectPoints::init()
{
    lines_.clear();
    points_.clear();
    directions_.clear();
}


/*提取标定板平面*/
bool DetectPoints::detectPlane()
{
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices()); //下标
    pcl::ExtractIndices<pcl::PointXYZI> filters;    //过滤器
    filters.setNegative(true);

    Eigen::VectorXf coeff;

    while(1)
    {
        //RANSAC算法提取平面信息
        pcl::SampleConsensusModelPlane<pcl::PointXYZI>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZI>(laser_ptr_));
        pcl::RandomSampleConsensus<pcl::PointXYZI> ransac(model_p);
        ransac.setDistanceThreshold(TOLERANCE);
        ransac.computeModel();
        inliers->indices.clear();
        ransac.getInliers(inliers->indices);

        ransac.getModelCoefficients(coeff);

        float angle = fabs(pcl::rad2deg(acos(coeff.head(3).dot(ground_norm))));
        //std::cout << "angle = " << angle << std::endl;
        if(angle < TOLERANCE_ANGLE || fabs(angle - 180.0) < TOLERANCE_ANGLE)
        {
            filters.setInputCloud(laser_ptr_);
            filters.setIndices(inliers);
            filters.filter(*laser_ptr_);
            continue;
        }
        else
        {
            break;
        }
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr plane_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::copyPointCloud(*laser_ptr_, inliers->indices, *plane_ptr);
    laser_ptr_ = plane_ptr;
    if(laser_ptr_->points.empty())
    {
        ROS_ERROR_STREAM("detect plane error");
        return false;
    }

    //投影
    pcl::PointCloud<pcl::PointXYZI>::Ptr project_plane_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    for(int i = 0; i < laser_ptr_->points.size(); i ++)
    {
        pcl::PointXYZI pointOri = laser_ptr_->points[i];
        pcl::PointXYZI pointSel;
        float a = coeff(0);
        float b = coeff(1);
        float c = coeff(2);
        float d = coeff(3);
  
        pointSel.x = ((b * b + c * c) * pointOri.x - a * (b * pointOri.y + c * pointOri.z + d)) / coeff.head(3).squaredNorm();
        pointSel.y = ((a * a + c * c) * pointOri.y - b * (a * pointOri.x + c * pointOri.z + d)) / coeff.head(3).squaredNorm();
        pointSel.z = ((a * a + b * b) * pointOri.z - c * (a * pointOri.x + b * pointOri.y + d)) / coeff.head(3).squaredNorm();
        pointSel.intensity = plane_ptr->points[i].intensity;
        project_plane_ptr->push_back(pointSel);
    }
  
    laser_ptr_ = project_plane_ptr;

    pcl::io::savePCDFile("plane.pcd", *laser_ptr_);

    return true;
}

/*提取标定板轮廓*/
bool DetectPoints::getBorder()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr border_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    int scan_id = -1, start_index = -1;
    for(int i = 0; i < laser_ptr_->points.size(); i ++)
    {
        pcl::PointXYZI cur_point = laser_ptr_->points[i];
        pcl::PointXYZI next_point = laser_ptr_->points[i + 1];
        if(cur_point.intensity != scan_id)
        {
            scan_id = cur_point.intensity;
            start_index = i;
        }
        if(next_point.intensity != scan_id)
        {
            next_point = laser_ptr_->points[start_index];
        }

        float cur_angle = pcl::rad2deg(atan2(cur_point.y, cur_point.x)) + 360.0;
        float next_angle = pcl::rad2deg(atan2(next_point.y, next_point.x)) + 360.0;
        float delta_angle = fabs(next_angle - cur_angle);
        delta_angle = (delta_angle > 360.0 ? delta_angle - 360.0 : delta_angle);

        if(delta_angle > RESOLUTION)
        {
            border_ptr->push_back(cur_point);
            border_ptr->push_back(next_point);
        }
    }
    laser_ptr_ = border_ptr;

    if(laser_ptr_->points.empty())
    {
        ROS_ERROR_STREAM("detect border error");
        return false;
    }
    pcl::io::savePCDFile("border.pcd", *laser_ptr_);
    return true;
}

//提取标定板边缘线
bool DetectPoints::detectLines()
{
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices()); //下标
    
    pcl::ExtractIndices<pcl::PointXYZI> filters;    //过滤器
    filters.setNegative(true);
    //四根线
    for(int i = 0; i < 4; i ++)
    {
        if(laser_ptr_->points.empty())
        {
            ROS_ERROR_STREAM("detect lines error");
            return false;
        }
        
        //RANSAC提取线
        pcl::SampleConsensusModelLine<pcl::PointXYZI>::Ptr model_l(new pcl::SampleConsensusModelLine<pcl::PointXYZI>(laser_ptr_));
        pcl::RandomSampleConsensus<pcl::PointXYZI> ransac(model_l);
        ransac.setDistanceThreshold(TOLERANCE);
        ransac.computeModel();

        inliers->indices.clear();
        ransac.getInliers(inliers->indices);

        if(inliers->indices.empty())
        {
            return false;
        }

        Eigen::VectorXf coeff;
        ransac.getModelCoefficients(coeff);
        
        lines_.push_back(coeff);

        filters.setInputCloud(laser_ptr_);
        filters.setIndices(inliers);
        filters.filter(*laser_ptr_);

    }

    return computeEdgePoints();
}

bool DetectPoints::checkEdgePoints()
{
    for(int i = 0; i < points_.size(); i ++)
    {
        float dis = (points_[(i + 1) % points_.size()] - points_[i]).norm();
        if(fabs(dis - length_) < TOLERANCE_DISTANCE || fabs(dis - width_) < TOLERANCE_DISTANCE)
        {
            continue;
        }
        else
        {
            ROS_ERROR_STREAM("distance is error");
            return false;
        }
    }
    return true;
}


//计算标定板角点
bool DetectPoints::computeEdgePoints()
{
    std::cout << "computeEdgePoints start " << lines_.size() << std::endl;
    for(int i = 0; i < lines_.size(); i ++)
    {
        for(int j = i + 1; j < lines_.size(); j ++)
        {
            //两根线之间的夹角
            float angle = fabs(pcl::rad2deg(acos(lines_[i].tail(3).dot(lines_[j].tail(3)))));
            std::cout << "angle : " << angle << std::endl;
            //判断垂直
            if(fabs(angle - 90.0) < TOLERANCE_ANGLE)
            {
                //第i个点的x,y坐标
                float xi = lines_[i](0); 
                float yi = lines_[i](1);

                //第i条线的方向向量前两维
                float ai = lines_[i](3);
                float bi = lines_[i](4);

                //第j个点的x,y坐标
                float xj = lines_[j](0);
                float yj = lines_[j](1);

                //第j条线的方向向量前两维
                float aj = lines_[j](3);
                float bj = lines_[j](4);

                float t = (aj * (yi - yj) - bj * (xi - xj)) / (ai * bj - aj * bi);

                Eigen::Vector3f edge_point = lines_[i].head(3) + t * lines_[i].tail(3);
                
                points_.push_back(edge_point);
            }
            //判断平行
            else if(fabs(angle - 180.0) < TOLERANCE_ANGLE || fabs(angle) < TOLERANCE_ANGLE)
            {
                Eigen::Vector3f direction;
                if(angle < TOLERANCE_ANGLE)
                {
                    direction = lines_[i].tail(3) + lines_[j].tail(3);
                }
                else
                {
                    direction = lines_[i].tail(3) - lines_[j].tail(3);
                }
                direction.normalize();
                directions_.push_back(direction);
            }
            else
            {
                ROS_ERROR_STREAM("cannot compute edge point");
                return false;
            }
        }
    }

    if(points_.empty())
    {
        return false;
    }

    sort(points_.begin(), points_.end(), compareVectorCol2);
    sort(points_.begin() + 1, points_.end(), compareVectorCol1);

    std::cout << "computeEdgePoints finished" << std::endl;
    return checkEdgePoints();
}


bool DetectPoints::getCornerPoints(std::vector<Eigen::Vector3f> &points)
{
    if(radius_ == 0 || length_ == 0 || width_ == 0)
    {
        ROS_ERROR("radius = %f, length = %f, width = %f", radius_, length_, width_);
        return false;
    }
    if(laser_ptr_->points.empty())
    {
        ROS_ERROR_STREAM("laser_ptr is empty");
        return false;
    }
   
    if(!getObjectPointCloud())
    {
        return false;
    }
   
    if(!detectPlane())
    {
        return false;
    }
   
    if(!getBorder())
    {
        return false;
    }
   
    if(!detectLines())
    {
        return false;
    }

    points.assign(points_.begin(), points_.end());
    return true;
}
