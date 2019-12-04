#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <opencv/cv.h>

#define TOLERANCE_DISTANCE 0.1     // 计算出来的角点位置误差
#define TOLERANCE_ANGLE 5.0         // 计算出来的方向向量角度误差
#define TOLERANCE 0.05              // 提取线或面特征时可接受的距离误差
#define RESOLUTION 0.5*3
#define N_SCANS 16


class DetectPoints
{
    public:
        
        //构造函数
        DetectPoints();
        DetectPoints(float min_angle = -180.0, float max_angle = 180.0, float min_distance = 0.0, float max_distance = 100000.0, float radius = 0);
        DetectPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr laser_ptr, float min_angle = -180.0, float max_angle = 180.0, float min_distance = 0.0, float max_distance = 100000.0, float radius = 0);
        
        //析构函数
        ~DetectPoints();
        
        //设置标定板长度
        void setLength(float length);
        
        //设置标定板宽度
        void setWidth(float width);

        //设置角点边长
        void setRadius(float radius);

        //设置点云
        void setPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr laser_ptr);
        
        //设置标定板所处的激光视场角
        void setFOV(float min_angle = -180.0, float max_angle = 180.0);
        
        //设置标定板所处的距离
        void setDistance(float min_distance = 0, float max_distance = 100000.0);
        
        //返回角点信息
        bool getFeaturePoints(std::vector<cv::Point3f> &features);
    
        bool getCornerPoints(std::vector<Eigen::Vector3f> &points);
    private:

        // point cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr laser_ptr_;


        //边缘线方程参数
        std::vector<Eigen::VectorXf> lines_;

        //四个角点和两个相互垂直的方向向量
        std::vector<Eigen::Vector3f> points_;
        std::vector<Eigen::Vector3f> directions_;
        
        // FOV
        float min_angle_ = -180.0;
        float max_angle_ = 180.0;
        
        // distance
        float min_distance_ = 0.0;
        float max_distance_ = 100.0;
        
        // 角点边长
        float radius_ = 0;

        //标定板长和宽
        float length_ = 0;
        float width_ = 0;

        //初始化
        void init();

        //目标点云
        bool getObjectPointCloud();

        //检测标定板
        bool detectPlane();

        //检测轮廓
        bool getBorder();
        
        //检测边缘线
        bool detectLines();

        //计算角点
        bool computeEdgePoints();

        //检查点的相对位置关系
        bool checkEdgePoints();
};


