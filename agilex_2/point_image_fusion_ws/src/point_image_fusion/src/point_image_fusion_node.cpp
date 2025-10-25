#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

class PointImageFusion {
private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::Mat extrinsic_matrix_;
    cv::Size image_size_;
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud_;
    cv::Mat current_image_;
    bool cloud_updated_;
    bool image_updated_;
    
    std::mutex mutex_;

public:
    PointImageFusion() : current_cloud_(new pcl::PointCloud<pcl::PointXYZI>), 
                        cloud_updated_(false), image_updated_(false) {
        // 读取标定参数
        if (!loadCalibrationData()) {
            ROS_ERROR("Failed to load calibration data!");
            ros::shutdown();
            return;
        }
        
        // 订阅点云
        cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
            "/rslidar_points", 1, &PointImageFusion::cloudCallback, this);
        
        // 订阅图像
        image_sub_ = nh_.subscribe<sensor_msgs::Image>(
            "/camera/color/image_raw", 1, &PointImageFusion::imageCallback, this);
        
        // 发布融合图像
        image_transport::ImageTransport it(nh_);
        image_pub_ = it.advertise("/point_image", 1);
        
        ROS_INFO("PointImageFusion node initialized successfully");
    }

    bool loadCalibrationData() {
        try {
            std::string calibration_file;
            if (!ros::param::get("~calibration_file", calibration_file)) {
                calibration_file = "config/result.yaml";
            }
            
            YAML::Node config = YAML::LoadFile(calibration_file);
            
            // 读取相机内参
            YAML::Node camera_mat = config["CameraMat"];
            std::vector<double> cam_data = camera_mat["data"].as<std::vector<double>>();
            camera_matrix_ = cv::Mat(3, 3, CV_64F);
            for (int i = 0; i < 9; i++) {
                camera_matrix_.at<double>(i/3, i%3) = cam_data[i];
            }
            
            // 读取畸变系数
            YAML::Node dist_coeff = config["DistCoeff"];
            std::vector<double> dist_data = dist_coeff["data"].as<std::vector<double>>();
            dist_coeffs_ = cv::Mat(1, 5, CV_64F);
            for (int i = 0; i < 5; i++) {
                dist_coeffs_.at<double>(0, i) = dist_data[i];
            }
            
            // 读取外参矩阵
            YAML::Node extrinsic_mat = config["CameraExtrinsicMat"];
            std::vector<double> ext_data = extrinsic_mat["data"].as<std::vector<double>>();
            extrinsic_matrix_ = cv::Mat(4, 4, CV_64F);
            for (int i = 0; i < 16; i++) {
                extrinsic_matrix_.at<double>(i/4, i%4) = ext_data[i];
            }
            
            // 读取图像尺寸
            std::vector<int> size_data = config["ImageSize"].as<std::vector<int>>();
            image_size_ = cv::Size(size_data[0], size_data[1]);
            
            ROS_INFO("Calibration data loaded successfully");
            ROS_INFO("Image size: %dx%d", image_size_.width, image_size_.height);
            
            return true;
        } catch (const YAML::Exception& e) {
            ROS_ERROR("YAML parsing error: %s", e.what());
            return false;
        } catch (const std::exception& e) {
            ROS_ERROR("Error loading calibration data: %s", e.what());
            return false;
        }
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // 转换点云格式
        pcl::fromROSMsg(*msg, *current_cloud_);
        cloud_updated_ = true;
        
        // 如果图像也已更新，进行融合
        if (image_updated_) {
            fusePointCloudAndImage();
        }
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        try {
            // 转换图像格式
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            current_image_ = cv_ptr->image.clone();
            image_updated_ = true;
            
            // 如果点云也已更新，进行融合
            if (cloud_updated_) {
                fusePointCloudAndImage();
            }
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void fusePointCloudAndImage() {
        if (current_cloud_->empty() || current_image_.empty()) {
            return;
        }
        
        // 创建融合图像
        cv::Mat fused_image = current_image_.clone();
        
        // 将点云从雷达坐标系转换到相机坐标系
        Eigen::Matrix4f transform;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                transform(i, j) = extrinsic_matrix_.at<double>(i, j);
            }
        }
        
        Eigen::Matrix4f transform_inv = transform.inverse();
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::transformPointCloud(*current_cloud_, *transformed_cloud, transform_inv);
        // pcl::transformPointCloud(*current_cloud_, *transformed_cloud, transform);
        
        // 投影点到图像平面
        std::vector<cv::Point3f> object_points;
        std::vector<float> intensities;
        
        for (const auto& point : transformed_cloud->points) {
            if (point.z > 0) {  // 只处理相机前方的点
                object_points.push_back(cv::Point3f(point.x, point.y, point.z));
                intensities.push_back(point.intensity);
            }
        }
        
        if (object_points.empty()) {
            return;
        }
        
        // 投影点
        std::vector<cv::Point2f> image_points;
        cv::projectPoints(object_points, cv::Vec3f(0,0,0), cv::Vec3f(0,0,0), 
                         camera_matrix_, dist_coeffs_, image_points);
        
        // 在图像上绘制点
        for (size_t i = 0; i < image_points.size(); i++) {
            cv::Point2f pt = image_points[i];
            
            // 检查点是否在图像范围内
            if (pt.x >= 0 && pt.x < image_size_.width && 
                pt.y >= 0 && pt.y < image_size_.height) {
                
                // 根据强度值设置颜色
                float intensity = intensities[i];
                cv::Scalar color;
                if (intensity < 30) {
                    color = cv::Scalar(255, 0, 0);    // 蓝色
                } else if (intensity < 60) {
                    color = cv::Scalar(0, 255, 0);    // 绿色
                } else if (intensity < 90) {
                    color = cv::Scalar(0, 255, 255); // 黄色
                } else {
                    color = cv::Scalar(0, 0, 255);    // 红色
                }
                
                // 绘制点
                cv::circle(fused_image, pt, 2, color, -1);
            }
        }
        
        // 添加文字说明
        cv::putText(fused_image, "Point-Image Fusion: Blue<30 < Green<60 < Yellow<90 < Red", 
                   cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255), 2);
        cv::putText(fused_image, "Points: " + std::to_string(image_points.size()), 
                   cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255), 2);
        
        // 发布融合图像
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", fused_image).toImageMsg();
        image_pub_.publish(msg);
        
        cloud_updated_ = false;
        image_updated_ = false;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_image_fusion");
    
    PointImageFusion node;
    
    ros::spin();
    
    return 0;
}