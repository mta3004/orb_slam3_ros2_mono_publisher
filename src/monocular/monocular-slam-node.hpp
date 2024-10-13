#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cv_bridge/cv_bridge.h>
#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"
#include "Atlas.h"  // Atlas.h dosyası eklendi
#include "std_msgs/msg/header.hpp"
#include "utility.hpp"
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sophus/se3.hpp>
#include <vector>  // Eksik include eklendi
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/odometry.hpp>  // Odometry mesajını ekle
#include "std_srvs/srv/empty.hpp" // Gerekli std_srvs başlığı eklendi

class MonocularSlamNode : public rclcpp::Node
{
public:
    MonocularSlamNode(ORB_SLAM3::System* pSLAM);
    ~MonocularSlamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;
    using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
    using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
    using OdometryMsg = nav_msgs::msg::Odometry;  // Odometry mesajı için alias

    rclcpp::Publisher<PoseStampedMsg>::SharedPtr m_camera_pose_publisher1;
    rclcpp::Publisher<PoseStampedMsg>::SharedPtr m_camera_pose_publisher2;
    rclcpp::Publisher<PointCloud2Msg>::SharedPtr tracked_mappoints_pub;
    rclcpp::Publisher<PointCloud2Msg>::SharedPtr all_mappoints_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr tracking_img_pub; // Takip edilen görüntü yayıcı
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr kf_markers_pub;
    rclcpp::Publisher<OdometryMsg>::SharedPtr odom_pub;  // Odometry publisher'ı
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr m_save_map_service; // save_map hizmeti
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr m_save_traj_service; // save_traj hizmeti
    

    ORB_SLAM3::System* m_SLAM;
    cv_bridge::CvImagePtr m_cvImPtr;

    rclcpp::Subscription<ImageMsg>::SharedPtr m_image_subscriber;

    Sophus::SE3f current_camera_pose;
    tf2_ros::TransformBroadcaster m_transform_broadcaster;
    
    void setup_services();  // Servisleri ayarlamak için fonksiyon

    // Yeni mesajın zamanını tutmak için bir değişken ekleyin
    rclcpp::Time msg_time; // msg_time değişkeni burada tanımlandı

    void GrabImage(const ImageMsg::SharedPtr msg);
    void publish_camera_pose(const Sophus::SE3f& Tcw_SE3f, const std_msgs::msg::Header& header);
    void publish_tracked_points(const std::vector<ORB_SLAM3::MapPoint*>& tracked_points, const rclcpp::Time& msg_time);
    void publish_all_points(const std::vector<ORB_SLAM3::MapPoint*>& all_points, const rclcpp::Time& msg_time);
    void publish_tracking_img(cv::Mat image, const rclcpp::Time& msg_time); // Takip edilen görüntüyü yayımlama
    void publish_kf_markers(const std::vector<Sophus::SE3f> vKFposes, const rclcpp::Time& msg_time);
    void publish_odometry(const Sophus::SE3f& Tcw_SE3f, const rclcpp::Time& msg_time);  // Odometry fonksiyonu

    sensor_msgs::msg::PointCloud2 mappoint_to_pointcloud(std::vector<ORB_SLAM3::MapPoint*> map_points, const rclcpp::Time& msg_time);
    void publish_all_points_from_atlas(const rclcpp::Time& msg_time);
    cv::Mat SE3f_to_cvMat(Sophus::SE3f T_SE3f);
    tf2::Transform SE3f_to_tfTransform(Sophus::SE3f T_SE3f);

    // Servis çağrısı için fonksiyon tanımı
    bool save_map_srv(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                      std::shared_ptr<std_srvs::srv::Empty::Response> response);
    bool save_traj_srv(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                      std::shared_ptr<std_srvs::srv::Empty::Response> response);
};

#endif // __MONOCULAR_SLAM_NODE_HPP__
