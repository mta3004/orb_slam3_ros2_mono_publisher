#ifndef __MONOCULAR_INERTIAL_SLAM_NODE_HPP__
#define __MONOCULAR_INERTIAL_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cv_bridge/cv_bridge.h>
#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"
#include "Atlas.h"
#include "ImuTypes.h"
#include "std_msgs/msg/header.hpp"
#include "utility.hpp"
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sophus/se3.hpp>
#include <Eigen/Dense>
#include <vector>
#include <deque>
#include <mutex>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "std_srvs/srv/empty.hpp"

class MonocularInertialSlamNode : public rclcpp::Node
{
public:
    explicit MonocularInertialSlamNode(ORB_SLAM3::System* slam_system);
    ~MonocularInertialSlamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;
    using ImuMsg = sensor_msgs::msg::Imu;
    using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
    using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
    using OdometryMsg = nav_msgs::msg::Odometry;

    // Publishers
    rclcpp::Publisher<PoseStampedMsg>::SharedPtr camera_pose_pub;
    rclcpp::Publisher<PointCloud2Msg>::SharedPtr tracked_mappoints_pub;
    rclcpp::Publisher<PointCloud2Msg>::SharedPtr all_mappoints_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr tracking_img_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr kf_markers_pub;
    rclcpp::Publisher<OdometryMsg>::SharedPtr odom_pub;

    // Services
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_map_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_traj_srv_;

    // Subscriptions
    rclcpp::Subscription<ImageMsg>::SharedPtr image_sub;
    rclcpp::Subscription<ImuMsg>::SharedPtr imu_sub;

    ORB_SLAM3::System* slam_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    Sophus::SE3f current_camera_pose_;

    // IMU buffer (time-ordered)
    std::deque<ORB_SLAM3::IMU::Point> imu_buffer_;
    std::mutex imu_mutex_;

    void handle_image(const ImageMsg::SharedPtr msg);
    void handle_imu(const ImuMsg::SharedPtr msg);

    void publish_camera_pose(const Sophus::SE3f& Tcw_SE3f, const std_msgs::msg::Header& header);
    void publish_tracked_points(const std::vector<ORB_SLAM3::MapPoint*>& tracked_points, const rclcpp::Time& msg_time);
    void publish_all_points(const std::vector<ORB_SLAM3::MapPoint*>& all_points, const rclcpp::Time& msg_time);
    void publish_tracking_img(cv::Mat image, const rclcpp::Time& msg_time);
    void publish_kf_markers(const std::vector<Sophus::SE3f> vKFposes, const rclcpp::Time& msg_time);
    void publish_odometry(const Sophus::SE3f& Tcw_SE3f, const rclcpp::Time& msg_time);
    sensor_msgs::msg::PointCloud2 mappoint_to_pointcloud(std::vector<ORB_SLAM3::MapPoint*> map_points, const rclcpp::Time& msg_time);

    // Services impl
    bool save_map(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                  std::shared_ptr<std_srvs::srv::Empty::Response> response);
    bool save_traj(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                   std::shared_ptr<std_srvs::srv::Empty::Response> response);
};

#endif // __MONOCULAR_INERTIAL_SLAM_NODE_HPP__


