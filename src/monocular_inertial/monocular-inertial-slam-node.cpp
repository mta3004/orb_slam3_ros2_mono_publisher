#include "monocular-inertial-slam-node.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cv_bridge/cv_bridge.h>

MonocularInertialSlamNode::MonocularInertialSlamNode(ORB_SLAM3::System* slam_system)
    : Node("ORB_SLAM3_ROS2_MI"), tf_broadcaster_(this)
{
    slam_ = slam_system;

    image_sub = this->create_subscription<ImageMsg>(
        "/anafi/camera/image",
        rclcpp::QoS(10),
        std::bind(&MonocularInertialSlamNode::handle_image, this, std::placeholders::_1));

    imu_sub = this->create_subscription<ImuMsg>(
        "/imu",
        rclcpp::SensorDataQoS(),
        std::bind(&MonocularInertialSlamNode::handle_imu, this, std::placeholders::_1));

    camera_pose_pub = this->create_publisher<PoseStampedMsg>("/pose_orb_mi", rclcpp::QoS(10));
    tracked_mappoints_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/tracked_mappoints", rclcpp::QoS(10));
    all_mappoints_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/all_mappoints", rclcpp::QoS(10));
    tracking_img_pub = this->create_publisher<sensor_msgs::msg::Image>("/tracking_image", rclcpp::QoS(10));
    kf_markers_pub = this->create_publisher<visualization_msgs::msg::Marker>(
        std::string(this->get_name()) + "/kf_markers", rclcpp::QoS(1000));
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(10));

    // Services
    save_map_srv_ = this->create_service<std_srvs::srv::Empty>(
        "save_map", std::bind(&MonocularInertialSlamNode::save_map, this, std::placeholders::_1, std::placeholders::_2));

    save_traj_srv_ = this->create_service<std_srvs::srv::Empty>(
        "save_traj", std::bind(&MonocularInertialSlamNode::save_traj, this, std::placeholders::_1, std::placeholders::_2));

    current_camera_pose_ = Sophus::SE3f();
}

MonocularInertialSlamNode::~MonocularInertialSlamNode()
{
    slam_->Shutdown();
    slam_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonocularInertialSlamNode::handle_imu(const ImuMsg::SharedPtr msg)
{
    const double t = Utility::StampToSec(msg->header.stamp);
    Eigen::Vector3f acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    Eigen::Vector3f gyr(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_buffer_.emplace_back(t, acc, gyr);

    const double window_sec = 5.0; // keep up to 5 seconds IMU
    while (!imu_buffer_.empty() && (imu_buffer_.back().t - imu_buffer_.front().t) > window_sec)
        imu_buffer_.pop_front();
}

void MonocularInertialSlamNode::handle_image(const ImageMsg::SharedPtr msg)
{
    try
    {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        cv::Mat gray_image;
        cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);

        const double t_frame = Utility::StampToSec(msg->header.stamp);

        std::vector<ORB_SLAM3::IMU::Point> imu_measurements;
        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            size_t count = 0;
            for (const auto &p : imu_buffer_)
            {
                if (p.t <= t_frame)
                {
                    imu_measurements.push_back(p);
                    count++;
                }
                else
                {
                    break;
                }
            }
            if (count > 0)
            {
                imu_buffer_.erase(imu_buffer_.begin(), imu_buffer_.begin() + static_cast<long>(count));
            }
        }

        current_camera_pose_ = slam_->TrackMonocular(gray_image, t_frame, imu_measurements);

        Eigen::Vector3f translation = current_camera_pose_.translation();
        Eigen::Matrix3f rotation = current_camera_pose_.rotationMatrix();

        PoseStampedMsg pose_msg;
        pose_msg.header = msg->header;
        pose_msg.header.frame_id = "map";
        pose_msg.pose.position.x = translation(0);
        pose_msg.pose.position.y = translation(1);
        pose_msg.pose.position.z = translation(2);

        Eigen::Quaternionf q(rotation);
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();
        camera_pose_pub->publish(pose_msg);

        Sophus::SE3f Twc = slam_->GetCamTwc();
        if (Twc.translation().array().isNaN()[0] || Twc.rotationMatrix().array().isNaN()(0, 0))
            return;

        rclcpp::Time msg_time = this->get_clock()->now();
        publish_odometry(Twc, msg_time);
        publish_camera_pose(Twc, msg->header);
        publish_tracked_points(slam_->GetTrackedMapPoints(), msg_time);
        publish_all_points(slam_->GetAllMapPoints(), msg_time);
        publish_tracking_img(slam_->GetCurrentFrame(), msg->header.stamp);
        publish_kf_markers(slam_->GetAllKeyframePoses(), msg_time);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void MonocularInertialSlamNode::publish_odometry(const Sophus::SE3f &Tcw_SE3f, const rclcpp::Time &msg_time)
{
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = msg_time;
    odom_msg.header.frame_id = "map";
    odom_msg.child_frame_id = "camera";
    odom_msg.pose.pose.position.x = Tcw_SE3f.translation().x();
    odom_msg.pose.pose.position.y = Tcw_SE3f.translation().y();
    odom_msg.pose.pose.position.z = Tcw_SE3f.translation().z();
    Eigen::Quaternionf quaternion(Tcw_SE3f.rotationMatrix());
    odom_msg.pose.pose.orientation.x = quaternion.x();
    odom_msg.pose.pose.orientation.y = quaternion.y();
    odom_msg.pose.pose.orientation.z = quaternion.z();
    odom_msg.pose.pose.orientation.w = quaternion.w();
    odom_pub->publish(odom_msg);
}

void MonocularInertialSlamNode::publish_tracking_img(cv::Mat image, const rclcpp::Time &msg_time)
{
    std_msgs::msg::Header header;
    header.stamp = msg_time;
    header.frame_id = "map";

    auto rendered_image_msg = std::make_unique<sensor_msgs::msg::Image>();
    rendered_image_msg->header = header;
    rendered_image_msg->height = image.rows;
    rendered_image_msg->width = image.cols;
    rendered_image_msg->encoding = "bgr8";
    rendered_image_msg->is_bigendian = 0;
    rendered_image_msg->step = image.step[0];
    rendered_image_msg->data.resize(image.total() * image.elemSize());
    std::memcpy(rendered_image_msg->data.data(), image.data, rendered_image_msg->data.size());
    tracking_img_pub->publish(std::move(rendered_image_msg));
}

void MonocularInertialSlamNode::publish_kf_markers(const std::vector<Sophus::SE3f> vKFposes, const rclcpp::Time &msg_time)
{
    int numKFs = vKFposes.size();
    if (numKFs == 0)
        return;

    visualization_msgs::msg::Marker kf_markers;
    kf_markers.header.frame_id = "map";
    kf_markers.header.stamp = msg_time;
    kf_markers.ns = "kf_markers";
    kf_markers.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    kf_markers.action = visualization_msgs::msg::Marker::ADD;
    kf_markers.pose.orientation.w = 1.0;
    kf_markers.lifetime = rclcpp::Duration::from_seconds(1.0);
    kf_markers.id = 0;
    kf_markers.scale.x = 0.05;
    kf_markers.scale.y = 0.05;
    kf_markers.scale.z = 0.05;
    kf_markers.color.g = 1.0;
    kf_markers.color.a = 1.0;

    for (int i = 0; i < numKFs; i++)
    {
        geometry_msgs::msg::Point kf_marker;
        kf_marker.x = vKFposes[i].translation().x();
        kf_marker.y = vKFposes[i].translation().y();
        kf_marker.z = vKFposes[i].translation().z();
        kf_markers.points.push_back(kf_marker);
    }

    kf_markers_pub->publish(kf_markers);
}

void MonocularInertialSlamNode::publish_camera_pose(const Sophus::SE3f &Tcw_SE3f, const std_msgs::msg::Header &header)
{
    geometry_msgs::msg::TransformStamped transform_msg;
    transform_msg.header.stamp = this->get_clock()->now();
    transform_msg.header.frame_id = "map";
    transform_msg.child_frame_id = "camera";
    transform_msg.transform.translation.x = Tcw_SE3f.translation().x();
    transform_msg.transform.translation.y = Tcw_SE3f.translation().y();
    transform_msg.transform.translation.z = Tcw_SE3f.translation().z();

    Eigen::Quaternionf corrected_rotation = Tcw_SE3f.unit_quaternion();
    transform_msg.transform.rotation.w = corrected_rotation.w();
    transform_msg.transform.rotation.x = corrected_rotation.x();
    transform_msg.transform.rotation.y = corrected_rotation.y();
    transform_msg.transform.rotation.z = corrected_rotation.z();
    tf_broadcaster_.sendTransform(transform_msg);

    PoseStampedMsg pose_msg2;
    pose_msg2.header.frame_id = "map";
    pose_msg2.header.stamp = transform_msg.header.stamp;
    pose_msg2.pose.position.x = transform_msg.transform.translation.x;
    pose_msg2.pose.position.y = transform_msg.transform.translation.y;
    pose_msg2.pose.position.z = transform_msg.transform.translation.z;
    pose_msg2.pose.orientation = transform_msg.transform.rotation;
    camera_pose_pub->publish(pose_msg2);
}

sensor_msgs::msg::PointCloud2 MonocularInertialSlamNode::mappoint_to_pointcloud(std::vector<ORB_SLAM3::MapPoint *> map_points, const rclcpp::Time &msg_time)
{
    const int num_channels = 3;
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = msg_time;
    cloud.header.frame_id = "map";
    cloud.height = 1;
    cloud.width = map_points.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(num_channels);

    for (int i = 0; i < num_channels; ++i)
    {
        cloud.fields[i].name = (i == 0) ? "x" : (i == 1) ? "y" : "z";
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud.fields[i].count = 1;
    }

    cloud.data.resize(cloud.width * cloud.point_step);
    unsigned char *ptr = cloud.data.data();

    for (size_t i = 0; i < map_points.size(); ++i)
    {
        ORB_SLAM3::MapPoint *pMP = map_points[i];
        if (pMP)
        {
            float *p = reinterpret_cast<float *>(ptr);
            p[0] = pMP->GetWorldPos()(0);
            p[1] = pMP->GetWorldPos()(1);
            p[2] = pMP->GetWorldPos()(2);
            ptr += cloud.point_step;
        }
    }

    return cloud;
}

void MonocularInertialSlamNode::publish_tracked_points(const std::vector<ORB_SLAM3::MapPoint *> tracked_points, const rclcpp::Time &msg_time)
{
    sensor_msgs::msg::PointCloud2 cloud = mappoint_to_pointcloud(tracked_points, msg_time);
    tracked_mappoints_pub->publish(cloud);
}

void MonocularInertialSlamNode::publish_all_points(const std::vector<ORB_SLAM3::MapPoint *> all_points, const rclcpp::Time &msg_time)
{
    sensor_msgs::msg::PointCloud2 cloud = mappoint_to_pointcloud(all_points, msg_time);
    all_mappoints_pub->publish(cloud);
}

bool MonocularInertialSlamNode::save_map(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                         std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    (void)request;
    (void)response;
    slam_->SaveMap("SavedMap.map");
    RCLCPP_INFO(this->get_logger(), "Map has been saved to 'SavedMap.map'");
    return true;
}

bool MonocularInertialSlamNode::save_traj(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                          std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    (void)request;
    (void)response;
    slam_->SaveKeyFrameTrajectoryTUM("SavedTrajectory.txt");
    RCLCPP_INFO(this->get_logger(), "Trajectory has been saved to 'SavedTrajectory.txt'");
    return true;
}


