#include "monocular-slam-node.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/imgproc.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include <nav_msgs/msg/odometry.hpp>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
: Node("ORB_SLAM3_ROS2"),
  m_transform_broadcaster(this)
{
    m_SLAM = pSLAM;

    // Image subscriber
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "/anafi/camera/image", 
        rclcpp::QoS(10), 
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1)
    );
    
    // First camera pose publisher for GrabImage
    m_camera_pose_publisher1 = this->create_publisher<PoseStampedMsg>("/pose_orb1", rclcpp::QoS(10));
    
    // Second camera pose publisher for publish_camera_pose
    m_camera_pose_publisher2 = this->create_publisher<PoseStampedMsg>("/pose_orb2", rclcpp::QoS(10));

    // Tracked map points publisher
    tracked_mappoints_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/tracked_mappoints", rclcpp::QoS(10));

    // All map points publisher
    all_mappoints_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/all_mappoints", rclcpp::QoS(10));
    
    // Tracking image publisher
    tracking_img_pub = this->create_publisher<sensor_msgs::msg::Image>("/tracking_image", rclcpp::QoS(10));
    
    kf_markers_pub = this->create_publisher<visualization_msgs::msg::Marker>(
        std::string(this->get_name()) + "/kf_markers", rclcpp::QoS(1000));
        
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(10));


    // Initialize the current camera pose
    current_camera_pose = Sophus::SE3f();
    setup_services();

}

MonocularSlamNode::~MonocularSlamNode()
{
    m_SLAM->Shutdown();
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    try
    {
        cv::Mat gray_image;
        if (msg->encoding == "bgr8")
        {
            cv::Mat bgr(msg->height, msg->width, CV_8UC3, const_cast<unsigned char*>(msg->data.data()), msg->step);
            cv::cvtColor(bgr, gray_image, cv::COLOR_BGR2GRAY);
        }
        else if (msg->encoding == "rgb8")
        {
            cv::Mat rgb(msg->height, msg->width, CV_8UC3, const_cast<unsigned char*>(msg->data.data()), msg->step);
            cv::cvtColor(rgb, gray_image, cv::COLOR_RGB2GRAY);
        }
        else if (msg->encoding == "mono8")
        {
            gray_image = cv::Mat(msg->height, msg->width, CV_8UC1, const_cast<unsigned char*>(msg->data.data()), msg->step);
        }
        else
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Unsupported image encoding: %s", msg->encoding.c_str());
            return;
        }

        current_camera_pose = m_SLAM->TrackMonocular(gray_image, Utility::StampToSec(msg->header.stamp));

        Eigen::Vector3f translation = current_camera_pose.translation();
        Eigen::Matrix3f rotation = current_camera_pose.rotationMatrix();

        PoseStampedMsg pose_msg1;
        pose_msg1.header.stamp = msg->header.stamp;
        pose_msg1.pose.position.x = translation(0);
        pose_msg1.pose.position.y = translation(1);
        pose_msg1.pose.position.z = translation(2);

        Eigen::Quaternionf quaternion(rotation);
        pose_msg1.pose.orientation.x = -quaternion.x();
        pose_msg1.pose.orientation.y = -quaternion.y();
        pose_msg1.pose.orientation.z = -quaternion.z();
        pose_msg1.pose.orientation.w = quaternion.w();

        m_camera_pose_publisher1->publish(pose_msg1);
        
        
        Sophus::SE3f Twc = m_SLAM->GetCamTwc();

        if (Twc.translation().array().isNaN()[0] || Twc.rotationMatrix().array().isNaN()(0,0)) // avoid publishing NaN
            return;
    
        // Odometry bilgilerini yayınla
        rclcpp::Time msg_time = this->get_clock()->now();
        publish_odometry(Twc, msg_time); // Odometry yayınlama
        
        publish_camera_pose(Twc, msg->header);

        publish_tracked_points(m_SLAM->GetTrackedMapPoints(), msg_time);
        publish_all_points(m_SLAM->GetAllMapPoints(), msg_time); // All points yayınlama

        // Get current frame and publish tracking image
        publish_tracking_img(m_SLAM->GetCurrentFrame(), msg->header.stamp);
        
        publish_kf_markers(m_SLAM->GetAllKeyframePoses(), msg_time);
        
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}


void MonocularSlamNode::publish_odometry(const Sophus::SE3f& Tcw_SE3f, const rclcpp::Time& msg_time)
{
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = msg_time;
    odom_msg.header.frame_id = "map"; // Frame ayarları
    odom_msg.child_frame_id = "camera"; // Çocuk frame

    // Pozisyon bilgileri
    odom_msg.pose.pose.position.x = Tcw_SE3f.translation().x();
    odom_msg.pose.pose.position.y = Tcw_SE3f.translation().y();
    odom_msg.pose.pose.position.z = Tcw_SE3f.translation().z();

    // Yönelim bilgileri
    Eigen::Quaternionf quaternion(Tcw_SE3f.rotationMatrix());
    odom_msg.pose.pose.orientation.x = quaternion.x();
    odom_msg.pose.pose.orientation.y = quaternion.y();
    odom_msg.pose.pose.orientation.z = quaternion.z();
    odom_msg.pose.pose.orientation.w = quaternion.w();

    // Odom mesajını yayınla
    odom_pub->publish(odom_msg);
}

void MonocularSlamNode::publish_tracking_img(cv::Mat image, const rclcpp::Time& msg_time)
{
    std_msgs::msg::Header header;
    
    header.stamp = msg_time;
    header.frame_id = "map";

    // İlk olarak, render edilen görüntü mesajını oluşturun
    auto rendered_image_msg = std::make_unique<sensor_msgs::msg::Image>();

    // Gerekli alanları doldurun
    rendered_image_msg->header = header; // Header'ı ayarlayın
    rendered_image_msg->height = image.rows; // Görüntü yüksekliği
    rendered_image_msg->width = image.cols; // Görüntü genişliği
    rendered_image_msg->encoding = "bgr8"; // Görüntü kodlaması
    rendered_image_msg->is_bigendian = 0; // Byte sıralaması
    rendered_image_msg->step = image.step[0]; // Her satırın byte cinsinden uzunluğu
    rendered_image_msg->data.resize(image.total() * image.elemSize()); // Veri boyutunu ayarlayın
    std::memcpy(rendered_image_msg->data.data(), image.data, rendered_image_msg->data.size()); // Veri kopyalama

    // Artık mesajı yayınlayabilirsiniz
    tracking_img_pub->publish(std::move(rendered_image_msg));
}

void MonocularSlamNode::publish_kf_markers(const std::vector<Sophus::SE3f> vKFposes, const rclcpp::Time& msg_time)
{
    int numKFs = vKFposes.size();
    if (numKFs == 0)
        return;
    
    visualization_msgs::msg::Marker kf_markers;
    kf_markers.header.frame_id = "map";
    kf_markers.header.stamp = msg_time;  // Zaman damgası ekle
    kf_markers.ns = "kf_markers";
    kf_markers.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    kf_markers.action = visualization_msgs::msg::Marker::ADD;
    kf_markers.pose.orientation.w = 1.0;
    kf_markers.lifetime = rclcpp::Duration::from_seconds(1.0);  // 1 saniye

    kf_markers.id = 0;
    kf_markers.scale.x = 0.05;
    kf_markers.scale.y = 0.05;
    kf_markers.scale.z = 0.05;
    kf_markers.color.g = 1.0;
    kf_markers.color.a = 1.0;

    for (int i = 0; i < numKFs; i++)  // Düzeltildi
    {
        geometry_msgs::msg::Point kf_marker;
        kf_marker.x = vKFposes[i].translation().x();
        kf_marker.y = vKFposes[i].translation().y();
        kf_marker.z = vKFposes[i].translation().z();
        kf_markers.points.push_back(kf_marker);
    }
    
    kf_markers_pub->publish(kf_markers);  // Düzeltildi
}

void MonocularSlamNode::publish_camera_pose(const Sophus::SE3f& Tcw_SE3f, const std_msgs::msg::Header& header)
{
    // TransformStamped mesajı oluştur
    geometry_msgs::msg::TransformStamped transform_msg;
    transform_msg.header.stamp = this->get_clock()->now();
    transform_msg.header.frame_id = "map";         // Referans çerçeve (dünya çerçevesi)
    transform_msg.child_frame_id = "camera";       // Kameraya ait çerçeve

    // Pozisyon bilgilerini SE3f'den al
    transform_msg.transform.translation.x = Tcw_SE3f.translation().x();
    transform_msg.transform.translation.y = Tcw_SE3f.translation().y();
    transform_msg.transform.translation.z = Tcw_SE3f.translation().z();

    // Y ekseni etrafında ters yönde 90 derece döndürme için quaternion oluştur
    Eigen::Quaternionf yaw_minus_90_deg(Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitY()));

    // Kameranın orijinal rotasyonu ile ters yönde 90 derece döndürülmüş rotasyonu çarp
    Eigen::Quaternionf corrected_rotation = yaw_minus_90_deg * Tcw_SE3f.unit_quaternion();

    // Oryantasyonu mesajlara aktar (quaternion)
    transform_msg.transform.rotation.w = corrected_rotation.w();
    transform_msg.transform.rotation.x = corrected_rotation.x();
    transform_msg.transform.rotation.y = corrected_rotation.y();
    transform_msg.transform.rotation.z = corrected_rotation.z();

    // Transform'u yayınla (TF transformasyonu)
    m_transform_broadcaster.sendTransform(transform_msg);

    // PoseStamped mesajı oluştur
    PoseStampedMsg pose_msg2;
    pose_msg2.header.frame_id = "map";  // Dünya çerçevesi
    pose_msg2.header.stamp = transform_msg.header.stamp; // Zaman damgası

    // Pozisyon bilgilerini aktar
    pose_msg2.pose.position.x = transform_msg.transform.translation.x;
    pose_msg2.pose.position.y = transform_msg.transform.translation.y;
    pose_msg2.pose.position.z = transform_msg.transform.translation.z;

    // Oryantasyonu aktar
    pose_msg2.pose.orientation = transform_msg.transform.rotation;

    // PoseStamped mesajını yayınla
    m_camera_pose_publisher2->publish(pose_msg2);
}




bool MonocularSlamNode::save_map_srv(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                     std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    m_SLAM->SaveMap("SavedMap.map");
    RCLCPP_INFO(this->get_logger(), "Map has been saved to 'SavedMap.map'");
    return true;
}

bool MonocularSlamNode::save_traj_srv(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                      std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    m_SLAM->SaveKeyFrameTrajectoryTUM("SavedTrajectory.txt");
    RCLCPP_INFO(this->get_logger(), "Trajectory has been saved to 'SavedTrajectory.txt'");
    return true;
}

void MonocularSlamNode::setup_services()
{
    m_save_map_service = this->create_service<std_srvs::srv::Empty>(
        "save_map", std::bind(&MonocularSlamNode::save_map_srv, this, std::placeholders::_1, std::placeholders::_2));

    m_save_traj_service = this->create_service<std_srvs::srv::Empty>(
        "save_traj", std::bind(&MonocularSlamNode::save_traj_srv, this, std::placeholders::_1, std::placeholders::_2));
}

void MonocularSlamNode::publish_tracked_points(const std::vector<ORB_SLAM3::MapPoint*>& tracked_points, const rclcpp::Time& msg_time)
{
    sensor_msgs::msg::PointCloud2 cloud = mappoint_to_pointcloud(tracked_points, msg_time);
    tracked_mappoints_pub->publish(cloud);
}

void MonocularSlamNode::publish_all_points(const std::vector<ORB_SLAM3::MapPoint*>& all_points, const rclcpp::Time& msg_time)
{
    sensor_msgs::msg::PointCloud2 cloud = mappoint_to_pointcloud(all_points, msg_time);
    all_mappoints_pub->publish(cloud);
}

sensor_msgs::msg::PointCloud2 MonocularSlamNode::mappoint_to_pointcloud(std::vector<ORB_SLAM3::MapPoint*> map_points, const rclcpp::Time& msg_time)
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

    for (int i = 0; i < num_channels; ++i) {
        cloud.fields[i].name = (i == 0) ? "x" : (i == 1) ? "y" : "z";
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud.fields[i].count = 1;
    }

    cloud.data.resize(cloud.width * cloud.point_step);
    unsigned char* ptr = cloud.data.data();

    for (size_t i = 0; i < map_points.size(); ++i) {
        ORB_SLAM3::MapPoint* pMP = map_points[i];
        if (pMP) {
            float* p = reinterpret_cast<float*>(ptr);
            p[0] = pMP->GetWorldPos()(0);
            p[1] = pMP->GetWorldPos()(1);
            p[2] = pMP->GetWorldPos()(2);
            ptr += cloud.point_step;
        }
    }

    return cloud;
}
