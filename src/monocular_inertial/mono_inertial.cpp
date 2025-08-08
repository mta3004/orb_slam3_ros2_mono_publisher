#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "monocular-inertial-slam-node.hpp"

#include "System.h"

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam3_pose mono_inertial path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);

    bool visualization = true;
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, visualization);

    auto node = std::make_shared<MonocularInertialSlamNode>(&SLAM);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}


