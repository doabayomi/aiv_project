#include <rclcpp/rclcpp.hpp>
#include "imu_calib/do_calib.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<imu_calib::DoCalib>());
  rclcpp::shutdown();
  return 0;
}

