#include "imu_calib/apply_calib.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<imu_calib::ApplyCalib>());
  rclcpp::shutdown();
  return 0;
}
