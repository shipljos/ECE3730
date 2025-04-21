#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

using std::placeholders::_1;

class PeripheralSubscriber : public rclcpp::Node
{
public:
  PeripheralSubscriber()
  : Node("peripheral_subscriber")
  {
    auto cam_callback = [this](const sensor_msgs::msg::Image::SharedPtr msg){ 
      // long long unsigned int image_timestamp = msg->header.stamp.nanosec;
      double t = msg->header.stamp.sec+ (_Float64)msg->header.stamp.nanosec*1e-9;
      RCLCPP_INFO(this->get_logger(), "Camera Image Timestamp: %.9f'", t);
      // RCLCPP_INFO(this->get_logger(), "Camera Image Timestamp: %llu ns'", image_timestamp);
    };
    auto controller_callback = [this](const sensor_msgs::msg::Joy::SharedPtr msg){
      bool button0_status = msg->buttons.size() > 0 ? msg->buttons[0] == 1 : false;
      RCLCPP_INFO(this->get_logger(), "Joy Button 0 status: %s", button0_status ? "Pressed" : "Not Pressed");
    };
    auto gps_callback = [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg){
      // long long unsigned int gps_timestamp = msg->header.stamp.nanosec;
      double t = msg->header.stamp.sec+ (_Float64)msg->header.stamp.nanosec*1e-9;
      double longitude = msg->longitude;
      RCLCPP_INFO(this->get_logger(), "GPS Timestamp: %.9f'", t); 
      // RCLCPP_INFO(this->get_logger(), "GPS Timestamp: %llu ns'", gps_timestamp); 
      RCLCPP_INFO(this->get_logger(), "GPS Longitude: %f'", longitude); 
    };

    cam_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", 10, cam_callback);

    controller_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, controller_callback);

    gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/fix", 10, gps_callback);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr controller_subscription_;  
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PeripheralSubscriber>());
  rclcpp::shutdown();
  return 0;
}
