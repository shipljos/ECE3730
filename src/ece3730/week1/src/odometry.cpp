// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class Odometry : public rclcpp::Node
{
public:
  Odometry()
  : Node("odometry"), vx_(0), x_(0), last_t_(0)
  {
    auto topic_callback =
      [this](sensor_msgs::msg::Imu::UniquePtr msg) -> void {
        double t = msg->header.stamp.sec+ (_Float64)msg->header.stamp.nanosec*1e-9;
        auto ax = msg->linear_acceleration.x;
        RCLCPP_INFO(this->get_logger(), "time is: '%.9f'", t);
        if(this->last_t_!=0){
          auto dt = t-this->last_t_;
          this->vx_ += ax * dt;
          this->x_ += this->vx_ * dt;
        }
        this->last_t_ = t;
        auto message = geometry_msgs::msg::PoseStamped();
        message.header.stamp = msg->header.stamp;
        message.pose.position.x = this->x_;
        message.header.frame_id = "odom";
        this->publisher_->publish(message);
      };
    subscription_ =
      this->create_subscription<sensor_msgs::msg::Imu>("imu", 10, topic_callback);
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  _Float64 vx_, x_; //global floating point vlaues for velocity of x and position of x
  double last_t_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}
