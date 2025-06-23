/*
RRST-NHK-Project 2025
PS4コントローラーの入力を取得するサンプルプログラム
*/

// 標準
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class PS4_Listener : public rclcpp::Node {
public:
    PS4_Listener()
        : Node("ps4_listener") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&PS4_Listener::ps4_listener_callback, this,
                      std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(),
                    "PS4 Listener initialized");
    }

private:
    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (msg->buttons.size() >= 14) {
            bool CROSS = msg->buttons[0];   // Xボタン
            bool CIRCLE = msg->buttons[1];  //○ボタン
            bool TRIANGLE = msg->buttons[2];  // △ボタン
            bool SQUARE = msg->buttons[3];  // □ボタン

            if (CROSS) {
                 std::cout << "CROSS" << std::endl; // X
            }
            if (CIRCLE) {
                std::cout << "CIRCLE" << std::endl;
            }
            if (TRIANGLE) {
                std::cout << "TRIANGLE" << std::endl;
            }
            if (SQUARE) {
                std::cout << "SQUARE" << std::endl;
            }
        }

        if (msg->axes.size() >= 5) {
            float left_stick_x = msg->axes[0];  // 左スティックX軸
            float left_stick_y = msg->axes[1];  // 左スティックY軸
            std::cout << "左スティック X: " << left_stick_x << " Y: " << left_stick_y << std::endl;

            float right_stick_x = msg->axes[3];  // 右スティックX軸
            float right_stick_y = msg->axes[4];  // 右スティックY軸
            std::cout << "右スティック X: " << right_stick_x << " Y: " << right_stick_y << std::endl;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exec;
    auto ps4_listener = std::make_shared<PS4_Listener>();
    exec.add_node(ps4_listener);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}