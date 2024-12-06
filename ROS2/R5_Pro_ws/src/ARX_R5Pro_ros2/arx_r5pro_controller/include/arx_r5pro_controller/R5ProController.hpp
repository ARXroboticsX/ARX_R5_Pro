#pragma once

#include <rclcpp/rclcpp.hpp>
#include "arx_r5pro_src/interfaces/InterfacesThread.hpp"
#include "arx_r5pro_msg/msg/robot_cmd.hpp"
#include "arx_r5pro_msg/msg/robot_status.hpp"
#include <chrono>
#include <memory>

namespace arx::r5
{
    class R5ProController : public rclcpp::Node
    {
    public:
        R5ProController();

        void CmdCallback(const arx_r5pro_msg::msg::RobotCmd::SharedPtr msg);
        void PubState();
        
    private:
        std::shared_ptr<InterfacesThread> interfaces_ptr_;

        rclcpp::Publisher<arx_r5pro_msg::msg::RobotStatus>::SharedPtr joint_state_publisher_;
        rclcpp::Subscription<arx_r5pro_msg::msg::RobotCmd>::SharedPtr joint_state_subscriber_;
        rclcpp::TimerBase::SharedPtr timer_;
    };
}