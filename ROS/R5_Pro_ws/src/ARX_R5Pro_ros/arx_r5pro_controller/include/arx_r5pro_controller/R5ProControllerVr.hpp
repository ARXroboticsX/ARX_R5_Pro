#pragma once

#include <ros/ros.h>
#include "arx_r5pro_src/interfaces/InterfacesThread.hpp"
#include "arm_control/PosCmd.h"
#include <chrono>
#include <memory>

namespace arx
{
    class R5proController
    {
    public:
        R5proController(ros::NodeHandle nh);

        void CmdCallback(const arm_control::PosCmd::ConstPtr& msg);
        void PubState(const ros::TimerEvent&);

    private:
        std::shared_ptr<r5::InterfacesThread> l5_pro_ineterfaces_ptr_;

        ros::Publisher joint_state_publisher_;
        ros::Subscriber joint_state_subscriber_;
        ros::Timer timer_;
    };
}
