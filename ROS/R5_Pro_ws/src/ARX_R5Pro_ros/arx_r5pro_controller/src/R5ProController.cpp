#include "arx_r5pro_controller/R5ProController.hpp"

// using namespace std::chrono_literals;

namespace arx
{
    R5proController::R5proController(ros::NodeHandle nh)
    {
        // 创建发布器
        joint_state_publisher_ = nh.advertise<arx_r5pro_msg::RobotStatus>("/r5pro_status", 10);
        // // 创建订阅器
        joint_state_subscriber_ = nh.subscribe<arx_r5pro_msg::RobotCmd>(
            "/r5pro_cmd", 10, &R5proController::CmdCallback, this);
        // 定时器，用于发布关节信息

        timer_ = nh.createTimer(ros::Duration(0.01), &R5proController::PubState, this);
        interfaces_thread_ptr_ = std::make_shared<r5::InterfacesThread>("can0", 0);
    }

    void R5proController::CmdCallback(const arx_r5pro_msg::RobotCmd::ConstPtr& msg)
    {
        double end_pos[6] = {
            msg->end_pos[0], msg->end_pos[1], msg->end_pos[2], msg->end_pos[3], msg->end_pos[4], msg->end_pos[5]
        };

        Eigen::Isometry3d transform = solve::Xyzrpy2Isometry(end_pos);

        interfaces_thread_ptr_->setEndPose(transform);

        std::vector<double> joint_positions = {0, 0, 0, 0, 0, 0};

        for (int i = 0; i < 6; i++)
        {
            joint_positions[i] = msg->joint_pos[i];
        }

        interfaces_thread_ptr_->setJointPositions(joint_positions);

        interfaces_thread_ptr_->setArmStatus(msg->mode);

        interfaces_thread_ptr_->setCatch(msg->gripper);
    }

    void R5proController::PubState(const ros::TimerEvent&)
    {
        arx_r5pro_msg::RobotStatus msg;
        msg.header.stamp = ros::Time::now();

        Eigen::Isometry3d transform = interfaces_thread_ptr_->getEndPose();

        /*
        // 提取四元数和位移
        Eigen::Quaterniond quat(transform.rotation());
        Eigen::Vector3d translation = transform.translation();

        // 创建长度为7的vector
        
        */

        // 填充vector
        boost::array<double, 6> result;

        std::vector<double> xyzrpy = {0, 0, 0, 0, 0, 0};
        xyzrpy = solve::Isometry2Xyzrpy(transform);

        result[0] = xyzrpy[0];
        result[1] = xyzrpy[1];
        result[2] = xyzrpy[2];
        result[3] = xyzrpy[3];
        result[4] = xyzrpy[4];
        result[5] = xyzrpy[5];

        msg.end_pos = result;

        std::vector<double> joint_pos_vector = interfaces_thread_ptr_->getJointPositons();
        for (int i = 0; i < 7; i++)
        {
            msg.joint_pos[i] = joint_pos_vector[i];
        }

        std::vector<double> joint_velocities_vector = interfaces_thread_ptr_->getJointVelocities();
        for (int i = 0; i < 7; i++)
        {
            msg.joint_vel[i] = joint_velocities_vector[i];
        }

        std::vector<double> joint_current_vector = interfaces_thread_ptr_->getJointCurrent();
        for (int i = 0; i < 7; i++)
        {
            msg.joint_cur[i] = joint_current_vector[i];
        }

        // 发布消息
        ROS_INFO("Publishing RobotStatus message");
        joint_state_publisher_.publish(msg);
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "r5pro_controller");
    ros::NodeHandle nh = ros::NodeHandle("~");
    arx::R5proController controller(nh);
    ros::spin();
    return 0;
}
