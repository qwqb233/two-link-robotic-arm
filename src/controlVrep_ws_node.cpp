#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <iostream>
#include "../include/json.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "Matrix.h"
using json = nlohmann::json;

// 宏定义相关函数
#define rad2deg(x) ((x) * 180.0 / M_PI)
#define deg2rad(x) ((x) * M_PI / 180.0)
#define MAX(x ,y) (x) = ((x) > (y) ? (x) : (y))
#define MIN(x ,y) (x) = ((x) < (y) ? (x) : (y))

// 控制V-REP工作空间节点
class controlVrepWsNode: public rclcpp::Node
{
    private:
        // 目标点和基座点的坐标类型
        typedef struct position
        {
            double x;
            double y;
            double z;
        } position_t;

        // 目标点和基座点的坐标
        position_t targetPos= {0.0, 0.0, 0.0};
        position_t basePos = {0.0, 0.0, 0.0};

        // 关节角
        double theta1 = 0.0;
        double theta2 = 0.0;

        // 创建订阅者
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr targetPosition;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr basePosition;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr DHTable;

        // 创建发布者
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr jointAnglesPublisher;

        // 创建定时器,用于周期性计算关节角并发布关节角
        rclcpp::TimerBase::SharedPtr jointAnglesTimer_;

    public:
        // 目标位置回调
        void targetPositionCallback(const std_msgs::msg::String::SharedPtr msg)
        {
            //RCLCPP_INFO(this->get_logger(), "Target Position: %s", msg->data.c_str());
            try
            {

                // 解析目标位置，并写入targetPos
                std::sscanf(msg->data.c_str(), "{x=%lf, y=%lf, z=%lf}", &targetPos.x, &targetPos.y, &targetPos.z);
                //RCLCPP_INFO(this->get_logger(), "Parsed Target Position: x=%lf, y=%lf, z=%lf", targetPos.x, targetPos.y, targetPos.z);
            }
            catch(...)
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid target position");
            }
            
        }
        // 基座位置回调
        void basePositionCallback(const std_msgs::msg::String::SharedPtr msg)
        {
            //RCLCPP_INFO(this->get_logger(), "Base Position: %s", msg->data.c_str());
            try
            {
                // 解析基座位置，并写入basePos
                std::sscanf(msg->data.c_str(), "{x=%lf, y=%lf, z=%lf}", &basePos.x, &basePos.y, &basePos.z);
                //RCLCPP_INFO(this->get_logger(), "Parsed Base Position: x=%lf, y=%lf, z=%lf", basePos.x, basePos.y, basePos.z);
            }
            catch(...)
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid base position");
            }
        }

        // 逆运动学
        void inverseKinematics()
        {
            // Implement inverse kinematics logic here
            // This function will use targetPos, basePos, and DHTable_list to compute the joint angles
            RCLCPP_INFO(this->get_logger(), "Inverse Kinematics computation is not implemented yet.");
            // 计算目标位置与基座位置之间的相对位置
            double d_x = targetPos.x  - basePos.x;
            double d_y = targetPos.y - basePos.y;

            // 计算斜边
            double LL = targetPos.x * targetPos.x + targetPos.y * targetPos.y; 
            if (LL < 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid target position");
                return;
            }

            // 设定连杆长度
            const double L1 = 1;
            const double L2 = 1;
            // 当目标位置超出工作空间不更新关节角
            if(sqrt(LL) > (L1 + L2))
            {
                RCLCPP_ERROR(this->get_logger(), "Target position is out of reach");
                return;
            }
            // 计算关节角
            double temp = (LL - L1 * L1 - L2 * L2)/(2 * L1 * L2);
            MIN(MAX(temp,-1),1);
            double temp_2 = sqrt(1 - temp * temp);
            theta2 = atan2(temp_2, temp);

            temp = L1 + L2 * temp;
            temp_2 = L2 * temp_2;
            theta1 = atan2(d_y, d_x) - atan2(temp_2, temp);

            // 将弧度转换为角度，用于显示
            double theta1_deg = rad2deg(theta1);
            double theta2_deg = rad2deg(theta2);
            RCLCPP_INFO(this->get_logger(), "Computed Joint Angles: theta1=%lf, theta2=%lf", theta1_deg, theta2_deg);
        }

        // 发布关节角
        void publishJointAngles()
        {
            // 定义消息
            std_msgs::msg::String jointAnglesMsg;
            // 将关节角转换为字符串并发布
            // 注意：这里假设关节角是以弧度为单位的
            jointAnglesMsg.data = "Joint Angles: [" + std::to_string(theta1) + ", " + std::to_string(theta2) + "]";
            jointAnglesPublisher->publish(jointAnglesMsg);
            RCLCPP_INFO(this->get_logger(), "Published Joint Angles: %s", jointAnglesMsg.data.c_str());
        }

        controlVrepWsNode()
        : Node("control_vrep_ws_node")
        {
            RCLCPP_INFO(this->get_logger(), "Control VREP WebSocket Node has been started.");
            // 创建订阅者,节点名："/target_position"和"/base_position"
            targetPosition = this->create_subscription<std_msgs::msg::String>("/target_position", 10,
                std::bind(&controlVrepWsNode::targetPositionCallback, this, std::placeholders::_1));
            basePosition = this->create_subscription<std_msgs::msg::String>("/base_position", 10,
                std::bind(&controlVrepWsNode::basePositionCallback, this, std::placeholders::_1));
            // DHTable = this->create_subscription<std_msgs::msg::String>("/dh_parameters", 10,
            //     std::bind(&controlVrepWsNode::DHTableCallback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Subscriptions to /target_position, /base_position, and /dh_table created.");
            // 创建发布者,节点名："/joint_angles"
            // 这里假设关节角是以弧度为单位的
            jointAnglesPublisher = this->create_publisher<std_msgs::msg::String>("/joint_angles", 10);
            //创建定时器，每100毫秒调用一次inverseKinematics和publishJointAngles函数
            jointAnglesTimer_ = this->create_wall_timer(
                std::chrono::milliseconds(100), 
                [this]() -> void{
                    this->inverseKinematics();
                    this->publishJointAngles();
                }
            );
        }

        ~controlVrepWsNode()
        {
            RCLCPP_INFO(this->get_logger(), "Control VREP WebSocket Node is being destroyed.");
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<controlVrepWsNode>());
    rclcpp::shutdown();
    return 0;
}
