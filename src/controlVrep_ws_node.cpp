#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <iostream>
#include "../include/json.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "Matrix.h"
using json = nlohmann::json;


class controlVrepWsNode: public rclcpp::Node
{
    private:
        typedef struct DHTable
        {
            double a;
            double alpha;
            double d;
            double theta;
        } DHTable_t;

        typedef struct position
        {
            double x;
            double y;
            double z;
        } position_t;

        DHTable_t DH_F = {0.0, 0.0, 0.0, 0.0};
        DHTable_t DH_B = {0.0, 0.0, 0.0, 0.0};
        DHTable_t DHTable_list[2] = {DH_F, DH_B};

        position_t targetPos= {0.0, 0.0, 0.0};
        position_t basePos = {0.0, 0.0, 0.0};

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr targetPosition;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr basePosition;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr DHTable;


    public:
        void targetPositionCallback(const std_msgs::msg::String::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "Target Position: %s", msg->data.c_str());
            try
            {
                std::sscanf(msg->data.c_str(), "{x=%lf, y=%lf, z=%lf}", &targetPos.x, &targetPos.y, &targetPos.z);
                RCLCPP_INFO(this->get_logger(), "Parsed Target Position: x=%lf, y=%lf, z=%lf", targetPos.x, targetPos.y, targetPos.z);
            }
            catch(...)
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid target position");
            }
            
        }
        void basePositionCallback(const std_msgs::msg::String::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "Base Position: %s", msg->data.c_str());
            try
            {
                std::sscanf(msg->data.c_str(), "{x=%lf, y=%lf, z=%lf}", &basePos.x, &basePos.y, &basePos.z);
                RCLCPP_INFO(this->get_logger(), "Parsed Base Position: x=%lf, y=%lf, z=%lf", basePos.x, basePos.y, basePos.z);
            }
            catch(...)
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid base position");
            }
        }
        void DHTableCallback(const std_msgs::msg::String::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "DH Table: %s", msg->data.c_str());
            json data = json::parse(msg->data);
            try
            {
                DH_F.a = (double)data["1"]["a"];
                DH_F.alpha = (double)data["1"]["alpha"];
                DH_F.d = (double)data["1"]["d"];
                DH_F.theta = (double)data["1"]["theta"];

                DH_B.a = (double)data["2"]["a"];
                DH_B.alpha = (double)data["2"]["alpha"];
                DH_B.d = (double)data["2"]["d"];
                DH_B.theta = (double)data["2"]["theta"];

                DHTable_list[0] = DH_F;
                DHTable_list[1] = DH_B;
            }
            catch(...)
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid DH parameters");
            }
            for(int i = 0;i < 2;i++)
            {
                int rot_dim[] = {4,4};
                int dim_size = 2;
                double TAData[] = {
                    1,0,0,DHTable_list[i].a,
                    0,cos(DHTable_list[i].alpha),-sin(DHTable_list[i].alpha),0,
                    0,sin(DHTable_list[i].alpha),cos(DHTable_list[i].alpha),0,
                    0,0,0,1
                };
                double TBData[] = {
                    cos(DHTable_list[i].theta),-sin(DHTable_list[i].theta),0,0,
                    sin(DHTable_list[i].theta),cos(DHTable_list[i].theta),0,0,
                    0,0,1,DHTable_list[i].d,
                    0,0,0,1
                };
                Matrix TA(rot_dim, dim_size, TAData);
                Matrix TB(rot_dim, dim_size, TBData);
                Matrix T = TA * TB;
            }
        }

        controlVrepWsNode()
        : Node("control_vrep_ws_node")
        {
            RCLCPP_INFO(this->get_logger(), "Control VREP WebSocket Node has been started.");
            targetPosition = this->create_subscription<std_msgs::msg::String>("/target_position", 10,
                std::bind(&controlVrepWsNode::targetPositionCallback, this, std::placeholders::_1));
            basePosition = this->create_subscription<std_msgs::msg::String>("/base_position", 10,
                std::bind(&controlVrepWsNode::basePositionCallback, this, std::placeholders::_1));
            // DHTable = this->create_subscription<std_msgs::msg::String>("/dh_parameters", 10,
            //     std::bind(&controlVrepWsNode::DHTableCallback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Subscriptions to /target_position, /base_position, and /dh_table created.");
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
