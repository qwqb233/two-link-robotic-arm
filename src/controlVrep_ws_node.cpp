#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <iostream>
#include "../include/json.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "Matrix.h"
using json = nlohmann::json;

#define rad2deg(x) ((x) * 180.0 / M_PI)
#define deg2rad(x) ((x) * M_PI / 180.0)
#define MAX(x ,y) (x) = ((x) > (y) ? (x) : (y))
#define MIN(x ,y) (x) = ((x) < (y) ? (x) : (y))

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

        double theta1 = 0.0;
        double theta2 = 0.0;

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr targetPosition;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr basePosition;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr DHTable;

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr jointAnglesPublisher;

        rclcpp::TimerBase::SharedPtr jointAnglesTimer_;

    public:
        void targetPositionCallback(const std_msgs::msg::String::SharedPtr msg)
        {
            //RCLCPP_INFO(this->get_logger(), "Target Position: %s", msg->data.c_str());
            try
            {
                std::sscanf(msg->data.c_str(), "{x=%lf, y=%lf, z=%lf}", &targetPos.x, &targetPos.y, &targetPos.z);
                //RCLCPP_INFO(this->get_logger(), "Parsed Target Position: x=%lf, y=%lf, z=%lf", targetPos.x, targetPos.y, targetPos.z);
            }
            catch(...)
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid target position");
            }
            
        }
        void basePositionCallback(const std_msgs::msg::String::SharedPtr msg)
        {
            //RCLCPP_INFO(this->get_logger(), "Base Position: %s", msg->data.c_str());
            try
            {
                std::sscanf(msg->data.c_str(), "{x=%lf, y=%lf, z=%lf}", &basePos.x, &basePos.y, &basePos.z);
                //RCLCPP_INFO(this->get_logger(), "Parsed Base Position: x=%lf, y=%lf, z=%lf", basePos.x, basePos.y, basePos.z);
            }
            catch(...)
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid base position");
            }
        }
        void DHTableCallback(const std_msgs::msg::String::SharedPtr msg)
        {
            //RCLCPP_INFO(this->get_logger(), "DH Table: %s", msg->data.c_str());
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

        void inverseKinematics()
        {
            // Implement inverse kinematics logic here
            // This function will use targetPos, basePos, and DHTable_list to compute the joint angles
            RCLCPP_INFO(this->get_logger(), "Inverse Kinematics computation is not implemented yet.");
            double d_x = targetPos.x;
            double d_y = targetPos.y;

            double LL = targetPos.x * targetPos.x + targetPos.y * targetPos.y; 


            if (LL < 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid target position");
                return;
            }

            const double L1 = 1;
            const double L2 = 1;
            if(sqrt(LL) > (L1 + L2))
            {
                RCLCPP_ERROR(this->get_logger(), "Target position is out of reach");
                return;
            }
            
            double temp = (LL - L1 * L1 - L2 * L2)/(2 * L1 * L2);
            MIN(MAX(temp,-1),1);
            double temp_2 = sqrt(1 - temp * temp);
            theta2 = atan2(temp_2, temp);

            temp = L1 + L2 * temp;
            temp_2 = L2 * temp_2;
            theta1 = atan2(d_y, d_x) - atan2(temp_2, temp);

            double theta1_deg = rad2deg(theta1);
            double theta2_deg = rad2deg(theta2);

            if(theta2 == nan("") || theta1 == nan(""))
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid joint angles computed");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Computed Joint Angles: theta1=%lf, theta2=%lf", theta1_deg, theta2_deg);
        }

        void publishJointAngles()
        {
            std_msgs::msg::String jointAnglesMsg;
            // Here you would set the joint angles based on the inverse kinematics computation
            // For now, we will just send a placeholder message
            jointAnglesMsg.data = "Joint Angles: [" + std::to_string(theta1) + ", " + std::to_string(theta2) + "]";
            jointAnglesPublisher->publish(jointAnglesMsg);
            RCLCPP_INFO(this->get_logger(), "Published Joint Angles: %s", jointAnglesMsg.data.c_str());
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
            jointAnglesPublisher = this->create_publisher<std_msgs::msg::String>("/joint_angles", 10);
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
