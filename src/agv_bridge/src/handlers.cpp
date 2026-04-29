#include "agv_bridge/device_handler.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <mutex>

namespace agv_bridge {

/**
 * @brief AGV 底盘处理器
 */
class ChassisHandler : public DeviceHandler {
public:
    void init(rclcpp::Node* node) override {
        node_ = node;
        // 手动控制：标准 Twist
        manual_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_manual", 10);
        // 系统任务控制：String JSON
        sys_pub_ = node->create_publisher<std_msgs::msg::String>("/mission/sys_command", 10);
        
        // 订阅状态话题用于自检
        sub_ = node->create_subscription<std_msgs::msg::String>(
            "/chassis/serial_status", 10, [this](const std_msgs::msg::String::SharedPtr) {
                last_update_ = node_->now();
            });
    }

    Json::Value handleCommand(const Json::Value& data, const agv_protocol::MessageHeader&) override {
        Json::Value res;
        std::string cmd = data["command"].asString();
        
        if (cmd == "move") {
            const auto& p = data.isMember("parameters") ? data["parameters"] : data["data"];
            auto msg = geometry_msgs::msg::Twist();
            double vx = p.isMember("vx") ? p["vx"].asDouble() : (p.isMember("x") ? p["x"].asDouble() : 0.0);
            double vy = p.isMember("vy") ? p["vy"].asDouble() : (p.isMember("y") ? p["y"].asDouble() : 0.0);
            double wz = p.isMember("wz") ? p["wz"].asDouble() : (p.isMember("yaw") ? p["yaw"].asDouble() : 0.0);
            msg.linear.x = vx; msg.linear.y = vy; msg.angular.z = wz;
            manual_pub_->publish(msg);
            res["result"] = "yes";
        } 
        else if (cmd == "single_scan") {
            // 完整转发 single_scan 载荷到系统命令话题
            auto msg = std_msgs::msg::String();
            Json::StreamWriterBuilder writer;
            msg.data = Json::writeString(writer, data);
            sys_pub_->publish(msg);
            res["result"] = "yes";
        }
        else {
            res["result"] = cmd + "_successed";
        }
        res["command"] = cmd;
        return res;
    }

    bool isOnline() override {
        if (!node_) return false;
        return (node_->now() - last_update_).seconds() < 2.0;
    }

private:
    rclcpp::Node* node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr manual_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sys_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::Time last_update_{0, 0, RCL_ROS_TIME};
};

/**
 * @brief 机械臂处理器
 */
class ArmHandler : public DeviceHandler {
public:
    void init(rclcpp::Node* node) override {
        node_ = node;
        joint_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("cmd_arm_joint", 10);
        cart_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("cmd_arm_cartesian", 10);
        sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, [this](const sensor_msgs::msg::JointState::SharedPtr) {
                last_update_ = node_->now();
            });
    }

    Json::Value handleCommand(const Json::Value& data, const agv_protocol::MessageHeader&) override {
        Json::Value res_array(Json::arrayValue);
        if (data.isArray()) {
            for (const auto& arm : data) {
                std::string cmd = arm["command"].asString();
                const auto& p = arm["parameters"];
                auto msg = std_msgs::msg::Float64MultiArray();
                if (cmd == "joint_move") {
                    for(int i=1; i<=6; ++i) msg.data.push_back(p["val"+std::to_string(i)].asDouble());
                    joint_pub_->publish(msg);
                } else {
                    msg.data = {p["x"].asDouble(), p["y"].asDouble(), p["z"].asDouble(), 
                                p["roll"].asDouble(), p["pitch"].asDouble(), p["yaw"].asDouble()};
                    cart_pub_->publish(msg);
                }
                Json::Value arm_res;
                arm_res["arm_id"] = arm["arm_id"];
                arm_res["command"] = cmd;
                arm_res["result"] = "yes";
                res_array.append(arm_res);
            }
        }
        return res_array;
    }

    bool isOnline() override {
        if (!node_) return false;
        return (node_->now() - last_update_).seconds() < 2.0;
    }

private:
    rclcpp::Node* node_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cart_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;

    rclcpp::Time last_update_{0, 0, RCL_ROS_TIME};
};

/**
 * @brief 雷达处理器
 */
class RadarHandler : public DeviceHandler {
public:
    void init(rclcpp::Node* node) override {
        node_ = node;
        auto_pub_ = node->create_publisher<std_msgs::msg::String>("/cmd_vel_auto", 10);
        sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr) {
                last_update_ = node_->now();
            });
    }

    Json::Value handleCommand(const Json::Value& data, const agv_protocol::MessageHeader&) override {
        Json::Value res;
        std::string cmd = data["command"].asString();
        if (cmd == "nav_path") {
            auto msg = std_msgs::msg::String();
            Json::StreamWriterBuilder writer;
            msg.data = Json::writeString(writer, data);
            auto_pub_->publish(msg);
            res["result"] = "yes";
        } else {
            res["result"] = "yes";
        }
        res["command"] = cmd;
        return res;
    }

    bool isOnline() override {
        if (!node_) return false;
        return (node_->now() - last_update_).seconds() < 2.0;
    }
private:
    rclcpp::Node* node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr auto_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    rclcpp::Time last_update_{0, 0, RCL_ROS_TIME};
};

/**
 * @brief 相机处理器
 */
class CameraHandler : public DeviceHandler {
public:
    void init(rclcpp::Node* node) override {
        node_ = node;
        sub_ = node->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", rclcpp::SensorDataQoS(), 
            [this](const sensor_msgs::msg::Image::SharedPtr) {
                last_update_ = node_->now();
            });
        sub_rect = node->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_rect_raw", rclcpp::SensorDataQoS(), 
            [this](const sensor_msgs::msg::Image::SharedPtr) {
                last_update_ = node_->now();
            });
    }

    Json::Value handleCommand(const Json::Value&, const agv_protocol::MessageHeader&) override {
        Json::Value res; res["result"] = "yes"; return res;
    }
    bool isOnline() override {
        if (!node_) return false;
        return (node_->now() - last_update_).seconds() < 2.0;
    }
private:
    rclcpp::Node* node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_rect;
    rclcpp::Time last_update_{0, 0, RCL_ROS_TIME};
};

} // namespace agv_bridge
