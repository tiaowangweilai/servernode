#pragma once

#include "agv_bridge/device_handler.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <mutex>

namespace agv_bridge {

/**
 * @brief 通用设备处理器基类，封装心跳判断逻辑
 */
class BaseHandler : public DeviceHandler {
public:
    void updateHeartbeat() {
        std::lock_guard<std::mutex> lock(mutex_);
        last_update_ = node_ ? node_->now() : rclcpp::Time(0, 0, RCL_ROS_TIME);
    }

    bool isOnline() override {
        if (!node_) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        return (node_->now() - last_update_).seconds() < 2.0;
    }

protected:
    rclcpp::Node* node_ = nullptr;
    rclcpp::Time last_update_{0, 0, RCL_ROS_TIME};
    std::mutex mutex_;
};

// ==========================================
// 底盘部分 (Chassis)
// ==========================================

class BaseChassisHandler : public BaseHandler {
protected:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sys_pub_;
};

/**
 * @brief 墙壁机器人底盘 (Vacuum Adsorption)
 */
class WallChassisHandler : public BaseChassisHandler {
public:
    void init(rclcpp::Node* node) override {
        node_ = node;
        manual_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_manual", 10);
        sys_pub_ = node->create_publisher<std_msgs::msg::String>("/mission/command", 10);
        
        sub_ = node->create_subscription<std_msgs::msg::String>(
            "/chassis/serial_status", 10, [this](const std_msgs::msg::String::SharedPtr) { updateHeartbeat(); });
    }

    Json::Value handleCommand(const Json::Value& data, const agv_protocol::MessageHeader&) override {
        Json::Value res;
        std::string cmd = data["command"].asString();
        
        if (cmd == "move") {
            const auto& p = data.isMember("parameters") ? data["parameters"] : data["data"];
            auto msg = geometry_msgs::msg::Twist();
            msg.linear.x = p.isMember("vx") ? p["vx"].asDouble() : (p.isMember("x") ? p["x"].asDouble() : 0.0);
            msg.linear.y = p.isMember("vy") ? p["vy"].asDouble() : (p.isMember("y") ? p["y"].asDouble() : 0.0);
            msg.angular.z = p.isMember("wz") ? p["wz"].asDouble() : (p.isMember("yaw") ? p["yaw"].asDouble() : 0.0);
            manual_pub_->publish(msg);
            res["result"] = "yes";
        } 
        else if (cmd == "single_scan") {
            Json::Value cmd_json;
            cmd_json["command"] = "single_scan";
            const auto& p = data.isMember("parameters") ? data["parameters"] : data;
            
            cmd_json["ig35_start"] = p.isMember("ig35_start") ? p["ig35_start"] : 11;
            cmd_json["ig35_end"] = p.isMember("ig35_end") ? p["ig35_end"] : 0;
            cmd_json["scan_speed"] = p.isMember("scan_speed") ? p["scan_speed"] : 20;
            
            auto msg = std_msgs::msg::String();
            Json::StreamWriterBuilder writer;
            msg.data = Json::writeString(writer, cmd_json);
            sys_pub_->publish(msg);
            res["result"] = "yes";
        }
        else {
            res["result"] = cmd + "_successed";
        }
        res["command"] = cmd;
        return res;
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr manual_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

/**
 * @brief 复合机器人底盘 (Mobile Dual Arm)
 */
class AgvChassisHandler : public BaseChassisHandler {
public:
    void init(rclcpp::Node* node) override {
        node_ = node;
        agv_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("cmd_vel", 10);
        sys_pub_ = node->create_publisher<std_msgs::msg::String>("/mission/sys_command", 10);
        
        sub_ = node->create_subscription<geometry_msgs::msg::Pose2D>(
            "/agv_pose", 10, [this](const geometry_msgs::msg::Pose2D::SharedPtr) { updateHeartbeat(); });
    }

    Json::Value handleCommand(const Json::Value& data, const agv_protocol::MessageHeader&) override {
        Json::Value res;
        std::string cmd = data["command"].asString();
        if (cmd == "move") {
            const auto& p = data.isMember("parameters") ? data["parameters"] : data["data"];
            auto msg = std_msgs::msg::Float64MultiArray();
            double vx = p.isMember("vx") ? p["vx"].asDouble() : (p.isMember("x") ? p["x"].asDouble() : 0.0);
            double vy = p.isMember("vy") ? p["vy"].asDouble() : (p.isMember("y") ? p["y"].asDouble() : 0.0);
            double wz = p.isMember("wz") ? p["wz"].asDouble() : (p.isMember("yaw") ? p["yaw"].asDouble() : 0.0);
            msg.data = {vx, vy, wz};
            agv_pub_->publish(msg);
            res["result"] = "yes";
        } else {
            res["result"] = cmd + "_successed";
        }
        res["command"] = cmd;
        return res;
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr agv_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr sub_;
};

// ==========================================
// 机械臂部分 (Arms)
// ==========================================

class ArmHandler : public BaseHandler {
public:
    void init(rclcpp::Node* node) override {
        node_ = node;
        joint_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("cmd_arm_joint", 10);
        cart_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("cmd_arm_cartesian", 10);
        
        auto qos = rclcpp::SensorDataQoS();
        sub_joint_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/arm_joint_states", qos, [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                updateHeartbeat();
                std::lock_guard<std::mutex> lock(data_mutex_);
                last_joint_ = msg;
            });

        sub_cart_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/arm_cartesian_pose", qos, [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                updateHeartbeat();
                std::lock_guard<std::mutex> lock(data_mutex_);
                last_cart_ = msg;
            });
            
        // 🌟 墙壁机器人可能使用标准的 /joint_states
        sub_standard_ = node->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, [this](const sensor_msgs::msg::JointState::SharedPtr) { updateHeartbeat(); });
    }

    Json::Value getReport() override {
        auto reports = getReports();
        return reports.empty() ? Json::Value(Json::arrayValue) : reports.front();
    }

    std::vector<Json::Value> getReports() override {
        std::lock_guard<std::mutex> lock(data_mutex_);
        std::vector<Json::Value> reports;
        if (last_joint_) reports.push_back(buildArmReport(last_joint_->data, "joint"));
        if (last_cart_) reports.push_back(buildArmReport(last_cart_->data, "cartesian"));
        return reports;
    }

    Json::Value handleCommand(const Json::Value& data, const agv_protocol::MessageHeader&) override {
        Json::Value res_array(Json::arrayValue);
        if (data.isArray()) {
            auto joint_msg = std_msgs::msg::Float64MultiArray();
            auto cart_msg = std_msgs::msg::Float64MultiArray();

            for (const auto& arm : data) {
                std::string cmd = arm["command"].asString();
                const auto& p = arm["parameters"];
                if (cmd == "joint_move") {
                    appendJointPoints(p, joint_msg.data);
                } else {
                    appendCartesianPoints(p, cart_msg.data);
                }
                Json::Value arm_res;
                arm_res["arm_id"] = arm["arm_id"];
                arm_res["command"] = cmd;
                arm_res["result"] = cmd + "_successed";
                res_array.append(arm_res);
            }

            if (!joint_msg.data.empty()) joint_pub_->publish(joint_msg);
            if (!cart_msg.data.empty()) cart_pub_->publish(cart_msg);
        }
        return res_array;
    }

private:
    static double numericOrZero(const Json::Value& value) { return value.isNumeric() ? value.asDouble() : 0.0; }

    static void appendJointPoint(const Json::Value& point, std::vector<double>& out) {
        for (int i = 1; i <= 6; ++i) out.push_back(numericOrZero(point["val" + std::to_string(i)]));
    }

    static void appendCartesianPoint(const Json::Value& point, std::vector<double>& out) {
        out.push_back(numericOrZero(point["x"]));
        out.push_back(numericOrZero(point["y"]));
        out.push_back(numericOrZero(point["z"]));
        out.push_back(point.isMember("rx") ? numericOrZero(point["rx"]) : numericOrZero(point["roll"]));
        out.push_back(point.isMember("ry") ? numericOrZero(point["ry"]) : numericOrZero(point["pitch"]));
        out.push_back(point.isMember("rz") ? numericOrZero(point["rz"]) : numericOrZero(point["yaw"]));
    }

    static void appendJointPoints(const Json::Value& parameters, std::vector<double>& out) {
        if (parameters.isArray()) { for (const auto& point : parameters) appendJointPoint(point, out); return; }
        if (parameters.isObject() && parameters["points"].isArray()) { for (const auto& point : parameters["points"]) appendJointPoint(point, out); return; }
        appendJointPoint(parameters, out);
    }

    static void appendCartesianPoints(const Json::Value& parameters, std::vector<double>& out) {
        if (parameters.isArray()) { for (const auto& point : parameters) appendCartesianPoint(point, out); return; }
        if (parameters.isObject() && parameters["points"].isArray()) { for (const auto& point : parameters["points"]) appendCartesianPoint(point, out); return; }
        appendCartesianPoint(parameters, out);
    }

    static double valueAt(const std::vector<double>& data, size_t index) { return index < data.size() ? data[index] : 0.0; }

    static Json::Value buildOneArm(const std::vector<double>& data, const std::string& arm_id, const std::string& pos_type, size_t offset) {
        Json::Value arm;
        arm["arm_id"] = arm_id;
        arm["pos_type"] = pos_type;
        if (pos_type == "joint") {
            for (int i = 0; i < 6; ++i) arm["joint" + std::to_string(i + 1)] = valueAt(data, offset + i);
        } else {
            arm["x"] = valueAt(data, offset + 0); arm["y"] = valueAt(data, offset + 1); arm["z"] = valueAt(data, offset + 2);
            arm["rx"] = valueAt(data, offset + 3); arm["ry"] = valueAt(data, offset + 4); arm["rz"] = valueAt(data, offset + 5);
        }
        return arm;
    }

    static Json::Value buildArmReport(const std::vector<double>& data, const std::string& pos_type) {
        Json::Value arms(Json::arrayValue);
        arms.append(buildOneArm(data, "left_arm", pos_type, 0));
        arms.append(buildOneArm(data, "right_arm", pos_type, 6));
        return arms;
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cart_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_joint_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_cart_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_standard_;
    std_msgs::msg::Float64MultiArray::SharedPtr last_joint_;
    std_msgs::msg::Float64MultiArray::SharedPtr last_cart_;
    std::mutex data_mutex_;
};

// ==========================================
// 雷达部分 (Radar)
// ==========================================

class BaseRadarHandler : public BaseHandler {
public:
    void init(rclcpp::Node* node) override {
        node_ = node;
        auto_pub_ = node->create_publisher<std_msgs::msg::String>("/cmd_vel_auto", 10);
        
        // 🌟 订阅主控规划好的路径话题
        path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
            "/mission/planned_path", 10, 
            [this](const nav_msgs::msg::Path::SharedPtr msg) {
                RCLCPP_INFO(node_->get_logger(), "📍 [ServerNode] 成功捕获主控下发的规划路径！点数: %zu", msg->poses.size());
            });
    }

    Json::Value getReport() override {
        Json::Value lidar;
        if (last_odom_) {
            lidar["x"] = last_odom_->pose.pose.position.x;
            lidar["y"] = last_odom_->pose.pose.position.y;
            lidar["angle"] = last_odom_->pose.pose.orientation.z; 
            lidar["target_idx"] = 0;
            lidar["loc_stat"] = isOnline() ? 1 : 0;
            lidar["state"] = isOnline() ? "running" : "stopped";
        }
        return lidar;
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

protected:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr auto_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    nav_msgs::msg::Odometry::SharedPtr last_odom_;
};

class RadarHandler : public BaseRadarHandler {
public:
    void init(rclcpp::Node* node) override {
        BaseRadarHandler::init(node);
        sub_odom_ = node->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                updateHeartbeat();
                last_odom_ = msg;
            });
    }
};

// ==========================================
// 相机部分 (Camera)
// ==========================================

class BaseCameraHandler : public BaseHandler {
public:
    void init(rclcpp::Node* node) override {
        node_ = node;
        click_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("/click_point", 10);
    }

    Json::Value handleCommand(const Json::Value& data, const agv_protocol::MessageHeader&) override {
        Json::Value res;
        std::string cmd = data["command"].asString();
        if (cmd == "image_pos") {
            auto msg = geometry_msgs::msg::PointStamped();
            msg.header.stamp = node_->now();
            msg.header.frame_id = "camera_color_optical_frame";
            msg.point.x = data["x"].asDouble();
            msg.point.y = data["y"].asDouble();
            msg.point.z = 0.0;
            click_pub_->publish(msg);
            res["result"] = "yes";
        } else {
            res["result"] = "yes";
        }
        res["command"] = cmd;
        return res;
    }

protected:
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr click_pub_;
};

class WallCameraHandler : public BaseCameraHandler {
public:
    void init(rclcpp::Node* node) override {
        BaseCameraHandler::init(node);
        auto qos = rclcpp::SensorDataQoS();
        sub_wall_ = node->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", qos, [this](const sensor_msgs::msg::Image::SharedPtr) { updateHeartbeat(); });
        sub_rect_ = node->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_rect_raw", qos, [this](const sensor_msgs::msg::Image::SharedPtr) { updateHeartbeat(); });
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_wall_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_rect_;
};

class AgvCameraHandler : public BaseCameraHandler {
public:
    void init(rclcpp::Node* node) override {
        BaseCameraHandler::init(node);
        auto qos = rclcpp::SensorDataQoS();
        sub_agv_ = node->create_subscription<sensor_msgs::msg::Image>(
            "/color/image_raw", qos, [this](const sensor_msgs::msg::Image::SharedPtr) { updateHeartbeat(); });
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_agv_;
};

} // namespace agv_bridge
