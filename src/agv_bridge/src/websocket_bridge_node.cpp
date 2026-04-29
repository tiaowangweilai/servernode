#include <rclcpp/rclcpp.hpp>
#include "agv_protocol/websocket_client.hpp"
#include "agv_protocol/websocket_server.hpp"
#include "agv_protocol/command_parser.hpp"
#include "agv_bridge/device_handler.hpp"

// 临时引入刚才定义的 Handler 实现 (实际工程建议拆分为独立头文件)
#include "handlers.cpp" 

namespace agv_bridge {

class WebSocketBridgeNode : public rclcpp::Node {
public:
    WebSocketBridgeNode() : Node("websocket_bridge_node") {
        this->declare_parameter("server_uri", "ws://192.168.137.134:9100");
        this->declare_parameter("local_server_port", 9001);

        // 1. 注册设备处理器 (以后加新设备，只需要加一行 registerHandler)
        registerHandler("agv", std::make_shared<ChassisHandler>());
        registerHandler("chassis", handlers_["agv"]); 
        registerHandler("arms", std::make_shared<ArmHandler>());
        registerHandler("radar", std::make_shared<RadarHandler>());
        registerHandler("lidar", handlers_["radar"]); 
        registerHandler("camera", std::make_shared<CameraHandler>());

        // 2. 初始化 WebSocket
        std::string server_uri = this->get_parameter("server_uri").as_string();
        int local_port = this->get_parameter("local_server_port").as_int();
        ws_client_ = std::make_unique<agv_protocol::WebSocketClientWrapper>(server_uri, 2000);
        ws_server_ = std::make_unique<agv_protocol::WebSocketServerWrapper>(local_port);

        auto cmd_handler = [this](const std::string& msg) { handleCommand(msg); };
        ws_client_->setMessageHandler(cmd_handler);
        ws_server_->setMessageHandler(cmd_handler);

        ws_client_->start();
        ws_server_->start();

        RCLCPP_INFO(this->get_logger(), "动态分发架构 WebSocket 桥接节点已启动。");
    }

    ~WebSocketBridgeNode() {
        if (launched_) {
            RCLCPP_INFO(this->get_logger(), "正在一键关闭机器人硬件系统...");
            // 使用变量接收返回值并强制转换为 void，以消除 warn_unused_result 警告
            int ret = 0;
            ret = std::system("pkill -9 -f agv_bringup");
            ret = std::system("pkill -9 -f wall_robot_pkg");
            (void)ret;
        }
    }

private:
    void startLaunch(const std::string& robot_id) {
        if (launched_) return;

        std::string cmd;
        if (robot_id == "vacuum_adsorption_robot") {
            cmd = "ros2 launch wall_robot_pkg system_bringup.launch.py &";
        } else {
            // 默认为双臂机器人
            cmd = "ros2 launch agv_bringup agv.launch.xml &";
        }

        RCLCPP_INFO(this->get_logger(), "正在根据自检信号启动系统: %s", cmd.c_str());
        if (std::system(cmd.c_str()) == 0) {
            launched_ = true;
        }
    }

    void registerHandler(const std::string& key, DeviceHandler::Ptr handler) {
        handler->init(this);
        handlers_[key] = handler;
    }

    void handleCommand(const std::string& json_str) {
        RCLCPP_INFO(this->get_logger(), "收到上位机指令: %s", json_str.c_str());
        
        auto raw_opt = parser_.parseRaw(json_str);
        if (!raw_opt) {
            RCLCPP_ERROR(this->get_logger(), "指令解析失败!");
            return;
        }
        const auto& raw_msg = *raw_opt;

        Json::Value response_payload;

        // --- 逻辑分发 ---
        if (raw_msg.header.msg_type == "check") {
            // 1. 触发自动启动
            startLaunch(raw_msg.header.robot_id);

            // 2. 精准获取请求中指定的设备状态
            for (const auto& key : raw_msg.payload.getMemberNames()) {
                if (handlers_.count(key)) {
                    Json::Value dev_res;
                    dev_res["command"] = raw_msg.payload[key]["command"].asString();
                    dev_res["result"] = handlers_[key]->isOnline() ? "yes" : "no";
                    response_payload[key] = dev_res;
                }
            }
        } else {
            // 根据 Payload 里的 Key 自动找到对应的 Handler 处理
            for (const auto& key : raw_msg.payload.getMemberNames()) {
                if (handlers_.count(key)) {
                    response_payload[key] = handlers_[key]->handleCommand(raw_msg.payload[key], raw_msg.header);
                }
            }
        }

        // --- 发送响应 ---
        if (!response_payload.empty()) {
            std::string resp = parser_.buildRawMessage(raw_msg.header.robot_id, "response", response_payload);
            RCLCPP_INFO(this->get_logger(), "回复上位机: %s", resp.c_str());
            ws_client_->send(resp);
        }
    }

    agv_protocol::CommandParser parser_;
    std::unique_ptr<agv_protocol::WebSocketClientWrapper> ws_client_;
    std::unique_ptr<agv_protocol::WebSocketServerWrapper> ws_server_;
    std::map<std::string, DeviceHandler::Ptr> handlers_;
    bool launched_ = false;
};

} // namespace agv_bridge

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<agv_bridge::WebSocketBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
