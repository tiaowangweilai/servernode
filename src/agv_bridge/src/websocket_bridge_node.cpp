#include <rclcpp/rclcpp.hpp>
#include "agv_protocol/websocket_client.hpp"
#include "agv_protocol/websocket_server.hpp"
#include "agv_protocol/command_parser.hpp"
#include "agv_bridge/robots.hpp"

namespace agv_bridge {

/**
 * @brief WebSocket 桥接节点 - 优化后的架构
 * 
 * 核心优化：
 * 1. 使用 Robot 抽象类封装不同机器人的差异逻辑 (启动命令、状态报告等)
 * 2. 使用 DeviceHandler 封装不同设备的底层操作 (指令解析、话题订阅等)
 * 3. 节点本身只负责数据传输和协议分发，不包含具体业务逻辑
 */
class WebSocketBridgeNode : public rclcpp::Node {
public:
    WebSocketBridgeNode() : Node("websocket_bridge_node") {
        this->declare_parameter("server_uri", "ws://192.168.137.65:9100");
        this->declare_parameter("local_server_port", 9001);

        // ... 初始化 WebSocket ...
        std::string server_uri = this->get_parameter("server_uri").as_string();
        int local_port = this->get_parameter("local_server_port").as_int();
        ws_client_ = std::make_unique<agv_protocol::WebSocketClientWrapper>(server_uri, 2000);
        ws_server_ = std::make_unique<agv_protocol::WebSocketServerWrapper>(local_port);

        auto cmd_handler = [this](const std::string& msg) { handleCommand(msg); };
        ws_client_->setMessageHandler(cmd_handler);
        ws_server_->setMessageHandler(cmd_handler);

        ws_client_->start();
        ws_server_->start();

        // 🌟 开启状态回传定时器 (5Hz)
        report_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200), std::bind(&WebSocketBridgeNode::sendPeriodicReport, this));

        RCLCPP_INFO(this->get_logger(), "模块化架构 WebSocket 桥接节点已启动。");
    }

    ~WebSocketBridgeNode() {
        if (launched_) {
            RCLCPP_INFO(this->get_logger(), "正在一键关闭机器人硬件系统...");
            int ret = 0;
            ret = std::system("pkill -9 -f agv_bringup");
            ret = std::system("pkill -9 -f wall_robot_pkg");
            (void)ret;
        }
    }

private:
    void sendPeriodicReport() {
        if (!launched_ || !robot_) return;

        // 通过 Robot 对象生成报告，支持多条消息并行发送
        auto reports = robot_->generateReports(parser_);
        for (const auto& msg : reports) {
            ws_client_->send(msg);
        }
    }

    void startLaunch(const std::string& robot_id) {
        if (launched_) return;

        // 1. 根据 ID 创建对应的机器人实例
        robot_ = RobotFactory::createRobot(robot_id);
        robot_->init(this);

        // 2. 获取并执行启动命令
        std::string cmd = robot_->getLaunchCommand();
        RCLCPP_INFO(this->get_logger(), "正在根据机器人类型 [%s] 启动系统: %s", robot_id.c_str(), cmd.c_str());
        
        if (std::system(cmd.c_str()) == 0) {
            launched_ = true;
        }
    }

    void handleCommand(const std::string& json_str) {
        RCLCPP_INFO(this->get_logger(), "收到上位机指令: %s", json_str.c_str());
        
        auto raw_opt = parser_.parseRaw(json_str);
        if (!raw_opt) {
            RCLCPP_ERROR(this->get_logger(), "指令解析失败!");
            return;
        }
        const auto& raw_msg = *raw_opt;
        const std::string& robot_id = raw_msg.header.robot_id;

        Json::Value response_payload;

        if (raw_msg.header.msg_type == "check") {
            startLaunch(robot_id);
            if (robot_) {
                response_payload = robot_->handleCheck(raw_msg);
            }
        } else {
            if (robot_) {
                response_payload = robot_->handleCommand(raw_msg);
            } else {
                RCLCPP_WARN(this->get_logger(), "收到指令但机器人尚未初始化 (未收到 check)!");
            }
        }

        if (!response_payload.empty()) {
            std::string resp = parser_.buildRawMessage(robot_id, "response", response_payload);
            RCLCPP_INFO(this->get_logger(), "回复上位机: %s", resp.c_str());
            ws_client_->send(resp);
        }
    }

    agv_protocol::CommandParser parser_;
    std::unique_ptr<agv_protocol::WebSocketClientWrapper> ws_client_;
    std::unique_ptr<agv_protocol::WebSocketServerWrapper> ws_server_;
    
    // 核心：使用抽象接口
    Robot::Ptr robot_;
    
    rclcpp::TimerBase::SharedPtr report_timer_;
    bool launched_ = false;
};

} // namespace agv_bridge

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<agv_bridge::WebSocketBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
