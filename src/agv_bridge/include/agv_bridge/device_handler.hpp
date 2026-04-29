#pragma once

#include <rclcpp/rclcpp.hpp>
#include <jsoncpp/json/json.h>
#include "agv_protocol/command_parser.hpp"
#include <memory>
#include <string>

namespace agv_bridge {

/**
 * @brief 设备处理器基类
 */
class DeviceHandler {
public:
    using Ptr = std::shared_ptr<DeviceHandler>;
    virtual ~DeviceHandler() = default;

    // 初始化：由 Node 调用，用于 Handler 创建自己的 Publisher/Subscriber
    virtual void init(rclcpp::Node* node) = 0;

    // 命令处理：处理 payload 中对应自己的部分
    // 返回值：处理结果 Json，将直接放入响应包的对应设备 Key 下
    virtual Json::Value handleCommand(const Json::Value& device_payload, const agv_protocol::MessageHeader& header) = 0;

    // 状态检查：处理 msg_type 为 "check" 时的在线/离线检测逻辑
    virtual bool isOnline() = 0;
};

} // namespace agv_bridge
