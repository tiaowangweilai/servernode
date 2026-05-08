#pragma once

#include "agv_bridge/robot.hpp"
#include "agv_bridge/handlers.hpp"

namespace agv_bridge {

/**
 * @brief 复合移动双臂机器人
 */
class MobileDualArmRobot : public BaseRobot {
public:
    MobileDualArmRobot() : BaseRobot("mobile_dual_arm_robot") {
        // 使用针对 AGV 特化的子类
        auto chassis = std::make_shared<AgvChassisHandler>();
        addHandler("agv", chassis);
        addHandler("chassis", chassis);
        
        auto arms = std::make_shared<ArmHandler>();
        addHandler("arms", arms);
        
        auto radar = std::make_shared<RadarHandler>();
        addHandler("radar", radar);
        addHandler("lidar", radar);
        
        addHandler("camera", std::make_shared<AgvCameraHandler>());
    }

    std::string getLaunchCommand() override {
        return "ros2 launch agv_bringup agv.launch.xml sim:=false 2>&1 | grep -v 'Overrun' & "
               "ros2 launch robot_go_target gotarget.launch.py type:=agv &";
    }

    std::vector<std::string> generateReports(agv_protocol::CommandParser& parser) override {
        std::vector<std::string> reports;
        
        if (handlers_.count("arms")) {
            for (const auto& arm_report : handlers_["arms"]->getReports()) {
                if (arm_report.empty()) continue;
                Json::Value arm_payload;
                arm_payload["arms"] = arm_report;
                reports.push_back(parser.buildRawMessage(robot_id_, "status_update", arm_payload));
            }
        }
        
        Json::Value other_payload;
        for (auto const& [name, handler] : handlers_) {
            if (name == "arms") continue;
            if (name == "chassis" || name == "lidar") continue; 
            
            Json::Value r = handler->getReport();
            if (!r.empty()) other_payload[name] = r;
        }
        
        if (!other_payload.empty()) {
            reports.push_back(parser.buildRawMessage(robot_id_, "status_update", other_payload));
        }

        return reports;
    }
};

/**
 * @brief 气吸附机器人
 */
class VacuumAdsorptionRobot : public BaseRobot {
public:
    VacuumAdsorptionRobot() : BaseRobot("vacuum_adsorption_robot") {
        // 使用针对 墙壁机器人 特化的子类
        auto chassis = std::make_shared<WallChassisHandler>();
        addHandler("agv", chassis);
        addHandler("chassis", chassis);
        
        auto radar = std::make_shared<RadarHandler>();
        addHandler("radar", radar);
        addHandler("lidar", radar);

        addHandler("camera", std::make_shared<WallCameraHandler>());
    }

    std::string getLaunchCommand() override {
        return "ros2 launch wall_robot_pkg system_bringup.launch.py &";
    }

    std::vector<std::string> generateReports(agv_protocol::CommandParser& parser) override {
        Json::Value payload;
        // 气吸附机器人上报底盘和雷达状态
        if (handlers_.count("chassis")) {
            payload["agv"] = handlers_["chassis"]->isOnline() ? "online" : "offline";
        }
        if (handlers_.count("lidar")) {
            payload["lidar"] = handlers_["lidar"]->getReport();
        }
        
        if (payload.empty()) return {};
        return { parser.buildRawMessage(robot_id_, "status_update", payload) };
    }
};

/**
 * @brief 机器人工厂
 */
class RobotFactory {
public:
    static Robot::Ptr createRobot(const std::string& robot_id) {
        if (robot_id == "vacuum_adsorption_robot") {
            return std::make_shared<VacuumAdsorptionRobot>();
        } else {
            return std::make_shared<MobileDualArmRobot>();
        }
    }
};

} // namespace agv_bridge
