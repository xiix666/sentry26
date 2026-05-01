#include "rclcpp/rclcpp.hpp"
#include <cstdlib>
#include <string>
#include <chrono>
#include <filesystem>
using namespace std::chrono_literals;

class MapAutoSaver : public rclcpp::Node
{
public:
    MapAutoSaver() : Node("map_auto_saver")
    {
        // ====================== 可修改参数 ======================
        save_interval_ = this->declare_parameter<double>("save_interval", 20.0);
        save_path_ = this->declare_parameter<std::string>(
            "save_path",
            "/home/chakfan/ros_ws/map/map"
        );
        map_topic_ = this->declare_parameter<std::string>(
            "map_topic",
            "/slam_map"
        );
        save_id_ = 0;
        // ========================================================
        while (
            std::filesystem::exists(save_path_ + std::to_string(save_id_) + ".yaml") ||
            std::filesystem::exists(save_path_ + std::to_string(save_id_) + ".pgm"))
        {
            save_id_++;
        }
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(save_interval_),
            std::bind(&MapAutoSaver::saveMapCallback, this)
        );

        RCLCPP_INFO(this->get_logger(), "✅ 自动地图保存启动（调用官方 map_saver_cli）");
    }

private:
    void saveMapCallback()
    {
        // ===================== 终极方案：直接调用官方命令 =====================
        std::string cmd = "ros2 run nav2_map_server map_saver_cli "
                          "-t " + map_topic_ + " "
                          "-f " + save_path_ + std::to_string(save_id_);

        RCLCPP_INFO(this->get_logger(), "执行保存命令: %s", cmd.c_str());
        
        int ret = system(cmd.c_str());
        
        if (ret == 0) {
            RCLCPP_INFO(this->get_logger(), "✅ 地图保存成功！");
            save_id_++;
        } else {
            RCLCPP_ERROR(this->get_logger(), "❌ 地图保存失败！");
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    double save_interval_;
    std::string save_path_;
    std::string map_topic_;
    int save_id_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapAutoSaver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}