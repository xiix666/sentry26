#include "angle_pub.hpp"

AnglePub::AnglePub() : 
    rclcpp::Node("angle_pub_node"),
    latest_velocity_direction_(0.0),
    send_angle_(0.0),
    current_shoot_mode_(0),
    current_special_number_(0)
{
    // 声明并获取参数（带默认值）
    this->declare_parameter("yaw_energy_", M_PI / 6.0);
    this->declare_parameter("yaw_tunnel_forward", M_PI / 6.0);
    this->declare_parameter("yaw_tunnel_backward", -M_PI / 6.0 * 5);

    // 获取参数值
    this->get_parameter("yaw_energy", yaw_energy_);
    this->get_parameter("yaw_tunnel_forward", yaw_tunnel_forward_);
    this->get_parameter("yaw_tunnel_backward", yaw_tunnel_backward_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel_nav2_result", rclcpp::SensorDataQoS(),
        std::bind(&AnglePub::cmdVelCallback, this, std::placeholders::_1));

    shoot_mode_sub_ = this->create_subscription<rm_interfaces::msg::ShootMsg>(
        "/shoot_pack", rclcpp::SensorDataQoS(),
        std::bind(&AnglePub::shootModeCallback, this, std::placeholders::_1));

    special_number_sub_ = this->create_subscription<rm_interfaces::msg::SpecialMsg>(
        "/special_pack", rclcpp::SensorDataQoS(),
        std::bind(&AnglePub::specialNumberCallback, this, std::placeholders::_1));

    angle_sp_pub_ = this->create_publisher<rm_interfaces::msg::AnglePubMsg>("angle_sp", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() { this->timerCallback(); });
}

double AnglePub::getYaw()
{
    try {
        auto transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

        tf2::Quaternion q(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w);
        double yaw, pitch, roll;
        tf2::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);
        return yaw;
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Transform lookup failed: %s", ex.what());
        return 0.0; 
    }
}

double AnglePub::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

void AnglePub::shootModeCallback(const rm_interfaces::msg::ShootMsg::SharedPtr msg)
{
    if (msg->shoot_mode == 1 || msg->shoot_mode == 2)
    {
        if (current_shoot_mode_ != msg->shoot_mode)
        {
            RCLCPP_DEBUG(this->get_logger(), "Shoot Mode is 1 or 2");
            current_shoot_mode_ = msg->shoot_mode;
            double current_yaw = getYaw();
            double delta_angle = angles::shortest_angular_distance(current_yaw, yaw_energy_);  
            delta_angle = delta_angle*180/M_PI;
            std::lock_guard<std::mutex> lock(angle_mutex_);
            send_angle_ = delta_angle;
            RCLCPP_WARN(this->get_logger(), "angle_sp: %f", delta_angle);
        }   
    }
    else
    {
        if (current_shoot_mode_ != msg->shoot_mode)
        {
            current_shoot_mode_ = msg->shoot_mode;
            std::lock_guard<std::mutex> lock(angle_mutex_);
            send_angle_ = 0.0;
        }
    }
}

void AnglePub::specialNumberCallback(const rm_interfaces::msg::SpecialMsg::SharedPtr msg)
{
    if (msg->special_number == 2)
    {
        if (current_special_number_ != msg->special_number)
        {
            current_special_number_ = msg->special_number;
            
            double current_yaw = getYaw();
            double target_yaw;
            {
                std::lock_guard<std::mutex> lock(cmd_vel_mutex_); 
                double sum = current_yaw + latest_velocity_direction_;
                sum = normalizeAngle(sum); 
                RCLCPP_WARN(this->get_logger(),"sum yaw: %f", sum);
                target_yaw = (sum > -M_PI/2 && sum < M_PI/2) ? yaw_tunnel_forward_ : yaw_tunnel_backward_;
            }
            double delta_angle = angles::shortest_angular_distance(current_yaw, target_yaw); 
            std::lock_guard<std::mutex> lock(angle_mutex_);
            delta_angle = delta_angle*180/M_PI;
            send_angle_ = delta_angle;
            RCLCPP_WARN(this->get_logger(),
                "sp angle: %f", delta_angle);
        }
    }
    else
    {
        if (current_special_number_ != msg->special_number)
        {
            current_special_number_ = msg->special_number;
            std::lock_guard<std::mutex> lock(angle_mutex_);
            send_angle_ = 0.0;
        }
    }
}

void AnglePub::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    latest_cmd_vel_ = *msg;
    {
        std::lock_guard<std::mutex> lock(cmd_vel_mutex_); 
        latest_velocity_direction_ = std::atan2(msg->linear.y, msg->linear.x);
    }
    RCLCPP_DEBUG(this->get_logger(),
        "Velocity Direction (radians): %f", latest_velocity_direction_);
}

void AnglePub::timerCallback()
{
    
    auto msg = std::make_unique<rm_interfaces::msg::AnglePubMsg>();
    {
        std::lock_guard<std::mutex> lock(angle_mutex_);
        msg->angle_sp = send_angle_;
    }
    angle_sp_pub_->publish(std::move(msg));
}
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AnglePub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}