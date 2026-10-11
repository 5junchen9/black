#include <chrono>
#include <functional>
#include <memory>
#include <array>

#include "rclcpp/rclcpp.hpp"          // ROS2 C++接口库
#include "sensor_msgs/msg/joy.hpp"

#include "robot_interface/msg/robot_command.hpp"
#include "robot_interface/msg/robot_state.hpp"

using namespace std::chrono_literals;

class Publisher : public rclcpp::Node
{
public:
    Publisher()
    : Node("robot_stand_pub")  // 初始化节点，命名为"robot_stand_pub"
    {

        joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy",
            10,
            std::bind(&Publisher::joy_callback, this, std::placeholders::_1)
        );

        state_sub_ = create_subscription<robot_interface::msg::RobotState>(
            "motor_state", 10,
            std::bind(&Publisher::state_callback, this, std::placeholders::_1)
        );
        command_pub_ = create_publisher<robot_interface::msg::RobotCommand>(
            "motor_command", 10);
            
        timer_ = this->create_wall_timer(
            10ms, std::bind(&Publisher::timer_callback, this));
    }

private:
    rclcpp::Publisher<robot_interface::msg::RobotCommand>::SharedPtr command_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<robot_interface::msg::RobotState>::SharedPtr state_sub_;
    std::array<double,12> start_angles_;
    std::chrono::steady_clock::time_point stand_start_time_;


    robot_interface::msg::RobotState current_state_;
    bool received_state_ = false;
    bool standing_ = false;
    bool previous_stand_button_ =false;
    bool previous_damping_button_ = false;
    double left_stick_x_ = 0.0;
    double left_stick_y_ = 0.0;


    // 每条腿依次是 hip（髋侧摆）、thigh（髋俯仰，带动大腿）、calf（膝关节，带动小腿）。
    // 下面的目标角度单位为弧度；FL/FR/RR/RL 分别是左前/右前/右后/左后腿。
    const std::array<double, 12> stand_angles_ = {
             0.1,  0.7, -1.2,  // FL
            -0.1, -0.7,  1.2,  // FR
             0.1, -0.7,  1.2,  // RR
            -0.1,  0.7, -1.2   // RL
    };
 

    void timer_callback()
    {
        if (!received_state_ ) {
            return;
        }
        robot_interface::msg::RobotCommand command;
        if (standing_) {
            double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - stand_start_time_).count();
            double phase = elapsed / 0.5;  // 0.5秒平滑过渡到站姿
            if (phase > 1.0) {
                phase = 1.0;
            }
            // phase 是时间进度：0表示刚开始，1表示0.5秒的过渡已结束。
            // blend 是平滑后的进度：起点为0、终点为1，且两端变化率为0，
            // 使目标角度缓慢开始变化、缓慢停止变化；phase=0.5时blend也为0.5。
            double blend = phase * phase * (3.0 - 2.0 * phase);

            for (int i = 0; i < 12; i++) {
                // i % 3：0是hip，1是thigh，2是calf；每三个电机对应一条腿。
                if (i % 3 == 0) {
                    command.motors[i].kp = 35.0;
                    command.motors[i].kd = 2.5;
                } else if(i<6){
                    command.motors[i].kp = 45.0;
                    command.motors[i].kd = 3.0;
                }
                else{
                    command.motors[i].kp = 45.0;
                    command.motors[i].kd = 3.0;
                }
                // 对起身瞬间的实测角度和站姿角度加权，得到本周期的目标角度。
                // blend=0时使用起始角度，blend=1时使用站姿角度；中间逐渐过渡。
                // 这里设置的是目标角度，不是直接修改机器人的实际角度。
                command.motors[i].q =
                    (1.0 - blend) * start_angles_[i] + blend * stand_angles_[i];
                command.motors[i].w = 0.0;
                command.motors[i].tau = 0.0;
            }
        } else {
            // kp=0、w=0：仿真计算的力矩为 -kd*dq，不追踪固定姿态。
            for (int i = 0; i < 12; i++) {
                command.motors[i].kp = 0.0;
                command.motors[i].kd = 5.0;
                command.motors[i].q = 0.0;
                command.motors[i].w = 0.0;
                command.motors[i].tau = 0.0;
            }
        }
        // 过渡结束后仍持续发布站姿目标。
        command_pub_->publish(command);
    }



    void joy_callback(sensor_msgs::msg::Joy::ConstSharedPtr msg)
    {
        if (msg->axes.size() < 2 || msg->buttons.size() < 2) {
            return;
        }
        left_stick_x_ = msg->axes[0];
        left_stick_y_ = msg->axes[1];
        

        // 处理接收到的Joy消息
        RCLCPP_INFO(this->get_logger(), "Received Joy message: axes[0]=%f, buttons[0]=%d", msg->axes[0], msg->buttons[0]);
    // msg->buttons 是按键状态数组
    // msg->axes 是摇杆、扳机等轴的数值数组
        if(msg->buttons[0] == 1){
            if (!previous_stand_button_ && !standing_ && received_state_) {
                for (int i = 0; i < 12; i++) {
                    start_angles_[i] = current_state_.motors[i].q;
                }
                stand_start_time_ = std::chrono::steady_clock::now();
                standing_ = true;
                RCLCPP_INFO(get_logger(), "开始起身");
            }
            previous_stand_button_ = true;
        }
        else{
            previous_stand_button_ = false;
        }
        if (msg->buttons[1] == 1) {
            if (!previous_damping_button_ && standing_) {
                standing_ = false;
                RCLCPP_INFO(get_logger(), "切换到阻尼");
            }
            previous_damping_button_ = true;
        } else {
            previous_damping_button_ = false;
        }
        

    }

    void state_callback(const robot_interface::msg::RobotState::ConstSharedPtr msg) {
        current_state_ = *msg;
        received_state_ = true;
    }

    
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Publisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
