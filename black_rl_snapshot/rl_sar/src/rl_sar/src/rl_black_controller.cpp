#include <chrono>
#include <functional>
#include <memory>
#include <array>
#include <vector>
#include <stdexcept>
#include <cmath>
#include "vector_math.hpp"
#include "rclcpp/rclcpp.hpp"          // ROS2 C++接口库
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "robot_interface/msg/robot_command.hpp"
#include "robot_interface/msg/robot_state.hpp"
#include "inference_runtime.hpp"

using namespace std::chrono_literals;

class Publisher : public rclcpp::Node
{
public:
    Publisher()
    : Node("rl_black_controller")  // 初始化节点，命名为
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

        imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
            "imu",10,
            std::bind(&Publisher::imu_callback, this, std::placeholders::_1)
        );

        command_pub_ = create_publisher<robot_interface::msg::RobotCommand>(
            "motor_command", 10);
        policy_model_ = InferenceRuntime::ModelFactory::load_model(
            "/home/wujuncheng/black/rl_sar/policy/black/robot_lab/best.pt"
        );

        if (!policy_model_) {
            throw std::runtime_error("无法加载 best.pt");
        }

        timer_ = this->create_wall_timer(
            5ms, std::bind(&Publisher::timer_callback, this));
    }

//=================================================================================================
private:
    rclcpp::Publisher<robot_interface::msg::RobotCommand>::SharedPtr command_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<robot_interface::msg::RobotState>::SharedPtr state_sub_;
    std::array<double,12> start_angles_;
    std::chrono::steady_clock::time_point stand_start_time_;
    std::unique_ptr<InferenceRuntime::Model> policy_model_;
    robot_interface::msg::RobotCommand policy_command_;
    int policy_tick_ = 0;

    robot_interface::msg::RobotState current_state_;
    sensor_msgs::msg::Imu current_imu_;
    std::vector<float> gravity_body_ = {0.0f, 0.0f, -1.0f};//重力在xyz轴下的分布
    std::array<float, 45> observation_{};
    std::array<float, 12> previous_actions_{};
    std::array<float,270> observation_history_{};
    bool history_initialized_ = false;
    bool received_imu_ = false;
    bool received_state_ = false;
    bool standing_ = false;
    bool previous_stand_button_ =false;
    bool previous_damping_button_ = false;
    bool policy_running_ = false;
    bool previous_policy_button_ = false;
    double left_stick_x_ = 0.0;
    double left_stick_y_ = 0.0;
    double right_stick_x_ = 0.0;

    //策略关节到仿真关节的映射
    const std::array<int, 12> policy_to_sim_ = {
        0, 1, 2,       // FL
        3, 4, 5,       // FR
        9, 10, 11,     // RL
        6, 7, 8        // RR
    };
    //策略默认角度
    const std::array<float, 12> policy_default_angles_ = {
        0.0f,  0.8014f, -1.527f,  // FL
        0.0f, -0.8014f,  1.527f,  // FR
        0.0f,  0.8014f, -1.527f,  // RL
        0.0f, -0.8014f,  1.527f   // RR
    };

    // 每条腿依次是 hip（髋侧摆）、thigh（髋俯仰，带动大腿）、calf（膝关节，带动小腿）。
    // 下面的目标角度单位为弧度；FL/FR/RR/RL 分别是左前/右前/右后/左后腿。
    const std::array<double, 12> stand_angles_ = {
        0.0,  0.8014, -1.527,  // FL
        0.0, -0.8014,  1.527,  // FR
        0.0, -0.8014,  1.527,  // RR
        0.0,  0.8014, -1.527   // RL
    };
    // 贝塞尔曲线的引导点：影响收腿轨迹，不要求准确到达或在这里停下。
    const std::array<double, 12> prepare_angles_ = {
            0.0,  1.4, -2.2,  // FL
            0.0, -1.4,  2.2,  // FR
            0.0, -1.4,  2.2,  // RR
            0.0,  1.4, -2.2   // RL
    };
/*===============================================================
        0-2运动命令
        3-5身体角速度v
        6-8重力方向分量
        9-20关节角度差
        21-32关节角速度
        33-44上一次动作
*/
    void build_observation(){
        //前后移动的速度，m/s
        observation_[0] = static_cast<float>(left_stick_y_) * 0.5f * 2.0f;
        //左右横移的速度 m/s
        observation_[1] = static_cast<float>(left_stick_x_) * 0.5f * 2.0f;
        //绕Z的旋转速度rad/s
        observation_[2] = static_cast<float>(right_stick_x_) * 0.8f * 0.25f;
        observation_[3] = static_cast<float>(current_imu_.angular_velocity.x) * 0.25f;
        observation_[4] = static_cast<float>(current_imu_.angular_velocity.y) * 0.25f;
        observation_[5] = static_cast<float>(current_imu_.angular_velocity.z) * 0.25f;
        for (int i = 0; i < 3; ++i) {
            observation_[6 + i] = gravity_body_[i];
        }
        for (int i = 0; i < 12; ++i) {
            int sim_index = policy_to_sim_[i];
            observation_[9 + i] = static_cast<float>(current_state_.motors[sim_index].q) - policy_default_angles_[i];
            observation_[21 + i] = static_cast<float>(current_state_.motors[sim_index].dq) * 0.05f;
            observation_[33 + i] = previous_actions_[i];
        }
    }

    void update_observation_history(){
        if (!history_initialized_) {
            for (int frame = 0; frame < 6; ++frame) {
                for (int i = 0; i < 45; ++i) {
                    observation_history_[frame * 45 + i] = observation_[i];
                }
            }
        history_initialized_ = true;
        return;
        }

        for(int i = 269; i >= 45; i--){
            observation_history_[i] = observation_history_[i - 45];
        }
        for(int i = 0; i < 45; i++){
            observation_history_[i] = observation_[i];
        }
        return;

    }

    void run_policy(robot_interface::msg::RobotCommand& command)
    {
        build_observation();
        update_observation_history();

        std::vector<float> input(observation_history_.begin(),observation_history_.end());

        std::vector<float> actions = policy_model_->forward({input});
        if (actions.size() != 12) {
        throw std::runtime_error("模型输出数量不是12");
        }

        for (float action : actions) {
            if (!std::isfinite(action)) {
                throw std::runtime_error("模型输出含有无效数值");
            }
        }
        for (int i = 0; i < 12; ++i) {
            int sim_index = policy_to_sim_[i];
            //把动作转化为目标角度
            command.motors[sim_index].q = policy_default_angles_[i] + 0.25f * actions[i];
            command.motors[sim_index].kp = 40.0;
            command.motors[sim_index].kd = 1.2;
            command.motors[sim_index].w = 0.0;
            command.motors[sim_index].tau = 0.0;
            previous_actions_[i] = actions[i];
        }

    }

    void timer_callback()
    {
        if (!received_state_ ) {
            return;
        }
        robot_interface::msg::RobotCommand command;
        if (policy_running_) {
            // 每四个控制周期推理一次，其余周期保持最近的策略目标。
            if (policy_tick_ == 0) {
                run_policy(policy_command_);
            }
            policy_tick_ = (policy_tick_ + 1) % 4;
            command = policy_command_;
        } else if (standing_) {
            double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - stand_start_time_).count();

            // 整个起身用0.4秒，不再在中途重置进度。
            double phase = elapsed / 0.4;
            if (phase > 1.0) {
                phase = 1.0;
            }
            // 只在整个动作开始和结束时缓起缓停，避免中途统一停下再启动。
            double blend = phase * phase * (3.0 - 2.0 * phase);
            double remaining = 1.0 - blend;

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
                    command.motors[i].kp = 55.0;
                    command.motors[i].kd = 4.0;
                }
                // 二次贝塞尔曲线：从实测起始角度连续过渡到站姿，准备姿态引导中间轨迹。
                // blend=0时是起始角度，blend=1时是站姿；不再分两段独立插值。
                // 这里只设置目标角度，不直接修改机器人的实际角度。
                command.motors[i].q =
                    remaining * remaining * start_angles_[i]
                    + 2.0 * remaining * blend * prepare_angles_[i]
                    + blend * blend * stand_angles_[i];

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
        // 每个控制周期都发布当前模式的目标。
        command_pub_->publish(command);
    }



    void joy_callback(sensor_msgs::msg::Joy::ConstSharedPtr msg)
    {
        if (msg->axes.size() < 4 || msg->buttons.size() < 4) {
            return;
        }
        left_stick_x_ = msg->axes[0];
        left_stick_y_ = msg->axes[1];
        // 当前 DualSense 的右摇杆水平轴。
        right_stick_x_ = msg->axes[3];


        // 处理接收到的Joy消息
        RCLCPP_INFO(this->get_logger(), "Received Joy message: axes[0]=%f, buttons[0]=%d", msg->axes[0], msg->buttons[0]);
        // msg->buttons 是按键状态数组
        // msg->axes 是摇杆、扳机等轴的数值数组
        // buttons[0]：起立。
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

        // buttons[2]：进入策略模式。
        if (msg->buttons[2] == 1) {
            if (!previous_policy_button_ && standing_ && received_imu_ && !policy_running_) {

                double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - stand_start_time_).count();

                if (elapsed >= 0.4) {
                    history_initialized_ = false;
                    previous_actions_.fill(0.0f);
                    policy_tick_ = 0;
                    policy_running_ = true;
                    RCLCPP_INFO(get_logger(), "进入策略模式");
                }
            }
            previous_policy_button_ = true;
        } else {
            previous_policy_button_ = false;
        }
        // buttons[1]：退出策略并进入阻尼，不执行坐下姿态。
        if (msg->buttons[1] == 1) {
            if (!previous_damping_button_ && standing_) {
                policy_running_ = false;
                standing_ = false;
                RCLCPP_INFO(get_logger(), "切换到阻尼");
            }
            previous_damping_button_ = true;
        } else {
            previous_damping_button_ = false;
        }


    }

    void imu_callback(sensor_msgs::msg::Imu::ConstSharedPtr msg)
    {
        current_imu_ = *msg;
        received_imu_ = true;
        std::vector<float> quaternion = {
        static_cast<float>(msg->orientation.w),
        static_cast<float>(msg->orientation.x),
        static_cast<float>(msg->orientation.y),
        static_cast<float>(msg->orientation.z)
        };
        gravity_body_ = QuatRotateInverse(quaternion, {0.0f, 0.0f, -1.0f}
        );
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
