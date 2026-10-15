#include <chrono>
#include <functional>
#include <memory>
#include <array>
#include <vector>
#include <stdexcept>
#include <cmath>
#include "fsm.hpp"
#include "fsm_black.hpp"
#include "vector_math.hpp"
#include "rclcpp/rclcpp.hpp"          // ROS2 C++接口库
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "robot_interface/msg/robot_command.hpp"
#include "robot_interface/msg/robot_state.hpp"
#include "inference_runtime.hpp"
#include "rl_sdk.hpp"

using namespace std::chrono_literals;

class Publisher : public rclcpp::Node, public BlackControl
{
public:
    Publisher()
    : Node("rl_black_controller")  // 初始化节点，命名为
    {
        InitJointNum(12);
        params.config_node["num_of_dofs"] = 12;
        policy_command_.motor_command.resize(12);

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

        RL* context = this;
        auto black_fsm = FSMManager::GetInstance().CreateFSM("black", context);
        if(!black_fsm) {
            throw std::runtime_error("无法创建 FSM");
        }
        fsm = *black_fsm;
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
    std::unique_ptr<InferenceRuntime::Model> policy_model_;
    RobotCommand<float> policy_command_;
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
    bool previous_stand_button_ =false;
    bool previous_damping_button_ = false;
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
        observation_[3] = robot_state.imu.gyroscope[0] * 0.25f;
        observation_[4] = robot_state.imu.gyroscope[1] * 0.25f;
        observation_[5] = robot_state.imu.gyroscope[2] * 0.25f;
        for (int i = 0; i < 3; ++i) {
            observation_[6 + i] = gravity_body_[i];
        }
        for (int i = 0; i < 12; ++i) {
            observation_[9 + i] =robot_state.motor_state.q[i] - policy_default_angles_[i];
            observation_[21 + i] = robot_state.motor_state.dq[i] * 0.05f;
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

    void GetState(RobotState<float>* state) override
    {
        state->motor_state.resize(12);
        for(int i = 0; i < 12; ++i) {
            // 转换关节状态
            int sim_index = policy_to_sim_[i];
            state->motor_state.q[i] = current_state_.motors[sim_index].q;
            state->motor_state.dq[i] = current_state_.motors[sim_index].dq;
        }

        // 内部四元数顺序为 w、x、y、z。
        state->imu.quaternion = {
            static_cast<float>(current_imu_.orientation.w),
            static_cast<float>(current_imu_.orientation.x),
            static_cast<float>(current_imu_.orientation.y),
            static_cast<float>(current_imu_.orientation.z)
        };
        state->imu.gyroscope = {
            static_cast<float>(current_imu_.angular_velocity.x),
            static_cast<float>(current_imu_.angular_velocity.y),
            static_cast<float>(current_imu_.angular_velocity.z)
        };
    }

    void SetCommand(const RobotCommand<float>* command) override
    {
        robot_interface::msg::RobotCommand message;
        for(int i = 0; i < 12; ++i) {
            int sim_index = policy_to_sim_[i];
            message.motors[sim_index].q = command->motor_command.q[i];
            message.motors[sim_index].kp = command->motor_command.kp[i];
            message.motors[sim_index].kd = command->motor_command.kd[i];
            message.motors[sim_index].w = command->motor_command.dq[i];
            message.motors[sim_index].tau = command->motor_command.tau[i];
        }
        command_pub_->publish(message);
    }

    void ResetControl() override
    {
        previous_actions_.fill(0.0f);
        history_initialized_ = false;
        policy_tick_ = 0;
    }

    void RunpolicyStep(RobotCommand<float>& command) override
    {
        if(policy_tick_ == 0){
            run_policy(policy_command_);
        }
        policy_tick_ = (policy_tick_ + 1) % 4;
        command = policy_command_;
    }

    std::vector<float> Forward() override
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
        return actions;
    }

    void run_policy(RobotCommand<float>& command)
    {
        std::vector<float> actions = Forward();
        for (int i = 0; i < 12; ++i) {
            //把动作转化为目标角度
            command.motor_command.q[i] = policy_default_angles_[i] + 0.25f * actions[i];
            command.motor_command.kp[i] = 40.0f;
            command.motor_command.kd[i] = 1.2f;
            command.motor_command.dq[i] = 0.0f;
            command.motor_command.tau[i] = 0.0f;
            previous_actions_[i] = actions[i];
        }

    }

    void timer_callback()
    {
        if(!received_state_) {
            return;
        }

        GetState(&robot_state);
        StateController(&robot_state, &robot_command);
        control.ClearInput();
        SetCommand(&robot_command);
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


        // 只在按下沿提交事件，长按不会重复切换。
        bool stand_pressed = msg->buttons[0] == 1 && !previous_stand_button_;
        bool policy_pressed = msg->buttons[2] == 1 && !previous_policy_button_;
        bool damping_pressed = msg->buttons[1] == 1 && !previous_damping_button_;

        previous_stand_button_ = msg->buttons[0] == 1;
        previous_policy_button_ = msg->buttons[2] == 1;
        previous_damping_button_ = msg->buttons[1] == 1;

        if (damping_pressed) {
            control.SetGamepad(Input::Gamepad::LB_X);
        } else if (control.current_gamepad != Input::Gamepad::LB_X) {
            // 未处理的阻尼事件不能被其他消息覆盖。
            if (policy_pressed && received_state_ && received_imu_) {
                control.SetGamepad(Input::Gamepad::RB_DPadUp);
            } else if (stand_pressed && received_state_) {
                control.SetGamepad(Input::Gamepad::A);
            }
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
