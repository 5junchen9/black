#ifndef BLACK_FSM_HPP
#define BLACK_FSM_HPP

#include <array>
#include <chrono>

#include "fsm.hpp"
#include "rl_sdk.hpp"

class BlackControl : public RL
{
public:
    virtual void ResetControl() = 0;// 重置控制器状态
    virtual void RunpolicyStep(RobotCommand<float>& command) = 0; // 执行控制策略，更新命令
};

namespace black_fsm
{

// 阻尼状态，继承自 RLFSMState
class RLFSMStatePassive : public RLFSMState
{
public:
    RLFSMStatePassive(RL *rl) ://创建一个阻尼状态，接收控制器的地址 rl；
        RLFSMState(*rl, "RLFSMStatePassive") {}//把这个控制器和状态名称交给父类初始化。

    void Enter() override
    {
        std::cout << LOGGER::NOTE << "Entered passive mode. Press '0' (Keyboard) or 'A' (Gamepad) to switch to RLFSMStateGetUp." << std::endl;
    }

    void Run() override
    {
        for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
        {
            fsm_command->motor_command.q[i] = 0.0f;
            fsm_command->motor_command.dq[i] = 0.0f;
            fsm_command->motor_command.kp[i] = 0.0f;
            fsm_command->motor_command.kd[i] = 5.0f;
            fsm_command->motor_command.tau[i] = 0.0f;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

class RLFSMStateGetUp : public RLFSMState
{
public:
    RLFSMStateGetUp(RL *rl) : RLFSMState(*rl, "RLFSMStateGetUp") {}

    const std::array<double, 12> stand_angles_ = {
        0.0,  0.8014, -1.527,  // FL
        0.0, -0.8014,  1.527,  // FR
        0.0,  0.8014, -1.527,  // RL
        0.0, -0.8014,  1.527   // RR
    };
    // 贝塞尔曲线的引导点：影响收腿轨迹，不要求准确到达或在这里停下。
    const std::array<double, 12> prepare_angles_ = {
            0.0,  1.4, -2.2,  // FL
            0.0, -1.4,  2.2,  // FR
            0.0,  1.4, -2.2,  // RL
            0.0, -1.4,  2.2   // RR
    };
    std::array<double, 12> start_angles_{};
    std::chrono::steady_clock::time_point stand_start_time_;
    bool getup_finished_ = false;

    void Enter() override
    {
        // 内部状态已采用 FL、FR、RL、RR 顺序，不再映射。
        for (int i = 0; i < 12; ++i) {
            start_angles_[i] = fsm_state->motor_state.q[i];
        }
        stand_start_time_ = std::chrono::steady_clock::now();
        getup_finished_ = false;
    }

    void Run() override
    {
        double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - stand_start_time_).count();
        double phase = elapsed / 0.4;
        if (phase > 1.0) {
            phase = 1.0;
        }
        // 表示轨迹时间走完，不代表实际关节已经到位。
        getup_finished_ = phase >= 1.0;
        double blend = phase * phase * (3.0 - 2.0 * phase);
        double remaining = 1.0 - blend;

        for (int i = 0; i < 12; ++i) {
            if (i % 3 == 0) {
                fsm_command->motor_command.kp[i] = 35.0f;
                fsm_command->motor_command.kd[i] = 2.5f;
            } else if (i < 6) {
                fsm_command->motor_command.kp[i] = 45.0f;
                fsm_command->motor_command.kd[i] = 3.0f;
            } else {
                fsm_command->motor_command.kp[i] = 55.0f;
                fsm_command->motor_command.kd[i] = 4.0f;
            }
            // 0.4 秒贝塞尔过渡，结束后持续输出站姿。
            fsm_command->motor_command.q[i] =
                remaining * remaining * start_angles_[i]
                + 2.0 * remaining * blend * prepare_angles_[i]
                + blend * blend * stand_angles_[i];
            fsm_command->motor_command.dq[i] = 0.0f;
            fsm_command->motor_command.tau[i] = 0.0f;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        if (getup_finished_)
        {
            if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
            {
                return "RLFSMStateRLLocomotion";
            }
        }
        return state_name_;
    }
};


// RL locomotion state, inherits from RLFSMState
class RLFSMStateRLLocomotion : public RLFSMState
{
public:
    RLFSMStateRLLocomotion(BlackControl& controller)
        : RLFSMState(controller, "RLFSMStateRLLocomotion"),
          black_control_(controller) {}


    void Enter() override
    {
        black_control_.ResetControl();
    }

    void Run() override
    {
        black_control_.RunpolicyStep(*fsm_command);

    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        return state_name_;
    }
private:
    BlackControl& black_control_;

};

} // namespace black_fsm

// FSM factory for the black robot
class BlackFSMFactory : public FSMFactory
{
public:
    BlackFSMFactory(const std::string& initial) : initial_state_(initial) {}
    std::shared_ptr<FSMState> CreateState(void *context, const std::string &state_name) override
    {
        RL *rl = static_cast<RL *>(context);
        if (state_name == "RLFSMStatePassive")
            return std::make_shared<black_fsm::RLFSMStatePassive>(rl);
        else if (state_name == "RLFSMStateGetUp")
            return std::make_shared<black_fsm::RLFSMStateGetUp>(rl);
        else if (state_name == "RLFSMStateRLLocomotion") {
            auto* controller = dynamic_cast<BlackControl*>(rl);
            if (!controller) {
                return nullptr;
            }
            return std::make_shared<black_fsm::RLFSMStateRLLocomotion>(*controller);
        }
        return nullptr;
    }
    std::string GetType() const override { return "black"; }
    std::vector<std::string> GetSupportedStates() const override
    {
        return {
            "RLFSMStatePassive",
            "RLFSMStateGetUp",
            "RLFSMStateRLLocomotion"
        };
    }
    std::string GetInitialState() const override { return initial_state_; }
private:
    std::string initial_state_;
};

REGISTER_FSM_FACTORY(BlackFSMFactory, "RLFSMStatePassive")


#endif // BLACK_FSM_HPP
