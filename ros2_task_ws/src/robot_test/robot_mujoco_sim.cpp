#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <array>
#include "robot_interface/msg/robot_state.hpp"
#include "robot_interface/msg/robot_command.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MujocoNode : public rclcpp::Node {
public:
    MujocoNode() : Node("mujoco_node") {
        
        char error[1000] = "";
        model = mj_loadXML(MUJOCO_MODEL_PATH, nullptr, error, sizeof(error));
        if (!model) {
            throw std::runtime_error(std::string("模型加载失败: ") + error);
        }

        const std::array<const char*, 12> joint_names = {
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"
        };


        for(int i=0; i<12; i++){
            // 获取关节 ID
            int joint_id = mj_name2id(model, mjOBJ_JOINT, joint_names[i]);
            if(joint_id == -1){
                mj_deleteModel(model);
                throw std::runtime_error("无法找到关节: " + std::string(joint_names[i]));
            }
            joint_qpos_[i] = model->jnt_qposadr[joint_id];
            joint_dof_[i] = model->jnt_dofadr[joint_id];

            // 获取执行器 ID
            std::string actuator_name = std::string(joint_names[i]) + "_motor";
            int actuator_id = mj_name2id(model, mjOBJ_ACTUATOR, actuator_name.c_str());
            if (actuator_id == -1) {
                mj_deleteModel(model);
                throw std::runtime_error("无法找到执行器: " + actuator_name);
            }
            actuator_ids_[i] = actuator_id;
        }

        // 创建数据结构，保存仿真状态。
        data = mj_makeData(model);
        if (!data) {
            mj_deleteModel(model);
            throw std::runtime_error("MuJoCo 数据初始化失败");
        }

        // 初始化 GLFW 窗口和 OpenGL 上下文。
        if (!glfwInit()) {
            mj_deleteData(data);
            mj_deleteModel(model);
            throw std::runtime_error("GLFW 初始化失败");
        }

        window = glfwCreateWindow(1200, 900, "MuJoCo ROS 2", nullptr, nullptr);
        if (!window) {
            glfwTerminate();
            mj_deleteData(data);
            mj_deleteModel(model);
            throw std::runtime_error("窗口创建失败");
        }
        glfwMakeContextCurrent(window);
        // 不等待显示器垂直同步，减少渲染对同一线程物理回调的阻塞。
        glfwSwapInterval(0);
        glfwSetKeyCallback(window, keyboard);

        mjv_defaultCamera(&cam);
        mjv_defaultOption(&opt);
        mjv_defaultScene(&scene);
        mjr_defaultContext(&con);
        mjv_makeScene(model, &scene, 2000);
        mjr_makeContext(model, &con, mjFONTSCALE_150);

        cam.type = mjCAMERA_FREE;
        cam.distance = 3.0;
        cam.azimuth = 90;
        cam.elevation = -20;
        cam.lookat[0] = 0.0;
        cam.lookat[1] = 0.0;
        cam.lookat[2] = 0.5;

        // 初始化控制输入为零，确保仿真从静止开始。
        for (int i = 0; i < model->nu; ++i) {
            data->ctrl[i] = 0.0;
        }

        const std::array<double, 12> prone_angles = {
            0.49,  1.44, -2.45,  // FL
            -0.49, -1.49,  2.45,  // FR
            0.49, -1.34,  2.45,  // RR
            -0.49,  1.35, -2.45   // RL
        };

        for(int i = 0;i <12 ;i++){
            data->qpos[joint_qpos_[i]] = prone_angles[i];
        }

        int root_id = mj_name2id(model, mjOBJ_JOINT, "root_free");
        if(root_id != -1){
            int root_qpos = model->jnt_qposadr[root_id];
            data->qpos[root_qpos]     = 0.0;   // x
            data->qpos[root_qpos + 1] = 0.0;   // y
            data->qpos[root_qpos + 2] = 0.45;  // z：离地高度
            data->qpos[root_qpos + 3] = 1.0;   // 四元数 w
            data->qpos[root_qpos + 4] = 0.0;
            data->qpos[root_qpos + 5] = 0.0;
            data->qpos[root_qpos + 6] = 0.0;
        }
        

        // 运行一次仿真，确保所有状态变量初始化。
        mj_forward(model, data);


        // 创建机器人状态发布器
        state_pub_ = create_publisher<robot_interface::msg::RobotState>("motor_state", 10);
       
        command_sub_ = create_subscription<robot_interface::msg::RobotCommand>(
            "motor_command", 10,
            std::bind(&MujocoNode::command_callback, this, std::placeholders::_1));

        // 每次物理回调推进一步，周期使用模型自身的 timestep。
        simulation_timer_ = create_wall_timer(
            std::chrono::duration<double>(model->opt.timestep),
            std::bind(&MujocoNode::simulation_step, this));

        // 显示以较低频率更新，不必每个物理步都画一帧。
        render_timer_ = create_wall_timer(
            std::chrono::milliseconds(16),
            std::bind(&MujocoNode::render, this));
    }

    ~MujocoNode() override {
        // OpenGL 资源必须在窗口及其上下文销毁之前释放。
        glfwMakeContextCurrent(window);
        mjr_freeContext(&con);
        mjv_freeScene(&scene);
        mj_deleteData(data);
        mj_deleteModel(model);
        glfwDestroyWindow(window);
        glfwTerminate();
    }

private:
    mjModel* model = nullptr;
    mjData* data = nullptr;
    GLFWwindow* window = nullptr;
    mjvCamera cam;
    mjvOption opt;
    mjvScene scene;
    mjrContext con;

    rclcpp::TimerBase::SharedPtr simulation_timer_;
    rclcpp::TimerBase::SharedPtr render_timer_;
    robot_interface::msg::RobotCommand current_command_;

    rclcpp::Publisher<robot_interface::msg::RobotState>::SharedPtr state_pub_;
    rclcpp::Subscription<robot_interface::msg::RobotCommand>::SharedPtr command_sub_;

    std::array<int, 12> joint_qpos_;
    std::array<int, 12> joint_dof_;
    std::array<int,12> actuator_ids_;

    // GLFW 不传入 this，因此使用不依赖对象的 static 成员函数。
    static void keyboard(GLFWwindow* window, int key, int, int action, int) {
        if (action == GLFW_PRESS && key == GLFW_KEY_ESCAPE) {
            glfwSetWindowShouldClose(window, GLFW_TRUE);
        }
    }

    void publish_robot_state() {
        robot_interface::msg::RobotState msg;
        for (int i = 0; i < 12; ++i) {
            robot_interface::msg::MotorState motor_state;
            motor_state.q = data->qpos[joint_qpos_[i]];
            motor_state.dq = data->qvel[joint_dof_[i]];
            // 这里假设 ddq、tau、cur 为零，实际应用中应根据需要计算或获取这些值。
            motor_state.ddq = data->qacc[joint_dof_[i]];
            motor_state.tau = 0.0;
            motor_state.cur = 0.0;
            msg.motors[i] = motor_state;
        }
        state_pub_->publish(msg);
    }   

    
    void command_callback(robot_interface::msg::RobotCommand::ConstSharedPtr msg)
    {
        current_command_ = *msg;
    }

    void simulation_step() {
        for(int i = 0; i<12 ;i++){
            double q = data->qpos[joint_qpos_[i]];
            double dq = data->qvel[joint_dof_[i]];
            const auto& command = current_command_.motors[i];
            double torque = command.kp * (command.q - q) + command.kd * (command .w - dq) + command.tau;
            data->ctrl[actuator_ids_[i]] = torque;
        }
        mj_step(model, data);
        publish_robot_state();
    }

    void render() {
        glfwPollEvents();
        if (glfwWindowShouldClose(window)) {
            simulation_timer_->cancel();
            render_timer_->cancel();
            rclcpp::shutdown();
            return;
        }

        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        mjrRect viewport = {0, 0, width, height};
        mjv_updateScene(model, data, &opt, nullptr, &cam, mjCAT_ALL, &scene);
        mjr_render(viewport, &scene, &con);
        glfwSwapBuffers(window);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    // 单线程 spin：物理、渲染和订阅回调依次执行，不同时修改 data。
    try {
        auto node = std::make_shared<MujocoNode>();
        rclcpp::spin(node);
    } catch (const std::exception& error) {
        RCLCPP_ERROR(rclcpp::get_logger("mujoco_node"), "%s", error.what());
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        return 1;
    }
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return 0;
}
