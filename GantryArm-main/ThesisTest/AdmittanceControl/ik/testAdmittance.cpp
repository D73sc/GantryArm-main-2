///**
// * @file main.cpp
// * @brief 导纳控制器使用示例
// */
//
//#include "admittance_factory.h"
//#include <iostream>
//#include <fstream>
//#include <vector>
//
//using namespace admittance;
//
//// 简单环境模拟
//class Environment {
//public:
//    Environment(double K, double z_surface) : K_(K), z_surface_(z_surface) {}
//
//    double getForceZ(double z) const {
//        return (z >= z_surface_) ? K_ * (z - z_surface_) : 0.0;
//    }
//
//private:
//    double K_, z_surface_;
//};
//
///**
// * @brief 示例1：Z轴单轴力控制（最常用）
// */
//void example1_z_axis_force_control() {
//    printf("\n========================================\n");
//    printf("示例1: Z轴力控制\n");
//    printf("========================================\n\n");
//
//    // 1. 创建Z轴导纳控制器（带积分）
//    auto ctrl = Factory::createZAxisIntegral(
//        1.0,    // M = 1kg
//        10.0,   // B = 10
//        100.0,  // K = 100
//        0.001,  // dt = 1ms
//        5.0     // eta = 5
//    );
//
//    // 2. 设置期望力（5N恒定接触力）
//    ctrl->setDesiredForce(DOF::Z, 5.0);
//    ctrl->printInfo();
//
//    // 3. 环境
//    Environment env(1000.0, 0.250);  // 刚度1000N/m，表面在0.25m
//
//    // 4. 机器人初始位姿
//    RobotPose robot_pose;
//    robot_pose.tcp_pose = Point6D(0.3, 0.0, 0.20, 0, 0, 0);  // 初始在0.2m高度
//    Point6D tcp_initial = robot_pose.tcp_pose;
//
//    printf("初始TCP: ");
//    print(tcp_initial);
//    printf("\n");
//
//    // 5. 仿真循环
//    std::ofstream file("example1_result.csv");
//    file << "Time,Z_TCP,F_Z,Z_Offset,Integral\n";
//
//    for (int i = 0; i < 5000; ++i) {
//        double t = i * 0.001;
//
//        // 5.1 测量当前Z轴力
//        double F_z = env.getForceZ(robot_pose.tcp_pose.z);
//
//        // 5.2 导纳控制器计算位姿偏移
//        Point6D delta = ctrl->updateZ(F_z);
//
//        // 5.3 计算新的TCP位姿（XY不变，Z=初始Z+偏移）
//        robot_pose.tcp_pose.x = tcp_initial.x;  // XY由其他控制器管
//        robot_pose.tcp_pose.y = tcp_initial.y;
//        robot_pose.tcp_pose.z = tcp_initial.z + delta.z;  // Z由导纳控制
//        robot_pose.tcp_pose.rx = tcp_initial.rx;
//        robot_pose.tcp_pose.ry = tcp_initial.ry;
//        robot_pose.tcp_pose.rz = tcp_initial.rz;
//
//        // 5.4 这里应该调用你的逆运动学
//        // bool ok = inverseKinematics(robot_pose.tcp_pose, robot_pose.joint_positions);
//        // if (ok) robot.moveJoints(robot_pose.joint_positions);
//
//        // 5.5 记录数据
//        if (i % 10 == 0) {
//            file << t << "," << robot_pose.tcp_pose.z << "," << F_z << ","
//                << delta.z << "," << ctrl->getIntegral()(2) << "\n";
//        }
//
//        if (i % 500 == 0) {
//            printf("t=%.2fs: F_z=%.3fN, z=%.6fm\n", t, F_z, robot_pose.tcp_pose.z);
//        }
//    }
//
//    file.close();
//
//    double final_force = env.getForceZ(robot_pose.tcp_pose.z);
//    printf("\n最终状态:\n");
//    printf("  接触力: %.4f N (期望5N)\n", final_force);
//    printf("  力误差: %.4f N\n", std::abs(final_force - 5.0));
//    printf("  数据保存: example1_result.csv\n");
//}
//
///**
// * @brief 示例2：圆形轨迹+恒定接触力
// */
//void example2_circular_trajectory() {
//    printf("\n========================================\n");
//    printf("示例2: 圆形轨迹+恒定接触力\n");
//    printf("========================================\n\n");
//
//    auto ctrl = Factory::createZAxisIntegral(1.0, 10.0, 100.0, 0.001, 5.0);
//    ctrl->setDesiredForce(DOF::Z, 8.0);  // 8N接触力
//
//    Environment env(1000.0, 0.25);
//
//    RobotPose robot_pose;
//    robot_pose.tcp_pose = Point6D(0.35, 0.0, 0.20, 0, 0, 0);
//    Point6D tcp_initial = robot_pose.tcp_pose;
//
//    // 圆形轨迹
//    // 圆形轨迹参数
//    double center_x = 0.35;
//    double center_y = 0.0;
//    double radius = 0.05;  // 5cm半径
//    double omega = 2.0 * M_PI * 0.2;  // 0.2Hz
//
//    std::ofstream file("example2_result.csv");
//    file << "Time,X,Y,Z,F_Z\n";
//
//    for (int i = 0; i < 10000; ++i) {
//        double t = i * 0.001;
//
//        // XY圆形轨迹（位置控制）
//        double x_desired = center_x + radius * std::cos(omega * t);
//        double y_desired = center_y + radius * std::sin(omega * t);
//
//        // Z轴力控制
//        double F_z = env.getForceZ(robot_pose.tcp_pose.z);
//        Point6D delta = ctrl->updateZ(F_z);
//
//        // 组合TCP位姿
//        robot_pose.tcp_pose.x = x_desired;
//        robot_pose.tcp_pose.y = y_desired;
//        robot_pose.tcp_pose.z = tcp_initial.z + delta.z;
//
//        // 调用逆运动学
//        // bool ok = inverseKinematics(robot_pose.tcp_pose, robot_pose.joint_positions);
//
//        if (i % 10 == 0) {
//            file << t << "," << robot_pose.tcp_pose.x << ","
//                << robot_pose.tcp_pose.y << "," << robot_pose.tcp_pose.z << ","
//                << F_z << "\n";
//        }
//    }
//
//    file.close();
//    printf("数据保存: example2_result.csv\n");
//}
//
///**
// * @brief 示例3：实际使用模板（集成到你的机器人系统）
// */
//void example3_real_usage_template() {
//    printf("\n========================================\n");
//    printf("示例3: 实际使用模板\n");
//    printf("========================================\n\n");
//
//    // 1. 创建控制器（根据任务选择）
//    auto ctrl = Factory::createZAxisIntegral(1.0, 10.0, 100.0, 0.001, 5.0);
//
//    // 2. 设置期望力（通常固定）
//    ctrl->setDesiredForce(DOF::Z, 5.0);
//
//    // 3. 准备你的机器人和传感器
//    // YourRobotClass robot;
//    // YourForceSensorClass force_sensor;
//
//    printf("控制循环模板:\n\n");
//    printf("```cpp\n");
//    printf("// 获取当前TCP位姿\n");
//    printf("RobotPose current_pose = robot.getCurrentPose();\n");
//    printf("Point6D tcp_initial = current_pose.tcp_pose;\n\n");
//
//    printf("// 控制循环（1kHz）\n");
//    printf("while (running) {\n");
//    printf("    // 1. 读取力传感器\n");
//    printf("    double F_z = force_sensor.readForceZ();\n\n");
//
//    printf("    // 2. 导纳控制计算位姿偏移\n");
//    printf("    Point6D delta = ctrl->updateZ(F_z);\n\n");
//
//    printf("    // 3. 计算目标TCP位姿\n");
//    printf("    Point6D tcp_target;\n");
//    printf("    tcp_target.x = xy_trajectory.getX();  // XY由轨迹规划\n");
//    printf("    tcp_target.y = xy_trajectory.getY();\n");
//    printf("    tcp_target.z = tcp_initial.z + delta.z;  // Z由导纳控制\n");
//    printf("    tcp_target.rx = tcp_initial.rx;  // 姿态保持\n");
//    printf("    tcp_target.ry = tcp_initial.ry;\n");
//    printf("    tcp_target.rz = tcp_initial.rz;\n\n");
//
//    printf("    // 4. 逆运动学求解\n");
//    printf("    std::array<double, 8> joint_target;\n");
//    printf("    if (!inverseKinematics(tcp_target, joint_target)) {\n");
//    printf("        // 逆解失败处理\n");
//    printf("        handleIKFailure();\n");
//    printf("        continue;\n");
//    printf("    }\n\n");
//
//    printf("    // 5. 发送关节指令\n");
//    printf("    robot.moveJoints(joint_target);\n\n");
//
//    printf("    // 6. 等待下一个周期\n");
//    printf("    sleep_until_next_cycle();\n");
//    printf("}\n");
//    printf("```\n\n");
//}
//
///**
// * @brief 示例4：不同预设场景对比
// */
//void example4_preset_comparison() {
//    printf("\n========================================\n");
//    printf("示例4: 预设场景对比\n");
//    printf("========================================\n\n");
//
//    Environment env(1000.0, 0.25);
//
//    // 三种预设
//    auto ctrl_polishing = Factory::createPreset(Factory::POLISHING);
//    auto ctrl_assembly = Factory::createPreset(Factory::ASSEMBLY);
//    auto ctrl_grinding = Factory::createPreset(Factory::GRINDING);
//
//    ctrl_polishing->setDesiredForce(DOF::Z, 5.0);
//    ctrl_assembly->setDesiredForce(DOF::Z, 5.0);
//    ctrl_grinding->setDesiredForce(DOF::Z, 5.0);
//
//    printf("对比三种预设场景:\n\n");
//
//    struct Result {
//        std::string name;
//        double settling_time;
//        double steady_force;
//    };
//
//    std::vector<Result> results;
//
//    auto test_preset = [&](std::unique_ptr<AdmittanceController>& ctrl,
//        const std::string& name) {
//            Point6D tcp_initial(0.3, 0.0, 0.20, 0, 0, 0);
//            Point6D tcp_current = tcp_initial;
//
//            ctrl->reset();
//            double settle_time = 0;
//            double final_force = 0;
//
//            for (int i = 0; i < 5000; ++i) {
//                double t = i * 0.001;
//                double F_z = env.getForceZ(tcp_current.z);
//                Point6D delta = ctrl->updateZ(F_z);
//                tcp_current.z = tcp_initial.z + delta.z;
//
//                // 检测调节时间（进入±2%误差带）
//                if (settle_time == 0 && std::abs(F_z - 5.0) < 0.1) {
//                    settle_time = t;
//                }
//
//                if (i == 4999) final_force = F_z;
//            }
//
//            results.push_back({ name, settle_time, final_force });
//        };
//
//    test_preset(ctrl_polishing, "Polishing");
//    test_preset(ctrl_assembly, "Assembly");
//    test_preset(ctrl_grinding, "Grinding");
//
//    printf("  场景      | 调节时间 | 稳态力 | 力误差\n");
//    printf("  ----------|----------|--------|--------\n");
//    for (const auto& r : results) {
//        printf("  %-10s| %.3fs    | %.3fN  | %.4fN\n",
//            r.name.c_str(), r.settling_time, r.steady_force,
//            std::abs(r.steady_force - 5.0));
//    }
//    printf("\n");
//}
//
///**
// * @brief 示例5：在线调整参数
// */
//void example5_online_tuning() {
//    printf("\n========================================\n");
//    printf("示例5: 在线参数调整\n");
//    printf("========================================\n\n");
//
//    auto ctrl = Factory::createZAxisIntegral(1.0, 10.0, 100.0, 0.001, 5.0);
//    ctrl->setDesiredForce(DOF::Z, 5.0);
//
//    Environment env(1000.0, 0.25);
//    Point6D tcp(0.3, 0.0, 0.20, 0, 0, 0);
//    Point6D tcp_initial = tcp;
//
//    std::ofstream file("example5_result.csv");
//    file << "Time,Eta,Z,Force\n";
//
//    printf("测试不同积分增益的影响:\n\n");
//
//    std::vector<double> eta_values = { 0, 2, 5, 10, 20 };
//
//    for (double eta : eta_values) {
//        printf("  测试 eta = %.0f...\n", eta);
//
//        // 修改参数
//        IntegralParams params = ctrl->getParams();
//        params.setIntegralDOF(DOF::Z, eta, 1.0);
//        ctrl->setParams(params);
//        ctrl->reset();
//
//        tcp = tcp_initial;
//
//        for (int i = 0; i < 3000; ++i) {
//            double t = i * 0.001;
//            double F_z = env.getForceZ(tcp.z);
//            Point6D delta = ctrl->updateZ(F_z);
//            tcp.z = tcp_initial.z + delta.z;
//
//            if (i % 30 == 0) {
//                file << t << "," << eta << "," << tcp.z << "," << F_z << "\n";
//            }
//        }
//    }
//
//    file.close();
//    printf("\n数据保存: example5_result.csv\n");
//}
//
///**
// * @brief 主函数
// */
//int main() {
//    printf("\n========================================\n");
//    printf("导纳控制器测试程序\n");
//    printf("========================================\n");
//
//    try {
//        example1_z_axis_force_control();
//        example2_circular_trajectory();
//        example3_real_usage_template();
//        example4_preset_comparison();
//        example5_online_tuning();
//
//        printf("\n========================================\n");
//        printf("所有测试完成！\n");
//        printf("========================================\n\n");
//
//    }
//    catch (const std::exception& e) {
//        printf("错误: %s\n", e.what());
//        return -1;
//    }
//
//    return 0;
//}