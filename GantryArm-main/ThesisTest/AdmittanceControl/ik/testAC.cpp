//#include <iostream>
//#include "AdmittanceController.h"
//
//int main() {
//    // 1. 创建并初始化参数
//    // 假设平动质量1kg，阻尼10，刚度100，旋转质量0.1kg·m²，阻尼1，刚度10
//    IntegralParams params(
//        10, 200, 4000,    // 平动参数 (M, B, K)
//        0.1, 1.0, 10.0,      // 旋转参数 (M, B, K)
//        1.1, 1.0, 5, 0.1, // 积分参数 (η, k_e,积分限幅, 衰减)
//        0.05                // 采样时间
//    );
//
//    // 2. 创建控制器，启用积分补偿
//    AdmittanceController controller(params, true);
//
//    // 3. 设置期望力为0（静态期望，无外力）
//    Eigen::Matrix<double, 6, 1> desired_force = Eigen::Matrix<double, 6, 1>::Zero();
//    desired_force[2] = -20;
//    controller.setDesiredWrench(desired_force);
//
//    // 4. 循环模拟测量力并调用update（假设周期1ms，模拟100个周期）
//    // 这里模拟一个沿Z轴受到的力扰动，比如5牛顿
//    for (int i = 0; i < 100; ++i) {
//        Eigen::Matrix<double, 6, 1> measured_force = Eigen::Matrix<double, 6, 1>::Zero();
//        measured_force(2) =0; // Z轴方向测量外力5N
//
//        Point6D offset = controller.update(measured_force);
//
//        // 输出位姿偏移量
//        std::cout << "Step " << i << ": Offset ["
//            << offset.x << ", " << offset.y << ", " << offset.z*1000 << ", "
//            << offset.rx << ", " << offset.ry << ", " << offset.rz << "]\n";
//
//        // 5. 在实际应用里，将offset叠加到当前机械臂末端位姿，
//        // 调用逆运动学求解新的关节角，命令机器人执行
//        // 伪代码示例：
//        // RobotPose current_pose = robot.getCurrentPose();
//        // RobotPose desired_pose = current_pose + offset;
//        // JointAngles q = inverseKinematics(desired_pose);
//        // robot.setJointAngles(q);
//    }
//
//    return 0;
//}