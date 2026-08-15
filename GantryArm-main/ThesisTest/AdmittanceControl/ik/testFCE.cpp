#include "ForceControlExecutor.h"
#include "AdmittanceController.h"
#include "TrajectoryOptimizer.h"
#include "RobotArm.h"
#include "CollisionChecker.h"
#include <iostream>
#include <thread>
#include <chrono>

// 模拟的机器人运动控制器（你需要替换成真实的）
class MockRobotController {
public:
    void sendTrajectory(const std::vector<std::array<double, 8>>& jointTraj) {
        std::cout << "[MockRobot] Received trajectory with " << jointTraj.size() << " points" << std::endl;
        // 这里应该发送给真实机器人控制器
        isMoving_ = true;
        moveStartTime_ = std::chrono::steady_clock::now();
    }

    bool waitForCompletion(int timeout_ms) {
        // 模拟运动耗时（实际应该查询机器人状态）
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - moveStartTime_).count();

        if (elapsed < 100) { // 模拟100ms运动时间
            std::this_thread::sleep_for(std::chrono::milliseconds(100 - elapsed));
        }

        isMoving_ = false;
        std::cout << "[MockRobot] Motion completed" << std::endl;
        return true;
    }

private:
    bool isMoving_ = false;
    std::chrono::steady_clock::time_point moveStartTime_;
};

// 模拟力传感器（你需要替换成真实的传感器读取）
class MockForceSensor {
public:
    Eigen::Matrix<double, 6, 1> readForce() {
        Eigen::Matrix<double, 6, 1> force;
        // 模拟传感器数据（实际应该从硬件读取）
        force << 0.5,-60, -60,  // Fx, Fy, Fz (N)
            0.0, 0.0, 0.0;   // Mx, My, Mz (N·m)
        return force;
    }
};

int main()
{
    try {
        std::cout << "========== 力控执行器示例 ==========" << std::endl;

        // ===== 1. 创建导纳控制器 =====
        IntegralParams admittanceParams;
        // 设置导纳参数（根据你的需求调整）
        admittanceParams.mass.head<3>().setConstant(1.0);      // 平动质量 1kg
        admittanceParams.damping.head<3>().setConstant(20.0);  // 平动阻尼
        admittanceParams.stiffness.head<3>().setConstant(100.0); // 平动刚度

        admittanceParams.mass.tail<3>().setConstant(0.1);      // 旋转质量
        admittanceParams.damping.tail<3>().setConstant(2.0);   // 旋转阻尼
        admittanceParams.stiffness.tail<3>().setConstant(10.0); // 旋转刚度

        admittanceParams.dt = 0.05; // 50ms控制周期

        AdmittanceController admittanceController(admittanceParams, false); // 不使用积分项

        // 设置期望力（例如Z方向期望-5N的接触力）
        Eigen::Matrix<double, 6, 1> desiredForce;
        desiredForce << 0, 0, -20, 0, 0, 0;
        admittanceController.setDesiredWrench(desiredForce);

        std::cout << "✓ 导纳控制器创建完成" << std::endl;

        // ===== 2. 创建轨迹规划器 =====
        // 这里需要你的RobotArm和CollisionSystem实例
        // 示例中假设你已经有这些对象
        RobotArm robotArm; // 你需要正确初始化
        CollisionSystem collisionSystem(robotArm); // 你需要正确初始化

        OptimizeParams optimizeParams;

        auto trajectoryOptimizer = TrajectoryOptimizer::create(
            "GA", robotArm, collisionSystem, optimizeParams);

        std::cout << "✓ 轨迹规划器创建完成" << std::endl;

        // ===== 3. 创建力控执行器 =====
        ForceControlExecutor forceExecutor(&admittanceController, trajectoryOptimizer.get());
        forceExecutor.setControlPeriod(50); // 50ms控制周期

        std::cout << "✓ 力控执行器创建完成" << std::endl;

        // ===== 4. 设置回调函数 =====
        MockRobotController mockRobot;

        // 日志回调
        forceExecutor.setLogCallback([](const std::string& msg) {
            std::cout << "[ForceControl] " << msg << std::endl;
            });

        // 发送运动指令回调
        forceExecutor.setSendMotionCallback(
            [&mockRobot](const std::vector<std::array<double, 8>>& jointTraj) {
                mockRobot.sendTrajectory(jointTraj);
            });

        // 等待运动完成回调
        forceExecutor.setIsMotionCompleteCallback(
            [&mockRobot]() -> bool {
                return mockRobot.waitForCompletion(1000);
            });

        std::cout << "✓ 回调函数设置完成" << std::endl;

        // ===== 5. 设置预定轨迹（弧形轨迹示例）=====
        std::vector<Eigen::Matrix4d> plannedTrajectory;

        // 生成一条简单的直线轨迹（5个点）
        for (int i = 0; i < 5; ++i) {
            Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
            pose(0, 3) = 500.0 + i * 10.0;  // X方向移动
            pose(1, 3) = 0.0;
            pose(2, 3) = 300.0;
            plannedTrajectory.push_back(pose);
        }

        forceExecutor.setPlannedTrajectory(plannedTrajectory);
        std::cout << "✓ 预定轨迹设置完成（" << plannedTrajectory.size() << "个点）" << std::endl;

        // ===== 6. 启动力控线程 =====
        forceExecutor.start();
        std::cout << "✓ 力控线程已启动" << std::endl;

        // ===== 7. 模拟传感器数据更新（在主线程中周期性更新）=====
        MockForceSensor forceSensor;

        std::cout << "\n开始力控执行，按Ctrl+C停止...\n" << std::endl;

        // 模拟传感器数据更新循环（实际应该在传感器采集线程中）
        for (int i = 0; i < 100 && forceExecutor.isRunning(); ++i) {
            // 读取传感器数据
            Eigen::Matrix<double, 6, 1> forceData = forceSensor.readForce();

            // 更新到力控执行器
            forceExecutor.updateSensorData(forceData);

            // 模拟传感器采样周期（例如10ms）
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // ===== 8. 等待轨迹执行完成或手动停止 =====
        std::this_thread::sleep_for(std::chrono::seconds(200));

        // ===== 9. 停止力控 =====
        forceExecutor.stop();
        std::cout << "\n✓ 力控执行器已停止" << std::endl;

        std::cout << "\n========== 示例执行完成 ==========" << std::endl;

    }
    catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}