//#include <iostream>
//#include <vector>
//#include <string>
//#include <chrono>
//#include <thread>
//#include "TrajectoryOptimizer.h"
//#include <Eigen/Dense>
//#include "curve_processor_eigen.h"
//#include "TrajectoryOptimizer.h"
///**
//* @brief 生成倾斜椭圆轨迹，末端指向中心点
//*
//* @param center 工作空间中心点 [x, y, z]
//* @param semi_major 长半轴（米）
//* @param semi_minor 短半轴（米）
//* @param tilt_angle 倾斜角度（度），绕X轴旋转
//* @param num_points 采样点数
//* @param height_offset 椭圆平面相对中心的高度偏移（米），默认0
//* @return std::vector<Eigen::Matrix4d> 轨迹点序列（4x4齐次变换矩阵）
//*/
//std::vector<Eigen::Matrix4d> generateTiltedEllipticalTrajectory(
//    const Eigen::Vector3d& center,
//    double semi_major,
//    double semi_minor,
//    double tilt_angle = 45.0,
//    int num_points = 100,
//    double height_offset = 0.0
//) {
//    std::vector<Eigen::Matrix4d> trajectory;
//    trajectory.reserve(num_points);
//
//    // 计算旋转矩阵（绕X轴）
//    double angle_rad = tilt_angle * M_PI / 180.0;
//    Eigen::Matrix3d rotation = Eigen::AngleAxisd(angle_rad, Eigen::Vector3d::UnitX()).toRotationMatrix();
//
//    for (int i = 0; i < num_points; ++i) {
//        double t = 2 * M_PI * i / num_points;
//
//        // 1. 标准椭圆上的点（XY平面）
//        Eigen::Vector3d p_local(semi_major * std::cos(t),
//            semi_minor * std::sin(t),
//            height_offset);
//
//        // 2. 旋转 + 平移到中心
//        Eigen::Vector3d position = center + rotation * p_local;
//
//        // 3. 计算姿态：Z轴指向中心
//        Eigen::Vector3d z_axis = (center - position).normalized();
//
//        // 选择参考方向计算X轴
//        Eigen::Vector3d reference = Eigen::Vector3d::UnitZ();
//        if (std::abs(z_axis.dot(reference)) > 0.99) {
//            reference = Eigen::Vector3d::UnitX();
//        }
//
//        Eigen::Vector3d x_axis = z_axis.cross(reference).normalized();
//        Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();
//
//        // 4. 构建4x4齐次变换矩阵
//        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
//        transform.block<3, 1>(0, 0) = x_axis;      // 第1列：X轴
//        transform.block<3, 1>(0, 1) = y_axis;      // 第2列：Y轴
//        transform.block<3, 1>(0, 2) = z_axis;      // 第3列：Z轴
//        transform.block<3, 1>(0, 3) = position;    // 第4列：位置
//
//        trajectory.push_back(transform);
//    }
//
//    return trajectory;
//}
//
//
//int main() {
//    try {
//        RobotArm robotArm(218.4);
//        CollisionSystem collisionSystem(robotArm);
//        OptimizeParams params;
//        params.population_size = 40;
//        vector<Vector3d> real_point = {
//        {694.67	,-1121.96,	672.52},
//        {658.29	,-1123.48,	436.29},
//        {1997.93,-1067.46,	672.25},
//        {2034.2	,-1065.94,	436},
//        {1851.52,-2274.63,	635.65},
//        {1887.82,-2273.11,	399.25},
//        {941.33	,-2312.69,	635.84},
//        {904.94	,-2314.21,	399.49}
//        };
//        for (auto& point : real_point)
//        {
//            point.x() = point.x() / 1000.0 - 0.5;
//            point.y() = point.y() / 1000.0;
//            point.z() /= 1000.0;
//        }
//        std::string processor_path = "sideTrajectory.csv";
//        CurveProcessor processor(processor_path);//12345678
//        processor.setRealCorners(real_point);
//        auto testTrajectory = processor.generateOrientInpIKTrajectory();
//        //Eigen::Vector3d center(900 , -1000, 900 - 2358);
//        //auto testTrajectory = generateTiltedEllipticalTrajectory(
//        //    center,      // 中心点
//        //    900,        // 长半轴
//        //    700,        // 短半轴
//        //    30,        // 倾斜角度
//        //    300,          // 采样点数
//        //    400
//        //);
//        int times = 100;
//        std::string alg = "MOEAD";
//        auto optimizer = TrajectoryOptimizer::create(alg, robotArm, collisionSystem);
//
//        if (testTrajectory.empty()) {
//            std::cerr << "轨迹为空，请先准备轨迹点" << std::endl;
//            return -1;
//        }
//        // 循环测试times次
//        for (int run_idx = 1; run_idx <= times; ++run_idx) {
//            std::cout << "开始第 " << run_idx << " 次异步优化..." << std::endl;
//
//            // 异步启动轨迹优化
//            AsyncOptimizeTask async_task = optimizer->optimizeTrajectoryAsync(testTrajectory,
//                [run_idx](const std::string& progress_msg) {
//                    std::cout << "[Run " << run_idx << "] Progress: " << progress_msg << std::endl;
//                });
//
//            // 等待任务完成
//            while (!async_task.isReady()) {
//                std::this_thread::sleep_for(std::chrono::milliseconds(100));
//                // 也可以加超时或用户中断处理
//            }
//
//            // 获取计算结果（关节角序列）
//            auto joint_trajectory = async_task.get();
//
//            // 再从优化器实例获取每点耗时（单位ms）
//            const auto& elapsed_times = optimizer->getPerPointElapsedTimes();
//
//            if (joint_trajectory.size() != elapsed_times.size()) {
//                std::cerr << "跑批数据大小不匹配，跳过导出" << std::endl;
//                continue;
//            }
//
//            // 组装输出文件名
//            std::string filename = "essay_RALLE/"+ alg +"/run_" + std::to_string(run_idx) + "_optimized.csv";
//
//            // 导出含时间的CSV文件
//            TrajectoryOptimizer::exportJointTrajectoryWithTime(joint_trajectory, elapsed_times, filename);
//
//            std::cout << "第 " << run_idx << " 次优化完成，结果已导出到：" << filename << std::endl;
//        }
//
//        std::cout << "全部times次优化结束。" << std::endl;
//
//    }
//    catch (const std::exception& e) {
//        std::cerr << "出现异常: " << e.what() << std::endl;
//        return -1;
//    }
//
//
//    return 0;
//}