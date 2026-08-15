
 //   #include "TrajectoryOptimizer.h"
 //   #include "AlgorithmComparator.h"
 //   #include <iostream>
 //   #include <iomanip>
 //   #include <fstream>
 //   #include <vector>
 //   #include <cmath>
 //   #include <Eigen/Dense>
 //   #include "curve_processor_eigen.h"
 // 

 //   // 新增轨迹输出函数（可封装为 RobotArm 类成员）
 //   void exportJointTrajectory(const std::vector<std::array<double, 8>>& trajectory,
 //       const std::string& filename = "joint_trajectory.csv")
 //   {
 //       std::ofstream file(filename);
 //       if (!file.is_open()) {
 //           throw std::runtime_error("无法创建输出文件");
 //       }

 //       // 设置高精度输出（参考机械臂精度需求）
 //       file << std::fixed << std::setprecision(6);

 //       for (const auto& joint : trajectory) {
 //           for (size_t i = 0; i < joint.size(); ++i) {
 //               file << joint[i];
 //               if (i != joint.size() - 1) file << ","; // 最后一位不加逗号
 //           }
 //           file << "\n";
 //       }
 //   }

 //   /**
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
 //   std::vector<Eigen::Matrix4d> generateTiltedEllipticalTrajectory(
 //       const Eigen::Vector3d& center,
 //       double semi_major,
 //       double semi_minor,
 //       double tilt_angle = 45.0,
 //       int num_points = 100,
 //       double height_offset = 0.0
 //   ) {
 //       std::vector<Eigen::Matrix4d> trajectory;
 //       trajectory.reserve(num_points);

 //       // 计算旋转矩阵（绕X轴）
 //       double angle_rad = tilt_angle * M_PI / 180.0;
 //       Eigen::Matrix3d rotation = Eigen::AngleAxisd(angle_rad, Eigen::Vector3d::UnitX()).toRotationMatrix();

 //       for (int i = 0; i < num_points; ++i) {
 //           double t = 2*M_PI * i / num_points;

 //           // 1. 标准椭圆上的点（XY平面）
 //           Eigen::Vector3d p_local(semi_major * std::cos(t),
 //               semi_minor * std::sin(t),
 //               height_offset);

 //           // 2. 旋转 + 平移到中心
 //           Eigen::Vector3d position = center + rotation * p_local;

 //           // 3. 计算姿态：Z轴指向中心
 //           Eigen::Vector3d z_axis = (center - position).normalized();

 //           // 选择参考方向计算X轴
 //           Eigen::Vector3d reference = Eigen::Vector3d::UnitZ();
 //           if (std::abs(z_axis.dot(reference)) > 0.99) {
 //               reference = Eigen::Vector3d::UnitX();
 //           }

 //           Eigen::Vector3d x_axis = z_axis.cross(reference).normalized();
 //           Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();

 //           // 4. 构建4x4齐次变换矩阵
 //           Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
 //           transform.block<3, 1>(0, 0) = x_axis;      // 第1列：X轴
 //           transform.block<3, 1>(0, 1) = y_axis;      // 第2列：Y轴
 //           transform.block<3, 1>(0, 2) = z_axis;      // 第3列：Z轴
 //           transform.block<3, 1>(0, 3) = position;    // 第4列：位置

 //           trajectory.push_back(transform);
 //       }

 //       return trajectory;
 //   }
 //   int main() {

 //       RobotArm arm(218.4);
 //       CollisionSystem collision_system(arm);
 //       OptimizeParams params;

 //       AlgorithmComparator comparator(arm, collision_system, params);
 //       vector<Vector3d> real_point = {
 //       {694.67	,-1121.96,	672.52},
 //       {658.29	,-1123.48,	436.29},
 //       {1997.93,-1067.46,	672.25},
 //       {2034.2	,-1065.94,	436},
 //       {1851.52,-2274.63,	635.65},
 //       {1887.82,-2273.11,	399.25},
 //       {941.33	,-2312.69,	635.84},
 //       {904.94	,-2314.21,	399.49}
 //           };
 //       for (auto& point : real_point)
 //       {
 //           point.x() = point.x() / 1000.0 - 0.5;
 //           point.y() = point.y() / 1000.0;
 //           point.z() /= 1000.0;
 //       }
 //       std::string processor_path =  "sideTrajectory.csv";
 //       CurveProcessor processor(processor_path);//12345678
 //       processor.setRealCorners(real_point);
 //       auto trajectory = processor.generateOrientInpIKTrajectory();


 //       // 设置要比较的算法
 //       //comparator.setAlgorithmTypes({ "GA", "ISGA", "MOEAD", "NSGA" });
 //       comparator.setAlgorithmTypes({ "GA","MOEAD"});

 //       


 //       //Eigen::Vector3d center(900 , -1000, 900 - 2358);

 //       //// 生成轨迹：长轴15cm，短轴10cm，倾斜45度，100个点
 //       //auto trajectory = generateTiltedEllipticalTrajectory(
 //       //    center,      // 中心点
 //       //    900,        // 长半轴
 //       //    700,        // 短半轴
 //       //    30,        // 倾斜角度
 //       //    300,          // 采样点数
 //       //    400
 //       //);
 //       for (int i = 0; i <50; i++)
 //       {
 //           auto results = comparator.compareAlgorithms(trajectory);
 //           // 执行比较
 //           //auto results = comparator.compareAlgorithms("2025-09-09-15-02-upperpoint.csv",true);
 //           //auto results = comparator.compareAlgorithms("sine_trajectory.csv", true);

 //           // 导出简化表格
 //           comparator.exportSimplifiedComparison(results, "curve_comparison.csv");
 //       }
 //       
 //     
 //       return 0;
 //       //RobotArm arm;
 //       //CollisionSystem collision_system(arm);

 //       //OptimizeParams params;
 //       auto optimizer = TrajectoryOptimizer::create("GA", arm, collision_system, params);

 //       try {
 //           // 方式1：同步优化，带字符串回调（在主线程执行）
 //           auto joint_trajectory_sync = optimizer->optimizeFromCSV(
 //               "test_Data_Trajectory3_RotationAroundVerticalAxis_DataFusion.csv",
 //               [](const std::string& progress) {
 //                   std::cout << "[同步] " << progress << std::endl;
 //               }
 //           );

 //           std::cout << "同步优化完成！" << std::endl;
 //           TrajectoryOptimizer::exportJointTrajectory(joint_trajectory_sync);

 //           // 方式2：异步优化，带字符串回调（回调在后台线程执行）
 //           auto task = optimizer->optimizeFromCSVAsync(
 //               "test_Data_Trajectory3_RotationAroundVerticalAxis_DataFusion.csv",
 //               [](const std::string& progress) {
 //                   // 这个回调在工作线程中执行，可以用于日志、更新UI（需线程安全）等
 //                   std::cout << "[异步] " << progress << std::endl;
 //               }
 //           );

 //           // 主线程可以做其他事，同时监控任务状态
 //           while (!task.isReady()) {
 //               // 可以在这里检查用户输入来取消
 //               // std::cout << "主线程空闲中... (按 'c' 取消)" << std::endl;
 //               // char input; std::cin >> input;
 //               // if (input == 'c') {
 //               //     task.cancel();
 //               //     break;
 //               // }

 //               std::this_thread::sleep_for(std::chrono::milliseconds(500));
 //           }

 //           if (task.isReady()) {
 //               auto joint_trajectory_async = task.get();
 //               std::cout << "异步优化完成！" << std::endl;
 //               TrajectoryOptimizer::exportJointTrajectory(joint_trajectory_async);
 //           }
 //           else {
 //               std::cout << "任务被取消或出错" << std::endl;
 //           }

 //       }
 //       catch (const std::exception& e) {
 //           std::cerr << "Error: " << e.what() << std::endl;
 //           return -1;
 //       }

 //       // 方式2：同步优化，带进度回调
 //       // auto joint_trajectory = optimizer->optimizeFromCSV(
 //       //     "your_file.csv",
 //       //     [](size_t current, size_t total) {
 //       //         std::cout << "Processing " << current << "/" << total << std::endl;
 //       //     }
 //       // );

 //       return 0;


 //       //RobotArm arm;
 //       //CollisionSystem collision_system(arm);
 //   
 //       //// 配置优化参数
 //       //TrajectoryOptimizer::OptimizeParams params;

 //       //// 创建优化器
 //       //auto optimizer = TrajectoryOptimizer::create("GA", arm, collision_system, params);
 //       ////auto optimizer = TrajectoryOptimizer::create("NSGA", arm, collision_system, params);
 //       ////auto optimizer = TrajectoryOptimizer::create("MOEAD", arm, collision_system, params);
 //       ////auto optimizer = TrajectoryOptimizer::create("ISGA", arm, collision_system, params);
 //       ////auto optimizer = TrajectoryOptimizer::create("DE", arm, collision_system, params);

 //       ////const auto trajectory = RobotArm::loadmmTrajectoryFromCSV("2025-09-09-15-02-upperpoint.csv");
 //       //const auto trajectory = RobotArm::loadTrajectoryFromCSV("test_Data_Trajectory3_RotationAroundVerticalAxis_DataFusion.csv");

 //       //if (trajectory.empty()) {
 //       //    throw runtime_error("轨迹数据加载失败");
 //       //}

 //       //Eigen::Matrix4d prev_target_pose = Eigen::Matrix4d::Identity();

 //       //RobotArm::Solution prev_solution;
 //       //std::vector<TrajectoryOptimizer::Individual> prev_population;
 //       //std::vector<std::array<double, 8>> joint_trajectory;

 //       //for (size_t i = 0; i < trajectory.size(); ++i) {
 //       //    const auto& pose = trajectory[i];

 //       //    // 输出进度信息
 //       //    std::cout << "Processing point " << (i + 1) << "/" << trajectory.size() << endl;

 //       //    auto optimal = optimizer->optimize(
 //       //        pose,           // 当前目标位姿
 //       //        prev_target_pose,    // 上一目标位姿
 //       //        prev_solution,  // 上一关节解
 //       //        prev_population // 上一代种群
 //       //    );
 //       //    joint_trajectory.push_back(optimal);

 //       //    // 更新参考参数
 //       //    prev_target_pose = pose; // 更新历史位姿
 //       //}
 //   
 //       //// 结果输出
 //       //exportJointTrajectory(joint_trajectory);
 //       //std::cout << "\nOptimization completed. " << joint_trajectory.size()
 //       //    << " points saved to optimized_joints.csv\n";

 //       //return 0;
 //   }
