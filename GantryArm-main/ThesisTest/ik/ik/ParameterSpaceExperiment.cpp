//#include "TrajectoryOptimizer.h"
//#include "AlgorithmComparator.h"
//#include <iostream>
//#include <iomanip>
//
//int main() {
//
//    RobotArm arm;
//    CollisionSystem collision_system(arm);
//    OptimizeParams params;
//    auto optimizer = TrajectoryOptimizer::create("GA", arm, collision_system, params);
//    std::cout << "\n========== 参数空间扫描实验（10000个点） ==========\n";
//    // 定义两个位姿
//    //Eigen::Matrix4d warmup_pose = Eigen::Matrix4d::Identity();
//    //warmup_pose.block<3, 1>(0, 3) << 500, 0, 300;
//    Eigen::Matrix4d warmup_pose=arm.poseToTransformMatrix(102.998 ,- 972.302	,1224.898	-2358,0.563289	,0.427436 ,- 0.500479	,0.499525);
//
//    //Eigen::Matrix4d test_pose = Eigen::Matrix4d::Identity();
//    //test_pose.block<3, 1>(0, 3) << 550, 100, 350;
//    Eigen::Matrix4d test_pose = arm.poseToTransformMatrix(225.51 ,- 989.22	,1243.405 - 2358,0.571443	-2358,0.416472 ,- 0.490734	,0.509102);
//    Eigen::Matrix4d actual_test_pose = arm.poseToTransformMatrix(347.207, - 1006.025	,1266.772 - 2358,0.579387	,0.405347 ,- 0.4808	,0.518493);
//    auto results = optimizer->scanParameterSpace(
//        warmup_pose,
//        test_pose,
//        actual_test_pose,
//        100,  // 种群规模：1-100
//        20,  // 迁移比例：0-1.0
//        5,    // 5ms超时
//        [](const std::string& msg) {
//            std::cout << msg << std::endl;
//        });
//
//    optimizer->exportExperimentResults(results, "experiment_results.csv");
//}
//
//
