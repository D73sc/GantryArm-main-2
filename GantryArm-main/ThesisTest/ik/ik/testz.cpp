//// RobotArm.h 中新增内容
////// main.cpp （新增部分）
//#include "RobotArm.h"
//#include <algorithm>
//#include <fstream>
//#include <random> // 新增随机数头文件
//// 角度转弧度
//constexpr double deg2rad(double deg) {
//    return deg * PI / 180.0;
//}
//        // 定义关节限位 (直接取自您的初始化)
//const array<pair<double, double>, 8> limits = { {
//    {0, 2500},    // d0
//    {0, 3000},    // d1
//    {0, 2000},    // d2
//    {-PI, PI},// theta3
//    {deg2rad(-45), deg2rad(100)},  // theta4
//    {-PI, PI},// theta5
//    {deg2rad(-120), deg2rad(120)}, // theta6
//    {-PI, PI} // theta7
//} };
//
//
//// main.cpp 新增优化逻辑
//struct OptimizationResult {
//    int valid_45;
//    int valid_47;
//    int valid;
//};
//// 新增函数：在 theta4/theta5 的关节空间进行网格化搜索，统计有效解分布
//void gridSearchInverseSolutions(
//    RobotArm& arm,
//    const Matrix4d& T_goal,
//    const string& method,
//    int grid_steps = 100
//) {
//
//    size_t axis1_idx, axis2_idx;
//    const double axis1_min = -45 * PI / 180;
//    const double axis1_max = 100 * PI / 180;
//    const double axis2_min = -180 * PI / 180;
//    const double axis2_max = 180 * PI / 180;
//
//    const double axis1_step = (axis1_max - axis1_min) / grid_steps;
//    const double axis2_step = (axis2_max - axis2_min) / grid_steps;
//
//
//    int total_valid = 0;
//    for (int i = 1; i <= grid_steps; ++i) {
//        const double axis1 = axis1_min + i * axis1_step;
//
//        for (int j = 1; j <= grid_steps; ++j) {
//            const double axis2 = axis2_min + j * axis2_step;
//
//            // 根据方法选择已知角度
//            array<double, 2> known_angles;
//            known_angles = { axis1, axis2 };
//            
//
//            const auto solutions = arm.inverseKinematics(T_goal, known_angles, method);
//
//            const bool has_valid = !solutions.empty();
//            const size_t count = solutions.size();
//            total_valid += has_valid ? 1 : 0;
//
//
//        }
//    }
//}
//
//OptimizationResult evaluateParameters(
//    RobotArm& arm,
//    const Matrix4d& T_goal,
//    double anz,
//    double aoz,
//    mt19937& gen,
//    const array<pair<double, double>, 8>& limits)
//{
//    // 更新当前参数
//    RobotArm::AdaptiveParams params{ anz, aoz };
//    arm.setAdaptiveParams(params);
//    int grid_steps = 50;
//    OptimizationResult result{0,0,0};
//    size_t axis1_idx, axis2_idx;
//    const double axis1_min = -45 * PI / 180;
//    const double axis1_max = 100 * PI / 180;
//    const double axis2_min = -180 * PI / 180;
//    const double axis2_max = 180 * PI / 180;
//
//    const double axis1_step = (axis1_max - axis1_min) / grid_steps;
//    const double axis2_step = (axis2_max - axis2_min) / grid_steps;
//
//
//    int total_valid = 0;
//    for (int i = 1; i <= grid_steps; ++i) {
//        const double axis1 = axis1_min + i * axis1_step;
//
//        for (int j = 1; j <= grid_steps; ++j) {
//            const double axis2 = axis2_min + j * axis2_step;
//
//            // 根据方法选择已知角度
//            array<double, 2> known_angles;
//            known_angles = { axis1, axis2 };
//
//
//            // 测试两个方法
//
//            auto solutions_45 = arm.inverseKinematics(T_goal, known_angles, "45");
//            auto solutions_47 = arm.inverseKinematics(T_goal, known_angles, "47");
//            auto solutions = arm.inverseKinematics(T_goal, known_angles, "auto");
//
//            const bool has_valid_45 = !solutions_45.empty();
//            result.valid_45 += has_valid_45 ? 1 : 0;
//            const bool has_valid_47 = !solutions_47.empty();
//            result.valid_47 += has_valid_47 ? 1 : 0;
//            const bool has_valid = !solutions.empty();
//            result.valid += has_valid ? 1 : 0;
//
//        }
//    }
//    return result;
//}
//
//void adaptiveParameterTuning(RobotArm& arm, const vector<Matrix4d>& test_set) {
//    const double INIT_ANZ = 5;
//    const double INIT_AOZ = 5;
//    const int MAX_ITER = 1;
//
//    double best_anz = INIT_ANZ;
//    double best_aoz = INIT_AOZ;
//    int max_valid = 0;
//    
//    mt19937 gen(0x1234ABCD);  // 确保可重复性测试
//
//    for (int iter = 0; iter < MAX_ITER; ++iter) {
//        OptimizationResult total{ 0,0 };
//        int ti = 0;
//        for (const auto& T : test_set) {
//            auto result = evaluateParameters(arm, T, best_anz, best_aoz, gen, limits);
//            double az = abs(T.col(2)[2]);
//            double oz = abs(T.col(1)[2]);
//            double nz = abs(T.col(0)[2]);
//            const int auto_valid = result.valid;
//            const int valid_47 = result.valid_47;
//            const int valid_45 = result.valid_45;
//            const int target_valid = std::max(valid_47, valid_45);
//            double anz = best_anz;
//            double aoz=best_aoz;
//            const double threshold = 0.4; // 触发 47 补偿解的临界比例
//            const double emergency_adjust = 0.2; // 急降系数
//            // 调整规则核心逻辑
//        // 计算当前解的构成比例
//            const double ratio_45 = static_cast<double>(valid_45) / (50*50);
//
//
//
//                if (auto_valid < target_valid) {
//                    const double adjust_factor = 1.0 - 0.1 * (1.0 -
//                        static_cast<double>(auto_valid) / target_valid);
//
//                    if (target_valid == valid_47) {
//                        best_anz *= adjust_factor;
//                        best_aoz *= adjust_factor;
//                    }
//                    else {
//                        best_anz /= adjust_factor;  // 逆向调整提升阈值
//                        best_aoz /= adjust_factor;
//                    }
//                }
//
//            
//            ti++;
//            cout << "Iter " << ti + 1 << " anz=" << best_anz
//                << " aoz=" << best_aoz << " (47="
//                << (result.valid_47) << " 45=" << result.valid_45 << ")\n";
//
//        }
//
//
//
//    }
//
//    arm.adaptive_params_.anz = best_anz;
//    arm.adaptive_params_.aoz = best_aoz;
//}
//
//// 修改后的主函数
//int main() {
//    try {
//        RobotArm arm(100.0);
//        //        // 初始化具有固定种子的伪随机数生成器（确保结果可重现）
//        constexpr uint32_t RANDOM_SEED = 0x1234ABCD; // 固定种子值（可自定义修改）
//        mt19937 gen(RANDOM_SEED);
//
//
//
//        // 创建随机数分布器
//        vector<uniform_real_distribution<>> dists;
//        for (const auto& limit : limits) {
//            dists.emplace_back(limit.first, limit.second);
//        }
//
//        // 生成 5组随机测试用例
//        int NUM_TESTS = 1000;
//        vector<Matrix4d> test_set;
//        for (int i = 0; i < NUM_TESTS; ++i) {
//            array<double, 8> q_rand;
//
//            // 填充随机关节值
//            for (size_t j = 0; j < 8; ++j) {
//                q_rand[j] = dists[j](gen);
//
//            }
//            Eigen::Matrix4d test = arm.forwardKinematics(q_rand);
//            //if (abs(test.col(2)[2]) > abs(test.col(1)[2]) && abs(test.col(2)[2]) > abs( test.col(0)[2]))
//            //{
//            //    test_set.push_back(test);
//
//            //}
//            //else
//            //    NUM_TESTS++;
//          test_set.push_back(test);
//
//        }
//
//        // 执行参数优化
//        adaptiveParameterTuning(arm, test_set);
//
//        // 输出最终优化结果
//        cout << "Optimized parameters: anz=" << arm.adaptive_params_.anz
//            << " aoz=" << arm.adaptive_params_.aoz << endl;
//    }
//    catch (const exception& e) { 
//
//    }
//}