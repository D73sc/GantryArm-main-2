//// main.cpp （新增部分）
//#include "RobotArm.h"
//#include "curve_processor_eigen.h"
//#include <algorithm>
//#include <fstream>
//#include <random> // 新增随机数头文件
//#include <filesystem>
//namespace fs = std::filesystem;
//
//// 角度转弧度
//constexpr double deg2rad(double deg) {
//    return deg * PI / 180.0;
//}
//
//
//// 修改后的主函数示例
//int main() {
//    try {
//        RobotArm arm(0);
//        // ========== 创建输出目录结构 ==========
//        fs::path results_dir = "results";
//        fs::create_directory(results_dir);
//
//        vector<string> methods = { "47", "45", "auto" };
//        for (const auto& method : methods) {
//            fs::create_directory(results_dir / method);
//        }
//        //// 初始化具有固定种子的伪随机数生成器
//        //constexpr uint32_t RANDOM_SEED = 0x1234ABCD;
//        //mt19937 gen(RANDOM_SEED);
//        //// 定义关节限位
//        //auto limits = arm.getJointLimits();
//
//        //// 创建随机数分布器
//        //vector<uniform_real_distribution<>> dists;
//        //for (int i = 0; i < 8; i++) {
//        //    dists.emplace_back(limits.min[i], limits.max[i]);
//        //}
//
//        //// 生成测试用例
//        //const int NUM_TESTS = 1000;
//        //vector<Matrix4d> trajectory;
//        //for (int i = 0; i < NUM_TESTS; ++i) {
//        //    array<double, 8> q_rand;
//        //    for (size_t j = 0; j < 8; ++j) {
//        //        q_rand[j] = dists[j](gen);
//        //    }
//        //    trajectory.push_back(arm.forwardKinematics(q_rand));
//        //}
//
//
//         vector<Vector3d> real_point = {
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
//        auto trajectory = processor.generateOrientInpIKTrajectory();
//
//        int grid_steps = 100;
//        if (trajectory.empty()) {
//            throw runtime_error("轨迹数据加载失败");
//        }
//
//        // ========== 定义三种方案 ==========
//        vector<string> methods = { "47", "45", "auto" };
//        map<string, vector<RobotArm::SolutionDiversity>> all_methods_stats;
//
//        // ========== 对每种方案进行测试 ==========
//        for (const auto& method : methods) {
//            cout << "\n\n";
//            cout << "╔══════════════════════════════════════════╗\n";
//            cout << "║     开始测试方案: " << setw(4) << method << "                  ║\n";
//            cout << "╚══════════════════════════════════════════╝\n";
//
//            vector<RobotArm::SolutionDiversity> method_stats;
//            string csv_filename = "grid_" + method + ".csv";
//
//            // 轨迹级计时开始
//            auto trajectory_start = chrono::high_resolution_clock::now();
//
//            for (size_t i = 0; i < trajectory.size(); i++) {
//                cout << "▶ 方案[" << method << "] 轨迹点 " << (i + 1)
//                    << "/" << trajectory.size() << "\r" << flush;
//
//                RobotArm::SolutionDiversity stats = arm.gridSearchInverseSolutions(
//                    trajectory[i], method, grid_steps, csv_filename, false);  // 不打印详细信息
//
//                method_stats.push_back(stats);
//            }
//
//            // 轨迹级计时结束
//            auto trajectory_end = chrono::high_resolution_clock::now();
//            auto trajectory_duration = chrono::duration_cast<chrono::milliseconds>(
//                trajectory_end - trajectory_start);
//            double total_time_ms = trajectory_duration.count();
//
//            all_methods_stats[method] = method_stats;
//
//            // 打印该方案的总体统计
//            cout << "\n\n方案 [" << method << "] 总体统计:\n";
//            cout << "─────────────────────────────────────\n";
//
//            double sum_success = 0, sum_time = 0;
//            for (auto& stat : method_stats) {
//                sum_success += stat.success_rate;
//                sum_time += stat.elapsed_time_ms;
//            }
//
//            int n = method_stats.size();
//            double avg_success = sum_success / n;
//            double avg_time = sum_time / n;
//
//            cout << "  总点数:       " << n << "\n";
//            cout << "  平均成功率:   " << fixed << setprecision(2) << avg_success << "%\n";
//            cout << "  总耗时:       " << fixed << setprecision(2) << total_time_ms << " ms\n";
//            cout << "  平均每点:     " << fixed << setprecision(2) << avg_time << " ms\n";
//            cout << "─────────────────────────────────────\n";
//        }
//
//        // ========== 导出对比表格 ==========
//        cout << "\n正在导出对比数据到 CSV...\n";
//
//        ofstream comparison_file("method_comparison.csv");
//        comparison_file << "Point_Index,Method_47_Success_Rate,Method_47_Time_ms,"
//            << "Method_45_Success_Rate,Method_45_Time_ms,"
//            << "Method_Auto_Success_Rate,Method_Auto_Time_ms\n";
//
//        for (size_t i = 0; i < trajectory.size(); i++) {
//            comparison_file << (i + 1) << ",";
//            comparison_file << fixed << setprecision(2)
//                << all_methods_stats["47"][i].success_rate << ","
//                << all_methods_stats["47"][i].elapsed_time_ms << ",";
//            comparison_file << all_methods_stats["45"][i].success_rate << ","
//                << all_methods_stats["45"][i].elapsed_time_ms << ",";
//            comparison_file << all_methods_stats["auto"][i].success_rate << ","
//                << all_methods_stats["auto"][i].elapsed_time_ms << "\n";
//        }
//        comparison_file.close();
//        cout << "✓ 对比数据已导出到: method_comparison.csv\n";
//
//        // ========== 导出统计摘要 ==========
//        ofstream summary_file("method_summary.csv");
//        summary_file << "Method,Avg_Success_Rate,Total_Time_ms,Avg_Time_Per_Point_ms,"
//            << "Min_Success_Rate,Max_Success_Rate,Std_Success_Rate\n";
//
//        for (const auto& method : methods) {
//            auto& stats = all_methods_stats[method];
//
//            double sum_success = 0, sum_time = 0;
//            double min_success = 100.0, max_success = 0.0;
//
//            for (auto& stat : stats) {
//                sum_success += stat.success_rate;
//                sum_time += stat.elapsed_time_ms;
//                min_success = min(min_success, stat.success_rate);
//                max_success = max(max_success, stat.success_rate);
//            }
//
//            double avg_success = sum_success / stats.size();
//            double avg_time = sum_time / stats.size();
//
//            // 计算标准差
//            double sum_sq_diff = 0;
//            for (auto& stat : stats) {
//                double diff = stat.success_rate - avg_success;
//                sum_sq_diff += diff * diff;
//            }
//            double std_success = sqrt(sum_sq_diff / stats.size());
//
//            summary_file << method << ","
//                << fixed << setprecision(2) << avg_success << ","
//                << sum_time << "," << avg_time << ","
//                << min_success << "," << max_success << ","
//                << std_success << "\n";
//        }
//        summary_file.close();
//        cout << "✓ 统计摘要已导出到: method_summary.csv\n";
//
//        // ========== 最终对比报告 ==========
//        cout << "\n\n";
//        cout << "╔══════════════════════════════════════════════════════╗\n";
//        cout << "║              三种方案综合对比报告                    ║\n";
//        cout << "╚══════════════════════════════════════════════════════╝\n\n";
//
//        cout << setw(15) << "指标" << " | "
//            << setw(12) << "方案47" << " | "
//            << setw(12) << "方案45" << " | "
//            << setw(12) << "方案Auto" << "\n";
//        cout << string(60, '─') << "\n";
//
//        for (const auto& method : methods) {
//            auto& stats = all_methods_stats[method];
//            double sum_success = 0, sum_time = 0;
//            for (auto& stat : stats) {
//                sum_success += stat.success_rate;
//                sum_time += stat.elapsed_time_ms;
//            }
//            double avg_success = sum_success / stats.size();
//
//            if (method == methods[0]) {
//                cout << setw(15) << "平均成功率" << " | ";
//            }
//            cout << setw(11) << fixed << setprecision(2) << avg_success << "%"
//                << (method == methods.back() ? "\n" : " | ");
//        }
//
//        for (const auto& method : methods) {
//            auto& stats = all_methods_stats[method];
//            double sum_time = 0;
//            for (auto& stat : stats) {
//                sum_time += stat.elapsed_time_ms;
//            }
//
//            if (method == methods[0]) {
//                cout << setw(15) << "总耗时(ms)" << " | ";
//            }
//            cout << setw(12) << fixed << setprecision(0) << sum_time
//                << (method == methods.back() ? "\n" : " | ");
//        }
//
//        cout << "\n";
//
//    }
//    catch (const exception& e) {
//        cerr << "Error: " << e.what() << endl;
//        return EXIT_FAILURE;
//    }
//
//    return EXIT_SUCCESS;
//}