//#include "AlgorithmComparator.h"
//#include "TrajectoryOptimizer.h"
//bool readJointAnglesAndTimesFromCSV(
//    const std::string& filename,
//    std::vector<std::array<double, 8>>& out_joint_angles,
//    std::vector<double>& out_times_ms,
//    bool skip_header = true)
//{
//    std::ifstream file(filename);
//    if (!file.is_open()) {
//        std::cerr << "无法打开文件: " << filename << std::endl;
//        return false;
//    }
//
//    out_joint_angles.clear();
//    out_times_ms.clear();
//
//    std::string line;
//    // Skip header line
//    std::getline(file, line);
//    while (std::getline(file, line)) {
//        std::stringstream ss(line);
//        std::string item;
//
//        // 读取“index”列（第一列）
//        if (!std::getline(ss, item, ',')) break;
//        int index_val = std::stoi(item);
//
//        if (skip_header&& index_val <= 2) {
//            // 跳过这行，继续下一行
//            continue;
//        }
//
//        std::array<double, 8> joints{};
//        double time_ms = 0.0;
//
//        // 读8个关节角
//        bool read_ok = true;
//        for (int i = 0; i < 8; ++i) {
//            if (!std::getline(ss, item, ',')) {
//                read_ok = false;
//                break;
//            }
//            joints[i] = std::stod(item);
//        }
//        if (!read_ok) break;
//
//        // 读耗时列
//        if (!std::getline(ss, item, ',')) break;
//        time_ms = std::stod(item);
//
//        out_joint_angles.push_back(joints);
//        out_times_ms.push_back(time_ms);
//    }
//
//    file.close();
//    return true;
//}
//bool readPlanningTimeAndJointAngles(
//    const std::string& filename,
//    std::vector<double>& out_planning_times_ms,
//    std::vector<std::array<double, 8>>& out_joint_angles)
//{
//    std::ifstream file(filename);
//    if (!file.is_open()) {
//        std::cerr << "无法打开文件: " << filename << std::endl;
//        return false;
//    }
//
//    out_planning_times_ms.clear();
//    out_joint_angles.clear();
//
//    std::string line;
//    // 读取并跳过表头行
//    if (!std::getline(file, line)) {
//        std::cerr << "文件为空或读取失败" << std::endl;
//        return false;
//    }
//
//    while (std::getline(file, line)) {
//        if (line.empty() || line[0] == '#') continue; // 跳过空行和注释
//
//        std::stringstream ss(line);
//        std::string token;
//        std::vector<std::string> tokens;
//
//        while (std::getline(ss, token, ',')) {
//            tokens.push_back(token);
//        }
//
//        if (tokens.size() < 14) { // 列数不足
//            std::cerr << "数据列不足: " << line << std::endl;
//            continue;
//        }
//
//        try {
//            double planning_time = std::stod(tokens[3]); // Planning_Time(ms)
//            std::array<double, 8> joints{};
//            for (int i = 0; i < 8; ++i) {
//                if(i<3)
//                joints[i] = std::stod(tokens[6 + i])*1000; // J0~J7
//                else
//                    joints[i] = std::stod(tokens[6 + i]) ; // J0~J7
//
//            }
//            // 根据需要单位转换，例如米->毫米，通常关节角度不需要转换
//
//            out_planning_times_ms.push_back(planning_time);
//            out_joint_angles.push_back(joints);
//        }
//        catch (const std::exception& e) {
//            std::cerr << "转换错误: " << e.what() << " 行内容: " << line << std::endl;
//            continue;
//        }
//    }
//    return true;
//}
//
//int main()
//{
//	RobotArm arm(218.4);
//    CollisionSystem collision_system(arm);
//    OptimizeParams params;
//	// 准备关节角轨迹数据
//	std::vector<std::array<double, 8>> joint_trajectory;
//	AlgorithmComparator comparator(arm, collision_system, params);
//    std::vector<double> elapsed_times;
//	std::vector<AlgorithmComparator::PerformanceMetrics> all_metrics;
//    int times = 100;
//    std::string alg = "KDL";
//    std::string base_path = "essay_RALLE/"+ alg+"/";  // 根据你的目录修改
//    for (int i = 1; i <= times; ++i) {
//        std::string filename = base_path + "run_" + std::to_string(i) + "_optimized.csv";
//
//        std::cout << "读取文件: " << filename << std::endl;
//        //bool ok = readJointAnglesAndTimesFromCSV(filename, joint_trajectory, elapsed_times, true);
//        bool ok = readPlanningTimeAndJointAngles(filename, elapsed_times, joint_trajectory);
//        if (!ok) {
//            std::cerr << "读取失败: " << filename << ", 跳过" << std::endl;
//            continue;
//        }
//
//        auto metrics = comparator.evaluateJointTrajectoryWithTimes("run_" + std::to_string(i), joint_trajectory, elapsed_times);
//        all_metrics.push_back(metrics);
//    }
//
//    std::string export_file = base_path + "batch_evaluation_summary.csv";
//    comparator.exportMetricsToCSV(all_metrics, export_file);
//
//    std::cout << "批量评估完成，汇总已导出到: " << export_file << std::endl;
//
//    return 0;
// //   std::string firstname = "NewTunnel";
// //   std::vector< std::string> names = { "NSGA2","MOEAD","MOEAD2"  };
// //   std::vector< std::string> ROS_names = { "BIO","TRAC","TRAC2" };
// //   //std::string firstname = "C2";
// //   //std::vector< std::string> names = { "GA2","MOEAD","MOEAD2" ,"MOEAD3" };
// //   //std::vector< std::string> ROS_names = { "BIO2","TRAC" };
//
//	//for (auto& name : names)
//	//{
//	//	// 方式1：从文件读取
//	//	joint_trajectory = arm.readJointTrajectoryFromCSV("thesis/"+ firstname+"/" + name + ".csv");
//	//	// 评估
//	//	allmetrics.push_back(comparator.evaluateJointTrajectory("thesis" + name, joint_trajectory));
//	//}
// //   for (auto& name : ROS_names)
// //   {
// //       joint_trajectory=read_joint_angles_from_csv("thesis/" + firstname + "/" + name + ".csv");
// //       allmetrics.push_back(comparator.evaluateJointTrajectory("thesis" + name, joint_trajectory));
//
// //   }
//
//	//comparator.exportMetricsToCSV(allmetrics, "thesis/" + firstname + "/Joint.csv");
//}