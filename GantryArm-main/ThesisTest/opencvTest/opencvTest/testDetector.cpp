//#include <iostream>
//#include <iomanip>  // 用于输出精度
//#include <opencv2/opencv.hpp>
//#include "FeatureDetector.h"      // 您的 FeatureDetector 头文件
//#include "calibration_types.h"    // 包含 CameraParams, CalibrationResult 等
//
//int main() {
//    std::cout << "=== FeatureDetector 调用示例 ===" << std::endl;
//
//    // 步骤 1: 准备相机内参 (CameraParams) - 示例数据 (实际从标定文件加载)
//    CameraParams cam_params;
//    cam_params.camera_matrix = (cv::Mat_<double>(3, 3) << 800.0, 0.0, 640.0,    // fx, 0, cx
//        0.0, 800.0, 480.0,    // 0, fy, cy
//        0.0, 0.0, 1.0);       // 0, 0, 1
//    cam_params.dist_coeffs = (cv::Mat_<double>(5, 1) << -0.3, 0.1, 0.0, 0.0, 0.0);  // k1, k2, p1, p2, k3
//    cam_params.reproj_error = 0.5;  // 重投影误差 (mm)
//
//    // 步骤 2: 准备手眼标定结果 (CalibrationResult) - 示例数据 (实际从标定结果获取)
//    CalibrationResult hand_eye_res;
//    hand_eye_res.success = true;  // 标定成功
//    hand_eye_res.reprojection_error = 0.8;  // 平均误差 (mm)
//    hand_eye_res.hand_eye_transform = Eigen::Matrix4d::Identity();  // 示例单位变换 (实际填充 4x4 矩阵)
//    hand_eye_res.hand_eye_transform(0, 3) = 50.0;   // 示例平移 X=50mm
//    hand_eye_res.hand_eye_transform(1, 3) = 0.0;    // Y=0
//    hand_eye_res.hand_eye_transform(2, 3) = 100.0;  // Z=100mm
//    hand_eye_res.condition_number = 10.0;  // 条件数
//
//    // 步骤 3: 初始化 FeatureDetector
//        FeatureDetector detector(cam_params, hand_eye_res);
//        std::cout << "FeatureDetector 初始化成功" << std::endl;
//
//
//    // 步骤 4: 配置检测参数 (DetectionParams)
//    FeatureDetector::DetectionParams params;
//    params.visualize = true;                       // 启用可视化
//    params.circle_z_depth_mm = 150.0;              // 圆心 Z 深度假设 150mm
//    params.circle_min_radius = 30;                 // 最小半径 30 像素
//    params.circle_max_radius = 80;                 // 最大半径 80 像素
//    params.compute_base_coords = true;             // 计算基坐标
//
//    // 示例机器人当前位姿 (4x4 单位矩阵；实际从机器人 API 获取)
//    params.current_robot_pose = cv::Mat::eye(4, 4, CV_64F);
//    params.current_robot_pose.at<double>(0, 3) = 200.0;  // 示例平移 X=200mm
//
//    // 棋盘格参数 (仅棋盘检测使用)
//    params.chess_params.board_size = cv::Size(8, 11);     // 9x6 内角点
//    params.chess_params.square_size = 6.0f;             // 25mm 方块
//
//    //// 步骤 5: 圆心检测示例
//    //std::cout << "\n--- 圆心检测 ---" << std::endl;
//    //cv::Mat circle_img = cv::imread("C000.jpg");  // 替换为您的圆形图像路径
//    //if (circle_img.empty()) {
//    //    std::cerr << "警告: 无法加载 circle_image.jpg，使用合成图像测试" << std::endl;
//    //    circle_img = cv::Mat::zeros(640, 1280, CV_8UC3);  // 合成测试图像
//    //    cv::circle(circle_img, cv::Point(400, 300), 50, cv::Scalar(0, 0, 255), -1);  // 绘制测试圆
//    //}
//
//    //std::vector<cv::Point3f> cam_circle_coords;  // 相机坐标输出
//    //std::optional<std::vector<cv::Point3f>> base_circle_coords;  // 基坐标输出 (可选)
//
//    //FeatureDetector detector(cam_params, hand_eye_res);  // 重新实例化 (示例中重复以示独立)
//    //bool circle_success = detector.detectAndComputeCircleCenters(circle_img, cam_circle_coords, base_circle_coords, params);
//
//    //if (circle_success) {
//    //    std::cout << "圆心检测成功！检测到 " << cam_circle_coords.size() << " 个点" << std::endl;
//    //    std::cout << "重投影误差: " << std::fixed << std::setprecision(2)
//    //        << detector.getLastResult().reproj_error_mm << " 像素" << std::endl;
//
//    //    // 打印相机坐标
//    //    std::cout << "相机坐标 (x, y, z mm):" << std::endl;
//    //    for (size_t i = 0; i < cam_circle_coords.size(); ++i) {
//    //        const auto& pt = cam_circle_coords[i];
//    //        std::cout << "  点 " << i << ": (" << pt.x << ", " << pt.y << ", " << pt.z << ")" << std::endl;
//    //    }
//
//    //    // 打印基坐标 (如果计算)
//    //    if (base_circle_coords) {
//    //        std::cout << "基坐标 (x, y, z mm):" << std::endl;
//    //        for (size_t i = 0; i < base_circle_coords->size(); ++i) {
//    //            const auto& pt = (*base_circle_coords)[i];
//    //            std::cout << "  点 " << i << ": (" << pt.x << ", " << pt.y << ", " << pt.z << ")" << std::endl;
//    //        }
//    //    }
//    //}
//    //else {
//    //    std::cerr << "圆心检测失败: " << detector.getLastResult().error_message << std::endl;
//    //}
//
//    // 步骤 6: 棋盘格检测示例
//    std::cout << "\n--- 棋盘格检测 ---" << std::endl;
//    cv::Mat chess_img = cv::imread("0.jpg");  // 替换为您的棋盘图像路径
//    if (chess_img.empty()) {
//        std::cerr << "警告: 无法加载 chessboard_image.jpg，使用合成图像测试" << std::endl;
//        chess_img = cv::Mat::zeros(640, 1280, CV_8UC3);  // 合成测试棋盘
//        std::vector<cv::Point3f> obj_pts;  // 生成物体点
//        for (int i = 0; i < 6; ++i) {
//            for (int j = 0; j < 9; ++j) {
//                obj_pts.emplace_back(static_cast<float>(j * 25), static_cast<float>(-i * 25), 0.0f);
//            }
//        }
//        // 简单投影绘制 (实际测试用真实图像)
//        cv::putText(chess_img, "使用真实棋盘图像测试", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
//    }
//
//    std::vector<cv::Point2f> img_chess_points;       // 图像点输出
//    std::vector<cv::Point3f> world_chess_coords;     // 世界坐标输出
//    std::optional<Eigen::Matrix4d> cam_chess_pose;   // 相机位姿输出
//    std::optional<Eigen::Matrix4d> base_chess_pose;  // 基位姿输出
//
//    bool chess_success = detector.detectAndComputeChessboardPoints(chess_img, img_chess_points, world_chess_coords,
//        cam_chess_pose, base_chess_pose, params);
//
//    if (chess_success) {
//        std::cout << "棋盘格检测成功！检测到 " << img_chess_points.size() << " 个角点" << std::endl;
//        std::cout << "重投影误差: " << std::fixed << std::setprecision(2)
//            << detector.getLastResult().reproj_error_mm << " 像素" << std::endl;
//
//        // 打印世界坐标 (前几个示例)
//        std::cout << "世界坐标 (x, y, z mm，前 5 个):" << std::endl;
//        for (size_t i = 0; i < std::min<size_t>(5, world_chess_coords.size()); ++i) {
//            const auto& pt = world_chess_coords[i];
//            std::cout << "  点 " << i << ": (" << pt.x << ", " << pt.y << ", " << pt.z << ")" << std::endl;
//        }
//
//        // 打印相机位姿平移
//        if (cam_chess_pose) {
//            Eigen::Vector3d trans = cam_chess_pose->col(3).head<3>();
//            std::cout << "棋盘格相机位姿平移: (" << std::fixed << std::setprecision(1)
//                << trans.x() << ", " << trans.y() << ", " << trans.z() << ") mm" << std::endl;
//        }
//
//        // 打印基位姿 (如果计算)
//        if (base_chess_pose) {
//            Eigen::Vector3d trans = base_chess_pose->col(3).head<3>();
//            std::cout << "棋盘格基位姿平移: (" << std::fixed << std::setprecision(1)
//                << trans.x() << ", " << trans.y() << ", " << trans.z() << ") mm" << std::endl;
//        }
//    }
//    else {
//        std::cerr << "棋盘格检测失败: " << detector.getLastResult().error_message << std::endl;
//    }
//
//    // 步骤 7: 等待可视化窗口关闭
//    std::cout << "\n按任意键关闭窗口..." << std::endl;
//    cv::waitKey(0);
//    cv::destroyAllWindows();
//
//    std::cout << "示例结束" << std::endl;
//    return 0;
//}