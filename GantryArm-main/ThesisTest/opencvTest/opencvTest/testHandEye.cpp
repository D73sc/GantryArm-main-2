#include "HandEyeCalibration.h"
#include <Eigen/Dense>
#include <vector>
#include <iostream>

int main() {
    try {
        // 1. 初始化配置参数
        CalibrationConfig params;
        params.boardX = 11;
        params.boardY = 8;
        params.squareSize = 5.0f;
        params.calibImgPath = "data/calibration/2026123/";
        params.robotPosesPath = "data/calibration/robot_poses5.csv";
        params.calibResultPath = "./calibResult";
        params.resultFileName = "hand_eye_calibration_result.txt";

        // 实例化标定器对象
        HandEyeCalibrator calibrator(params);

        // 2. 准备数据: 读取图像与生成理论点

        // 生成图像文件列表
        std::vector<cv::String> filenames;
        for (int i = 1; i < 22; i++) {
            filenames.push_back(params.calibImgPath + "0" + std::to_string(i) + ".jpg");
        }
        if (filenames.empty()) {
            std::cerr << "未找到任何标定图像" << std::endl;
            return -1;
        }

        // 核心算法: 检测所有图像中的棋盘格角点
        std::cout << "开始检测棋盘格角点..." << std::endl;
        auto imagePoints = calibrator.findAllChessboard(filenames, params.boardX, params.boardY);
        if (imagePoints.size() < 3) {
            std::cerr << "有效图像不足3张，无法标定" << std::endl;
            return -1;
        }

        // 核心算法: 生成标定板的理论三维点
        auto singleObjectPoints = calibrator.generateObjectPoints(params.boardX, params.boardY, params.squareSize);
        std::vector<std::vector<cv::Point3f>> objectPoints(imagePoints.size(), singleObjectPoints);

        // 3. 计算相机外参 (SolvePnP)

        // 设置相机内参和畸变系数 (此处为Matlab标定结果)
        cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 2120.6, 0.0, 1198.5, 0.0, 2120.6, 1042.6, 0.0, 0.0, 1.0);
        cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << -0.1633, 0.0908, 0.0, 0.0, 0.0);

        // 流程封装: 批量计算每张图片的外参 (rvecs, tvecs)
        std::cout << "开始计算每张图片的外参 (solvePnP)..." << std::endl;
        std::vector<cv::Mat> rvecs, tvecs;
        calibrator.batchSolvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvecs, tvecs);

        // 4. 准备手眼标定数据

        // 静态工具: 读取机器人位姿
        std::cout << "读取并处理机器人位姿..." << std::endl;
        auto eigenPoses = HandEyeCalibrator::readPoint6DFromCSV(params.robotPosesPath, true);
        if (eigenPoses.size() != imagePoints.size()) {
            std::cerr << "数据数量不匹配: 图片数 " << imagePoints.size() << " vs 位姿数 " << eigenPoses.size() << std::endl;
            return -1;
        }

        // 流程封装: 将数据转换为 calibrateHandEye 所需的 R/T 矩阵列表
        std::vector<cv::Mat> R_gripper2base, t_gripper2base, R_target2cam, t_target2cam;
        calibrator.prepareHandEyeData(eigenPoses, rvecs, tvecs, R_gripper2base, t_gripper2base, R_target2cam, t_target2cam);

        // 5. 执行手眼标定

        std::cout << "开始手眼标定..." << std::endl;
        cv::Mat R_cam2gripper, t_cam2gripper;

        // OpenCV 手眼标定函数 (Tsai方法)
        cv::calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam,
            R_cam2gripper, t_cam2gripper, cv::CALIB_HAND_EYE_TSAI);

        std::cout << "标定完成。" << std::endl;
        std::cout << "R_cam2gripper:\n" << R_cam2gripper << std::endl;
        std::cout << "t_cam2gripper:\n" << t_cam2gripper << std::endl;

        // 6. 验证与保存

        // 流程封装: 计算并打印误差统计信息
        calibrator.computeCalibrationError(imagePoints, rvecs, tvecs, cameraMatrix, distCoeffs,
            singleObjectPoints, R_cam2gripper, t_cam2gripper,
            R_gripper2base, t_gripper2base);

        // 静态工具: 保存结果到文件
        std::string savePath = params.calibResultPath + "/" + params.resultFileName;
        HandEyeCalibrator::saveCalibrationResults(savePath, cameraMatrix, distCoeffs, R_cam2gripper, t_cam2gripper);

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return -1;
    }
}
