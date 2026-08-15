#include "hand_eye_calibration.h"
#include <Eigen/Dense>
#include <vector>
#include <iostream>

// 标定参数结构体
struct CalibrationParams {
    int boardX = 8;          // 棋盘格X方向角点数
    int boardY = 11;          // 棋盘格Y方向角点数
    float squareSize = 6.0f; // 棋盘格方块尺寸，单位mm
    int imageWidth = 320;   // 图像宽度
    int imageHeight = 240;   // 图像高度"E:\GIT\GantryArm\lib\data\calibration\Djn"
    std::string calibImgPath = "../../../lib/data/calibration/Djn"; // 标定图像路径
    std::string robotPosesPath = "../../../lib/data/calibration/Djn/robot_poses.csv"; // 机器人位姿CSV文件路径
    std::string calibResultPath = "../../../lib/data/calibration/Djn/calibResult"; // 标定结果保存路径
    std::string resultFileName = "../../../lib/data/calibration/Djn/hand_eye_calibration_result.txt"; // 结果文件名
    double depth = 100.0;    // 标定板到相机的距离，单位mm
};

int main() {
    try {
        // 初始化标定参数
        CalibrationParams params;

        // 步骤1: 读取标定图像
        std::vector<cv::String> filenames;
        cv::glob(params.calibImgPath, filenames);

        if (filenames.empty()) {
            std::cerr << "未找到任何标定图像，请检查路径: " << params.calibImgPath << std::endl;
            return -1;
        }

        std::cout << "找到 " << filenames.size() << " 张标定图像" << std::endl;

        // 步骤2: 检测所有图像中的棋盘格角点
        std::cout << "开始检测棋盘格角点..." << std::endl;
        std::vector<std::vector<cv::Point2f>> imagePoints =
            findAllChessboard(filenames, params.boardX, params.boardY);

        if (imagePoints.size() < 3) {
            std::cerr << "检测到的有效棋盘格图像不足，至少需要3张" << std::endl;
            return -1;
        }

        std::cout << "成功检测到 " << imagePoints.size() << " 张有效棋盘格图像" << std::endl;

        // 步骤3: 生成标定板三维坐标点
        std::vector<cv::Point3f> singleObjectPoints =
            generateObjectPoints(params.boardX, params.boardY, params.squareSize);

        std::vector<std::vector<cv::Point3f>> objectPoints;
        for (size_t i = 0; i < imagePoints.size(); i++) {
            objectPoints.push_back(singleObjectPoints);
        }

        // 步骤4: 相机标定
        std::cout << "开始相机标定..." << std::endl;
        cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
        std::vector<cv::Mat> rvecs, tvecs;

        double rms = cv::calibrateCamera(objectPoints, imagePoints,
            cv::Size(params.imageWidth, params.imageHeight),
            cameraMatrix, distCoeffs, rvecs, tvecs);

        std::cout << "相机标定完成，重投影误差 RMS: " << rms << std::endl;

        // 步骤5: 读取机器人末端位姿（使用Eigen矩阵）
        std::cout << "读取机器人末端位姿..." << std::endl;
        std::vector<Eigen::Matrix4d> eigenPoses =
            CameraCalibration::readPosesFromCSV(params.robotPosesPath);

        // 检查位姿数量是否与有效图像匹配
        if (eigenPoses.size() != imagePoints.size()) {
            std::cerr << "机器人位姿数量与有效图像数量不匹配: "
                << eigenPoses.size() << " vs " << imagePoints.size() << std::endl;
            return -1;
        }

        // 步骤6: 转换Eigen矩阵为OpenCV格式，并提取旋转和平移分量
        std::vector<cv::Mat> R_gripper2base, t_gripper2base, R_target2cam, t_target2cam;
        std::vector<cv::Mat> robotPoses;

        for (size_t i = 0; i < imagePoints.size(); i++) {
            // 将Eigen矩阵转换为OpenCV的cv::Mat（4x4齐次矩阵）
            cv::Mat pose = CameraCalibration::eigenToCvMat(eigenPoses[i]);
            robotPoses.push_back(pose);

            // 从4x4齐次矩阵中提取旋转矩阵（3x3）和平移向量（3x1）
            cv::Mat R = pose(cv::Rect(0, 0, 3, 3)).clone(); // 左上角3x3为旋转矩阵
            cv::Mat t = pose(cv::Rect(3, 0, 1, 3)).clone(); // 第4列前3行为平移向量
            t.convertTo(t, CV_64F); // 确保数据类型为double

            R_gripper2base.push_back(R);
            t_gripper2base.push_back(t);

            // 处理相机外参（旋转向量转旋转矩阵）
            cv::Mat R_cam;
            cv::Rodrigues(rvecs[i], R_cam);
            R_target2cam.push_back(R_cam);
            t_target2cam.push_back(tvecs[i]);
        }

        // 步骤7: 执行手眼标定
        std::cout << "开始手眼标定..." << std::endl;
        cv::Mat R_cam2gripper, t_cam2gripper;

        cv::calibrateHandEye(R_gripper2base, t_gripper2base,
            R_target2cam, t_target2cam,
            R_cam2gripper, t_cam2gripper,
            cv::CALIB_HAND_EYE_TSAI);

        std::cout << "手眼标定完成" << std::endl;
        std::cout << "相机到夹具的旋转矩阵:\n" << R_cam2gripper << std::endl;
        std::cout << "相机到夹具的平移向量:\n" << t_cam2gripper << std::endl;

        // 步骤8: 计算水平误差
        std::cout << "\n开始计算水平误差..." << std::endl;
        double meanXError, meanYError, maxXError, maxYError;

        calculateHorizontalErrors(imagePoints.back(), singleObjectPoints,
            params.depth, cameraMatrix, distCoeffs,
            R_cam2gripper, t_cam2gripper,
            R_gripper2base.back(), t_gripper2base.back(),
            meanXError, meanYError, maxXError, maxYError);

        // 输出误差统计结果
        std::cout << "\n水平误差统计结果 (单位: mm):" << std::endl;
        std::cout << "平均X方向误差: " << meanXError << std::endl;
        std::cout << "平均Y方向误差: " << meanYError << std::endl;
        std::cout << "最大X方向误差: " << maxXError << std::endl;
        std::cout << "最大Y方向误差: " << maxYError << std::endl;

        // 步骤9: 保存标定结果
        std::string savePath = params.calibResultPath + "/" + params.resultFileName;
        saveCalibrationResults(savePath, cameraMatrix, distCoeffs,
            R_cam2gripper, t_cam2gripper);

        std::cout << "\n手眼标定流程完成" << std::endl;
        return 0;
    }
    catch (const cv::Exception& e) {
        std::cerr << "OpenCV 错误: " << e.what() << std::endl;
        return -1;
    }
    catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return -1;
    }
}
