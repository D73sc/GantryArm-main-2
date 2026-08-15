#pragma once
#ifndef HAND_EYE_CALIBRATION_H
#define HAND_EYE_CALIBRATION_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <algorithm>
#include <numeric>
#include <cmath>

/**
 * @brief 标定配置参数结构体
 * 包含所有进行手眼标定所需的参数，如棋盘格规格、图像路径、文件路径等。
 */
struct CalibrationConfig {
    int boardX = 8;           // 棋盘格X方向内角点数量 (cols - 1)
    int boardY = 11;          // 棋盘格Y方向内角点数量 (rows - 1)
    float squareSize = 5.0f;  // 棋盘格方块的实际物理尺寸，单位通常为mm
    int imageWidth = 2448;    // 相机图像宽度
    int imageHeight = 2048;   // 相机图像高度
    std::string calibImgPath; // 存放标定图像的文件夹路径
    std::string robotPosesPath;// 机器人位姿数据的CSV文件路径
    std::string calibResultPath; // 标定结果保存的文件夹路径
    std::string resultFileName;  // 标定结果保存的文件名
};

/**
 * @brief 手眼标定类
 */
class HandEyeCalibrator {
public:
    /**
     * @brief 构造函数
     * @param cfg 标定配置参数
     */
    HandEyeCalibrator(const CalibrationConfig& cfg);
    ~HandEyeCalibrator() = default;

    // 获取当前配置对象的副本
    CalibrationConfig getConfig() const { return config; }

    // ==========================================
    //  1. 核心视觉算法函数 (普通成员函数)
    // ==========================================

    /**
     * @brief 生成标定板的理论三维坐标点 (Z=0平面)
     */
    std::vector<cv::Point3f> generateObjectPoints(int boardX, int boardY, float squareSize);

    /**
     * @brief 批量读取图像并查找棋盘格角点
     * @return 返回每张图中检测到的角点集合
     */
    std::vector<std::vector<cv::Point2f>> findAllChessboard(const std::vector<cv::String>& filenames, int boardX, int boardY);

    /**
     * @brief 像素坐标转换为相机坐标 (基于solvePnP计算出的外参)
     * 计算像素点对应的射线与标定板平面的交点。
     */
    bool pixelToCameraCoord(const cv::Point2f& pixel, const cv::Mat& cameraMatrix,
        const cv::Mat& distCoeffs, const cv::Mat& rvec, const cv::Mat& tvec,
        const std::vector<cv::Point3f>& objectPoints, cv::Point3d& camCoord);

    /**
     * @brief 像素坐标转换为机器人基坐标
     * 流程: Pixel -> Camera Frame -> Gripper Frame -> Base Frame
     */
    bool pixelToRobotBaseCoord(const cv::Point2f& pixel, const cv::Mat& cameraMatrix,
        const cv::Mat& distCoeffs, const cv::Mat& rvec, const cv::Mat& tvec,
        const std::vector<cv::Point3f>& objectPoints,
        const cv::Mat& R_cam2gripper, const cv::Mat& t_cam2gripper,
        const cv::Mat& R_gripper2base, const cv::Mat& t_gripper2base,
        cv::Point3d& baseCoord);

    // ==========================================
    //  2. 静态工具函数 (通用工具，不依赖对象状态)
    // ==========================================

    // 从RT矩阵(3x4 or 4x4)中分离旋转矩阵R和平移向量T
    static void RT2R_T(const cv::Mat& rt, cv::Mat& R, cv::Mat& T);

    // 检查矩阵是否为有效的旋转矩阵
    static bool isRotationMatrix(const cv::Mat& R);

    // 从文本文件读取矩阵数据
    static cv::Mat_<double> readMatrixFromFile(const std::string& filePath, int rows, int cols);

    // 读取CSV文件中的位姿数据，返回Eigen格式
    static std::vector<Eigen::Matrix4d> readPosesFromCSV(const std::string& filePath);
    static std::vector<Eigen::Matrix4d> readPoint6DFromCSV(const std::string& filePath, bool hasImageIndexCol = true);

    // Eigen矩阵转OpenCV矩阵格式
    static cv::Mat eigenToCvMat(const Eigen::Matrix4d& eigenMat);

    // 将标定结果保存到文件
    static void saveCalibrationResults(const std::string& filePath, const cv::Mat& cameraMatrix,
        const cv::Mat& distCoeffs, const cv::Mat& R_cam2gripper,
        const cv::Mat& t_cam2gripper);

    // ==========================================
    //  3. 流程封装函数 (简化Main函数逻辑)
    // ==========================================

    /**
     * @brief 批量计算每张图片的外参 (SolvePnP Loop)
     * 遍历所有图像的角点，使用 solvePnP 计算相机相对于标定板的位姿 (rvec, tvec)。
     */
    void batchSolvePnP(const std::vector<std::vector<cv::Point3f>>& objectPoints,
        const std::vector<std::vector<cv::Point2f>>& imagePoints,
        const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
        std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs);

    /**
     * @brief 准备手眼标定所需的数据格式 (Data Conversion Loop)
     * 将读取到的机器人位姿(Eigen)和计算出的相机外参(rvec/tvec)转换为 calibrateHandEye 函数所需的格式。
     */
    void prepareHandEyeData(const std::vector<Eigen::Matrix4d>& eigenPoses,
        const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
        std::vector<cv::Mat>& R_gripper2base, std::vector<cv::Mat>& t_gripper2base,
        std::vector<cv::Mat>& R_target2cam, std::vector<cv::Mat>& t_target2cam);

    /**
     * @brief 计算并打印误差统计 (Error Calculation Loop)
     * 验证标定结果，通过将特定像素点转换到基坐标系，计算点集的聚合程度(RMSE)。
     */
    void computeCalibrationError(const std::vector<std::vector<cv::Point2f>>& imagePoints,
        const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
        const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
        const std::vector<cv::Point3f>& singleObjectPoints,
        const cv::Mat& R_cam2gripper, const cv::Mat& t_cam2gripper,
        const std::vector<cv::Mat>& R_gripper2base,
        const std::vector<cv::Mat>& t_gripper2base);

private:
    CalibrationConfig config; // 内部保存的配置副本
};

#endif // HAND_EYE_CALIBRATION_H



