#pragma once
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "robot_types.h"
/**
 * @brief 棋盘格参数结构体 (独立定义，不依赖 CameraCalibration)
 */
struct ChessboardParams {
    cv::Size board_size = cv::Size(8, 11);  // 内角点数量 (宽 x 高)
    float square_size =6.0f;             // 方块大小 (mm)
};

// 相机内参结构体
struct CameraParams {
    cv::Mat camera_matrix;  // 3x3内参矩阵
    cv::Mat dist_coeffs;    // 5x1畸变系数
    double reproj_error;    // 重投影误差
};

/**
 * @brief 标定结果结构
 */
struct CalibrationResult {
    Eigen::Matrix4d hand_eye_transform;     // 相机到法兰盘的变换矩阵
    double reprojection_error;              // 平均重投影误差 (mm)
    std::vector<double> point_errors;       // 每个点的误差 (mm)
    bool success;                           // 标定是否成功
    std::string error_message;              // 错误信息
    double condition_number;                // 系统条件数

    CalibrationResult() : reprojection_error(0), success(false), condition_number(0) {}
};

// TCP标定结果
struct TCPResult {
    Point6D theoretical_tcp;      // 理论TCP偏移（输入）
    Point6D actual_tcp;          // 实际TCP偏移（标定结果）
    Point6D tcp_correction;      // TCP修正量
    Point6D reference_point;     // 参考点位置
    double accuracy;             // 标定精度（RMS误差）
    size_t point_count;          // 标定点数量

    TCPResult();
};
//静态方法实现
static cv::Mat eigenToCvMat(const Eigen::Matrix4d& eigen_mat) {
    cv::Mat cv_mat(4, 4, CV_64F);
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            cv_mat.at<double>(i, j) = eigen_mat(i, j);
        }
    }
    return cv_mat;
}

static Eigen::Matrix4d cvToEigenMat(const cv::Mat& cv_mat) {
    Eigen::Matrix4d eigen_mat;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            eigen_mat(i, j) = cv_mat.at<double>(i, j);
        }
    }
    return eigen_mat;
}
