#ifndef HAND_EYE_CALIBRATION_H
#define HAND_EYE_CALIBRATION_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Core>

// 生成标定板的三维坐标点
std::vector<cv::Point3f> generateObjectPoints(int boardX, int boardY, float squareSize);

// 查找所有图像中的棋盘格角点
std::vector<std::vector<cv::Point2f>> findAllChessboard(const std::vector<cv::String>& filenames, int boardX, int boardY);

// 从文件读取矩阵
cv::Mat_<double> readMatrixFromFile(const std::string& filePath, int rows, int cols);

// 从RT矩阵中提取旋转和平移分量
void RT2R_T(const cv::Mat& rt, cv::Mat& R, cv::Mat& T);

// 检查是否为旋转矩阵
bool isRotationMatrix(const cv::Mat& R);

// 保存标定结果
void saveCalibrationResults(const std::string& filePath, const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs, const cv::Mat& R_cam2gripper,
    const cv::Mat& t_cam2gripper);

// 像素坐标转换为相机坐标
bool pixelToCameraCoord(const cv::Point2f& pixel, const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs, double depth, cv::Point3d& camCoord);

// 像素坐标转换为机器人基坐标
bool pixelToRobotBaseCoord(const cv::Point2f& pixel, double depth,
    const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
    const cv::Mat& R_cam2gripper, const cv::Mat& t_cam2gripper,
    const cv::Mat& R_gripper2base, const cv::Mat& t_gripper2base,
    cv::Point3d& baseCoord);

// 计算水平误差（X和Y方向）
void calculateHorizontalErrors(const std::vector<cv::Point2f>& imagePoints,
    const std::vector<cv::Point3f>& objectPoints,
    double depth, const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs, const cv::Mat& R_cam2gripper,
    const cv::Mat& t_cam2gripper, const cv::Mat& R_gripper2base,
    const cv::Mat& t_gripper2base, double& meanXError,
    double& meanYError, double& maxXError, double& maxYError);

// 命名空间：相机标定工具函数
namespace CameraCalibration {
    // 从CSV文件读取机器人位姿（返回Eigen的4x4齐次矩阵）
    std::vector< Eigen::Matrix4d>readPosesFromCSV(const std::string& filePath);

    // 将Eigen矩阵转换为OpenCV的cv::Mat
    cv::Mat eigenToCvMat(const Eigen::Matrix4d& eigenMat);
}

#endif // HAND_EYE_CALIBRATION_H
