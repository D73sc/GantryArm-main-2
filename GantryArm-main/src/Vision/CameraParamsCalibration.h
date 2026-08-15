#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "calibration_types.h"




class CameraParamsCalibrator {
private:
    ChessboardParams chessboardParams_;

    CameraParams params;                // 相机参数
    double fixed_depth=1.0;
    // 齐次矩阵乘法
    cv::Mat multiplyHomogeneous(const cv::Mat& a, const cv::Mat& b);

    // 求齐次矩阵的逆
    cv::Mat inverseHomogeneous(const cv::Mat& mat);

public:
    // 构造函数
    CameraParamsCalibrator(const ChessboardParams chessboardParams);

    // 将旋转向量和平移向量转换为4x4齐次矩阵
    cv::Mat rtvecToHomogeneous(const cv::Mat& rvec, const cv::Mat& tvec);

    // 相机内参标定
    bool calibrateCamera(const std::vector<std::string>& image_paths);

    // 对图像进行畸变校正
    void undistortImage(const cv::Mat& input_image, cv::Mat& output_image);

    void setFixedDepth(double depth);
    cv::Point3f pixelToCamera(const cv::Point2f& pixel);

    // 从单张图像中求解相机位姿
    bool getCameraPoseFromImage(const cv::Mat& image, cv::Mat& camera_pose);

    // 保存标定参数到文件
    bool saveParams(const std::string& filename);

    // 从文件加载标定参数
    bool loadParams(const std::string& filename);

    // 获取相机参数
    const CameraParams& getCameraParams() const;

};



