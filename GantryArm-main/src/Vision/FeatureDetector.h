#pragma once
#ifndef FEATURE_DETECTOR_H
#define FEATURE_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>
#include <optional>
#include <Eigen/Dense>
#include "calibration_types.h"  // 包含 CameraParams, CalibrationResult 等
#include "robot_types.h"
// 复用 Point6D (若需 6D 输出，可扩展；此处用 Point3D 简化)
using Point3D = cv::Point3f;  // 3D 点 (float 类型，单位 mm)



/**
 * FeatureDetector 类：独立特征检测和坐标计算 (不依赖 CameraCalibration)
 * - 圆心：Hough 检测 + 验证 + 3D 转换
 * - 棋盘格：角点检测 + PnP 位姿 + 世界坐标生成
 * 输入：内参 (CameraParams) 和手眼结果 (CalibrationResult)
 */
class FeatureDetector {
public:
    // 检测参数配置结构体 (扩展以支持圆半径范围等)
    struct DetectionParams {
        double circle_z_depth_mm = 100.0;      // 圆心 Z 深度假设 (mm)
        int circle_min_radius = 20;            // 圆最小半径 (像素)
        int circle_max_radius = 100;           // 圆最大半径 (像素)
        bool compute_base_coords = true;       // 计算基坐标 (需手眼 success=true)
        bool visualize = false;                // 可视化结果
        cv::Mat current_robot_pose;            // 当前机器人位姿 (4x4 cv::Mat，默认单位矩阵)
        ChessboardParams chess_params;         // 棋盘格参数 (仅棋盘检测使用)
        DetectionParams() : current_robot_pose(cv::Mat::eye(4, 4, CV_64F)) {}
    };

    // 检测结果 (兼容 CalibrationResult)
    struct DetectionResult {
        bool success = false;                  // 检测成功
        double reproj_error_mm = 0.0;          // 重投影误差 (像素单位，近似 mm)
        size_t num_points = 0;                 // 点数量
        std::string error_message;             // 错误信息
        std::vector<double> point_errors;      // 每个点误差 (可选)
    };

    // 构造函数：传入内参和手眼结果
    FeatureDetector(const CameraParams& cam_params, const CalibrationResult& hand_eye_res);
    ~FeatureDetector() = default;

    /**
     * 圆心检测和坐标计算
     * @param image 输入图像
     * @param camera_coords 输出：相机 3D 坐标
     * @param base_coords_opt 输出：基坐标 3D (可选)
     * @param params 参数
     * @return true 如果成功
     */
    bool detectAndComputeCircleCenters(const cv::Mat& image,
        std::vector<Point3D>& camera_coords,
        std::optional<std::vector<Point3D>>& base_coords_opt ,
        const DetectionParams& params = DetectionParams());

    /**
     * 棋盘格检测和坐标计算
     * @param image 输入图像
     * @param image_points 输出：2D 图像点
     * @param world_coords 输出：3D 世界点
     * @param board_pose_camera_opt 输出：相机位姿 (Eigen 4x4)
     * @param board_pose_base_opt 输出：基位姿 (可选)
     * @param params 参数
     * @return true 如果成功
     */
    bool detectAndComputeChessboardPoints(const cv::Mat& image,
        std::vector<cv::Point2f>& image_points,
        std::vector<Point3D>& world_coords,
        std::optional<Eigen::Matrix4d>& board_pose_camera_opt ,
        std::optional<Eigen::Matrix4d>& board_pose_base_opt ,
        const DetectionParams& params = DetectionParams());

    cv::Mat visualizeChessboardDetection(const cv::Mat& image, const std::vector<cv::Point2f>& image_points, const cv::Size& board_size, bool success, int max_width, int max_height) const;

    /**
     * 单个点相机到基坐标转换
     * @param cam_point 输入相机点
     * @param robot_pose 机器人位姿 (4x4 cv::Mat)
     * @return 基坐标点
     */
    Point3D cameraToBase(const Point3D& cam_point, const cv::Mat& robot_pose) const;

    // 获取上次结果
    DetectionResult getLastResult() const { return last_result_; }

    /**
     * 可视化：绘制点到图像
     * @param image 原图像
     * @param points 2D 点
     * @return 绘制图像
     */
    cv::Mat visualizeDetection(const cv::Mat& image, const std::vector<cv::Point2f>& points) const;

private:
    CameraParams cam_params_;              // 相机内参
    CalibrationResult hand_eye_res_;       // 手眼结果
    DetectionResult last_result_;          // 上次结果

    // 私有辅助
    cv::Mat preprocessImage(const cv::Mat& image) const;  // 图像预处理
    bool detectCircles(const cv::Mat& image, std::vector<cv::Point2f>& centers, const DetectionParams& params) ;  // 圆检测
    cv::Mat drawDetectedCircles(const cv::Mat& image, const std::vector<cv::Vec3f>& circles);
    void showImage(const std::string& winname, const cv::Mat& img, int delay);
    std::vector<cv::Vec3f> postProcessCircles(const cv::Mat gray, const std::vector<cv::Vec3f>& circles, const cv::Size& imageSize);
    bool validateCircleInterior(const cv::Mat& image, const cv::Point2f& center, float radius);
    float calculateCircleQuality(const cv::Mat& image, const cv::Point2f& center, float radius);
    float calculateGradientMagnitude(const cv::Mat& image, const cv::Point2f& point);
    bool detectChessboardCorners(const cv::Mat& image, std::vector<cv::Point2f>& corners, const ChessboardParams& ch_params);  // 棋盘检测
    bool validateCornerArrangement(const std::vector<cv::Point2f>& corners, const cv::Size& patternSize);
    std::vector<Point3D> pixelToCamera3D(const std::vector<cv::Point2f>& pixels, double z_depth) const;  // 像素转 3D
    Eigen::Matrix4d computePnPpose(const std::vector<cv::Point3f>& obj_pts, const std::vector<cv::Point2f>& img_pts) const;  // PnP 位姿
    bool isHandEyeValid() const { return hand_eye_res_.success; }  // 手眼有效性
};

#endif // FEATURE_DETECTOR_H