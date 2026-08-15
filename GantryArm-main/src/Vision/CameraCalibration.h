#ifndef CAMERA_CALIBRATION_H
#define CAMERA_CALIBRATION_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <memory>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <fstream>
#include <sstream>
class CameraCalibration {
public:
    // 棋盘格参数结构体
    struct ChessboardParams {
        cv::Size boardSize;      // 棋盘格内角点数量（宽x高）
        float squareSize;        // 棋盘格方块大小(mm)
        bool fastCheck;          // 是否使用快速检查模式
        int cornerFlags;         // 角点检测标志
        ChessboardParams() :
            boardSize(9,6),
            squareSize(25.0f),
            fastCheck(true),
            cornerFlags(cv::CALIB_CB_ADAPTIVE_THRESH +
                        cv::CALIB_CB_NORMALIZE_IMAGE +
                        cv::CALIB_CB_FAST_CHECK) {}
    };

    // 相机参数结构体
    struct CameraParams {
        cv::Mat cameraMatrix;    // 内参矩阵
        cv::Mat distCoeffs;      // 畸变系数
        std::vector<cv::Mat> rVecs; // 旋转向量
        std::vector<cv::Mat> tVecs; // 平移向量
        double reprojectionError; // 重投影误差
        CameraParams() : reprojectionError(0.0) {}
    };

public:
    CameraCalibration();
    ~CameraCalibration();
    // 参数设置函数
    void setChessboardParams(const ChessboardParams& params);
    ChessboardParams getChessboardParams() const;
    CameraParams getCameraParams() const;

    // 标定相关函数
    bool calibrateCamera(const std::vector<std::string>& imageFiles);
    bool calibrateCamera(const std::vector<cv::Mat>& images);
    bool loadCalibrationData(const std::string& filename);
    bool saveCalibrationData(const std::string& filename) const;

    // 图像处理函数
    cv::Mat undistortImage(const cv::Mat& src) const;
    bool detectFeaturePoints(const cv::Mat& image,
                             std::vector<cv::Point2f>& points,
                             const std::string& method = "CHESSBOARD");

    // 坐标转换函数
    std::vector<cv::Point3f> pixel2Camera(
        const std::vector<cv::Point2f>& pixelPoints,
        double Z_depth_mm = 100.0) const;
    std::vector<cv::Point3f> camera2Base(const std::vector<cv::Point3f>& cameraPoints,
                                         const Eigen::Matrix4d& T_base_camera) const;

    // 状态查询函数
    bool isCalibrated() const;
    double getReprojectionError() const;

    // 辅助函数
    static void showImage(const std::string& winname, const cv::Mat& img, int delay = 0);
    static void drawPoints(cv::Mat& image, const std::vector<cv::Point2f>& points,
                           const cv::Scalar& color = cv::Scalar(0,255,0));

private:
    bool calibrated;             // 标定状态
    ChessboardParams chessParams;// 棋盘格参数
    CameraParams camParams;      // 相机参数

    // 辅助函数
    void drawDebugInfo(cv::Mat& image,
        const std::vector<cv::Point2f>& corners);
    void drawDetectedCircles(const cv::Mat& image, const std::vector<cv::Vec3f>& circles);
    std::vector<cv::Vec3f> postProcessCircles(const std::vector<cv::Vec3f>& circles,
        const cv::Size& imageSize);
    bool validateCircleInterior(const cv::Mat& image, const cv::Point2f& center, float radius);
    bool validateCircleEdge(const cv::Mat& image, const cv::Point2f& center, float radius);

    // 计算指定点的梯度强度
    float calculateGradientMagnitude(const cv::Mat& image, const cv::Point2f& point);

    // 计算圆的整体质量评分
    float calculateCircleQuality(const cv::Mat& image, const cv::Point2f& center, float radius);
    bool validateCornerArrangement(
        const std::vector<cv::Point2f>& corners,
        const cv::Size& patternSize);
    bool detectChessboardCorners(const cv::Mat& image,
                                 std::vector<cv::Point2f>& corners);
    bool detectCircles(const cv::Mat& image,
                       std::vector<cv::Point2f>& centers);
    cv::Mat preprocessImage(const cv::Mat& image);

    cv::Mat preprocessImage(const cv::Mat& image) const;
    void generateObjectPoints(std::vector<cv::Point3f>& objectPoints) const;

public:
    // 结果可视化函数
    void visualizeResults(const cv::Mat& image,
                          const std::vector<cv::Point2f>& imagePoints,
                          const std::vector<cv::Point3f>& worldPoints = std::vector<cv::Point3f>(),
                          const cv::Mat& T_base_camera = cv::Mat());

private:
    cv::Mat gray;
    // 辅助绘制函数
    void drawCoordinateSystem(cv::Mat& image, const cv::Mat& rvec,
                              const cv::Mat& tvec, float length = 100.0f);
    void drawPointsWithLabels(cv::Mat& image,
                              const std::vector<cv::Point2f>& points,
                              const std::vector<cv::Point3f>& worldPoints = std::vector<cv::Point3f>());
    cv::Mat createInfoPanel(const std::vector<cv::Point2f>& imagePoints,
                            const std::vector<cv::Point3f>& worldPoints);

// 在手眼标定中添加以下内容
public:
    // 手眼标定数据结构体
    struct HandEyeData {
        cv::Mat handEyeTransform;        // 手眼变换矩阵 T_gripper_camera (4x4)
        double calibrationError;         // 标定误差
        bool isCalibrated;               // 是否已标定

        HandEyeData() : calibrationError(0.0), isCalibrated(false) {}
    };

public:
    // 手眼标定主要函数
    bool performHandEyeCalibration(const std::vector<cv::Mat>& images,
                                   const std::vector<cv::Mat>& robotPoses);

    void printMatrix(const cv::Mat& matrix, const std::string& name);

    // 手眼标定数据管理
    bool saveHandEyeData(const std::string& filename) const;
    bool loadHandEyeData(const std::string& filename);

    // 获取手眼标定结果
    HandEyeData getHandEyeData() const;
    cv::Mat getHandEyeTransform() const;
    bool isHandEyeCalibrated() const;

    // 使用手眼标定结果进行坐标变换
    std::vector<cv::Point3f> pixelToRobotBase(const std::vector<cv::Point2f>& pixelPoints,
                                              const cv::Mat& currentRobotPose,
                                              double Z_depth_mm = 100.0) const;
    // 从取位姿矩阵
    static std::vector<Eigen::Matrix4d> readPosesFromCSV(const std::string& csvPath) {
        std::vector<Eigen::Matrix4d> poses;
        std::ifstream file(csvPath);

        if (!file.is_open()) {
            std::cerr << "Error: Cannot open CSV file: " << csvPath << std::endl;
            return poses;
        }

        std::string line;
        int poseIndex = 0;

        while (std::getline(file, line)) {
            // 跳过空行和注释行
            if (line.empty() || line[0] == '#') {
                continue;
            }

            std::stringstream ss(line);
            std::string cell;
            std::vector<double> values;

            // 解析每一行的数值
            while (std::getline(ss, cell, ',')) {
                try {
                    values.push_back(std::stod(cell));
                } catch (const std::exception& e) {
                    std::cerr << "Error parsing value: " << cell << std::endl;
                    continue;
                }
            }

            // 检查是否有足够的数值（16个数值构成4x4矩阵）
            if (values.size() == 16) {
                Eigen::Matrix4d pose;
                // 按行填充矩阵（CSV中按行存储）
                for (int i = 0; i < 4; i++) {
                    for (int j = 0; j < 4; j++) {
                        pose(i, j) = values[i * 4 + j];
                    }
                }
                poses.push_back(pose);
                poseIndex++;
            } else if (values.size() == 12) {
                // 如果只有12个值，假设是3x4的变换矩阵（缺少底行[0,0,0,1]）
                Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 4; j++) {
                        pose(i, j) = values[i * 4 + j];
                    }
                }
                poses.push_back(pose);
                poseIndex++;
            } else {
                std::cerr << "Warning: Line " << poseIndex + 1
                          << " has " << values.size()
                          << " values, expected 16 or 12" << std::endl;
            }
        }

        file.close();
        std::cout << "Successfully loaded " << poses.size() << " poses from " << csvPath << std::endl;
        return poses;
    }

    // // Eigen::Matrix4d转换为cv::Mat
    // static cv::Mat eigenToCvMat(const Eigen::Matrix4d& eigenMat) {
    //     cv::Mat cvMat(4, 4, CV_64F);
    //     for (int i = 0; i < 4; i++) {
    //         for (int j = 0; j < 4; j++) {
    //             cvMat.at<double>(i, j) = eigenMat(i, j);
    //         }
    //     }
    //     return cvMat;
    // }


private:
    HandEyeData handEyeData;  // 手眼标定数据

    // 简化的辅助函数
    bool validateRobotPose(const cv::Mat& pose) const;
    cv::Mat calculateCorrectedPose(const cv::Mat& originalPose, const cv::Mat& handEyeTransform);
    cv::Mat calculatePoseDifference(const cv::Mat& pose1, const cv::Mat& pose2);
    double calculateReprojectionError(const cv::Mat& R_gripper2base, const cv::Mat& t_gripper2base, const cv::Mat& R_target2cam, const cv::Mat& t_target2cam, const cv::Mat& handEyeTransform);
    cv::Mat solvePnPForImage(const cv::Mat& image);

};

#endif // CAMERA_CALIBRATION_H
