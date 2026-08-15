#include "hand_eye_calibration.h"
#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

// 生成标定板的三维坐标点
std::vector<cv::Point3f> generateObjectPoints(int boardX, int boardY, float squareSize) {
    std::vector<cv::Point3f> objectPoints;
    for (int i = 0; i < boardY; ++i) {
        for (int j = 0; j < boardX; ++j) {
            objectPoints.emplace_back(j * squareSize, i * squareSize, 0.0f);
        }
    }
    return objectPoints;
}

// 查找所有图像中的棋盘格角点
std::vector<std::vector<cv::Point2f>> findAllChessboard(const std::vector<cv::String>& filenames, int boardX, int boardY) {
    std::vector<std::vector<cv::Point2f>> allCorners;
    cv::Size patternSize(boardX, boardY);

    for (const auto& filename : filenames) {
        cv::Mat img = cv::imread(filename);
        if (img.empty()) {
            std::cerr << "无法读取图像: " << filename << std::endl;
            continue;
        }

        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(gray, patternSize, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);

        if (found) {
            // 亚像素精确化
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
            allCorners.push_back(corners);

            // 可视化角点（可选）
            cv::drawChessboardCorners(img, patternSize, corners, found);
            cv::imshow("Chessboard Corners", img);
            cv::waitKey(50);
        }
        else {
            std::cerr << "未在图像中找到棋盘格角点: " << filename << std::endl;
        }
    }
    cv::destroyAllWindows();
    return allCorners;
}

// 从文件读取矩阵
cv::Mat_<double> readMatrixFromFile(const std::string& filePath, int rows, int cols) {
    std::ifstream file(filePath);
    if (!file.is_open()) {
        throw std::runtime_error("无法打开文件: " + filePath);
    }

    cv::Mat_<double> matrix(rows, cols);
    std::string line;
    int row = 0;

    while (std::getline(file, line) && row < rows) {
        std::stringstream ss(line);
        std::string value;
        int col = 0;

        while (std::getline(ss, value, ',') && col < cols) {
            try {
                matrix(row, col) = std::stod(value);
            }
            catch (const std::exception& e) {
                throw std::runtime_error("文件格式错误，解析数值失败: " + std::string(e.what()));
            }
            col++;
        }

        if (col != cols) {
            throw std::runtime_error("文件列数与预期不符: " + std::to_string(col) + " vs " + std::to_string(cols));
        }
        row++;
    }

    if (row != rows) {
        throw std::runtime_error("文件行数与预期不符: " + std::to_string(row) + " vs " + std::to_string(rows));
    }

    return matrix;
}

// 从RT矩阵中提取旋转和平移分量
void RT2R_T(const cv::Mat& rt, cv::Mat& R, cv::Mat& T) {
    if (rt.rows == 3 && rt.cols == 4) {
        R = rt(cv::Rect(0, 0, 3, 3)).clone();
        T = rt(cv::Rect(3, 0, 1, 3)).clone();
    }
    else if (rt.rows == 4 && rt.cols == 4) {
        R = rt(cv::Rect(0, 0, 3, 3)).clone();
        T = rt(cv::Rect(3, 0, 1, 3)).clone();
    }
    else {
        throw std::invalid_argument("RT矩阵必须是3x4或4x4的矩阵");
    }
}

// 检查是否为旋转矩阵
bool isRotationMatrix(const cv::Mat& R) {
    if (R.rows != 3 || R.cols != 3) return false;

    cv::Mat Rt;
    cv::transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat identity = cv::Mat::eye(3, 3, shouldBeIdentity.type());

    return cv::norm(identity - shouldBeIdentity) < 1e-6;
}

// 保存标定结果
void saveCalibrationResults(const std::string& filePath, const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs, const cv::Mat& R_cam2gripper,
    const cv::Mat& t_cam2gripper) {
    std::ofstream file(filePath);
    if (!file.is_open()) {
        throw std::runtime_error("无法打开文件用于写入: " + filePath);
    }

    file << "相机内参矩阵 (3x3):" << std::endl;
    file << cameraMatrix << std::endl << std::endl;

    file << "畸变系数 (1x5):" << std::endl;
    file << distCoeffs << std::endl << std::endl;

    file << "相机到夹具的旋转矩阵 R_cam2gripper (3x3):" << std::endl;
    file << R_cam2gripper << std::endl << std::endl;

    file << "相机到夹具的平移向量 t_cam2gripper (3x1):" << std::endl;
    file << t_cam2gripper << std::endl << std::endl;

    std::cout << "标定结果已保存至: " << filePath << std::endl;
}

// 像素坐标转换为相机坐标
bool pixelToCameraCoord(const cv::Point2f& pixel, const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs, double depth, cv::Point3d& camCoord) {
    if (cameraMatrix.empty() || cameraMatrix.rows != 3 || cameraMatrix.cols != 3 || cameraMatrix.type() != CV_64F) {
        std::cerr << "无效的相机内参矩阵" << std::endl;
        return false;
    }
    if (depth <= 0) {
        std::cerr << "深度值必须为正" << std::endl;
        return false;
    }

    std::vector<cv::Point2f> srcPoints = { pixel };
    std::vector<cv::Point2f> dstPoints;

    // 去畸变
    try {
        cv::undistortPoints(srcPoints, dstPoints, cameraMatrix, distCoeffs, cv::noArray(), cameraMatrix);
    }
    catch (const cv::Exception& e) {
        std::cerr << "去畸变失败: " << e.what() << std::endl;
        return false;
    }

    if (dstPoints.empty()) {
        std::cerr << "去畸变后点为空" << std::endl;
        return false;
    }
    cv::Point2f undistortedPixel = dstPoints[0];

    // 提取内参
    double fx = cameraMatrix.at<double>(0, 0);
    double fy = cameraMatrix.at<double>(1, 1);
    double cx = cameraMatrix.at<double>(0, 2);
    double cy = cameraMatrix.at<double>(1, 2);

    if (fx == 0 || fy == 0) {
        std::cerr << "相机内参 fx 或 fy 为零" << std::endl;
        return false;
    }

    // 计算相机坐标
    double x_normalized = (undistortedPixel.x - cx) / fx;
    double y_normalized = (undistortedPixel.y - cy) / fy;

    camCoord.x = x_normalized * depth;
    camCoord.y = y_normalized * depth;
    camCoord.z = depth;

    return true;
}

// 像素坐标转换为机器人基坐标
bool pixelToRobotBaseCoord(const cv::Point2f& pixel, double depth,
    const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
    const cv::Mat& R_cam2gripper, const cv::Mat& t_cam2gripper,
    const cv::Mat& R_gripper2base, const cv::Mat& t_gripper2base,
    cv::Point3d& baseCoord) {
    // 像素坐标 -> 相机坐标
    cv::Point3d P_cam;
    if (!pixelToCameraCoord(pixel, cameraMatrix, distCoeffs, depth, P_cam)) {
        std::cerr << "像素到相机坐标转换失败" << std::endl;
        return false;
    }

    // 构造齐次坐标
    cv::Mat P_cam_h = (cv::Mat_<double>(4, 1) << P_cam.x, P_cam.y, P_cam.z, 1.0);

    // 相机 -> 夹具的变换矩阵
    cv::Mat T_ee_cam = cv::Mat::eye(4, 4, CV_64F);
    R_cam2gripper.copyTo(T_ee_cam(cv::Rect(0, 0, 3, 3)));
    t_cam2gripper.copyTo(T_ee_cam(cv::Rect(3, 0, 1, 3)));

    // 夹具 -> 基坐标的变换矩阵
    cv::Mat T_base_ee = cv::Mat::eye(4, 4, CV_64F);
    R_gripper2base.copyTo(T_base_ee(cv::Rect(0, 0, 3, 3)));
    t_gripper2base.copyTo(T_base_ee(cv::Rect(3, 0, 1, 3)));

    // 计算基坐标下的齐次坐标
    cv::Mat P_base_h = T_base_ee * T_ee_cam * P_cam_h;

    // 提取结果
    baseCoord.x = P_base_h.at<double>(0);
    baseCoord.y = P_base_h.at<double>(1);
    baseCoord.z = P_base_h.at<double>(2);

    return true;
}

// 计算水平误差
void calculateHorizontalErrors(const std::vector<cv::Point2f>& imagePoints,
    const std::vector<cv::Point3f>& objectPoints,
    double depth, const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs, const cv::Mat& R_cam2gripper,
    const cv::Mat& t_cam2gripper, const cv::Mat& R_gripper2base,
    const cv::Mat& t_gripper2base, double& meanXError,
    double& meanYError, double& maxXError, double& maxYError) {
    if (imagePoints.size() != objectPoints.size()) {
        throw std::invalid_argument("图像点和目标点数量不匹配");
    }

    meanXError = 0.0;
    meanYError = 0.0;
    maxXError = 0.0;
    maxYError = 0.0;
    int validCount = 0;

    // 以第一个点为原点计算相对坐标
    cv::Point3d originBase;
    if (!pixelToRobotBaseCoord(imagePoints[0], depth, cameraMatrix, distCoeffs,
        R_cam2gripper, t_cam2gripper, R_gripper2base, t_gripper2base, originBase)) {
        throw std::runtime_error("原点坐标转换失败");
    }

    for (size_t i = 0; i < imagePoints.size(); ++i) {
        // 转换当前像素点到机器人基坐标
        cv::Point3d currentBase;
        if (!pixelToRobotBaseCoord(imagePoints[i], depth, cameraMatrix, distCoeffs,
            R_cam2gripper, t_cam2gripper, R_gripper2base, t_gripper2base, currentBase)) {
            std::cerr << "跳过无效点 " << i << std::endl;
            continue;
        }

        // 计算相对坐标（相对于第一个点）
        double x = currentBase.x - originBase.x;
        double y = currentBase.y - originBase.y;

        // 理论相对坐标（来自标定板）
        double X = objectPoints[i].x - objectPoints[0].x;
        double Y = objectPoints[i].y - objectPoints[0].y;

        // 计算误差
        double xError = std::abs(x - X);
        double yError = std::abs(y - Y);

        // 累加误差
        meanXError += xError;
        meanYError += yError;
        maxXError = std::max(maxXError, xError);
        maxYError = std::max(maxYError, yError);

        validCount++;
        std::cout << "点 " << i << " 误差: X=" << xError << "mm, Y=" << yError << "mm" << std::endl;
    }

    if (validCount == 0) {
        throw std::runtime_error("没有有效点用于计算误差");
    }

    // 计算平均误差
    meanXError /= validCount;
    meanYError /= validCount;
}

// 实现Eigen与OpenCV转换函数
namespace CameraCalibration {
    // 从CSV文件读取机器人位姿（每行16个数值，对应4x4矩阵的按行存储）
    std::vector<Eigen::Matrix4d> readPosesFromCSV(const std::string& filePath) {
        std::vector<Eigen::Matrix4d> poses;
        std::ifstream file(filePath);

        if (!file.is_open()) {
            std::cerr << "Error: Cannot open CSV file: " << filePath << std::endl;
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
                }
                catch (const std::exception& e) {
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
            }
            else if (values.size() == 12) {
                // 如果只有12个值，假设是3x4的变换矩阵（缺少底行[0,0,0,1]）
                Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 4; j++) {
                        pose(i, j) = values[i * 4 + j];
                    }
                }
                poses.push_back(pose);
                poseIndex++;
            }
            else {
                std::cerr << "Warning: Line " << poseIndex + 1
                    << " has " << values.size()
                    << " values, expected 16 or 12" << std::endl;
            }
        }

        file.close();
        std::cout << "Successfully loaded " << poses.size() << " poses from " << filePath << std::endl;
        return poses;
    }

    // 将Eigen的4x4矩阵转换为OpenCV的cv::Mat（CV_64F类型）
    cv::Mat eigenToCvMat(const Eigen::Matrix4d& eigenMat) {
        cv::Mat cvMat(4, 4, CV_64F);
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                cvMat.at<double>(i, j) = eigenMat(i, j);
            }
        }
        return cvMat;
    }
}
