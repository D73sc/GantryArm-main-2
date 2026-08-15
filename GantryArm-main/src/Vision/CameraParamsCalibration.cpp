#include "CameraParamsCalibration.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

CameraParamsCalibrator::CameraParamsCalibrator(const ChessboardParams chessboardParams) :chessboardParams_(chessboardParams)
{
    // 初始化内参矩阵和畸变系数
    params.camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    params.dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
    params.reproj_error = 0.0;
}

// 添加设置固定深度的方法
void CameraParamsCalibrator::setFixedDepth(double depth) {
    fixed_depth = depth;
}

// 添加像素坐标到相机坐标的转换函数（使用固定深度）
cv::Point3f CameraParamsCalibrator::pixelToCamera(const cv::Point2f& pixel) {
    // 检查内参矩阵是否有效
    if (params.camera_matrix.empty() || params.camera_matrix.rows != 3 || params.camera_matrix.cols != 3) {
        std::cerr << "相机内参矩阵无效，无法进行坐标转换" << std::endl;
        return cv::Point3f(0, 0, 0);
    }

    // 获取内参
    double fx = params.camera_matrix.at<double>(0, 0);
    double fy = params.camera_matrix.at<double>(1, 1);
    double cx = params.camera_matrix.at<double>(0, 2);
    double cy = params.camera_matrix.at<double>(1, 2);

    // 去畸变
    std::vector<cv::Point2f> distorted_points = { pixel };
    std::vector<cv::Point2f> undistorted_points;
    cv::undistortPoints(distorted_points, undistorted_points,
        params.camera_matrix, params.dist_coeffs);

    // 计算相机坐标 (使用固定深度)
    cv::Point2f undistorted = undistorted_points[0];
    double X = (undistorted.x - cx) * fixed_depth / fx;
    double Y = (undistorted.y - cy) * fixed_depth / fy;
    double Z = fixed_depth;

    return cv::Point3f(X, Y, Z);
}



//转化为齐次矩阵
cv::Mat CameraParamsCalibrator::rtvecToHomogeneous(const cv::Mat& rvec, const cv::Mat& tvec) {
    cv::Mat R;
    cv::Rodrigues(rvec, R);  // 旋转向量转旋转矩阵

    cv::Mat H = cv::Mat::eye(4, 4, CV_64F);
    R.copyTo(H(cv::Rect(0, 0, 3, 3)));
    tvec.copyTo(H(cv::Rect(3, 0, 1, 3)));

    return H;
}

//齐次矩阵的乘积
cv::Mat CameraParamsCalibrator::multiplyHomogeneous(const cv::Mat& a, const cv::Mat& b) {
    cv::Mat result = cv::Mat::eye(4, 4, CV_64F);

    // 旋转部分: R = R1 * R2
    cv::Mat R1 = a(cv::Rect(0, 0, 3, 3));
    cv::Mat R2 = b(cv::Rect(0, 0, 3, 3));
    cv::Mat R = R1 * R2;

    // 平移部分: t = R1 * t2 + t1
    cv::Mat t1 = a(cv::Rect(3, 0, 1, 3));
    cv::Mat t2 = b(cv::Rect(3, 0, 1, 3));
    cv::Mat t = R1 * t2 + t1;

    R.copyTo(result(cv::Rect(0, 0, 3, 3)));
    t.copyTo(result(cv::Rect(3, 0, 1, 3)));

    return result;
}

//齐次矩阵的逆
cv::Mat CameraParamsCalibrator::inverseHomogeneous(const cv::Mat& mat) {
    cv::Mat result = cv::Mat::eye(4, 4, CV_64F);

    // 旋转部分的逆是转置
    cv::Mat R = mat(cv::Rect(0, 0, 3, 3));
    cv::Mat R_inv = R.t();

    // 平移部分的逆
    cv::Mat t = mat(cv::Rect(3, 0, 1, 3));
    cv::Mat t_inv = -R_inv * t;

    R_inv.copyTo(result(cv::Rect(0, 0, 3, 3)));
    t_inv.copyTo(result(cv::Rect(3, 0, 1, 3)));

    return result;
}

//相机内参标定
bool CameraParamsCalibrator::calibrateCamera(const std::vector<std::string>& image_paths) {
    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points;

    // 生成棋盘格的三维坐标
    std::vector<cv::Point3f> obj;
    for (int i = 0; i < chessboardParams_.board_size.height; ++i) {
        for (int j = 0; j < chessboardParams_.board_size.width; ++j) {
            obj.push_back(cv::Point3f(j * chessboardParams_.square_size, i * chessboardParams_.square_size, 0));
        }
    }

    // 处理每张图像，检测棋盘格角点
    for (const std::string& path : image_paths) {
        cv::Mat image = cv::imread(path);
        if (image.empty()) {
            std::cerr << "无法读取图像: " << path << std::endl;
            continue;
        }

        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(gray, chessboardParams_.board_size, corners);

        if (found) {
            // 亚像素级角点检测，提高精度
            cv::cornerSubPix(gray, corners, cv::Size(11, 11),
                cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 0.001));

            image_points.push_back(corners);
            object_points.push_back(obj);

            // 可视化检测到的角点
            /*cv::drawChessboardCorners(image, board_size, cv::Mat(corners), found);
            cv::imshow("Chessboard Corners", image);
            cv::waitKey(100);*/
        }
        else {
            std::cerr << "在图像 " << path << " 中未找到棋盘格" << std::endl;
        }
    }

    cv::destroyAllWindows();

    if (image_points.size() < 5) {  // 至少需要5张图像进行标定
        std::cerr << "用于标定的有效图像不足" << std::endl;
        return false;
    }

    // 相机标定
    std::vector<cv::Mat> rvecs, tvecs;
    cv::Size image_size = cv::imread(image_paths[0]).size();  // 从第一张图像获取尺寸
    params.reproj_error = cv::calibrateCamera(object_points, image_points,
        image_size,
        params.camera_matrix, params.dist_coeffs, rvecs, tvecs);

    std::cout << "相机内参标定完成，平均重投影误差: " << params.reproj_error << std::endl;
    std::cout << "相机内参矩阵:\n" << params.camera_matrix << std::endl;
    std::cout << "畸变系数:\n" << params.dist_coeffs << std::endl;

    return true;
}


void CameraParamsCalibrator::undistortImage(const cv::Mat& input_image, cv::Mat& output_image) {
    cv::undistort(input_image, output_image, params.camera_matrix, params.dist_coeffs);
}


bool CameraParamsCalibrator::getCameraPoseFromImage(const cv::Mat& image, cv::Mat& camera_pose) {
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::Point2f> corners;
    bool found = cv::findChessboardCorners(gray, chessboardParams_.board_size, corners);

    if (!found) {
        std::cerr << "未找到棋盘格角点" << std::endl;
        return false;
    }

    // 亚像素级角点检测
    cv::cornerSubPix(gray, corners, cv::Size(11, 11),
        cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 0.001));

    // 生成棋盘格三维坐标
    std::vector<cv::Point3f> obj;
    for (int i = 0; i < chessboardParams_.board_size.height; ++i) {
        for (int j = 0; j < chessboardParams_.board_size.width; ++j) {
            obj.push_back(cv::Point3f(j * chessboardParams_.square_size, i * chessboardParams_.square_size, 0));
        }
    }

    // 求解位姿
    cv::Mat rvec, tvec;
    cv::solvePnP(obj, corners, params.camera_matrix, params.dist_coeffs, rvec, tvec);

    // 转换为齐次矩阵
    camera_pose = rtvecToHomogeneous(rvec, tvec);
    return true;
}

bool CameraParamsCalibrator::saveParams(const std::string& filename) {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cerr << "无法打开文件 " << filename << " 进行写入" << std::endl;
        return false;
    }

    // 保存相机参数
    fs << "camera_matrix" << params.camera_matrix;
    fs << "dist_coeffs" << params.dist_coeffs;
    fs << "reproj_error" << params.reproj_error;

    // 保存棋盘格参数和固定深度
    fs << "board_width" << chessboardParams_.board_size.width;
    fs << "board_height" << chessboardParams_.board_size.height;
    fs << "square_size" << chessboardParams_.square_size;
    fs << "fixed_depth" << fixed_depth;  // 保存固定深度值

    fs.release();
    return true;
}

bool CameraParamsCalibrator::loadParams(const std::string& filename) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "无法打开文件 " << filename << " 进行读取" << std::endl;
        return false;
    }

    // 加载相机参数
    fs["camera_matrix"] >> params.camera_matrix;
    fs["dist_coeffs"] >> params.dist_coeffs;
    fs["reproj_error"] >> params.reproj_error;

    // 加载棋盘格参数和固定深度
    int w, h;
    fs["board_width"] >> w;
    fs["board_height"] >> h;
    chessboardParams_.board_size = cv::Size(w, h);

    fs["square_size"] >> chessboardParams_.square_size;

    // 加载固定深度值，如果不存在则使用默认值
    if (!fs["fixed_depth"].isNone()) {
        fs["fixed_depth"] >> fixed_depth;
    }

    fs.release();
    return true;
}

const CameraParams& CameraParamsCalibrator::getCameraParams() const {
    return params;
}


