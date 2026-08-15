#include "HandEyeCalibration.h"
#include "robot_types.h"
#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

// 构造函数：初始化配置
HandEyeCalibrator::HandEyeCalibrator(const CalibrationConfig& cfg) : config(cfg) {}


// 生成标定板的三维坐标点
std::vector<cv::Point3f> HandEyeCalibrator::generateObjectPoints(int boardX, int boardY, float squareSize) {
    std::vector<cv::Point3f> objectPoints;
    for (int i = 0; i < boardY; ++i) {
        for (int j = 0; j < boardX; ++j) {
            objectPoints.emplace_back(j * squareSize, i * squareSize, 0.0f);
        }
    }
    return objectPoints;
}

// 查找所有图像中的棋盘格角点
std::vector<std::vector<cv::Point2f>> HandEyeCalibrator::findAllChessboard(const std::vector<cv::String>& filenames, int boardX, int boardY) {
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
        // 增强检测参数，提升角点检测稳定性
        bool found = cv::findChessboardCorners(gray, patternSize, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE +
            cv::CALIB_CB_FAST_CHECK + cv::CALIB_CB_EXHAUSTIVE);

        if (found) {
            // 亚像素精确化
            cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 0.0001);
            cv::cornerSubPix(gray, corners, cv::Size(15, 15), cv::Size(-1, -1), criteria);

            allCorners.push_back(corners);

            // 可视化验证
            cv::Mat imgCopy = img.clone();
            for (int i = 0; i < corners.size(); ++i) {
                cv::circle(imgCopy, corners[i], 5, cv::Scalar(0, 0, 255), -1);
                cv::putText(imgCopy, std::to_string(i),
                    cv::Point(corners[i].x + 5, corners[i].y - 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
            }
            // --- 修改部分开始：解决图片太大窗口显示不全的问题 ---
            std::string winName = "OpenCV Native Sorted Corners";

            // 1. 创建窗口，参数设置为 WINDOW_NORMAL (允许手动改变窗口大小)
            cv::namedWindow(winName, cv::WINDOW_NORMAL);

            // 2. 强制将窗口调整为固定大小（例如宽1000，高800）
            // 这样无论图片分辨率是4K还是更高，窗口都会适配这个尺寸
            cv::resizeWindow(winName, 1600, 1200);

            // 3. 显示图像
            cv::imshow(winName, imgCopy);
            // --- 修改部分结束 ---

            cv::waitKey(3000); // 建议稍微增加延时(例如100ms)，以便人眼能看清弹出的窗口
        }
        else {
            std::cerr << "未在图像中找到棋盘格角点: " << filename << std::endl;
        }
    }
    cv::destroyAllWindows();
    return allCorners;
}

// 像素坐标转换为相机坐标（solvePnP外参版）
bool HandEyeCalibrator::pixelToCameraCoord(const cv::Point2f& pixel, const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs, const cv::Mat& rvec, const cv::Mat& tvec,
    const std::vector<cv::Point3f>& objectPoints, cv::Point3d& camCoord) {

    // 1. 相机内参合法性检查
    if (cameraMatrix.empty() || cameraMatrix.rows != 3 || cameraMatrix.cols != 3 || cameraMatrix.type() != CV_64F) return false;
    if (rvec.empty() || rvec.rows != 3 || rvec.cols != 1 || rvec.type() != CV_64F ||
        tvec.empty() || tvec.rows != 3 || tvec.cols != 1 || tvec.type() != CV_64F) return false;

    // 2. 像素点去畸变
    std::vector<cv::Point2f> srcPoints = { pixel };
    std::vector<cv::Point2f> dstPoints;
    try {
        cv::undistortPoints(srcPoints, dstPoints, cameraMatrix, distCoeffs, cv::noArray(), cameraMatrix);
    }
    catch (...) { return false; }

    if (dstPoints.empty()) return false;
    cv::Point2f undistortedPixel = dstPoints[0];

    // 3. 计算相机坐标系下的射线方向
    double fx = cameraMatrix.at<double>(0, 0);
    double fy = cameraMatrix.at<double>(1, 1);
    double cx = cameraMatrix.at<double>(0, 2);
    double cy = cameraMatrix.at<double>(1, 2);

    if (fabs(fx) < 1e-8 || fabs(fy) < 1e-8) return false;

    cv::Mat ray = (cv::Mat_<double>(3, 1) << (undistortedPixel.x - cx) / fx, (undistortedPixel.y - cy) / fy, 1.0);

    // 4. 将solvePnP求出的rvec/tvec 转换为旋转矩阵R和位移T
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::Mat T = tvec;

    // 5. 计算标定板平面的方程
    cv::Mat n_c = R.col(2);
    double num = n_c.dot(T);
    double den = n_c.dot(ray);

    if (fabs(den) < 1e-8) return false;

    double s = num / den; // 深度

    // 6. 得到相机坐标系下的三维坐标
    ray *= s;
    camCoord.x = ray.at<double>(0, 0);
    camCoord.y = ray.at<double>(1, 0);
    camCoord.z = ray.at<double>(2, 0);

    return true;
}

// 像素坐标转基坐标（solvePnP外参版）
bool HandEyeCalibrator::pixelToRobotBaseCoord(const cv::Point2f& pixel, const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs, const cv::Mat& rvec, const cv::Mat& tvec,
    const std::vector<cv::Point3f>& objectPoints,
    const cv::Mat& R_cam2gripper, const cv::Mat& t_cam2gripper,
    const cv::Mat& R_gripper2base, const cv::Mat& t_gripper2base,
    cv::Point3d& baseCoord) {

    // 1. 先转到相机坐标系
    cv::Point3d camCoord;
    if (!pixelToCameraCoord(pixel, cameraMatrix, distCoeffs, rvec, tvec, objectPoints, camCoord)) {
        return false;
    }

    // 2. 相机坐标 -> 夹具坐标
    cv::Mat P_cam = (cv::Mat_<double>(3, 1) << camCoord.x, camCoord.y, camCoord.z);
    cv::Mat P_gripper = R_cam2gripper * P_cam + t_cam2gripper;

    // 3. 夹具坐标 -> 基坐标
    cv::Mat P_base = R_gripper2base * P_gripper + t_gripper2base;

    baseCoord.x = P_base.at<double>(0, 0);
    baseCoord.y = P_base.at<double>(1, 0);
    baseCoord.z = P_base.at<double>(2, 0);

    return true;
}

// 2. 静态工具函数实现
// 从文件读取矩阵
cv::Mat_<double> HandEyeCalibrator::readMatrixFromFile(const std::string& filePath, int rows, int cols) {
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
        if (col != cols) throw std::runtime_error("文件列数与预期不符");
        row++;
    }
    if (row != rows) throw std::runtime_error("文件行数与预期不符");
    return matrix;
}

// 从RT矩阵中提取旋转和平移分量
void HandEyeCalibrator::RT2R_T(const cv::Mat& rt, cv::Mat& R, cv::Mat& T) {
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
bool HandEyeCalibrator::isRotationMatrix(const cv::Mat& R) {
    if (R.rows != 3 || R.cols != 3) return false;
    cv::Mat Rt;
    cv::transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat identity = cv::Mat::eye(3, 3, shouldBeIdentity.type());
    return cv::norm(identity - shouldBeIdentity) < 1e-6;
}

void HandEyeCalibrator::saveCalibrationResults(const std::string& filePath, const cv::Mat& cameraMatrix,
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

// 从CSV文件读取机器人位姿
std::vector<Eigen::Matrix4d> HandEyeCalibrator::readPosesFromCSV(const std::string& filePath) {
    std::vector<Eigen::Matrix4d> poses;
    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "无法打开机器人位姿文件: " << filePath << std::endl;
        return poses;
    }
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string val;
        std::vector<double> values;
        while (std::getline(ss, val, ',')) {
            values.push_back(std::stod(val));
        }
        if (values.size() >= 6) {}
    }
    return poses;
}

// 从CSV读取6D位姿（x,y,z,rx,ry,rz）- 支持图像编号列，保证与标定图像顺序一一对应
std::vector<Eigen::Matrix4d> HandEyeCalibrator::readPoint6DFromCSV(const std::string& filePath, bool hasImageIndexCol)
{
    // 存储 <图像编号, 位姿矩阵> 对，用于排序
    std::vector<std::pair<int, Eigen::Matrix4d>> indexedPoses;
    std::ifstream file(filePath);
    if (!file.is_open()) {
        throw std::runtime_error("无法打开6D位姿文件: " + filePath);
    }

    std::string line;
    int lineNum = 0;
    while (std::getline(file, line)) {
        lineNum++;
        std::stringstream ss(line);
        std::string value;
        double x, y, z, rx, ry, rz;
        int imgIndex = -1; // 图像编号（对应0001.jpg的1、0002.jpg的2...）
        int fieldIdx = 0;

        // 逐字段读取（支持两种格式：有编号列/无编号列）
        while (std::getline(ss, value, ',') && (hasImageIndexCol ? fieldIdx < 7 : fieldIdx < 6)) {
            try {
                double val = std::stod(value);
                if (hasImageIndexCol && fieldIdx == 0) {
                    // 第一列是图像编号（必须为正整数）
                    imgIndex = static_cast<int>(val);
                    if (imgIndex <= 0) {
                        std::cerr << "第" << lineNum << "行：图像编号必须为正整数，跳过该行 | 数值：" << val << std::endl;
                        break;
                    }
                }
                else {
                    // 后续列是6D位姿（x,y,z,rx,ry,rz）
                    int poseFieldIdx = hasImageIndexCol ? fieldIdx - 1 : fieldIdx;
                    switch (poseFieldIdx) {
                    case 0: x = val; break;
                    case 1: y = val; break;
                    case 2: z = val; break;
                    case 3: rx = val; break;
                    case 4: ry = val; break;
                    case 5: rz = val; break;
                    }
                }
                fieldIdx++;
            }
            catch (const std::exception& e) {
                std::cerr << "第" << lineNum << "行：解析数值失败，跳过该行 | 错误：" << e.what() << " | 内容：" << line << std::endl;
                break;
            }
        }

        // 校验字段数量是否完整
        int expectedFieldCnt = hasImageIndexCol ? 7 : 6;
        if (fieldIdx != expectedFieldCnt) {
            std::cerr << "第" << lineNum << "行：字段数不匹配（预期" << expectedFieldCnt << "个，实际" << fieldIdx << "个），跳过该行 | 内容：" << line << std::endl;
            continue;
        }

        // 无编号列时，按行号作为图像编号（第一行对应1，第二行对应2...）
        if (!hasImageIndexCol) {
            imgIndex = lineNum;
        }

        //// 核心：旋转矢量 -> 旋转矩阵（罗德里格斯变换）
        //cv::Mat rvec_cv = (cv::Mat_<double>(3, 1) << rx, ry, rz);
        //cv::Mat R_cv(3, 3, CV_64F);
        //cv::Rodrigues(rvec_cv, R_cv);

        //// 转换为Eigen旋转矩阵
        //Eigen::Matrix3d R_eigen;
        //for (int i = 0; i < 3; ++i) {
        //    for (int j = 0; j < 3; ++j) {
        //        R_eigen(i, j) = R_cv.at<double>(i, j);
        //    }
        //}

        //// 构建4x4齐次变换矩阵
        //Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        //pose.block<3, 3>(0, 0) = R_eigen;  // 旋转部分
        //pose.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z); // 平移部分

        // 构造Point6D对象
        Point6D currentPoint = { x, y, z, rx, ry, rz };

        // 欧拉角
        Eigen::Matrix4d pose = toEigenMatrix(currentPoint);

        // 检查图像编号是否重复
        bool isDuplicate = false;
        for (const auto& p : indexedPoses) {
            if (p.first == imgIndex) {
                isDuplicate = true;
                break;
            }
        }
        if (isDuplicate) {
            std::cerr << "第" << lineNum << "行：图像编号" << imgIndex << "重复，跳过该行" << std::endl;
            continue;
        }

        indexedPoses.emplace_back(imgIndex, pose);
    }

    if (indexedPoses.empty()) {
        throw std::runtime_error("未读取到有效的6D位姿数据");
    }

    // 按图像编号升序排序（关键：保证与0001.jpg→0002.jpg...顺序一致）
    std::sort(indexedPoses.begin(), indexedPoses.end(),
        [](const std::pair<int, Eigen::Matrix4d>& a, const std::pair<int, Eigen::Matrix4d>& b) {
            return a.first < b.first;
        });

    // 提取排序后的位姿矩阵（去除编号）
    std::vector<Eigen::Matrix4d> poses;
    for (const auto& p : indexedPoses) {
        poses.push_back(p.second);
    }

    // 输出排序后的编号映射（调试用）
    std::cout << "读取并排序后的位姿映射（图像编号→行号）：";
    for (size_t i = 0; i < indexedPoses.size(); ++i) {
        std::cout << indexedPoses[i].first << "→" << (i + 1) << " ";
    }
    std::cout << std::endl;

    return poses;
}

 //将Eigen的4x4矩阵转换为OpenCV的cv::Mat（CV_64F类型）  
cv::Mat HandEyeCalibrator::eigenToCvMat(const Eigen::Matrix4d& eigenMat) {
    cv::Mat cvMat(4, 4, CV_64F);
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            cvMat.at<double>(i, j) = eigenMat(i, j);
    return cvMat;
}

// 3. 流程封装函数实现 

void HandEyeCalibrator::batchSolvePnP(const std::vector<std::vector<cv::Point3f>>& objectPoints,
    const std::vector<std::vector<cv::Point2f>>& imagePoints,
    const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
    std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs) {

    rvecs.reserve(imagePoints.size());
    tvecs.reserve(imagePoints.size());

    for (size_t i = 0; i < imagePoints.size(); ++i) {
        cv::Mat rvec, tvec;
        // 使用 ITERATIVE 方法求解，这是比较通用的方法
        bool success = cv::solvePnP(objectPoints[i], imagePoints[i], cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);

        if (!success) {
            std::cerr << "警告: 第 " << i + 1 << " 张图片 solvePnP 求解失败，使用零向量填充。" << std::endl;
            rvec = cv::Mat::zeros(3, 1, CV_64F);
            tvec = cv::Mat::zeros(3, 1, CV_64F);
        }
        rvecs.push_back(rvec);
        tvecs.push_back(tvec);
    }
    std::cout << "所有图片外参计算完成。" << std::endl;
}

void HandEyeCalibrator::prepareHandEyeData(const std::vector<Eigen::Matrix4d>& eigenPoses,
    const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
    std::vector<cv::Mat>& R_gripper2base, std::vector<cv::Mat>& t_gripper2base,
    std::vector<cv::Mat>& R_target2cam, std::vector<cv::Mat>& t_target2cam) {
    for (size_t i = 0; i < rvecs.size(); i++) {
        // 将Eigen矩阵转换为OpenCV的cv::Mat（4x4齐次矩阵）
        cv::Mat pose = HandEyeCalibrator::eigenToCvMat(eigenPoses[i]);

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
}

void HandEyeCalibrator::computeCalibrationError(const std::vector<std::vector<cv::Point2f>>& imagePoints,
    const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
    const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
    const std::vector<cv::Point3f>& singleObjectPoints,
    const cv::Mat& R_cam2gripper, const cv::Mat& t_cam2gripper,
    const std::vector<cv::Mat>& R_gripper2base,
    const std::vector<cv::Mat>& t_gripper2base) {

    std::vector<cv::Point3d> validBaseCoords;

    for (size_t imgIdx = 0; imgIdx < imagePoints.size(); imgIdx++) {
        // 获取外参
        cv::Mat rvec = rvecs[imgIdx];
        cv::Mat tvec = tvecs[imgIdx];

        // 获取当前图片的第2个角点 (index 1)
        cv::Point2f secondPixel = imagePoints[imgIdx][1];

        // 计算该点在基坐标系下的坐标
        cv::Point3d baseCoord;
        bool convertSuccess = pixelToRobotBaseCoord(secondPixel, cameraMatrix, distCoeffs,
            rvec, tvec, singleObjectPoints,
            R_cam2gripper, t_cam2gripper,
            R_gripper2base[imgIdx], t_gripper2base[imgIdx],
            baseCoord);

        if (convertSuccess) {
            std::cout << "\n图片 " << imgIdx + 1 << ":" << std::endl;
            std::cout << "  基坐标系坐标 (mm): X=" << baseCoord.x
                << ", Y=" << baseCoord.y
                << ", Z=" << baseCoord.z << std::endl;
            double depth = tvec.at<double>(2);
            std::cout << "  标定板到相机的深度: " << depth << " mm" << std::endl;
            validBaseCoords.push_back(baseCoord);
        }
        else {
            std::cerr << "图片 " << imgIdx + 1 << " 坐标转换失败，跳过" << std::endl;
        }
    }

    if (!validBaseCoords.empty()) {
        std::cout << "\n================ 误差统计 ================" << std::endl;
        cv::Point3d meanCoord(0, 0, 0);
        for (const auto& pt : validBaseCoords) {
            meanCoord.x += pt.x; meanCoord.y += pt.y; meanCoord.z += pt.z;
        }
        meanCoord.x /= validBaseCoords.size();
        meanCoord.y /= validBaseCoords.size();
        meanCoord.z /= validBaseCoords.size();

        std::cout << "拟合中心点 (Mean): [" << meanCoord.x << ", " << meanCoord.y << ", " << meanCoord.z << "]" << std::endl;

        double sumErrorSq = 0.0, sumError = 0.0, maxError = 0.0;
        for (const auto& pt : validBaseCoords) {
            double dist = std::sqrt(std::pow(pt.x - meanCoord.x, 2) + std::pow(pt.y - meanCoord.y, 2) + std::pow(pt.z - meanCoord.z, 2));
            sumError += dist;
            sumErrorSq += dist * dist;
            if (dist > maxError) maxError = dist;
        }

        double meanError = sumError / validBaseCoords.size();
        double rmse = std::sqrt(sumErrorSq / validBaseCoords.size());

        std::cout << "样本数量: " << validBaseCoords.size() << std::endl;
        std::cout << "平均误差 (Mean Error): " << meanError << " mm" << std::endl;
        std::cout << "最大误差 (Max Error):  " << maxError << " mm" << std::endl;
        std::cout << "均方根误差 (RMSE):     " << rmse << " mm" << std::endl;
        std::cout << "==========================================" << std::endl;
    }
    else {
        std::cout << "\n没有有效的坐标数据，无法计算误差。" << std::endl;
    }
}
