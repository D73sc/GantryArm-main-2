#include "CameraCalibration.h"
#include <fstream>
#include <iostream>
#include <numeric>
CameraCalibration::CameraCalibration() : calibrated(false) {}

CameraCalibration::~CameraCalibration() {}

void CameraCalibration::setChessboardParams(const ChessboardParams& params) {
    chessParams = params;
}

CameraCalibration::ChessboardParams CameraCalibration::getChessboardParams() const {
    return chessParams;
}

CameraCalibration::CameraParams CameraCalibration::getCameraParams() const {
    return camParams;
}

bool CameraCalibration::calibrateCamera(const std::vector<std::string>& imageFiles) {
    std::vector<cv::Mat> images;
    for(const auto& file : imageFiles) {
        cv::Mat img = cv::imread(file);
        if(img.empty()) {
            std::cerr << "Failed to load image: " << file << std::endl;
            continue;
        }
        images.push_back(img);
    }
    return calibrateCamera(images);
}

bool CameraCalibration::calibrateCamera(const std::vector<cv::Mat>& images) {
    if(images.empty()) {
        std::cerr << "No images provided for calibration" << std::endl;
        return false;
    }

    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> imagePoints;
    cv::Size imageSize = images[0].size();

    // 生成标定板的三维点
    std::vector<cv::Point3f> obj;
    generateObjectPoints(obj);

    // 检测所有图像中的角点
    for(const auto& image : images) {
        std::vector<cv::Point2f> corners;
        bool found = detectChessboardCorners(image, corners);

        if(found) {
            imagePoints.push_back(corners);
            objectPoints.push_back(obj);

#ifdef DEBUG_MODE
            cv::Mat debugImg = image.clone();
            drawPoints(debugImg, corners);
            showImage("Corners", debugImg, 100);
#endif
        }
    }

    if(imagePoints.empty()) {
        std::cerr << "No corners detected in any image" << std::endl;
        return false;
    }

    // 执行标定
    camParams.rVecs.clear();
    camParams.tVecs.clear();
    camParams.reprojectionError = cv::calibrateCamera(objectPoints, imagePoints, imageSize,
                                                      camParams.cameraMatrix, camParams.distCoeffs,
                                                      camParams.rVecs, camParams.tVecs);

    calibrated = true;
    return true;
}

bool CameraCalibration::loadCalibrationData(const std::string& filename) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if(!fs.isOpened()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return false;
    }

    fs["camera_matrix"] >> camParams.cameraMatrix;
    fs["distortion_coefficients"] >> camParams.distCoeffs;
    fs["reprojection_error"] >> camParams.reprojectionError;

    calibrated = !camParams.cameraMatrix.empty();
    fs.release();
    return calibrated;
}

bool CameraCalibration::saveCalibrationData(const std::string& filename) const {
    if(!calibrated) {
        std::cerr << "No calibration data to save" << std::endl;
        return false;
    }

    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if(!fs.isOpened()) {
        std::cerr << "Failed to create file: " << filename << std::endl;
        return false;
    }

    fs << "camera_matrix" << camParams.cameraMatrix;
    fs << "distortion_coefficients" << camParams.distCoeffs;
    fs << "reprojection_error" << camParams.reprojectionError;

    fs.release();
    return true;
}

cv::Mat CameraCalibration::undistortImage(const cv::Mat& src) const {
    if(!calibrated) {
        std::cerr << "Camera not calibrated" << std::endl;
        return src;
    }

    cv::Mat dst;
    cv::undistort(src, dst, camParams.cameraMatrix, camParams.distCoeffs);
    return dst;
}

bool CameraCalibration::detectFeaturePoints(const cv::Mat& image,
                                            std::vector<cv::Point2f>& points,
                                            const std::string& method) {
    if(method == "CHESSBOARD") {
        return detectChessboardCorners(image, points);
    }
    else if(method == "CIRCLE") {
        return detectCircles(image, points);
    }
    return false;
}

cv::Mat CameraCalibration::preprocessImage(const cv::Mat& image) {
    cv::Mat gray, processed;

    // 步骤1: 转换为灰度图
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }
    else {
        gray = image.clone();
    }

    // 步骤2: 去噪 - 使用双边滤波保持边缘清晰
    cv::Mat denoised;
    cv::bilateralFilter(gray, denoised, 9, 75, 75);

    // 步骤3: 对比度增强 - CLAHE自适应直方图均衡
    cv::Mat enhanced;
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
    clahe->apply(denoised, enhanced);

    // 步骤4: 锐化处理 - 增强圆的边缘
    cv::Mat sharpened;
    cv::Mat sharpenKernel = (cv::Mat_<float>(3, 3) <<
        0, -1, 0,
        -1, 5, -1,
        0, -1, 0);
    cv::filter2D(enhanced, sharpened, -1, sharpenKernel);

    // 步骤5: 智能二值化 - 先二值化再形态学处理
    cv::Mat binary;
    cv::Scalar meanVal = cv::mean(sharpened);
    cv::Mat stddevMat;
    cv::meanStdDev(sharpened, cv::noArray(), stddevMat);
    double contrast = stddevMat.at<double>(0, 0);

    if (contrast > 50) {
        // 高对比度 - 使用Otsu自动阈值
        cv::threshold(sharpened, binary, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
    }
    else if (meanVal[0] > 100) {
        // 亮背景暗圆 - 反向自适应阈值
        cv::adaptiveThreshold(sharpened, binary, 255,
            cv::ADAPTIVE_THRESH_GAUSSIAN_C,
            cv::THRESH_BINARY_INV, 15, 10);
    }
    else {
        // 暗背景亮圆 - 正向自适应阈值
        cv::adaptiveThreshold(sharpened, binary, 255,
            cv::ADAPTIVE_THRESH_GAUSSIAN_C,
            cv::THRESH_BINARY, 15, 10);
    }

    // 检查二值化结果
    int whitePixels = cv::countNonZero(binary);
    int totalPixels = binary.rows * binary.cols;
    double whiteRatio = static_cast<double>(whitePixels) / totalPixels;

    // 如果白色像素太少或太多，调整二值化策略
    if (whiteRatio < 0.01 || whiteRatio > 0.99) {
        // 使用简单的固定阈值
        cv::threshold(sharpened, binary, 127, 255, cv::THRESH_BINARY);

        // 重新检查
        whitePixels = cv::countNonZero(binary);
        whiteRatio = static_cast<double>(whitePixels) / totalPixels;

        if (whiteRatio < 0.25) {
            // 如果还是太少，降低阈值
            cv::threshold(sharpened, binary, 35, 255, cv::THRESH_BINARY);
        }
        else if (whiteRatio > 0.75) {
            // 如果太多，提高阈值
            cv::threshold(sharpened, binary, 180, 255, cv::THRESH_BINARY);
        }
    }

    // 步骤6: 轻微的形态学处理 - 只去除很小的噪声
    cv::Mat morphed;
    // 使用更小的核，避免把目标也去掉
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4, 4)); // 改为2x2
    cv::morphologyEx(binary, morphed, cv::MORPH_OPEN, element);

    // 检查形态学处理后是否还有内容
    int remainingPixels = cv::countNonZero(morphed);
    if (remainingPixels < whitePixels * 0.1) {
        // 如果形态学处理去掉了太多内容，跳过这一步
        morphed = binary.clone();
    }

    // 步骤7: 清除边界噪声
    cv::Mat cleaned = binary.clone();
    int borderSize = std::max(2, (int)(std::min(cleaned.rows, cleaned.cols) * 0.02));

    // 清除四个边界的噪声
    cv::rectangle(cleaned, cv::Rect(0, 0, cleaned.cols, borderSize), cv::Scalar(0), -1);
    cv::rectangle(cleaned, cv::Rect(0, cleaned.rows - borderSize, cleaned.cols, borderSize), cv::Scalar(0), -1);
    cv::rectangle(cleaned, cv::Rect(0, 0, borderSize, cleaned.rows), cv::Scalar(0), -1);
    cv::rectangle(cleaned, cv::Rect(cleaned.cols - borderSize, 0, borderSize, cleaned.rows), cv::Scalar(0), -1);

    // 步骤8: 最终形态学闭运算 - 填补圆内的小孔洞
    cv::Mat final;
    cv::Mat closeElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(cleaned, final, cv::MORPH_CLOSE, closeElement);

    //// 可选：调试显示
    //    // 创建多步骤显示图像
    //    cv::Mat debugDisplay;
    //    std::vector<cv::Mat> steps = { gray, denoised, enhanced, sharpened, binary, final };
    //    std::vector<std::string> labels = { "Original", "Denoised", "Enhanced", "Sharpened", "Binary", "Final" };

    //    // 调整图像大小用于显示
    //    int displayHeight = 200;
    //    std::vector<cv::Mat> resizedSteps;
    //    for (const auto& step : steps) {
    //        cv::Mat resized;
    //        double scale = static_cast<double>(displayHeight) / step.rows;
    //        cv::resize(step, resized, cv::Size(), scale, scale);

    //        // 添加标签
    //        cv::Mat labeled = resized.clone();
    //        if (labeled.channels() == 1) {
    //            cv::cvtColor(labeled, labeled, cv::COLOR_GRAY2BGR);
    //        }
    //        cv::putText(labeled, labels[resizedSteps.size()],
    //            cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX,
    //            0.6, cv::Scalar(0, 255, 0), 2);
    //        resizedSteps.push_back(labeled);
    //    }

    //    // 水平拼接显示
    //        cv::imshow("1-Original", gray);
    //        cv::waitKey(1);

    //        cv::imshow("2-Denoised", denoised);
    //        cv::waitKey(1);

    //        cv::imshow("3-Enhanced", enhanced);
    //        cv::waitKey(1);

    //        cv::imshow("4-Sharpened", sharpened);
    //        cv::waitKey(1);

    //        cv::imshow("5-Binary", binary);
    //        cv::waitKey(1);

    //        cv::imshow("6-Final", final);
    //        cv::waitKey(1);
    //

    return final;
}

bool CameraCalibration::detectChessboardCorners(const cv::Mat& image,
    std::vector<cv::Point2f>& corners) {
    cv::Mat gray;

    // 1. 高质量的图像预处理
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }
    else {
        gray = image.clone();
    }

    // 2. 增强棋盘格对比度
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    clahe->apply(gray, gray);

    // 3. 使用专门针对标定板的参数
    int flags = cv::CALIB_CB_ADAPTIVE_THRESH | // 使用自适应阈值
        cv::CALIB_CB_NORMALIZE_IMAGE;  // 归一化图像亮度

    // 4. 设置固定的棋盘格参数
    cv::Size patternSize(chessParams.boardSize); // 例如(9,6)

    // 5. 角点检测
    bool found = cv::findChessboardCorners(gray, patternSize, corners, flags);

    if (found) {
        // 6. 亚像素精确化
        // 使用较大的搜索窗口以提高精度
        cv::Size winSize(11, 11);
        cv::Size zeroZone(-1, -1);
        cv::TermCriteria criteria(
            cv::TermCriteria::EPS + cv::TermCriteria::COUNT,
            100,  // 最大迭代次数
            0.001 // 精度要求
        );

        cv::cornerSubPix(gray, corners, winSize, zeroZone, criteria);

        // 7. 验证角点排列的正确性
        if (!validateCornerArrangement(corners, patternSize)) {
            return false;
        }
    }

    return found;
}

// 验证角点排列是否正确
bool CameraCalibration::validateCornerArrangement(
    const std::vector<cv::Point2f>& corners,
    const cv::Size& patternSize) {

    if (corners.size() != patternSize.width * patternSize.height) {
        return false;
    }

    // 检查角点间距是否合理
    double avgSpacingX = 0, avgSpacingY = 0;
    int count = 0;

    // 计算水平方向平均间距
    for (int i = 0; i < patternSize.height; i++) {
        for (int j = 0; j < patternSize.width - 1; j++) {
            int idx = i * patternSize.width + j;
            double dx = corners[idx + 1].x - corners[idx].x;
            avgSpacingX += std::abs(dx);
            count++;
        }
    }
    avgSpacingX /= count;

    // 计算垂直方向平均间距
    count = 0;
    for (int i = 0; i < patternSize.height - 1; i++) {
        for (int j = 0; j < patternSize.width; j++) {
            int idx = i * patternSize.width + j;
            int idx2 = (i + 1) * patternSize.width + j;
            double dy = corners[idx2].y - corners[idx].y;
            avgSpacingY += std::abs(dy);
            count++;
        }
    }
    avgSpacingY /= count;

    // 检查每个角点的间距是否在合理范围内
    const double tolerance = 0.3; // 允许30%的偏差

    for (int i = 0; i < patternSize.height; i++) {
        for (int j = 0; j < patternSize.width - 1; j++) {
            int idx = i * patternSize.width + j;
            double dx = std::abs(corners[idx + 1].x - corners[idx].x);
            if (std::abs(dx - avgSpacingX) > avgSpacingX * tolerance) {
                return false;
            }
        }
    }

    for (int i = 0; i < patternSize.height - 1; i++) {
        for (int j = 0; j < patternSize.width; j++) {
            int idx = i * patternSize.width + j;
            int idx2 = (i + 1) * patternSize.width + j;
            double dy = std::abs(corners[idx2].y - corners[idx].y);
            if (std::abs(dy - avgSpacingY) > avgSpacingY * tolerance) {
                return false;
            }
        }
    }

    return true;
}


// 可以添加调试信息的功能
void CameraCalibration::drawDebugInfo(cv::Mat& image,
    const std::vector<cv::Point2f>& corners) {

    if (corners.empty()) return;

    // 绘制检测到的角点
    cv::drawChessboardCorners(image, chessParams.boardSize, corners, true);

    // 添加角点编号
    for (size_t i = 0; i < corners.size(); i++) {
        cv::putText(image, std::to_string(i),
            cv::Point(corners[i].x + 5, corners[i].y + 5),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    }

    // 添加检测质量信息
    std::string qualityInfo = "Detected Points: " + std::to_string(corners.size());
    cv::putText(image, qualityInfo, cv::Point(20, 30),
        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
}

//bool CameraCalibration::detectCircles(const cv::Mat& image,
//                                      std::vector<cv::Point2f>& centers) {
//    cv::Mat gray = preprocessImage(image); // 预处理图像
//    // showImage("Gray", gray, 100);        // 显示灰度图
//
//    cv::GaussianBlur(gray, gray, cv::Size(9, 9), 2, 2); // 高斯模糊去噪声
//    // showImage("Blurred", gray, 100);     // 显示模糊图像
//
//    std::vector<cv::Vec3f> circles;
//    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1,
//                     4000,  // 最小圆心距离
//                     45, 35,        // Canny 阈值
//                     550, 650);      // 半径范围
//
//    centers.clear();
//    for(const auto& circle : circles) {
//        centers.push_back(cv::Point2f(circle[0], circle[1]));
//    }
//
//    // showImage("Detected Circles", image, 100);  // 显示检测到的圆
//    // 输出结果
//    return !centers.empty();
//}
bool CameraCalibration::detectCircles(const cv::Mat& image,
    std::vector<cv::Point2f>& centers) {
    gray = preprocessImage(image);

    // 自适应高斯模糊 - 根据图像尺寸调整
    int blurSize = std::max(5, static_cast<int>(image.cols * 0.01)) | 1; // 确保为奇数
    cv::GaussianBlur(gray, gray, cv::Size(blurSize, blurSize), 0);

    std::vector<cv::Vec3f> circles;


    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT,
        2,           // 累加器分辨率比例
        500,      // 最小圆心距离  
        75,       // Canny高阈值
        65,       // 累加器阈值
        350,    // 最小半径
        500);   // 最大半径

    // 后处理：过滤和排序
    circles = postProcessCircles(circles, image.size());

    centers.clear();
    centers.reserve(circles.size());

    for (const auto& circle : circles) {
        centers.emplace_back(circle[0], circle[1]);
    }

    // 可选：绘制检测结果用于调试
    drawDetectedCircles(image, circles);
    

    return !centers.empty();
}

// 后处理：去除重复和无效的圆，验证圆的内部特征
std::vector<cv::Vec3f> CameraCalibration::postProcessCircles(const std::vector<cv::Vec3f>& circles,
    const cv::Size& imageSize) {

    std::vector<cv::Vec3f> filtered;

    for (const auto& circle : circles) {
        cv::Point2f center(circle[0], circle[1]);
        float radius = circle[2];

        // 1. 边界检查：确保圆完全在图像内
        if (center.x - radius < 0 || center.y - radius < 0 ||
            center.x + radius >= imageSize.width || center.y + radius >= imageSize.height) {
            continue;
        }

        // 2. 内部一致性检查：验证圆内部颜色是否一致
        if (!validateCircleInterior(gray, center, radius)) {
            continue;
        }



        // 4. 重复检查：避免检测到相似的圆
        bool isDuplicate = false;
        for (const auto& existing : filtered) {
            cv::Point2f existingCenter(existing[0], existing[1]);
            float distance = cv::norm(center - existingCenter);
            float radiusDiff = std::abs(radius - existing[2]);

            // 如果圆心距离小于半径的一半且半径相似，认为是重复
            if (distance < std::max(radius, existing[2]) * 0.5 &&
                radiusDiff < std::max(radius, existing[2]) * 0.3) {
                isDuplicate = true;
                break;
            }
        }

        if (!isDuplicate) {
            filtered.push_back(circle);
        }
    }

    // 按圆的质量评分排序
    std::sort(filtered.begin(), filtered.end(),
        [&](const cv::Vec3f& a, const cv::Vec3f& b) {
            float scoreA = calculateCircleQuality(gray, cv::Point2f(a[0], a[1]), a[2]);
            float scoreB = calculateCircleQuality(gray, cv::Point2f(b[0], b[1]), b[2]);
            return scoreA > scoreB;
        });

    return filtered;
}
// 验证圆内部颜色一致性
bool CameraCalibration::validateCircleInterior(const cv::Mat& image, const cv::Point2f& center, float radius) {
    // 在圆内部采样多个点
    std::vector<uchar> interiorPixels;
    int sampleRadius = static_cast<int>(radius * 0.4); // 采样内圆，避免边缘影响

    // 采样策略：同心圆采样
    for (int r = 5; r < sampleRadius; r += 5) {
        int numSamples = std::max(8, r * 2); // 根据半径调整采样点数
        for (int i = 0; i < numSamples; ++i) {
            float angle = 2 * CV_PI * i / numSamples;
            int x = static_cast<int>(center.x + r * std::cos(angle));
            int y = static_cast<int>(center.y + r * std::sin(angle));

            if (x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
                interiorPixels.push_back(image.at<uchar>(y, x));
            }
        }
    }

    if (interiorPixels.size() < 10) return false; // 采样点太少

    // 计算内部像素的统计特征
    cv::Mat pixelMat(interiorPixels);
    cv::Scalar meanVal, stdVal;
    cv::meanStdDev(pixelMat, meanVal, stdVal);

    // 检查一致性：标准差应该较小
    double consistency = stdVal[0];
    double threshold = 100; // 可调整的一致性阈值

    // 检查是否主要是黑色（对于黑色圆点）或白色（对于白色圆点）
    bool isBlackCircle = meanVal[0] < 100;  // 平均值小于100认为是黑圆
    bool isWhiteCircle = meanVal[0] > 155;  // 平均值大于155认为是白圆

    return (consistency < threshold) && (isBlackCircle || isWhiteCircle);
}



// 计算指定点的梯度强度
float CameraCalibration::calculateGradientMagnitude(const cv::Mat& image, const cv::Point2f& point) {
    int x = static_cast<int>(point.x);
    int y = static_cast<int>(point.y);

    // Sobel算子计算梯度
    float gx = (image.at<uchar>(y, x + 1) - image.at<uchar>(y, x - 1)) / 2.0f;
    float gy = (image.at<uchar>(y + 1, x) - image.at<uchar>(y - 1, x)) / 2.0f;

    return std::sqrt(gx * gx + gy * gy);
}

// 计算圆的整体质量评分
float CameraCalibration::calculateCircleQuality(const cv::Mat& image, const cv::Point2f& center, float radius) {
    float interiorScore = 0.0f;
    float edgeScore = 0.0f;
    float sizeScore = 0.0f;

    // 1. 内部一致性评分 (0-1)
    std::vector<uchar> interiorPixels;
    int sampleRadius = static_cast<int>(radius * 0.6);

    for (int r = 5; r < sampleRadius; r += 8) {
        int numSamples = r * 2;
        for (int i = 0; i < numSamples; ++i) {
            float angle = 2 * CV_PI * i / numSamples;
            int x = static_cast<int>(center.x + r * std::cos(angle));
            int y = static_cast<int>(center.y + r * std::sin(angle));

            if (x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
                interiorPixels.push_back(image.at<uchar>(y, x));
            }
        }
    }

    if (!interiorPixels.empty()) {
        cv::Mat pixelMat(interiorPixels);
        cv::Scalar meanVal, stdVal;
        cv::meanStdDev(pixelMat, meanVal, stdVal);
        interiorScore = std::max(0.0f, 1.0f - static_cast<float>(stdVal[0]) / 50.0f);
    }

    // 2. 边缘强度评分 (0-1)
    std::vector<float> edgeGradients;
    int numSamples = 24;

    for (int i = 0; i < numSamples; ++i) {
        float angle = 2 * CV_PI * i / numSamples;
        cv::Point2f edgePoint(center.x + radius * std::cos(angle),
            center.y + radius * std::sin(angle));

        if (edgePoint.x >= 1 && edgePoint.x < image.cols - 1 &&
            edgePoint.y >= 1 && edgePoint.y < image.rows - 1) {
            float gradient = calculateGradientMagnitude(image, edgePoint);
            edgeGradients.push_back(gradient);
        }
    }

    if (!edgeGradients.empty()) {
        float avgGradient = std::accumulate(edgeGradients.begin(), edgeGradients.end(), 0.0f) / edgeGradients.size();
        edgeScore = std::min(1.0f, avgGradient / 50.0f);
    }


    // 综合评分
    return interiorScore * 0.4f + edgeScore * 0.4f + sizeScore * 0.2f;
}



// 调试用：绘制检测到的圆
void CameraCalibration::drawDetectedCircles(const cv::Mat& image, const std::vector<cv::Vec3f>& circles) {
    cv::Mat display = image.clone();

    for (size_t i = 0; i < circles.size(); ++i) {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);

        // 绘制圆心
        cv::circle(display, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
        // 绘制圆周
        cv::circle(display, center, radius, cv::Scalar(0, 0, 255), 2, 8, 0);

        // 标注序号
        cv::putText(display, std::to_string(i),
            cv::Point(center.x + 5, center.y - 5),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
    }

    showImage("Detected Circles", display, 100);
}

std::vector<cv::Point3f> CameraCalibration::pixel2Camera(
    const std::vector<cv::Point2f>& pixelPoints,
    double Z_depth_mm) const {

    std::vector<cv::Point3f> cameraPoints;

    // 1. 获取相机内参
    double fx = camParams.cameraMatrix.at<double>(0, 0);
    double fy = camParams.cameraMatrix.at<double>(1, 1);
    double cx = camParams.cameraMatrix.at<double>(0, 2);
    double cy = camParams.cameraMatrix.at<double>(1, 2);

    // 2. 对每个像素点进行转换
    for (const auto& pt : pixelPoints) {
        // 考虑畸变校正
        cv::Point2f undistorted_pt;
        std::vector<cv::Point2f> src_pt = { pt };
        std::vector<cv::Point2f> dst_pt;
        cv::undistortPoints(src_pt, dst_pt, camParams.cameraMatrix,
            camParams.distCoeffs);
        undistorted_pt = dst_pt[0];

        // 计算归一化平面坐标
        double x_normalized = undistorted_pt.x;
        double y_normalized = undistorted_pt.y;
        // 计算相机坐标系下的3D坐标
        double X = x_normalized * Z_depth_mm;
        double Y = y_normalized * Z_depth_mm;

        cameraPoints.push_back(cv::Point3f(X, Y, Z_depth_mm));
    }

    return cameraPoints;
}


std::vector<cv::Point3f> CameraCalibration::camera2Base(
    const std::vector<cv::Point3f>& cameraPoints,
    const Eigen::Matrix4d& T_base_camera) const {
    std::vector<cv::Point3f> basePoints;



    for (const auto& pt : cameraPoints) {
        // 使用Eigen进行计算
        Eigen::Vector3d pt_cam_homo(pt.x, pt.y, pt.z);

        // 构建相机相对于末端执行器的变换矩阵
        Eigen::Matrix4d T_end_camera = Eigen::Matrix4d::Identity();
        T_end_camera.block<3, 1>(0, 3) = pt_cam_homo;  // 设置平移部分

        // 计算变换后的矩阵
        Eigen::Matrix4d T_result = T_base_camera * T_end_camera;

        // 从结果矩阵中提取平移部分作为基坐标系中的点
        Eigen::Vector3d pt_base_homo = T_result.block<3, 1>(0, 3);

        cv::Point3f base_pt(
            pt_base_homo(0),
            pt_base_homo(1),
            pt_base_homo(2)
        );

        std::cout << "Camera point: " << pt << std::endl;
        std::cout << "Base point: " << base_pt << std::endl << std::endl;

        basePoints.push_back(base_pt);
    }

    return basePoints;
}

bool CameraCalibration::isCalibrated() const {
    return calibrated;
}

double CameraCalibration::getReprojectionError() const {
    return camParams.reprojectionError;
}

void CameraCalibration::showImage(const std::string& winname,
                                  const cv::Mat& img,
                                  int delay) {
    cv::imshow(winname, img);
    cv::waitKey(delay);
}

void CameraCalibration::drawPoints(cv::Mat& image,
                                   const std::vector<cv::Point2f>& points,
                                   const cv::Scalar& color) {
    for(const auto& pt : points) {
        cv::circle(image, pt, 3, color, -1);
    }
}

cv::Mat CameraCalibration::preprocessImage(const cv::Mat& image) const {
    cv::Mat gray;
    if(image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image.clone();
    }
    return gray;
}

void CameraCalibration::generateObjectPoints(std::vector<cv::Point3f>& objectPoints) const {
    objectPoints.clear();
    for(int i = 0; i < chessParams.boardSize.height; i++) {
        for(int j = 0; j < chessParams.boardSize.width; j++) {
            objectPoints.push_back(cv::Point3f(j * chessParams.squareSize,
                                               i * chessParams.squareSize, 0));
        }
    }
}

void CameraCalibration::visualizeResults(const cv::Mat& image,
    const std::vector<cv::Point2f>& imagePoints,
    const std::vector<cv::Point3f>& worldPoints,
    const cv::Mat& T_base_camera) {
    try {
        if (image.empty() || imagePoints.empty()) {
            std::cerr << "Invalid input for visualization" << std::endl;
            return;
        }

        // 创建可视化窗口
        cv::namedWindow("Calibration Results", cv::WINDOW_NORMAL);

        // 创建显示图像的副本
        cv::Mat displayImg = image.clone();
        if (displayImg.channels() == 1) {
            cv::cvtColor(displayImg, displayImg, cv::COLOR_GRAY2BGR);
        }

        // 绘制检测到的点
        drawPointsWithLabels(displayImg, imagePoints, worldPoints);

        // 如果有变换矩阵，绘制坐标系
        if (!T_base_camera.empty()) {
            cv::Mat rvec, tvec;
            cv::Rodrigues(T_base_camera(cv::Rect(0, 0, 3, 3)), rvec);
            tvec = T_base_camera(cv::Rect(3, 0, 1, 3));
            drawCoordinateSystem(displayImg, rvec, tvec);
        }

        // 创建信息面板
        cv::Mat infoPanel = createInfoPanel(imagePoints, worldPoints);

        // 调整信息面板大小以匹配显示图像的高度
        cv::Mat resizedInfoPanel;
        double scale = static_cast<double>(displayImg.rows) / infoPanel.rows;
        cv::resize(infoPanel, resizedInfoPanel,
            cv::Size(static_cast<int>(infoPanel.cols * scale), displayImg.rows));

        // 检查并确保两个图像的高度相同
        if (displayImg.rows != resizedInfoPanel.rows) {
            std::cerr << "Image heights don't match after resize" << std::endl;
            // 显示原始图像作为备选
            cv::imshow("Calibration Results", displayImg);
            cv::waitKey(1);
            return;
        }
        cv::imshow("Calibration Results", displayImg);

        //    // 合并显示图像和信息面板
        //    cv::Mat finalDisplay;
        //    try {
        //        cv::hconcat(std::vector<cv::Mat>{displayImg, resizedInfoPanel}, finalDisplay);
        //    }
        //    catch (const cv::Exception& e) {
        //        std::cerr << "Error concatenating images: " << e.what() << std::endl;
        //        std::cerr << "displayImg size: " << displayImg.size() <<
        //            " type: " << displayImg.type() << std::endl;
        //        std::cerr << "infoPanel size: " << resizedInfoPanel.size() <<
        //            " type: " << resizedInfoPanel.type() << std::endl;

        //        // 显示原始图像作为备选
        //        cv::imshow("Calibration Results", displayImg);
        //        cv::waitKey(1);
        //        return;
        //    }

        //    // 显示结果
        //    cv::imshow("Calibration Results", finalDisplay);
        //    cv::waitKey(1);
        //}
        //catch (const cv::Exception& e) {
        //    std::cerr << "OpenCV exception in visualizeResults: " << e.what() << std::endl;
        //}
        //catch (const std::exception& e) {
        //    std::cerr << "Standard exception in visualizeResults: " << e.what() << std::endl;
        //}
    }
    catch (...) {
        std::cerr << "Unknown exception in visualizeResults" << std::endl;
    }
}

void CameraCalibration::drawCoordinateSystem(cv::Mat& image,
                                             const cv::Mat& rvec,
                                             const cv::Mat& tvec,
                                             float length) {
    if(!calibrated) return;

    std::vector<cv::Point3f> axisPoints;
    axisPoints.push_back(cv::Point3f(0,0,0));
    axisPoints.push_back(cv::Point3f(length,0,0));  // X axis (Red)
    axisPoints.push_back(cv::Point3f(0,length,0));  // Y axis (Green)
    axisPoints.push_back(cv::Point3f(0,0,length));  // Z axis (Blue)

    std::vector<cv::Point2f> imageAxisPoints;
    cv::projectPoints(axisPoints, rvec, tvec,
                     camParams.cameraMatrix, camParams.distCoeffs,
                     imageAxisPoints);

    // 绘制坐标轴
    cv::line(image, imageAxisPoints[0], imageAxisPoints[1],
            cv::Scalar(0,0,255), 2);  // X axis - Red
    cv::line(image, imageAxisPoints[0], imageAxisPoints[2],
            cv::Scalar(0,255,0), 2);  // Y axis - Green
    cv::line(image, imageAxisPoints[0], imageAxisPoints[3],
            cv::Scalar(255,0,0), 2);  // Z axis - Blue

    // 添加坐标轴标签
    cv::putText(image, "X", imageAxisPoints[1],
               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,255), 2);
    cv::putText(image, "Y", imageAxisPoints[2],
               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,0), 2);
    cv::putText(image, "Z", imageAxisPoints[3],
               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,0,0), 2);
}

void CameraCalibration::drawPointsWithLabels(cv::Mat& image,
                                             const std::vector<cv::Point2f>& points,
                                             const std::vector<cv::Point3f>& worldPoints) {
    for(size_t i = 0; i < points.size(); i++) {
        // 绘制点
        cv::circle(image, points[i], 3, cv::Scalar(0,255,255), -1);

        // 添加标签
        std::string label;
        if(i < worldPoints.size()) {
            label = cv::format("P%d (%.1f,%.1f,%.1f)", (int)i,
                               worldPoints[i].x, worldPoints[i].y, worldPoints[i].z);
        } else {
            label = cv::format("P%d (%.1f,%.1f)", (int)i,
                               points[i].x, points[i].y);
        }

        cv::putText(image, label, points[i] + cv::Point2f(5,5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,255,255), 1);
    }
}

cv::Mat CameraCalibration::createInfoPanel(const std::vector<cv::Point2f>& imagePoints,
    const std::vector<cv::Point3f>& worldPoints) {
    // 创建白色背景的信息面板，确保与输入图像类型相同
    cv::Mat panel = cv::Mat(400, 300, CV_8UC3, cv::Scalar(255, 255, 255));

    try {
        int y = 30;
        const int lineHeight = 20;

        // 添加标题
        cv::putText(panel, "Calibration Information", cv::Point(10, y),
            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);
        y += 2 * lineHeight;

        // 添加相机内参信息
        if (calibrated) {
            cv::putText(panel, "Camera Matrix:", cv::Point(10, y),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
            y += lineHeight;

            for (int i = 0; i < 3; i++) {
                std::string matRow = cv::format("%.2f, %.2f, %.2f",
                    camParams.cameraMatrix.at<double>(i, 0),
                    camParams.cameraMatrix.at<double>(i, 1),
                    camParams.cameraMatrix.at<double>(i, 2));
                cv::putText(panel, matRow, cv::Point(10, y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
                y += lineHeight;
            }
            y += lineHeight;

            // 添加重投影误差
            std::string errorText = cv::format("Reprojection Error: %.3f",
                camParams.reprojectionError);
            cv::putText(panel, errorText, cv::Point(10, y),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
        }

        // 添加检测点信息
        y += 2 * lineHeight;
        std::string pointsText = cv::format("Detected Points: %d", (int)imagePoints.size());
        cv::putText(panel, pointsText, cv::Point(10, y),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    }
    catch (const cv::Exception& e) {
        std::cerr << "Error creating info panel: " << e.what() << std::endl;
        // 返回一个简单的错误信息面板
        cv::putText(panel, "Error creating info panel", cv::Point(10, 30),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    }

    return panel;
}

// 手眼标定

bool CameraCalibration::performHandEyeCalibration(const std::vector<cv::Mat>& images,
                                                  const std::vector<cv::Mat>& robotPoses) {
    // 1. 基本验证
    if(images.size() != robotPoses.size()) {
        std::cerr << "图像数量和机器人位姿数量不匹配" << std::endl;
        return false;
    }

    if(images.size() < 3) {
        std::cerr << "手眼标定至少需要3组图像-位姿对" << std::endl;
        return false;
    }

    if(!calibrated) {
        std::cerr << "请先完成相机标定" << std::endl;
        return false;
    }

    // 2. 验证所有机器人位姿
    for(size_t i = 0; i < robotPoses.size(); i++) {
        if(!validateRobotPose(robotPoses[i])) {
            std::cerr << "第" << i+1 << "个机器人位姿无效" << std::endl;
            return false;
        }
    }

    // 3. 计算每个图像中标定板相对于相机的位姿
    std::vector<cv::Mat> R_gripper2base, t_gripper2base;  // 机器人位姿
    std::vector<cv::Mat> R_target2cam, t_target2cam;      // 标定板到相机

    std::cout << "开始处理图像和位姿..." << std::endl;

    for(size_t i = 0; i < images.size(); i++) {
        std::cout << "处理第" << (i+1) << "/" << images.size() << "张图像" << std::endl;

        // 使用现有函数计算标定板位姿
        cv::Mat boardPose = solvePnPForImage(images[i]);
        if(boardPose.empty()) {
            std::cerr << "第" << i +1<< "张图像未能检测到标定板" << std::endl;
            continue;
        }

        // 提取机器人位姿的旋转和平移
        cv::Mat R_robot = robotPoses[i](cv::Rect(0, 0, 3, 3)).clone();
        cv::Mat t_robot = robotPoses[i](cv::Rect(3, 0, 1, 3)).clone();

        R_gripper2base.push_back(R_robot);
        t_gripper2base.push_back(t_robot);

        // 提取标定板位姿的旋转和平移
        cv::Mat R_board = boardPose(cv::Rect(0, 0, 3, 3)).clone();
        cv::Mat t_board = boardPose(cv::Rect(3, 0, 1, 3)).clone();

        R_target2cam.push_back(R_board);
        t_target2cam.push_back(t_board);
    }

    // 4. 执行手眼标定 (使用TSAI方法)
    cv::Mat R_cam2gripper, t_cam2gripper;

    try {
        std::cout << "执行手眼标定..." << std::endl;
        cv::calibrateHandEye(R_gripper2base, t_gripper2base,
                             R_target2cam, t_target2cam,
                             R_cam2gripper, t_cam2gripper,
                             cv::CALIB_HAND_EYE_TSAI);
    }
    catch(const cv::Exception& e) {
        std::cerr << "手眼标定失败: " << e.what() << std::endl;
        return false;
    }

    // 5. 构建手眼变换矩阵
    handEyeData.handEyeTransform = cv::Mat::eye(4, 4, CV_64F);
    R_cam2gripper.copyTo(handEyeData.handEyeTransform(cv::Rect(0, 0, 3, 3)));
    t_cam2gripper.copyTo(handEyeData.handEyeTransform(cv::Rect(3, 0, 1, 3)));

    // 6. 计算并输出修正后的位姿矩阵
    std::cout << "\n========== 手眼标定修正结果 ==========" << std::endl;
    std::cout << "手眼变换矩阵 (相机到夹具):" << std::endl;
    printMatrix(handEyeData.handEyeTransform, "T_cam2gripper");

    // 计算逆变换矩阵 (夹具到相机)
    cv::Mat T_gripper2cam;
    cv::invert(handEyeData.handEyeTransform, T_gripper2cam);
    std::cout << "\n逆变换矩阵 (夹具到相机):" << std::endl;
    printMatrix(T_gripper2cam, "T_gripper2cam");

    // 输出修正后的位姿对比
    std::cout << "\n========== 位姿修正前后对比 ==========" << std::endl;
    for (size_t i = 0; i < robotPoses.size() && i < R_gripper2base.size(); i++) {
        // 计算修正后的位姿
        cv::Mat correctedPose = calculateCorrectedPose(robotPoses[i],
            handEyeData.handEyeTransform);

        std::cout << "\n--- 第" << (i + 1) << "组位姿 ---" << std::endl;
        std::cout << "原始机器人位姿:" << std::endl;
        printMatrix(robotPoses[i], "Original_Pose_" + std::to_string(i + 1));

        std::cout << "修正后位姿:" << std::endl;
        printMatrix(correctedPose, "Corrected_Pose_" + std::to_string(i + 1));

        // 计算位姿修正量
        cv::Mat poseDiff = calculatePoseDifference(robotPoses[i], correctedPose);
        std::cout << "位姿修正量:" << std::endl;
        printMatrix(poseDiff, "Pose_Correction_" + std::to_string(i + 1));
    }

    // 7. 计算标定精度评估
    double totalError = 0.0;
    double maxError = 0.0;
    std::vector<double> errors;

    std::cout << "\n========== 标定精度评估 ==========" << std::endl;
    for (size_t i = 0; i < R_gripper2base.size(); i++) {
        double error = calculateReprojectionError(R_gripper2base[i], t_gripper2base[i],
            R_target2cam[i], t_target2cam[i],
            handEyeData.handEyeTransform);
        errors.push_back(error);
        totalError += error;
        maxError = std::max(maxError, error);

        std::cout << "第" << (i + 1) << "组重投影误差: " << error << " mm" << std::endl;
    }

    handEyeData.calibrationError = totalError / errors.size();
    handEyeData.isCalibrated = true;

    std::cout << "\n========== 标定结果汇总 ==========" << std::endl;
    std::cout << "平均重投影误差: " << handEyeData.calibrationError << " mm" << std::endl;
    std::cout << "最大重投影误差: " << maxError << " mm" << std::endl;
    std::cout << "手眼标定完成！" << std::endl;

    return true;
}

// 辅助函数：格式化输出矩阵
void CameraCalibration::printMatrix(const cv::Mat& matrix, const std::string& name) {
    std::cout << name << ":" << std::endl;
    for (int i = 0; i < matrix.rows; i++) {
        std::cout << "[";
        for (int j = 0; j < matrix.cols; j++) {
            std::cout << std::fixed << std::setprecision(6) << matrix.at<double>(i, j);
            if (j < matrix.cols - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }
    std::cout << std::endl;
}

// 辅助函数：计算修正后的位姿
cv::Mat CameraCalibration::calculateCorrectedPose(const cv::Mat& originalPose,
    const cv::Mat& handEyeTransform) {
    // 这里可以根据具体的修正策略来实现
    // 示例：应用手眼变换来修正位姿
    cv::Mat correctedPose = originalPose * handEyeTransform;
    return correctedPose;
}

// 辅助函数：计算位姿差异
cv::Mat CameraCalibration::calculatePoseDifference(const cv::Mat& pose1, const cv::Mat& pose2) {
    cv::Mat diff = cv::Mat::zeros(4, 4, CV_64F);

    // 计算平移差异
    cv::Mat t_diff = pose2(cv::Rect(3, 0, 1, 3)) - pose1(cv::Rect(3, 0, 1, 3));
    t_diff.copyTo(diff(cv::Rect(3, 0, 1, 3)));

    // 计算旋转差异 (简化版本)
    cv::Mat R1 = pose1(cv::Rect(0, 0, 3, 3));
    cv::Mat R2 = pose2(cv::Rect(0, 0, 3, 3));
    cv::Mat R_diff = R2 * R1.t();
    R_diff.copyTo(diff(cv::Rect(0, 0, 3, 3)));

    diff.at<double>(3, 3) = 1.0;
    return diff;
}

// 辅助函数：计算重投影误差
double CameraCalibration::calculateReprojectionError(const cv::Mat& R_gripper2base,
    const cv::Mat& t_gripper2base,
    const cv::Mat& R_target2cam,
    const cv::Mat& t_target2cam,
    const cv::Mat& handEyeTransform) {
    // 通过手眼关系验证一致性
    cv::Mat R_cam2gripper = handEyeTransform(cv::Rect(0, 0, 3, 3));
    cv::Mat t_cam2gripper = handEyeTransform(cv::Rect(3, 0, 1, 3));

    // 计算预测的标定板位姿
    cv::Mat R_predicted = R_cam2gripper.t() * R_gripper2base.t() * R_target2cam;
    cv::Mat t_predicted = R_cam2gripper.t() * (R_gripper2base.t() * t_target2cam - t_cam2gripper);

    // 计算误差（简化版本 - 仅考虑平移误差）
    cv::Mat error = t_predicted - t_target2cam;
    return cv::norm(error) * 1000; // 转换为毫米
}

cv::Mat CameraCalibration::solvePnPForImage(const cv::Mat& image) {
    // 使用现有的角点检测函数
    std::vector<cv::Point2f> imagePoints;
    bool found = detectChessboardCorners(image, imagePoints);

    if(!found) {
        return cv::Mat();
    }

    // 使用现有的生成三维点函数
    std::vector<cv::Point3f> objectPoints;
    generateObjectPoints(objectPoints);

    // 计算位姿
    cv::Mat rvec, tvec;
    bool success = cv::solvePnP(objectPoints, imagePoints,
                                camParams.cameraMatrix, camParams.distCoeffs,
                                rvec, tvec);

    if(!success) {
        return cv::Mat();
    }

    // 转换为4x4变换矩阵
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    cv::Mat pose = cv::Mat::eye(4, 4, CV_64F);
    R.copyTo(pose(cv::Rect(0, 0, 3, 3)));
    tvec.copyTo(pose(cv::Rect(3, 0, 1, 3)));

    return pose;
}

bool CameraCalibration::validateRobotPose(const cv::Mat& pose) const {
    if(pose.rows != 4 || pose.cols != 4) {
        return false;
    }

    if(pose.type() != CV_64F && pose.type() != CV_32F) {
        return false;
    }

    // 简单验证：检查最后一行是否接近[0,0,0,1]
    double val = pose.at<double>(3, 3);
    return std::abs(val - 1.0) < 0.01;
}

std::vector<cv::Point3f> CameraCalibration::pixelToRobotBase(
    const std::vector<cv::Point2f>& pixelPoints,
    const cv::Mat& currentRobotPose,
    double Z_depth_mm) const {

    if(!handEyeData.isCalibrated) {
        std::cerr << "手眼标定尚未完成" << std::endl;
        return {};
    }

    // 1. 复用现有函数：像素坐标转相机坐标
    std::vector<cv::Point3f> cameraPoints = pixel2Camera(pixelPoints, Z_depth_mm);

    // 2. 相机坐标转机器人基座坐标
    std::vector<cv::Point3f> basePoints;

    for(const auto& pt : cameraPoints) {
        // 构建齐次坐标
        cv::Mat pt_cam = (cv::Mat_<double>(4, 1) << pt.x, pt.y, pt.z, 1.0);

        // 变换: 相机 -> 末端执行器 -> 机器人基座
        cv::Mat pt_base = currentRobotPose * handEyeData.handEyeTransform * pt_cam;

        basePoints.push_back(cv::Point3f(
            pt_base.at<double>(0, 0),
            pt_base.at<double>(1, 0),
            pt_base.at<double>(2, 0)
            ));
    }

    return basePoints;
}

// 数据保存和加载
bool CameraCalibration::saveHandEyeData(const std::string& filename) const {
    if(!handEyeData.isCalibrated) {
        std::cerr << "没有手眼标定数据可保存" << std::endl;
        return false;
    }

    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if(!fs.isOpened()) {
        std::cerr << "无法创建文件: " << filename << std::endl;
        return false;
    }

    fs << "hand_eye_transform" << handEyeData.handEyeTransform;
    fs << "calibration_error" << handEyeData.calibrationError;
    fs << "is_calibrated" << handEyeData.isCalibrated;

    fs.release();
    return true;
}

bool CameraCalibration::loadHandEyeData(const std::string& filename) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if(!fs.isOpened()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return false;
    }

    fs["hand_eye_transform"] >> handEyeData.handEyeTransform;
    fs["calibration_error"] >> handEyeData.calibrationError;
    fs["is_calibrated"] >> handEyeData.isCalibrated;

    fs.release();
    return handEyeData.isCalibrated && !handEyeData.handEyeTransform.empty();
}

// 获取函数
CameraCalibration::HandEyeData CameraCalibration::getHandEyeData() const {
    return handEyeData;
}

cv::Mat CameraCalibration::getHandEyeTransform() const {
    return handEyeData.handEyeTransform;
}

bool CameraCalibration::isHandEyeCalibrated() const {
    return handEyeData.isCalibrated;
}
