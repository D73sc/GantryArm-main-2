#include "FeatureDetector.h"
#include <iostream>
#include <iomanip>
#include <numeric>
#include <stdexcept>
#include <cmath>  // 用于 sqrt, cos 等

// 构造函数：验证输入
FeatureDetector::FeatureDetector(const CameraParams& cam_params, const CalibrationResult& hand_eye_res)
    : cam_params_(cam_params), hand_eye_res_(hand_eye_res) {
    if (cam_params_.camera_matrix.empty() || cam_params_.dist_coeffs.empty()) {
        throw std::invalid_argument("相机内参无效 (camera_matrix 或 dist_coeffs 为空)");
    }
    std::cout << "FeatureDetector 初始化成功，重投影误差: " << std::fixed << std::setprecision(2)
        << cam_params_.reproj_error << std::endl;
}

// 图像预处理：独立实现 (去噪、增强、锐化、二值化)
cv::Mat FeatureDetector::preprocessImage(const cv::Mat& image) const {
    cv::Mat gray, processed;

    // 1. 转灰度
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

// 圆检测：独立 Hough + 后处理验证
bool FeatureDetector::detectCircles(const cv::Mat& image, std::vector<cv::Point2f>& centers, const DetectionParams& params)  {
    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }
    else {
        gray = image.clone();
    }

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
    circles = postProcessCircles(gray,circles, image.size());

    centers.clear();
    centers.reserve(circles.size());

    for (const auto& circle : circles) {
        centers.emplace_back(circle[0], circle[1]);
    }

    // 可选：绘制检测结果用于调试
    showImage("绘制检测结果", drawDetectedCircles(image, circles),1);


    return !centers.empty();
}

// 调试用：绘制检测到的圆
cv::Mat FeatureDetector::drawDetectedCircles(const cv::Mat& image, const std::vector<cv::Vec3f>& circles) {
    int max_width = 1920, max_height = 1080;
    cv::Mat display = image.clone();
    if (display.channels() == 1) {
        cv::cvtColor(display, display, cv::COLOR_GRAY2BGR);  // 确保彩色
    }

    // 步骤 1: 缩放图像以适应屏幕比例 (保持宽高比)
    double scale = 1.0;
    if (display.cols > max_width || display.rows > max_height) {
        double scale_w = static_cast<double>(max_width) / display.cols;
        double scale_h = static_cast<double>(max_height) / display.rows;
        scale = std::min(scale_w, scale_h);  // 取较小比例

        cv::Size new_size(static_cast<int>(display.cols * scale), static_cast<int>(display.rows * scale));
        cv::resize(display, display, new_size, 0, 0, cv::INTER_LINEAR);  // 线性插值
    }

    // 步骤 2: 缩放圆数据 (中心和半径)
    std::vector<cv::Vec3f> scaled_circles;
    scaled_circles.reserve(circles.size());
    for (const auto& circle : circles) {
        float scaled_x = circle[0] * scale;
        float scaled_y = circle[1] * scale;
        float scaled_r = circle[2] * scale;
        scaled_circles.push_back(cv::Vec3f(scaled_x, scaled_y, scaled_r));
    }

    // 步骤 3: 绘制缩放后的圆
    for (size_t i = 0; i < scaled_circles.size(); ++i) {
        cv::Point center(cvRound(scaled_circles[i][0]), cvRound(scaled_circles[i][1]));
        int radius = cvRound(scaled_circles[i][2]);
        int center_radius = static_cast<int>(3 * scale);  // 缩放圆心大小
        int line_thickness = static_cast<int>(2 * scale);  // 缩放线宽

        // 绘制圆心 (绿色填充)
        cv::circle(display, center, center_radius, cv::Scalar(0, 255, 0), -1, 8, 0);

        // 绘制圆周 (红色线条)
        cv::circle(display, center, radius, cv::Scalar(0, 0, 255), line_thickness, 8, 0);

        // 标注序号 (黄色文本，缩放位置和字体)
        float font_scale = std::max(0.3f, static_cast<float>(scale * 0.5f));
        int text_thickness = static_cast<int>(1 * scale);
        cv::Point text_pos(center.x + static_cast<int>(5 * scale), center.y - static_cast<int>(5 * scale));
        cv::putText(display, std::to_string(i), text_pos, cv::FONT_HERSHEY_SIMPLEX,
            font_scale, cv::Scalar(255, 255, 0), text_thickness);
    }

    // 步骤 4: 添加总体信息 (左上角，蓝色)
    std::string info = "Detected Circles: " + std::to_string(circles.size()) +
        " (Scaled: " + std::to_string(static_cast<int>(scale * 100)) + "%)";
    float info_font_scale = std::max(0.5f, static_cast<float>(scale * 0.8f));
    cv::putText(display, info, cv::Point(static_cast<int>(10 * scale), static_cast<int>(30 * scale)),
        cv::FONT_HERSHEY_SIMPLEX, info_font_scale, cv::Scalar(0, 0, 255),
        static_cast<int>(2 * scale));

    // 步骤 5: 添加屏幕适应提示 (底部，灰色)
    std::string scale_tip = "Adapted to screen (" + std::to_string(max_width) + "x" + std::to_string(max_height) + ")";
    cv::putText(display, scale_tip, cv::Point(static_cast<int>(10 * scale),
        static_cast<int>(display.rows - static_cast<int>(10 * scale))),
        cv::FONT_HERSHEY_SIMPLEX, info_font_scale * 0.7f, cv::Scalar(128, 128, 128), 1);

    return display;
}

void FeatureDetector::showImage(const std::string& winname,
    const cv::Mat& img,
    int delay) {
    int max_width = 1920, max_height = 1080;

    cv::Mat display_img = img.clone();  // 复制以避免修改原图

    // 步骤 1: 缩放图像 (如果需要)
    if (display_img.cols > max_width || display_img.rows > max_height) {
        double scale_w = static_cast<double>(max_width) / display_img.cols;
        double scale_h = static_cast<double>(max_height) / display_img.rows;
        double scale = std::min(scale_w, scale_h);

        cv::Size new_size(static_cast<int>(display_img.cols * scale), static_cast<int>(display_img.rows * scale));
        cv::resize(display_img, display_img, new_size, 0, 0, cv::INTER_LINEAR);
    }

    // 步骤 2: 创建自适应窗口并显示
    cv::namedWindow(winname, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);  // 正常窗口，保持比例
    cv::imshow(winname, display_img);

    // 步骤 3: 等待 (delay=0 为非阻塞，按键关闭)
    if (delay > 0) {
        cv::waitKey(delay);
    }
    else {
        cv::waitKey(0);  // 阻塞直到按键
    }

    // 可选：自动调整窗口大小适应图像 (如果图像小)
    cv::resizeWindow(winname, display_img.cols, display_img.rows);
}

// 后处理：去除重复和无效的圆，验证圆的内部特征
std::vector<cv::Vec3f> FeatureDetector::postProcessCircles(const cv::Mat gray,const std::vector<cv::Vec3f>& circles,
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
bool FeatureDetector::validateCircleInterior(const cv::Mat& image, const cv::Point2f& center, float radius) {
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

// 计算圆的整体质量评分
float FeatureDetector::calculateCircleQuality(const cv::Mat& image, const cv::Point2f& center, float radius) {
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

// 计算指定点的梯度强度
float FeatureDetector::calculateGradientMagnitude(const cv::Mat& image, const cv::Point2f& point) {
    int x = static_cast<int>(point.x);
    int y = static_cast<int>(point.y);

    // Sobel算子计算梯度
    float gx = (image.at<uchar>(y, x + 1) - image.at<uchar>(y, x - 1)) / 2.0f;
    float gy = (image.at<uchar>(y + 1, x) - image.at<uchar>(y - 1, x)) / 2.0f;

    return std::sqrt(gx * gx + gy * gy);
}

// 棋盘格角点检测：独立实现
bool FeatureDetector::detectChessboardCorners(const cv::Mat& image, std::vector<cv::Point2f>& corners, const ChessboardParams& ch_params) {
    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }
    else {
        gray = image.clone();
    }

    // CLAHE 增强 (针对棋盘格)
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    clahe->apply(gray, gray);

    // 3. 使用专门针对标定板的参数
    int flags = cv::CALIB_CB_ADAPTIVE_THRESH | // 使用自适应阈值
        cv::CALIB_CB_NORMALIZE_IMAGE;  // 归一化图像亮度

    // 4. 设置固定的棋盘格参数
    cv::Size patternSize(ch_params.board_size); // 例如(9,6)

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
        return true;
    }
    return false;
}

// 验证角点排列是否正确
bool FeatureDetector::validateCornerArrangement(
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

// 像素转相机 3D：使用内参反投影
std::vector<Point3D> FeatureDetector::pixelToCamera3D(const std::vector<cv::Point2f>& pixels, double z_depth) const {
    if (pixels.empty()) return {};

    std::vector<cv::Point2f> undistorted;
    cv::undistortPoints(pixels, undistorted, cam_params_.camera_matrix, cam_params_.dist_coeffs);

    std::vector<Point3D> camera_pts;
    camera_pts.reserve(pixels.size());
    for (const auto& pt : undistorted) {
        float x = static_cast<float>(pt.x * z_depth);
        float y = static_cast<float>(pt.y * z_depth);
        camera_pts.emplace_back(x, y, static_cast<float>(z_depth));
    }
    return camera_pts;
}

// PnP 计算位姿
Eigen::Matrix4d FeatureDetector::computePnPpose(const std::vector<cv::Point3f>& obj_pts, const std::vector<cv::Point2f>& img_pts) const {
    if (obj_pts.size() != img_pts.size() || obj_pts.empty()) {
        return Eigen::Matrix4d::Identity();
    }

    cv::Mat rvec, tvec;
    bool success = cv::solvePnP(obj_pts, img_pts, cam_params_.camera_matrix, cam_params_.dist_coeffs, rvec, tvec);

    if (!success) {
        std::cerr << "PnP 求解失败，返回单位矩阵" << std::endl;
        return Eigen::Matrix4d::Identity();
    }

    cv::Mat R;
    cv::Rodrigues(rvec, R);

    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            pose(i, j) = R.at<double>(i, j);
        }
        pose(i, 3) = tvec.at<double>(i, 0);
    }
    return pose;
}

// 相机点转基坐标点：使用手眼变换
Point3D FeatureDetector::cameraToBase(const Point3D& cam_point, const cv::Mat& robot_pose) const {
    if (!isHandEyeValid()) {
        std::cerr << "手眼标定无效，无法转换到基坐标" << std::endl;
        return cam_point;  // 回退
    }

    // 手眼变换 T_gripper_cam (从 CalibrationResult)
    cv::Mat hand_eye_cv = eigenToCvMat(hand_eye_res_.hand_eye_transform);

    // 构建齐次坐标
    cv::Mat pt_cam_h(4, 1, CV_64F, cv::Scalar(0));
    pt_cam_h.at<double>(0) = cam_point.x;
    pt_cam_h.at<double>(1) = cam_point.y;
    pt_cam_h.at<double>(2) = cam_point.z;
    pt_cam_h.at<double>(3) = 1.0;

    // 变换：pt_base = T_base_gripper * T_gripper_cam * pt_cam
    cv::Mat pt_base_h = robot_pose * hand_eye_cv * pt_cam_h;

    return Point3D(static_cast<float>(pt_base_h.at<double>(0)),
        static_cast<float>(pt_base_h.at<double>(1)),
        static_cast<float>(pt_base_h.at<double>(2)));
}

// 圆心检测主函数
bool FeatureDetector::detectAndComputeCircleCenters(const cv::Mat& image,
    std::vector<Point3D>& camera_coords,
    std::optional<std::vector<Point3D>>& base_coords_opt,
    const DetectionParams& params) {
    last_result_.success = false;
    last_result_.num_points = 0;
    last_result_.error_message.clear();
    last_result_.point_errors.clear();
    camera_coords.clear();

    if (image.empty()) {
        last_result_.error_message = "输入图像为空";
        return false;
    }

    // 步骤 1: 预处理 (可选)
    cv::Mat processed = preprocessImage(image);

    // 步骤 2: 检测 2D 圆心
    std::vector<cv::Point2f> centers_2d;
    bool detected = detectCircles(processed, centers_2d, params);

    if (!detected || centers_2d.empty()) {
        last_result_.error_message = "未检测到圆形特征";
        return false;
    }

    // 步骤 3: 像素转相机 3D
    camera_coords = pixelToCamera3D(centers_2d, params.circle_z_depth_mm);
    last_result_.num_points = camera_coords.size();

    // 步骤 4: 计算重投影误差 (简化：假设原点位姿，投影回像素)
    double total_error = 0.0;
    last_result_.point_errors.reserve(centers_2d.size());
    cv::Mat rvec_zero(3, 1, CV_64F, cv::Scalar(0));  // 无旋转
    cv::Mat tvec_zero(3, 1, CV_64F, cv::Scalar(0));  // 原点
    for (size_t i = 0; i < centers_2d.size(); ++i) {
        std::vector<cv::Point3f> obj_pt = { camera_coords[i] };
        std::vector<cv::Point2f> projected;
        cv::projectPoints(obj_pt, rvec_zero, tvec_zero, cam_params_.camera_matrix, cam_params_.dist_coeffs, projected);
        double err = cv::norm(centers_2d[i] - projected[0]);
        last_result_.point_errors.push_back(err);
        total_error += err;
    }
    last_result_.reproj_error_mm = total_error / centers_2d.size();  // 平均像素误差 (近似 mm，实际需像素尺寸校准)

    // 步骤 5: 可选基坐标转换
    cv::Mat robot_pose = params.current_robot_pose.empty() ? cv::Mat::eye(4, 4, CV_64F) : params.current_robot_pose;
    if (params.compute_base_coords && isHandEyeValid() && base_coords_opt) {
        std::vector<Point3D> base_coords;
        base_coords.reserve(camera_coords.size());
        for (const auto& pt : camera_coords) {
            base_coords.push_back(cameraToBase(pt, robot_pose));
        }
        *base_coords_opt = base_coords;
    }

    last_result_.success = true;
    std::cout << "圆心检测成功： " << last_result_.num_points << " 个点，平均误差: "
        << std::fixed << std::setprecision(2) << last_result_.reproj_error_mm << " 像素" << std::endl;

    // 步骤 6: 可视化
    if (params.visualize) {
        cv::Mat vis = visualizeDetection(image, centers_2d);
        cv::imshow("圆心检测结果", vis);
        cv::waitKey(1);
    }

    return true;
}

// 棋盘格检测主函数
bool FeatureDetector::detectAndComputeChessboardPoints(const cv::Mat& image,
    std::vector<cv::Point2f>& image_points,
    std::vector<Point3D>& world_coords,
    std::optional<Eigen::Matrix4d>& board_pose_camera_opt,
    std::optional<Eigen::Matrix4d>& board_pose_base_opt,
    const DetectionParams& params) {
    last_result_.success = false;
    last_result_.num_points = 0;
    last_result_.error_message.clear();
    last_result_.point_errors.clear();
    image_points.clear();
    world_coords.clear();

    if (image.empty()) {
        last_result_.error_message = "输入图像为空";
        return false;
    }



    // 步骤 2: 检测 2D 角点
    bool detected = detectChessboardCorners(image, image_points, params.chess_params);

    if (!detected || image_points.empty()) {
        last_result_.error_message = "未检测到棋盘格角点";
        return false;
    }

    // 步骤 3: 生成 3D 世界坐标 (基于棋盘几何)
    const auto& ch_params = params.chess_params;
    world_coords.reserve(image_points.size());
    for (int i = 0; i < ch_params.board_size.height; ++i) {
        for (int j = 0; j < ch_params.board_size.width; ++j) {
            float x = static_cast<float>(j * ch_params.square_size);
            float y = static_cast<float>(i * ch_params.square_size); 
            world_coords.emplace_back(x, y, 0.0f);
        }
    }
    last_result_.num_points = world_coords.size();

    // 步骤 4: 计算相机位姿 (PnP)
    Eigen::Matrix4d board_pose_cam = computePnPpose(world_coords, image_points);
    if (board_pose_camera_opt) {
        *board_pose_camera_opt = board_pose_cam;
    }

    // 步骤 5: 计算重投影误差
    cv::Mat rvec, tvec;
    cv::solvePnP(world_coords, image_points, cam_params_.camera_matrix, cam_params_.dist_coeffs, rvec, tvec);
    std::vector<cv::Point2f> projected;
    cv::projectPoints(world_coords, rvec, tvec, cam_params_.camera_matrix, cam_params_.dist_coeffs, projected);
    double total_reproj = 0.0;
    last_result_.point_errors.reserve(image_points.size());
    for (size_t i = 0; i < image_points.size(); ++i) {
        double err = cv::norm(image_points[i] - projected[i]);
        last_result_.point_errors.push_back(err);
        total_reproj += err;
    }
    last_result_.reproj_error_mm = total_reproj / image_points.size();

    // 步骤 6: 可选基坐标位姿
    if (params.compute_base_coords && isHandEyeValid() && board_pose_base_opt) {
        // T_cam_board = board_pose_cam.inverse() (相机到板)
        Eigen::Matrix4d T_cam_board = board_pose_cam.inverse();
        // T_gripper_cam = hand_eye_res_.hand_eye_transform
        // T_base_board = T_base_gripper * T_gripper_cam * T_cam_board
        cv::Mat robot_pose = params.current_robot_pose.empty() ? cv::Mat::eye(4, 4, CV_64F) : params.current_robot_pose;
        Eigen::Matrix4d T_base_gripper = cvToEigenMat(robot_pose);
        *board_pose_base_opt = T_base_gripper * hand_eye_res_.hand_eye_transform * T_cam_board;
    }

    last_result_.success = true;
    std::cout << "棋盘格检测成功： " << last_result_.num_points << " 个点，平均误差: "
        << std::fixed << std::setprecision(2) << last_result_.reproj_error_mm << " 像素" << std::endl;

    // 步骤 7: 可视化 (专用函数，确保点线对齐 + 屏幕缩放)
    if (params.visualize) {
        const auto& ch_params = params.chess_params;  // 棋盘参数
        cv::Mat vis = visualizeChessboardDetection(image, image_points, ch_params.board_size,
            last_result_.success, 1920, 1080);  // 适应屏幕
        cv::namedWindow("棋盘格检测结果", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);  // 自适应窗口
        cv::imshow("棋盘格检测结果", vis);
        cv::waitKey(1);  // 非阻塞
    }

    return true;
}

// 专用可视化：棋盘格检测结果 (解决点线对齐问题)
cv::Mat FeatureDetector::visualizeChessboardDetection(const cv::Mat& image, const std::vector<cv::Point2f>& image_points,
                                                      const cv::Size& board_size, bool success,
                                                      int max_width, int max_height) const {
    cv::Mat display = image.clone();
    if (display.channels() == 1) {
        cv::cvtColor(display, display, cv::COLOR_GRAY2BGR);  // 确保彩色
    }

    // 步骤 1: 创建局部可修改点集 (复制输入，避免 const 问题)
    std::vector<cv::Point2f> points_to_draw = image_points;  // 复制 (高效，OpenCV 点小)

    // 步骤 2: 缩放图像和点以适应屏幕 (保持宽高比)
    double scale = 1.0;
    if (display.cols > max_width || display.rows > max_height) {
        double scale_w = static_cast<double>(max_width) / display.cols;
        double scale_h = static_cast<double>(max_height) / display.rows;
        scale = std::min(scale_w, scale_h);

        cv::Size new_size(static_cast<int>(display.cols * scale), static_cast<int>(display.rows * scale));
        cv::resize(display, display, new_size, 0, 0, cv::INTER_LINEAR);

        // 缩放点坐标 (赋值给局部 points_to_draw)
        std::vector<cv::Point2f> scaled_points;
        scaled_points.reserve(points_to_draw.size());
        for (const auto& pt : points_to_draw) {
            scaled_points.emplace_back(static_cast<float>(pt.x * scale), static_cast<float>(pt.y * scale));
        }
        points_to_draw = std::move(scaled_points);  // 移动赋值 (高效，避免额外复制)
    }

    // 步骤 3: 绘制标准棋盘格 (线 + 角点，高亮如果 success) - 使用缩放后的 points_to_draw
    cv::drawChessboardCorners(display, board_size, points_to_draw, success);  // 绿色线 + 小方块角点

    // 步骤 4: 叠加自定义标签 (序号，不重复绘制点，避免冲突)
    float font_scale = std::max(0.4f, static_cast<float>(scale * 0.6f));  // 适应缩放
    int text_thickness = static_cast<int>(2 * scale);
    for (size_t i = 0; i < points_to_draw.size(); ++i) {
        std::string label = std::to_string(i);
        cv::Point label_pos(static_cast<int>(points_to_draw[i].x + 10 * scale),
            static_cast<int>(points_to_draw[i].y - 10 * scale));
        cv::putText(display, label, label_pos, cv::FONT_HERSHEY_SIMPLEX, font_scale,
            cv::Scalar(255, 0, 0), text_thickness);  // 红色标签
    }

    // 步骤 5: 添加总体信息 (蓝色，左上角)
    std::string info = "Chessboard Corners: " + std::to_string(points_to_draw.size()) +
        (success ? " (Success)" : " (Failed)") +
        " (Scaled: " + std::to_string(static_cast<int>(scale * 100)) + "%)";
    float info_font_scale = std::max(0.5f, static_cast<float>(scale * 0.8f));
    cv::putText(display, info, cv::Point(static_cast<int>(10 * scale), static_cast<int>(30 * scale)),
        cv::FONT_HERSHEY_SIMPLEX, info_font_scale, cv::Scalar(0, 0, 255),
        static_cast<int>(2 * scale));

    // 步骤 6: 添加屏幕适应提示 (底部，灰色)
    std::string scale_tip = "Adapted to screen (" + std::to_string(max_width) + "x" + std::to_string(max_height) + ")";
    cv::putText(display, scale_tip, cv::Point(static_cast<int>(10 * scale),
        static_cast<int>(display.rows - static_cast<int>(10 * scale))),
        cv::FONT_HERSHEY_SIMPLEX, info_font_scale * 0.7f, cv::Scalar(128, 128, 128), 1);

    return display;
}

// 可视化检测结果：绘制点和标签
cv::Mat FeatureDetector::visualizeDetection(const cv::Mat& image, const std::vector<cv::Point2f>& points) const {
    int max_width = 1920, max_height = 1080;
    cv::Mat display = image.clone();
    if (display.channels() == 1) {
        cv::cvtColor(display, display, cv::COLOR_GRAY2BGR);  // 确保彩色
    }

    // 步骤 1: 缩放图像以适应屏幕比例 (保持宽高比)
    double scale = 1.0;
    if (display.cols > max_width || display.rows > max_height) {
        // 计算缩放比例 (优先宽度，保持比例)
        double scale_w = static_cast<double>(max_width) / display.cols;
        double scale_h = static_cast<double>(max_height) / display.rows;
        scale = std::min(scale_w, scale_h);  // 取较小比例，避免裁剪

        cv::Size new_size(static_cast<int>(display.cols * scale), static_cast<int>(display.rows * scale));
        cv::resize(display, display, new_size, 0, 0, cv::INTER_LINEAR);  // 线性插值保持质量
    }

    // 步骤 2: 缩放点坐标 (与图像同步)
    std::vector<cv::Point2f> scaled_points;
    scaled_points.reserve(points.size());
    for (const auto& pt : points) {
        scaled_points.emplace_back(static_cast<float>(pt.x * scale), static_cast<float>(pt.y * scale));
    }

    // 步骤 3: 绘制点 (绿色圆点 + 黑色边框)
    for (const auto& pt : scaled_points) {
        cv::circle(display, pt, static_cast<int>(5 * scale), cv::Scalar(0, 255, 0), -1);  // 填充绿色
        cv::circle(display, pt, static_cast<int>(5 * scale), cv::Scalar(0, 0, 0), 2);     // 黑色边框
    }

    // 步骤 4: 添加序号标签 (红色，缩放字体大小)
    float font_scale = std::max(0.4f, static_cast<float>(scale * 0.6f));  // 适应缩放
    for (size_t i = 0; i < scaled_points.size(); ++i) {
        std::string label = std::to_string(i);
        cv::Point label_pos(static_cast<int>(scaled_points[i].x + 10 * scale),
            static_cast<int>(scaled_points[i].y - 10 * scale));
        cv::putText(display, label, label_pos, cv::FONT_HERSHEY_SIMPLEX, font_scale,
            cv::Scalar(255, 0, 0), static_cast<int>(2 * scale));
    }

    // 步骤 5: 添加总体信息 (蓝色，左上角)
    std::string info = "Detected Points: " + std::to_string(points.size()) +
        " (Scaled: " + std::to_string(static_cast<int>(scale * 100)) + "%)";
    float info_font_scale = std::max(0.5f, static_cast<float>(scale * 0.8f));
    cv::putText(display, info, cv::Point(10 * scale, 30 * scale), cv::FONT_HERSHEY_SIMPLEX,
        info_font_scale, cv::Scalar(0, 0, 255), static_cast<int>(2 * scale));

    // 步骤 6: 添加屏幕适应提示 (可选，小字体)
    std::string scale_tip = "Adapted to screen ratio (" + std::to_string(max_width) + "x" + std::to_string(max_height) + ")";
    cv::putText(display, scale_tip, cv::Point(10 * scale, static_cast<int>(display.rows - 10 * scale)),
        cv::FONT_HERSHEY_SIMPLEX, info_font_scale * 0.7f, cv::Scalar(128, 128, 128), 1);

    return display;
}