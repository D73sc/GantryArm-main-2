#include "ForceControlExecutor.h"
#include <chrono>
#include <thread>
#include <sstream>

ForceControlExecutor::ForceControlExecutor(AdmittanceController* admittanceController,
    TrajectoryOptimizer* trajectoryOptimizer)
    : admittanceController_(admittanceController),
    trajectoryOptimizer_(trajectoryOptimizer),
    running_(false)
{
    if (!admittanceController_ || !trajectoryOptimizer_) {
        throw std::invalid_argument("AdmittanceController or TrajectoryOptimizer pointer is null!");
    }
    latestSensorData_.setZero();
}

ForceControlExecutor::~ForceControlExecutor()
{
    stop();
}

void ForceControlExecutor::setControlPeriod(int period_ms)
{
    if (period_ms <= 0) return;
    controlPeriodMs_ = period_ms;
}

void ForceControlExecutor::setPlannedTrajectory(const std::vector<Eigen::Matrix4d>& traj)
{
    std::lock_guard<std::mutex> lock(trajMutex_);
    plannedTrajectory_ = traj;
    currentTrajectoryIndex_ = 0;
    detectTurningPointsFromTrajectory(plannedTrajectory_);
}

void ForceControlExecutor::updateSensorData(const Eigen::Matrix<double, 6, 1>& data)
{
    std::lock_guard<std::mutex> lock(sensorDataMutex_);
    latestSensorData_ = data;
}

void ForceControlExecutor::setLogCallback(LogCallback cb)
{
    logCallback_ = std::move(cb);
}

void ForceControlExecutor::setSendMotionCallback(SendMotionFunc func)
{
    sendMotionFunc_ = std::move(func);
}

void ForceControlExecutor::setIsMotionCompleteCallback(std::function<bool()> func)
{
    checkMotionCompleteFunc_ = std::move(func);
}

void ForceControlExecutor::setGetCurrentJointsCallback(GetCurrentJointsFunc func)
{
    getCurrentJointsFunc_ = std::move(func);
}

void ForceControlExecutor::start()
{
    if (running_) {
        log("Control thread already running.");
        return;
    }
    running_ = true;
    globalOffset_ = Eigen::Matrix4d::Identity();
    controlStartTime_ = std::chrono::steady_clock::now();
    sensorDataLog_.clear();

    // 清空队列
    {
        std::lock_guard<std::mutex> lock(queueMutex_);
        sentJointQueue_.clear();
    }

    controlThread_ = std::thread(&ForceControlExecutor::controlLoop, this);
    log("Control thread started.");
}

void ForceControlExecutor::stop()
{
    if (!running_) return;
    running_ = false;
    if (controlThread_.joinable()) {
        controlThread_.join();
    }
    log("Control thread stopped.");
    globalOffset_ = Eigen::Matrix4d::Identity();

    if (trajectoryOptimizer_) {
        trajectoryOptimizer_->clearWarmupState();
    }
}


void ForceControlExecutor::log(const std::string& msg)
{
    if (logCallback_) {
        logCallback_(msg);
    }
    else {
        // 可选：捕获到无回调时的默认输出，方便调试
        std::cout << "[ForceControlExecutor] " << msg << std::endl;
    }
}




std::string ForceControlExecutor::getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  now.time_since_epoch()) % 1000;

    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S")
       << "_" << std::setfill('0') << std::setw(3) << ms.count();
    return ss.str();
}

Eigen::Matrix4d ForceControlExecutor::toEigenMatrix(const Point6D& offset) {
    // 小角度近似（假设旋转很小）
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();

    // 位置
    mat(0, 3) = offset.x;
    mat(1, 3) = offset.y;
    mat(2, 3) = offset.z;

    // 旋转矩阵（使用旋转向量）
    Eigen::AngleAxisd angleAxis(
        Eigen::Vector3d(offset.rx, offset.ry, offset.rz).norm(),
        Eigen::Vector3d(offset.rx, offset.ry, offset.rz).normalized()
        );
    mat.block<3, 3>(0, 0) = angleAxis.toRotationMatrix();

    return mat;
}

Point6D ForceControlExecutor::toPoint6D(const Eigen::Vector3d& pos,
                                        const Eigen::Vector3d& rot) {
    return {pos.x(), pos.y(), pos.z(), rot.x(), rot.y(), rot.z()};
}

void ForceControlExecutor::reorthogonalizeMatrix(Eigen::Matrix4d& mat) {
    // SVD 正交化旋转部分
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        mat.block<3, 3>(0, 0),
        Eigen::ComputeFullU | Eigen::ComputeFullV
        );
    mat.block<3, 3>(0, 0) = svd.matrixU() * svd.matrixV().transpose();
}


bool ForceControlExecutor::waitForMotionCompletion(std::function<bool()> isMotionComplete, int timeoutMs, int pollIntervalMs ) {
    using namespace std::chrono;
    auto startTime = steady_clock::now();

    while (true) {
        std::this_thread::sleep_for(milliseconds(100));

        if (isMotionComplete()) {
            return true;  // 动作完成
        }

        auto elapsed = duration_cast<milliseconds>(steady_clock::now() - startTime).count();
        if (elapsed >= timeoutMs) {
            return false; // 超时失败
        }

        std::this_thread::sleep_for(milliseconds(pollIntervalMs));
    }
}

bool ForceControlExecutor::waitForMotionStart(std::function<bool()> isMotionComplete, int timeoutMs, int pollIntervalMs ) {
    using namespace std::chrono;
    auto startTime = steady_clock::now();

    while (true) {
        std::this_thread::sleep_for(milliseconds(100));

        if (!isMotionComplete()) {
            return true;  // 动作开始
        }

        auto elapsed = duration_cast<milliseconds>(steady_clock::now() - startTime).count();
        if (elapsed >= timeoutMs) {
            return false; // 超时失败
        }

        std::this_thread::sleep_for(milliseconds(pollIntervalMs));
    }
}
//停下的力控
// void ForceControlExecutor::controlLoop() {
//     while (running_) {
//         auto cycleStart = std::chrono::steady_clock::now();

//         Eigen::Matrix4d nextPlannedPose;
//         size_t trajIndex;
//         {
//             std::lock_guard<std::mutex> lock(trajMutex_);
//             if (currentTrajectoryIndex_ >= plannedTrajectory_.size()) {
//                 log("Trajectory complete.");
//                 running_ = false;
//                 break;
//             }
//             nextPlannedPose = plannedTrajectory_[currentTrajectoryIndex_];
//             trajIndex = currentTrajectoryIndex_;
//         }

//         Eigen::Matrix<double, 6, 1> forceCopy;
//         {
//             std::lock_guard<std::mutex> lock(sensorDataMutex_);
//             forceCopy = latestSensorData_;
//         }

//         if (!(sendMotionFunc_ && checkMotionCompleteFunc_)) {
//             log("Send motion function or completion checker not set. Aborting control loop.");
//             break;
//         }
//         // 判断当前轨迹点是否为转弯点
//         bool isTurning = false;
//         if (trajIndex < turningPoints_.size()) {
//             isTurning = turningPoints_[trajIndex];
//         }

//         if (!motionInProgress_) {
//             // 当前无正在进行的运动，可以发送动作指令
//             if (checkMotionCompleteFunc_()) {  // 确认机械臂空闲，准备新运动
//                 Eigen::Matrix4d actualTargetPose;
//                 Point6D offsetCur;
//                 if (isTurning) {
//                     // 转弯点禁用力控，直接轨迹跟踪
//                     globalOffset_ = Eigen::Matrix4d::Identity();
//                     admittanceController_->reset();
//                     actualTargetPose = nextPlannedPose;
//                 } else {
//                     // 正常力控逻辑
//                     offsetCur = admittanceController_->update(forceCopy);
//                     Eigen::Matrix4d offsetMatrix = toEigenMatrix(offsetCur);
//                     globalOffset_ = globalOffset_ * offsetMatrix;
//                     actualTargetPose = nextPlannedPose * globalOffset_;
//                 }
//                 {
//                     std::lock_guard<std::mutex> lock(statusMutex_);
//                     currentStatus_.current_index = trajIndex;
//                     currentStatus_.plannedPose = nextPlannedPose;
//                     currentStatus_.admittanceOffset = offsetCur;
//                     currentStatus_.actualTargetPose = actualTargetPose;
//                 }

//                 if (statusCallback_) {
//                     statusCallback_(currentStatus_.toString());
//                 }

//                 log("Sending motion command for trajectory point " + std::to_string(trajIndex));
//                 auto joint = trajectoryOptimizer_->optimizeSinglePoint(actualTargetPose);
//                 JointTrajectory jointTrajectory{ joint };
//                 sendMotionFunc_(jointTrajectory);

//                 motionInProgress_ = true;  // 标记运动开始
//                 bool motionSucceeded = waitForMotionStart(checkMotionCompleteFunc_, 5000);
//                 if (!motionSucceeded) {
//                     log("Warning: motion completion wait timed out");
//                     break;
//                 }
//             }
//         } else {
//             // 当前正在运动，等待完成
//             int timeoutMs = 100000;  // 合理超时，比如5秒
//             bool motionSucceeded = waitForMotionCompletion(checkMotionCompleteFunc_, timeoutMs);
//             if (!motionSucceeded) {
//                 log("Warning: motion completion wait timed out");
//                 break;
//             }
//             // 运动完成，准备下一个轨迹点
//             {
//                 std::lock_guard<std::mutex> lock(trajMutex_);
//                 currentTrajectoryIndex_++;
//             }
//             motionInProgress_ = false;  // 释放运动锁，允许发送下一动作
//         }

//         auto cycleEnd = std::chrono::steady_clock::now();
//         auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(cycleEnd - cycleStart).count();
//         int sleepTime = controlPeriodMs_ - static_cast<int>(elapsedMs);
//         if (sleepTime > 0) {
//             std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));
//         }
//     }

//     log("Control thread exiting.");
// }

void ForceControlExecutor::controlLoop() {
    // ==================== 稳定参数 ====================
    const size_t MIN_BUFFER = 4;
    const size_t MAX_BUFFER = 8;
    const size_t BATCH_SEND = 2;
    const double MOVE_THRESH = 0.35;
    const double PREDICT_THRESH = 0.7;
    const int CONTROL_PERIOD_MS = 10;
    const int INDEX_STUCK_TIMEOUT_MS = 60000;
    const int FINAL_POINT_TIMEOUT_MS = 60000;
    const double FORCE_CHANGE_EPS = 1e-4;
    const double MAX_POS_OFFSET = 10;      // 10mm
    const double MAX_ROT_OFFSET = 5;      // 5rad
    const int OFFSET_REORTHO_INTERVAL = 100;
    // ===================================================

    lastExecutedQueueIdx_ = 0;
    bool isFirstCycle = true;
    int reorthoCounter = 0;
    int logCounter = 0;
    size_t trajTotal = plannedTrajectory_.size();

    Eigen::Matrix<double, 6, 1> lastSensorData_ = Eigen::Matrix<double, 6, 1>::Zero();
    std::chrono::steady_clock::time_point lastIndexTime = std::chrono::steady_clock::now();

    // Clear queue at initialization
    {
        std::lock_guard<std::mutex> lock(queueMutex_);
        sentJointQueue_.clear();
    }

    log("========== Control loop started ==========");

    while (running_) {
        auto cycleStart = std::chrono::steady_clock::now();
        size_t queueSize = 0;
        double minDistSqr = 1e10;
        size_t closestIdx = lastExecutedQueueIdx_;

        const double moveThreshSqr = MOVE_THRESH * MOVE_THRESH;
        const double predThreshSqr = PREDICT_THRESH * PREDICT_THRESH;

        // ===== 1. 获取当前关节位置 =====
        std::array<double, 8> currJoints;
        if (!getCurrentJointsFunc_) {
            log("Error: getCurrentJointsFunc_ not set!");
            std::this_thread::sleep_for(std::chrono::milliseconds(CONTROL_PERIOD_MS));
            continue;
        }
        currJoints = getCurrentJointsFunc_();

        // ===== 2. 队列搜索与索引推进 =====
        bool indexAdvancedThisFrame = false;
        {
            std::lock_guard<std::mutex> lock(queueMutex_);
            queueSize = sentJointQueue_.size();

            if (queueSize > 0) {
                // 搜索最接近的点
                for (size_t i = lastExecutedQueueIdx_; i < queueSize; ++i) {
                    double d2 = 0.0;
                    for (int j = 0; j < 8; ++j) {
                        double df = currJoints[j] - sentJointQueue_[i][j];
                        d2 += df * df;
                    }
                    if (d2 < minDistSqr) {
                        minDistSqr = d2;
                        closestIdx = i;
                    }
                }

                bool canStep = (lastExecutedQueueIdx_ < queueSize - 1);
                auto now = std::chrono::steady_clock::now();
                int stuckMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  now - lastIndexTime).count();

                // 多条件平滑推进（每帧最多+1）
                bool shouldStep = false;
                // 检查是否是最后一个点
                bool isLastPoint = (currentTrajectoryIndex_ >= trajTotal) &&
                                   (lastExecutedQueueIdx_ == queueSize - 1);

                if (isLastPoint) {
                    // 最后一个点已经到达，直接推进
                    if (minDistSqr < moveThreshSqr) {
                        shouldStep = true;
                    }
                } else if (minDistSqr < moveThreshSqr && closestIdx > lastExecutedQueueIdx_) {
                    shouldStep = true;
                } else if (minDistSqr < predThreshSqr && canStep) {
                    shouldStep = true;
                } else if (stuckMs > INDEX_STUCK_TIMEOUT_MS && canStep) {
                    shouldStep = true;
                    log("Index stuck, advancing | Time: " +
                        std::to_string(stuckMs) + "ms");
                }

                if (shouldStep) {
                    lastExecutedQueueIdx_++;
                    lastIndexTime = now;
                    indexAdvancedThisFrame = true;
                }

                // 定期日志（50帧一次）
                if (indexAdvancedThisFrame || (++logCounter % 50 == 0)) {
                    double dist = sqrt(minDistSqr);
                    log("Index: " + std::to_string(lastExecutedQueueIdx_) +
                        " | Queue: " + std::to_string(queueSize) +
                        " | Remaining: " + std::to_string(queueSize - lastExecutedQueueIdx_) +
                        " | Dist: " + std::to_string(dist));
                }
            }
        }

        // ===== 3. 检查剩余轨迹 =====
        bool hasMoreTraj = false;
        size_t trajSent = 0;
        {
            std::lock_guard<std::mutex> lock(trajMutex_);
            trajSent = currentTrajectoryIndex_;
            hasMoreTraj = (trajSent < trajTotal);
        }

        // ===== 4. 计算本次需要发送的点数 =====
        size_t needSend = 0;
        size_t unexec = queueSize - lastExecutedQueueIdx_;
        if (hasMoreTraj) {
            if (isFirstCycle || unexec < MIN_BUFFER) {
                needSend = std::min<size_t>({
                    MAX_BUFFER - unexec,
                    BATCH_SEND,
                    trajTotal - trajSent
                });
                isFirstCycle = false;
            }
        }

        // ===== 5. 退出条件（已修复）=====
        bool shouldExit = false;
        if (!hasMoreTraj) {
            if (queueSize == 0) {
                log("Queue empty, no more trajectory");
                shouldExit = true;
                running_ = false;
                break;
            }
            else if (lastExecutedQueueIdx_ >= queueSize) {
                log("Exit: All points executed");
                shouldExit = true;
                running_ = false;
                break;
            }

            // 改进的最后一点检查：有超时机制
            bool lastPointReached = (lastExecutedQueueIdx_ >= queueSize - 1);
            int finalWaitMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  std::chrono::steady_clock::now() - lastIndexTime).count();

            if (lastPointReached && finalWaitMs > FINAL_POINT_TIMEOUT_MS) {
                log("All trajectory executed (final wait: " +
                    std::to_string(finalWaitMs) + "ms)");
                shouldExit = true;
                running_ = false;
                break;
            }
        }

        // ===== 6. 生成并发送轨迹点 =====
        if (needSend > 0) {
            std::vector<std::array<double, 8>> batch;
            Eigen::Matrix<double, 6, 1> currSensor;

            // 读取传感器数据
            {
                std::lock_guard<std::mutex> lock(sensorDataMutex_);
                currSensor = latestSensorData_;
            }

            // 修复：计算固定偏移（每帧一次）
            Point6D fixedOffset{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

            // 这里应该调用你的导纳控制器
            fixedOffset = admittanceController_->update(currSensor);

            // 限制fixedOffset大小
            Eigen::Vector3d posOff(fixedOffset.x, fixedOffset.y, fixedOffset.z);
            if (posOff.norm() > MAX_POS_OFFSET) {
                posOff = posOff.normalized() * MAX_POS_OFFSET;
                fixedOffset.x = posOff.x();
                fixedOffset.y = posOff.y();
                fixedOffset.z = posOff.z();
            }

            Eigen::Vector3d rotOff(fixedOffset.rx, fixedOffset.ry, fixedOffset.rz);
            if (rotOff.norm() > MAX_ROT_OFFSET) {
                rotOff = rotOff.normalized() * MAX_ROT_OFFSET;
                fixedOffset.rx = rotOff.x();
                fixedOffset.ry = rotOff.y();
                fixedOffset.rz = rotOff.z();
            }

            // 用限制后的fixedOffset生成矩阵
            Eigen::Matrix4d offsetMat = toEigenMatrix(fixedOffset);
            globalOffset_ = globalOffset_ * offsetMat;

            // 再限制globalOffset_本身
            Eigen::Vector3d globalPos = globalOffset_.block<3,1>(0,3);
            if (globalPos.norm() > MAX_POS_OFFSET*2) {
                globalPos = globalPos.normalized() * MAX_POS_OFFSET;
                globalOffset_.block<3,1>(0,3) = globalPos;
            }

            lastSensorData_ = currSensor;

            if (++reorthoCounter % OFFSET_REORTHO_INTERVAL == 0) {
                reorthogonalizeMatrix(globalOffset_);
            }

            Eigen::Matrix4d useGlobal = globalOffset_;

            // 生成点批次
            {
                std::lock_guard<std::mutex> lock(trajMutex_);
                size_t gen = 0;
                size_t startIdx = currentTrajectoryIndex_;

                while (gen < needSend && startIdx  + gen < trajTotal) {
                    size_t idx = startIdx  + gen;
                    const Eigen::Matrix4d& plannedPose = plannedTrajectory_[idx];
                    bool isTurning = (idx < turningPoints_.size()) ? turningPoints_[idx] : false;

                    Eigen::Matrix4d actualPose;
                    Point6D logOffset;

                    if (isTurning) {
                        // 创建安全后退矩阵
                        Eigen::Matrix4d safetyOffset = Eigen::Matrix4d::Identity();
                        safetyOffset(2, 3) = -10.0;  // Z轴后退20mm（机械臂坐标系）
                        // 应用到计划轨迹上
                        actualPose = plannedPose * safetyOffset;
                        // 日志记录
                        logOffset = {0.0, 0.0, -30.0, 0.0, 0.0, 0.0};
                        // 重置全局偏移
                        globalOffset_ = Eigen::Matrix4d::Identity();

                    } else {
                        logOffset = fixedOffset;
                        actualPose = plannedPose * useGlobal;
                    }

                    // 更新状态
                    {
                        std::lock_guard<std::mutex> lock(statusMutex_);
                        currentStatus_.current_index = idx;
                        currentStatus_.plannedPose = plannedPose;
                        currentStatus_.admittanceOffset = logOffset;
                        currentStatus_.actualTargetPose = actualPose;
                    }

                    if (statusCallback_) {
                        statusCallback_(currentStatus_.toString());
                    }

                    // 求解关节并添加到批次
                    std::array<double, 8> jointSol = trajectoryOptimizer_->optimizeSinglePoint(actualPose);
                    batch.push_back(jointSol);

                    // 收集传感器数据用于日志
                    double elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                           std::chrono::steady_clock::now() - controlStartTime_).count();

                    SensorDataFrame frame;
                    frame.timestamp_ms = elapsedMs;
                    frame.sensorData = {
                        currSensor(0), currSensor(1), currSensor(2),
                        currSensor(3), currSensor(4), currSensor(5)
                    };
                    frame.jointPos = currJoints;
                    frame.admittanceOffset = logOffset;
                    frame.targetPos = {
                        actualPose(0, 3), actualPose(1, 3), actualPose(2, 3)
                    };

                    {
                        std::lock_guard<std::mutex> lock(sensorDataMutex_);
                        sensorDataLog_.push_back(frame);
                    }

                    gen++;
                }
                currentTrajectoryIndex_ += gen;

            }

            // 发送点到伺服
            if (!batch.empty() && sendMotionFunc_) {
                sendMotionFunc_(batch);

                {
                    std::lock_guard<std::mutex> lock(queueMutex_);
                    sentJointQueue_.insert(sentJointQueue_.end(), batch.begin(), batch.end());
                }

                log("Sent: " + std::to_string(batch.size()) +
                    " points | Total: " + std::to_string(currentTrajectoryIndex_) +
                    " | Queue size: " + std::to_string(sentJointQueue_.size()));
            }
        }

        // ===== 7. 控制周期休眠 =====
        auto cycleEnd = std::chrono::steady_clock::now();
        int usedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                         cycleEnd - cycleStart).count();
        int sleepMs = std::max(1, CONTROL_PERIOD_MS - usedMs);
        std::this_thread::sleep_for(std::chrono::milliseconds(sleepMs));
    }

    // ===== 8. 保存传感器数据到CSV =====
    log("========== Saving sensor data ==========");
    saveSensorDataToCSV();

    log("========== Control thread exited ==========");
}

void ForceControlExecutor::saveSensorDataToCSV() {
    // 创建目录
    std::filesystem::path dirPath = "lib/data/sensor";
    try {
        std::filesystem::create_directories(dirPath);
    } catch (const std::exception& e) {
        log("Error creating directory: " + std::string(e.what()));
        return;
    }

    // 生成文件名
    std::string filename = getCurrentTimestamp() + ".csv";
    std::filesystem::path filePath = dirPath / filename;

    // 写入CSV
    std::ofstream csvFile(filePath.string());
    if (!csvFile.is_open()) {
        log("Error opening CSV file: " + filePath.string());
        return;
    }

    // 写入头部
    csvFile << "Timestamp(ms),"
            << "Fx(N),Fy(N),Fz(N),Tx(Nm),Ty(Nm),Tz(Nm),"
            << "J1(rad),J2(rad),J3(rad),J4(rad),J5(rad),J6(rad),J7(rad),J8(rad),"
            << "OffsetX(m),OffsetY(m),OffsetZ(m),"
            << "OffsetRx(rad),OffsetRy(rad),OffsetRz(rad),"
            << "TargetX(m),TargetY(m),TargetZ(m)\n";

    // 写入数据
    for (const auto& frame : sensorDataLog_) {
        csvFile << std::fixed << std::setprecision(6) << frame.timestamp_ms << ",";

        // 传感器数据
        for (int i = 0; i < 6; ++i) {
            csvFile << frame.sensorData[i];
            if (i < 5) csvFile << ",";
        }
        csvFile << ",";

        // 关节数据
        for (int i = 0; i < 8; ++i) {
            csvFile << frame.jointPos[i];
            if (i < 7) csvFile << ",";
        }
        csvFile << ",";

        // 偏移量
        csvFile << frame.admittanceOffset.x << ","
                << frame.admittanceOffset.y << ","
                << frame.admittanceOffset.z << ","
                << frame.admittanceOffset.rx << ","
                << frame.admittanceOffset.ry << ","
                << frame.admittanceOffset.rz << ",";

        // 目标位置
        csvFile << frame.targetPos[0] << ","
                << frame.targetPos[1] << ","
                << frame.targetPos[2] << "\n";
    }

    csvFile.close();
    log("Sensor data saved to: " + filePath.string() +
        " (" + std::to_string(sensorDataLog_.size()) + " frames)");
}

size_t ForceControlExecutor::estimateCurrentExecutedIndex() {
    std::array<double, 8> currentJoints = getCurrentJointsFunc_();  // 实时从机械臂读

    size_t nearestIdx = 0;
    double minDistSqr = 1;

    for (size_t i = 0; i < sentJointQueue_.size(); ++i) {
        double distSqr = 0.0;
        for (int j = 0; j < 8; ++j) {
            double diff = currentJoints[j] - sentJointQueue_[i][j];
            distSqr += diff * diff;
        }
        if (distSqr < minDistSqr) {
            minDistSqr = distSqr;
            nearestIdx = i;
        }
    }
    return nearestIdx;  // 队列内的索引，也对应总体轨迹索引偏移（需加基准）
}

std::vector<std::array<double, 8>> ForceControlExecutor::computeSingleForceControlJoint() {
    Eigen::Matrix<double, 6, 1> forceCopy;
    {
        std::lock_guard<std::mutex> lock(sensorDataMutex_);
        forceCopy = latestSensorData_;
    }

    Eigen::Matrix4d currentPose;
    size_t trajIndex;
    {
        std::lock_guard<std::mutex> lock(trajMutex_);
        if (currentTrajectoryIndex_ >= plannedTrajectory_.size()) {
            log("No current trajectory pose available.");
            return {};
        }
        currentPose = plannedTrajectory_[currentTrajectoryIndex_];
        trajIndex = currentTrajectoryIndex_;
    }

    Point6D offsetCur = admittanceController_->update(forceCopy);
    Eigen::Matrix4d offsetMatrix = toEigenMatrix(offsetCur);
    Eigen::Matrix4d actualTargetPose = currentPose * offsetMatrix;

    // 更新状态结构，保持实时状态同步
    {
        std::lock_guard<std::mutex> lock(statusMutex_);
        currentStatus_.current_index = trajIndex;
        currentStatus_.plannedPose = currentPose;
        currentStatus_.admittanceOffset = offsetCur;
        currentStatus_.actualTargetPose = actualTargetPose;
    }

    // 状态回调（非阻塞最好，或推入队列异步处理）
    if (statusCallback_) {
        statusCallback_(currentStatus_.toString());
    }

    log("Computed admittance offset and updated status.");

    auto joint = trajectoryOptimizer_->optimizeSinglePoint(actualTargetPose);

    log("Inverse kinematics computed joint solution (no motion sent).");
    std::vector<std::array<double, 8>> jointSolution{joint};
    return jointSolution;
}

ForceControlExecutor::ControlStatus ForceControlExecutor::getControlStatus() const {
    std::lock_guard<std::mutex> lock(statusMutex_);
    return currentStatus_; // 结构体整体返回，拷贝安全
}

void ForceControlExecutor::setStatusCallback(StatusCallback cb) {
    std::lock_guard<std::mutex> lock(statusMutex_);
    statusCallback_ = std::move(cb);
}

std::vector<bool> ForceControlExecutor::detectTurningPointsFromTrajectory(
    const std::vector<Eigen::Matrix4d>& plannedTrajectory,
    double turning_angle_threshold)  // 默认阈值0.1弧度 (约5.7度)
{
    size_t n = plannedTrajectory.size();
    std::vector<bool> isTurningPoint(n, false);

    if (n < 2) return isTurningPoint;  // 少于2个点无意义

    for (size_t i = 1; i < n; ++i) {
        const Eigen::Matrix3d& R_prev = plannedTrajectory[i - 1].block<3, 3>(0, 0);
        const Eigen::Matrix3d& R_curr = plannedTrajectory[i].block<3, 3>(0, 0);

        // 计算相对旋转矩阵
        Eigen::Matrix3d R_delta = R_prev.transpose() * R_curr;

        // 转成角轴表示，提取旋转角
        Eigen::AngleAxisd aa(R_delta);
        double angle = std::abs(aa.angle());

        bool isTurning = (angle > turning_angle_threshold);
        isTurningPoint[i] = isTurning;

        std::cout << "Point " << i << ": rotation difference = "
                  << angle << " rad, turning = " << (isTurning ? "YES" : "no") << std::endl;
    }
    turningPoints_=isTurningPoint;
    return isTurningPoint;
}
