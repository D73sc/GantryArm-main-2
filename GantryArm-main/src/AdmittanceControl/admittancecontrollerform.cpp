#include "admittancecontrollerform.h"
#include "ui_admittancecontrollerform.h"

AdmittanceControllerForm::AdmittanceControllerForm(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::AdmittanceControllerForm)
{
    qRegisterMetaType<std::vector<std::array<double,8>>>("std::vector<std::array<double,8>>");
    iniRead = new QSettings("lib/param.ini", QSettings::IniFormat);
    ui->setupUi(this);
    compensator=new ForceTorqueGravityCompensator();
    id_param = loadIdentificationParam(iniRead);
    compensator->setIdentification_param(id_param);
    IntegralParams params(
        10, 200, 4000,    // 平动参数 (M, B, K)
        0.1, 1.0, 10.0,      // 旋转参数 (M, B, K)
        1.1, 1.0, 5, 0.1, // 积分参数 (η, k_e,积分限幅, 衰减)
        0.05                // 采样时间
        );
    controller=new AdmittanceController(params, true);
    // 3. 设置期望力为0（静态期望，无外力）
    Eigen::Matrix<double, 6, 1> desired_force = Eigen::Matrix<double, 6, 1>::Zero();
    desired_force[2] = -20;
    controller->setDesiredWrench(desired_force);
}

AdmittanceControllerForm::~AdmittanceControllerForm()
{
    delete ui;
}

void AdmittanceControllerForm::on_btnOpenData_clicked()
{
    QFileDialog dlg(this);
    QString currentPath = QDir::currentPath();
    QString dataQFile = QDir(currentPath).filePath("lib/data");

    // 打开文件对话框，让用户选择一个 CSV 文件
    QString fileName = dlg.getOpenFileName(
        nullptr,                    // 父窗口指针，这里为 nullptr 表示没有父窗口
        tr("Open CSV File"),       // 对话框标题
        dataQFile,                    // 默认目录
        tr("CSV Files (*.csv);;All Files (*)")  // 文件过滤器，只显示 CSV 文件和所有文件
        );

    // 检查是否选择了文件
    if (!fileName.isEmpty()) {
        qDebug() << "Selected file:" << fileName;
        // 将文件路径赋值给变量
        // QString csvfilePath = fileName;
        ui->LineEditDataAddress->setText(fileName);
    } else {
        qDebug() << "No file selected.";
    }
}


void AdmittanceControllerForm::on_btnLoadData_clicked()
{
    QString dataQFile =ui->LineEditDataAddress->text();
    trajectory.clear();
    trajectory = loadTrajectoryFromCSV(dataQFile.toStdString());
    forceExecutor->setPlannedTrajectory(trajectory);
    updateCurrentPosiSpin();

}


void AdmittanceControllerForm::on_btnLoadmmData_clicked()
{
    // auto turningPoints =forceExecutor->detectTurningPointsFromTrajectory(trajectory,0.1);

    // for (size_t i = 0; i < turningPoints.size(); ++i) {
    //     std::cout << "Trajectory point " << i
    //               << (turningPoints[i] ? " is a turning point." : " is NOT a turning point.")
    //               << std::endl;
    // }
    QString dataQFile =ui->LineEditDataAddress->text();
    trajectory.clear();
    trajectory = loadTrajectoryFromCSV(dataQFile.toStdString(),true,0);
    forceExecutor->setPlannedTrajectory(trajectory);
    updateCurrentPosiSpin();

}


void AdmittanceControllerForm::on_btnClearPosiData_clicked()
{
    emit requestPosition();
    savePoint6DToCSV("lib/data/AdmittanceControl/robot_poses.csv", robotPose.tcp_pose, false);
}


void AdmittanceControllerForm::on_btnSavePosiData_clicked()
{
    emit requestPosition();
    savePoint6DToCSV("lib/data/AdmittanceControl/robot_poses.csv", robotPose.tcp_pose, true);
}


void AdmittanceControllerForm::on_btnClearSensorData_clicked()
{

    std::vector<double> stdvec(forceData.constBegin(), forceData.constEnd());
    saveVectorToCSV("lib/data/AdmittanceControl/force_data.csv", stdvec, false,"FX,FY,FZ,MX,MY,MZ");
}


void AdmittanceControllerForm::on_btnSaveSensorData_clicked()
{
    std::vector<double> stdvec(forceData.constBegin(), forceData.constEnd());
    saveVectorToCSV("lib/data/AdmittanceControl/force_data.csv", stdvec, true,"FX,FY,FZ,MX,MY,MZ");
}


void AdmittanceControllerForm::on_btnForceTorqueGravityCompensator_clicked()
{
    std::string pose_csv="lib/data/AdmittanceControl/robot_poses.csv";
    std::string ft_csv="lib/data/AdmittanceControl/force_data.csv";

    std::vector<Point6D> poses;
    if (!readPoint6DFromCSV(pose_csv, poses)) {
        std::cerr << "Failed to load pose data!\n";
        return ;
    }

    std::vector<std::vector<double>> ft_raw;
    if (!readVectorsFromCSV(ft_csv, ft_raw, true)) {  // 假设 force-torque csv有表头
        std::cerr << "Failed to load force-torque data!\n";
        return ;
    }
    std::vector<Eigen::Matrix3d> rotation_list;
    for (const auto& p : poses) {
        Eigen::Matrix4d pose_mat = toEigenMatrix(p);//!!!!!!!!!!!!!!!!!
        Eigen::Matrix3d R = getRotation(pose_mat);
        rotation_list.push_back(R);

    }
    // for (size_t i = 0; i < rotation_list.size(); ++i) {
    //     std::cout << "Rotation matrix " << (i + 1) << ":\n";
    //     std::cout << rotation_list[i] << "\n\n";
    // }
    // 将ft_raw转换成Eigen::MatrixXd
    const size_t rows = ft_raw.size();
    if (rows == 0) {
        std::cerr << "No force-torque data!\n";
        return ;
    }
    const size_t cols = ft_raw[0].size();

    Eigen::MatrixXd ft_data(cols, rows);
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            ft_data(j, i) = ft_raw[i][j];  // 注意这里是 j,i 赋值
        }
    }


    compensator->identifyGravityParams(ft_data, rotation_list);// Eigen 向量 -> 逗号分隔字符串的基本套路

    // 使用示例
    for (size_t i = 0; i < rotation_list.size(); ++i) {
        std::cout << "----- 补偿第 " << i + 1 << " 组 -----" << std::endl;
        Vector6d ft_d = ft_data.col(i);
        Vector6d compensated = compensator->compensate(ft_d, rotation_list[i], true);

        QStringList parts;
        for (int k = 0; k < 6; ++k) {
            parts << QString::number(compensated[k], 'f', 6);  // 每个元素转字符串
        }
        QString vecStr = parts.join(", ");
        UpdateUI("Compensated force data:" + vecStr);
    }

}


void AdmittanceControllerForm::on_btnClearData_clicked()
{
    trajectory.clear();
    forceExecutor->setPlannedTrajectory(trajectory);

}

void AdmittanceControllerForm::on_btnMoveTrajectoryOnce_clicked()
{
    emit MoveJointTraj(jointOnce);
}


void AdmittanceControllerForm::on_btnAdmittanceControllerOnce_clicked()
{

    // Eigen::Matrix<double, 6, 1> measured_force = Eigen::Matrix<double, 6, 1>::Zero();
    // measured_force(2) =forceData[2];
    // offset = controller->update(measured_force);
    // // 格式化字符串并更新UI
    // QString str = QString("Offset - Position: [%1, %2, %3] mm, Rotation: [%4, %5, %6] rad")
    //                   .arg(offset.x, 0, 'f', 3)
    //                   .arg(offset.y, 0, 'f', 3)
    //                   .arg(offset.z, 0, 'f', 3)
    //                   .arg(offset.rx, 0, 'f', 4)
    //                   .arg(offset.ry, 0, 'f', 4)
    //                   .arg(offset.rz, 0, 'f', 4);

    // UpdateUI(str);
    emit requestPosition();
    optimizer_->setWarmupState(robotPose.joint_positions);
    auto joint=forceExecutor->computeSingleForceControlJoint();
    jointOnce={joint};
}

identification_param AdmittanceControllerForm::loadIdentificationParam(QSettings *iniRead)
{
    identification_param param;
    iniRead->beginGroup("IdentificationParam");

    param.G = iniRead->value("G", 9.81).toDouble();  // 默认9.81

    param.L[0] = iniRead->value("L_x", 0.0).toDouble();
    param.L[1] = iniRead->value("L_y", 0.0).toDouble();
    param.L[2] = iniRead->value("L_z", 0.0).toDouble();

    param.U = iniRead->value("U", 0.0).toDouble();
    param.V = iniRead->value("V", 0.0).toDouble();

    param.mass_center[0] = iniRead->value("mass_center_x", 0.0).toDouble();
    param.mass_center[1] = iniRead->value("mass_center_y", 0.0).toDouble();
    param.mass_center[2] = iniRead->value("mass_center_z", 0.0).toDouble();

    for (int i = 0; i < 6; ++i) {
        // value默认0.0，key名为 zero_point_0 到 zero_point_5
        param.zero_point[i] = iniRead->value(QString("zero_point_%1").arg(i), 0.0).toDouble();
    }

    iniRead->endGroup();

    return param;
}

void AdmittanceControllerForm::saveIdentificationParam(QSettings *iniWrite, const identification_param &param)
{
    iniWrite->beginGroup("IdentificationParam");

    iniWrite->setValue("G", param.G);

    iniWrite->setValue("L_x", param.L[0]);
    iniWrite->setValue("L_y", param.L[1]);
    iniWrite->setValue("L_z", param.L[2]);

    iniWrite->setValue("U", param.U);
    iniWrite->setValue("V", param.V);

    iniWrite->setValue("mass_center_x", param.mass_center[0]);
    iniWrite->setValue("mass_center_y", param.mass_center[1]);
    iniWrite->setValue("mass_center_z", param.mass_center[2]);

    for (int i = 0; i < 6; ++i) {
        iniWrite->setValue(QString("zero_point_%1").arg(i), param.zero_point[i]);
    }

    iniWrite->endGroup();

    iniWrite->sync(); // 立刻写入文件
}

void AdmittanceControllerForm::onPositionUpdated(const RobotPose &pose)
{
    robotPose = pose;
    // // 使用最新的robotPose数据，更新界面逻辑等
    // qDebug() << "收到最新robotPose，TCP坐标："
    //          << robotPose.tcp_pose.x << robotPose.tcp_pose.y << robotPose.tcp_pose.z;
}

void AdmittanceControllerForm::onForceDataReady(const QVector<double> &data)
{
    forceData=data;
    Eigen::Matrix<double, 6, 1> eData;
    // 手动赋值（QVector 顺序赋给 Eigen 向量）
    for (int i = 0; i < 6; ++i) {
            eData(i) = data[i];
    }
    auto compensateData=compensator->compensate(eData, getRotation(toEigenMatrix(robotPose.tcp_pose)));
    if(compensateData.size()==3)
    {
        ui->Sensor_XForce->setText(QString::number(compensateData[0], 'f', 2));
        ui->Sensor_YForce->setText(QString::number(compensateData[1], 'f', 2));
        ui->Sensor_ZForce->setText(QString::number(compensateData[2], 'f', 2));
    }
    else if(compensateData.size()==6)
    {
        ui->Sensor_XForce->setText(QString::number(compensateData[0], 'f', 2));
        ui->Sensor_YForce->setText(QString::number(compensateData[1], 'f', 2));
        ui->Sensor_ZForce->setText(QString::number(compensateData[2], 'f', 2));
        ui->Sensor_XMForce->setText(QString::number(compensateData[3], 'f', 2));
        ui->Sensor_YMForce->setText(QString::number(compensateData[4], 'f', 2));
        ui->Sensor_ZMForce->setText(QString::number(compensateData[5], 'f', 2));
    }
    else
        qWarning()<<"ForceDataError!!!";
    // forceData=compensateData;

    // 创建一个 6x1 的 Eigen 向量
    Eigen::Matrix<double, 6, 1> eigenData;

    // 手动赋值（QVector 顺序赋给 Eigen 向量）
    for (int i = 0; i < 6; ++i) {
        if(i==2)
            eigenData(i) = compensateData[i];
        else
            eigenData(i)=0;
    }
    forceExecutor->updateSensorData(eigenData);
}

void AdmittanceControllerForm::receiveTrajectory(const std::vector<Matrix4d> &traj)
{
    trajectory=traj;
    forceExecutor->setPlannedTrajectory(trajectory);
    updateCurrentPosiSpin();
}

void AdmittanceControllerForm::receiveIdle(const bool idle)
{
    idle_=idle;
}

void AdmittanceControllerForm::receiveCurrentEndMoveJoint(const std::array<double, 8> currentEndMoveJoint)
{
    currentEndMoveJoint_=currentEndMoveJoint;
}


void AdmittanceControllerForm::on_btnSaveResult_clicked()
{
    id_param=compensator->getIdentificationParam();
    saveIdentificationParam(iniRead,id_param);
}


void AdmittanceControllerForm::on_btnLoadResult_clicked()
{
    id_param = loadIdentificationParam(iniRead);
    compensator->setIdentification_param(id_param);
}

void AdmittanceControllerForm::saveIntegralParams(QSettings* iniWrite, const IntegralParams& params)
{
    iniWrite->beginGroup("IntegralParams");

    // mass 每个元素写一个键
    for (int i = 0; i < 6; ++i) {
        iniWrite->setValue(QString("mass_%1").arg(i), params.mass(i));
    }

    for (int i = 0; i < 6; ++i) {
        iniWrite->setValue(QString("damping_%1").arg(i), params.damping(i));
    }

    for (int i = 0; i < 6; ++i) {
        iniWrite->setValue(QString("stiffness_%1").arg(i), params.stiffness(i));
    }

    for (int i = 0; i < 6; ++i) {
        iniWrite->setValue(QString("eta_%1").arg(i), params.eta(i));
    }

    for (int i = 0; i < 6; ++i) {
        iniWrite->setValue(QString("k_e_%1").arg(i), params.k_e(i));
    }

    for (int i = 0; i < 6; ++i) {
        iniWrite->setValue(QString("integral_max_%1").arg(i), params.integral_max(i));
    }

    for (int i = 0; i < 6; ++i) {
        iniWrite->setValue(QString("integral_decay_%1").arg(i), params.integral_decay(i));
    }

    iniWrite->setValue("dt", params.dt);

    iniWrite->endGroup();

    iniWrite->sync(); // 立刻写入文件
}

IntegralParams AdmittanceControllerForm::loadIntegralParams(QSettings* iniRead)
{
    IntegralParams params;
    iniRead->beginGroup("IntegralParams");

    for (int i = 0; i < 6; ++i) {
        params.mass(i) = iniRead->value(QString("mass_%1").arg(i), params.mass(i)).toDouble();
        params.damping(i) = iniRead->value(QString("damping_%1").arg(i), params.damping(i)).toDouble();
        params.stiffness(i) = iniRead->value(QString("stiffness_%1").arg(i), params.stiffness(i)).toDouble();

        params.eta(i) = iniRead->value(QString("eta_%1").arg(i), params.eta(i)).toDouble();
        params.k_e(i) = iniRead->value(QString("k_e_%1").arg(i), params.k_e(i)).toDouble();
        params.integral_max(i) = iniRead->value(QString("integral_max_%1").arg(i), params.integral_max(i)).toDouble();
        params.integral_decay(i) = iniRead->value(QString("integral_decay_%1").arg(i), params.integral_decay(i)).toDouble();
    }

    params.dt = iniRead->value("dt", params.dt).toDouble();

    iniRead->endGroup();

    return params;
}

void AdmittanceControllerForm::updateCurrentPosiSpin()
{
    ui->spbCurrentPosi->setValue(0);
    int index=ui->spbCurrentPosi->value();
    auto forceTrajectory=forceExecutor->getPlannedTrajectory();
    if(index>=forceTrajectory.size())
    {
        ui->spbCurrentPosi->setValue(forceTrajectory.size()-1);
        return;
    }
    forceExecutor->setCurrentTrajectoryIndex(index);
    auto posi=getPosition(forceTrajectory[ui->spbCurrentPosi->value()]);
    ui->lblCurrentPosiX->setText(QString::number(posi(0), 'f', 3));
    ui->lblCurrentPosiY->setText(QString::number(posi(1), 'f', 3));
    ui->lblCurrentPosiZ->setText(QString::number(posi(2), 'f', 3));
}

void AdmittanceControllerForm::on_btnSetParams_clicked()
{
    bool ok = false;

    double m_t = ui->massTransEdit->text().toDouble(&ok);
    if (!ok) { QMessageBox::warning(this, "错误", "平动质量m_t输入格式错误"); return; }

    double d_t = ui->dampingTransEdit->text().toDouble(&ok);
    if (!ok) { QMessageBox::warning(this, "错误", "平动阻尼d_t输入格式错误"); return; }

    double k_t = ui->stiffnessTransEdit->text().toDouble(&ok);
    if (!ok) { QMessageBox::warning(this, "错误", "平动刚度k_t输入格式错误"); return; }

    double m_r = ui->massRotEdit->text().toDouble(&ok);
    if (!ok) { QMessageBox::warning(this, "错误", "旋转质量m_r输入格式错误"); return; }

    double d_r = ui->dampingRotEdit->text().toDouble(&ok);
    if (!ok) { QMessageBox::warning(this, "错误", "旋转阻尼d_r输入格式错误"); return; }

    double k_r = ui->stiffnessRotEdit->text().toDouble(&ok);
    if (!ok) { QMessageBox::warning(this, "错误", "旋转刚度k_r输入格式错误"); return; }

    double eta_val = ui->etaEdit->text().toDouble(&ok);
    if (!ok) { QMessageBox::warning(this, "错误", "积分增益η输入格式错误"); return; }

    double ke_val = ui->k_eEdit->text().toDouble(&ok);
    if (!ok) { QMessageBox::warning(this, "错误", "积分增益k_e输入格式错误"); return; }

    double int_max = ui->integralMaxEdit->text().toDouble(&ok);
    if (!ok) { QMessageBox::warning(this, "错误", "积分限幅输入格式错误"); return; }

    double decay = ui->integralDecayEdit->text().toDouble(&ok);
    if (!ok) { QMessageBox::warning(this, "错误", "积分衰减输入格式错误"); return; }

    double dt_val = ui->dtEdit->text().toDouble(&ok);
    if (!ok) { QMessageBox::warning(this, "错误", "采样周期dt输入格式错误"); return; }


    integralParams = IntegralParams(m_t, d_t, k_t, m_r, d_r, k_r, eta_val, ke_val, int_max, decay, dt_val);

}


void AdmittanceControllerForm::on_btnSaveParams_clicked()
{
    bool ok = false;

    double m_t = ui->massTransEdit->text().toDouble(&ok);
    if (!ok) { QMessageBox::warning(this, "错误", "平动质量m_t输入格式错误"); return; }

    double d_t = ui->dampingTransEdit->text().toDouble(&ok);
    if (!ok) { QMessageBox::warning(this, "错误", "平动阻尼d_t输入格式错误"); return; }

    double k_t = ui->stiffnessTransEdit->text().toDouble(&ok);
    if (!ok) { QMessageBox::warning(this, "错误", "平动刚度k_t输入格式错误"); return; }

    double m_r = ui->massRotEdit->text().toDouble(&ok);
    if (!ok) { QMessageBox::warning(this, "错误", "旋转质量m_r输入格式错误"); return; }

    double d_r = ui->dampingRotEdit->text().toDouble(&ok);
    if (!ok) { QMessageBox::warning(this, "错误", "旋转阻尼d_r输入格式错误"); return; }

    double k_r = ui->stiffnessRotEdit->text().toDouble(&ok);
    if (!ok) { QMessageBox::warning(this, "错误", "旋转刚度k_r输入格式错误"); return; }

    double eta_val = ui->etaEdit->text().toDouble(&ok);
    if (!ok) { QMessageBox::warning(this, "错误", "积分增益η输入格式错误"); return; }

    double ke_val = ui->k_eEdit->text().toDouble(&ok);
    if (!ok) { QMessageBox::warning(this, "错误", "积分增益k_e输入格式错误"); return; }

    double int_max = ui->integralMaxEdit->text().toDouble(&ok);
    if (!ok) { QMessageBox::warning(this, "错误", "积分限幅输入格式错误"); return; }

    double decay = ui->integralDecayEdit->text().toDouble(&ok);
    if (!ok) { QMessageBox::warning(this, "错误", "积分衰减输入格式错误"); return; }

    double dt_val = ui->dtEdit->text().toDouble(&ok);
    if (!ok)
            return;
    // 构造参数
    auto params= IntegralParams(m_t, d_t, k_t, m_r, d_r, k_r, eta_val, ke_val, int_max, decay, dt_val);
    saveIntegralParams(iniRead,params);
}


void AdmittanceControllerForm::on_btnLoadParams_clicked()
{
    auto params=loadIntegralParams(iniRead);
    ui->massTransEdit->setText(QString::number(params.mass(0)));
    ui->dampingTransEdit->setText(QString::number(params.damping(0)));
    ui->stiffnessTransEdit->setText(QString::number(params.stiffness(0)));

    ui->massRotEdit->setText(QString::number(params.mass(3)));
    ui->dampingRotEdit->setText(QString::number(params.damping(3)));
    ui->stiffnessRotEdit->setText(QString::number(params.stiffness(3)));

    ui->etaEdit->setText(QString::number(params.eta(0)));
    ui->k_eEdit->setText(QString::number(params.k_e(0)));
    ui->integralMaxEdit->setText(QString::number(params.integral_max(0)));
    ui->integralDecayEdit->setText(QString::number(params.integral_decay(0)));

    ui->dtEdit->setText(QString::number(params.dt));

}

void AdmittanceControllerForm::setOptimizer(TrajectoryOptimizer *optimizer)
{
    optimizer_ = optimizer;
    forceExecutor=new ForceControlExecutor(controller,optimizer_);
    forceExecutor->setControlPeriod(50); // 50ms控制周期
    forceExecutor->setLogCallback([this](const std::string& msg) {
        UpdateUI( QString::fromStdString("[ForceControl] "+ msg ));
    });
    forceExecutor->setStatusCallback([this](const std::string& msg) {
        UpdateUI( QString::fromStdString("[ForceControl] "+ msg ));
    });
    // 发送运动指令回调
    forceExecutor->setSendMotionCallback(
        [this](const std::vector<std::array<double, 8>>& jointTraj) {
            emit MoveJointTraj(jointTraj);
        });
    // 等待运动完成回调
    forceExecutor->setIsMotionCompleteCallback(
        [this](){
            emit requestIdle();
            return idle_;
        });
    // 在外部调用时设置回调
    // forceExecutor->setGetCurrentJointsCallback([this](){
    //     emit requestCurrentEndMoveJoint();
    //     return currentEndMoveJoint_;
    // });
    forceExecutor->setGetCurrentJointsCallback([this](){
        emit requestPosition();
        return robotPose.joint_positions;
    });
}

void AdmittanceControllerForm::on_btnCurrentPosition_clicked()
{
    emit requestPosition();
    int index=ui->spbCurrentPosi->value();
    if(trajectory.empty())
        return;
    optimizer_->setWarmupState(robotPose.joint_positions);

    auto joint=optimizer_->optimizeSinglePoint(trajectory[index]);
    std::vector<std::array<double, 8>> jointTraj{joint};

    emit MoveJointTraj(jointTraj);
}


void AdmittanceControllerForm::on_spbCurrentPosi_editingFinished()
{
    int index=ui->spbCurrentPosi->value();
    auto forceTrajectory=forceExecutor->getPlannedTrajectory();
    if(index>=forceTrajectory.size())
    {
        ui->spbCurrentPosi->setValue(forceTrajectory.size()-1);
        return;
    }
    forceExecutor->setCurrentTrajectoryIndex(index);
    auto posi=getPosition(forceTrajectory[ui->spbCurrentPosi->value()]);

    ui->lblCurrentPosiX->setText(QString::number(posi(0), 'f', 3));
    ui->lblCurrentPosiY->setText(QString::number(posi(1), 'f', 3));
    ui->lblCurrentPosiZ->setText(QString::number(posi(2), 'f', 3));
}


void AdmittanceControllerForm::on_btnStopForceExecute_clicked()
{
    // forceExecutor->stop();
    std::thread([this]() {
        forceExecutor->stop();
    }).detach();
}


void AdmittanceControllerForm::on_btnStartForceExecute_clicked()
{
    emit requestPosition();
    optimizer_->setWarmupState(robotPose.joint_positions);
    forceExecutor->start();
}


void AdmittanceControllerForm::on_btnLoadMainWindowData_clicked()
{
    emit requestTrajectory();
}

