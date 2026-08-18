#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    iniRead = new QSettings("lib/param.ini", QSettings::IniFormat);
    ui->setupUi(this);

    timerInit();
    ZMotionInit();
    iniInit();
    robotInit();
    btnInit();
    SerialPortInit();
    MDHInit();
    AdmittanceControllerInit();
    //cameraInit();


}

MainWindow::~MainWindow() {
        mvGetTimer->stop();

    if (zm) {
        zm->stopCallbackThread();
        zm->cleanup();
    }

    delete ui;

}
void MainWindow::ZMotionInit()
{
    zm=ZMotionControl::getinstance();

    allAxisStatus=new AxisStatus[ZMotionControl::AxisNum];
    pastallAxisStatus=new AxisStatus[ZMotionControl::AxisNum];
    for(int i=0;i<ZMotionControl::AxisNum;i++)
    {
        allAxisStatus[i]=AxisStatus(i);
        pastallAxisStatus[i]=AxisStatus(i);
    }
    // 启动回调线程，并传入主线程的回调函数
    // 使用 lambda 表达式包装成员函数指针
    zm->startCallbackThread([this](ZmotionStatus* status) {
        handleDataFetched(status);
    });    //回调函数获取数据
}

void MainWindow::getPosition()
{
    //获取实时的轴坐标
    robotPose.joint_positions = {allAxisStatus[0].posi*10, allAxisStatus[1].posi*10+200, allAxisStatus[2].posi*10, allAxisStatus[3].posi/180.0*PI, allAxisStatus[4].posi/180.0*PI, allAxisStatus[5].posi/180.0*PI, allAxisStatus[6].posi/180.0*PI, allAxisStatus[7].posi/180.0*PI};
    //坐标计算
    robotPose.flange_matrix4d = arm->forwardKinematics(robotPose.joint_positions);
    robotPose.world_pose=fromEigenMatrix(robotPose.flange_matrix4d);
    robotPose.tcp_pose=fromEigenMatrix(arm->getTCPTransform(robotPose.joint_positions));

    ui->lblFlangePosi->setText(QString::number(robotPose.world_pose.x,'f',3)+","
                               +QString::number(robotPose.world_pose.y,'f',3)+","
                               +QString::number(robotPose.world_pose.z,'f',3));
    ui->lblFlangeAngle->setText(QString::number(robotPose.world_pose.rx,'f',3)+","
                                +QString::number(robotPose.world_pose.ry,'f',3)+","
                                +QString::number(robotPose.world_pose.rz,'f',3));
    ui->lblTCPPosi->setText(QString::number(robotPose.tcp_pose.x,'f',3)+","
                            +QString::number(robotPose.tcp_pose.y,'f',3)+","
                            +QString::number(robotPose.tcp_pose.z,'f',3));
    emit positionUpdated(robotPose);

}

void MainWindow::on_btnBackward_pressed(int i)
{
    if(!zm->GetConnectStatus())
    {
        UpdateUI("未连接控制器");
        qInfo()<<"未连接控制器";
        return;
    }
    zm->SingleVMove(i,Backword);
}

void MainWindow::on_btnForward_pressed(int i)
{
    if(!zm->GetConnectStatus())
    {
        UpdateUI("未连接控制器");
        qInfo()<<"未连接控制器";
        return;
    }
    zm->SingleVMove(i,Forward);
}

void MainWindow::on_btnZero_clicked(int i)
{
    if(!zm->GetConnectStatus())
    {
        UpdateUI("未连接控制器");
        qInfo()<<"未连接控制器";
        return;
    }
    zm->MoveZero(i);
}

void MainWindow::on_btnInit_clicked(int i)
{
    if(!zm->GetConnectStatus())
    {
        UpdateUI("未连接控制器");
        qInfo()<<"未连接控制器";
        return;
    }
    zm->MoveInit(i);
}

void MainWindow::on_btn_released(int i)
{
    if(!zm->GetConnectStatus())
    {
        UpdateUI("未连接控制器");
        qInfo()<<"未连接控制器";
        return;
    }
    zm->SingleVMove(i,Cancel);
}


// 该函数将label控件变成一个圆形指示灯，需要指定颜色color以及直径size
// color 0:grey 1:red 2:green 3:yellow
// size  单位是像素
void MainWindow::setLED(QLabel* label, int color, int size)
{
    // 将label中的文字清空
    label->setText("");
    // 先设置矩形大小
    // 如果ui界面设置的label大小比最小宽度和高度小，矩形将被设置为最小宽度和最小高度；
    // 如果ui界面设置的label大小比最小宽度和高度大，矩形将被设置为最大宽度和最大高度；
    QString min_width = QString("min-width: %1px;").arg(size);              // 最小宽度：size
    QString min_height = QString("min-height: %1px;").arg(size);            // 最小高度：size
    QString max_width = QString("max-width: %1px;").arg(size);              // 最小宽度：size
    QString max_height = QString("max-height: %1px;").arg(size);            // 最小高度：size
    // 再设置边界形状及边框
    QString border_radius = QString("border-radius: %1px;").arg(size/2);    // 边框是圆角，半径为size/2
    QString border = QString("border:1px solid black;");                    // 边框为1px黑色
    // 最后设置背景颜色
    QString background = "background-color:";
    switch (color) {
    case 0:
        // 灰色
        background += "rgb(190,190,190)";
        break;
    case 1:
        // 红色
        background += "rgb(255,0,0)";
        break;
    case 2:
        // 绿色
        background += "rgb(0,255,0)";
        break;
    case 3:
        // 黄色
        background += "rgb(255,255,0)";
        break;
    default:
        break;
    }

    const QString SheetStyle = min_width + min_height + max_width + max_height + border_radius + border + background;
    label->setStyleSheet(SheetStyle);
}

void MainWindow::btnInit()
{
    QPushButton *buttonName[8][4]={
        {ui->btnForward0,ui->btnBackward0,ui->btnZero0,ui->btnInit0},
        {ui->btnForward1,ui->btnBackward1,ui->btnZero1,ui->btnInit1},
        {ui->btnForward2,ui->btnBackward2,ui->btnZero2,ui->btnInit2},
        {ui->btnForward3,ui->btnBackward3,ui->btnZero3,ui->btnInit3},
        {ui->btnForward4,ui->btnBackward4,ui->btnZero4,ui->btnInit4},
        {ui->btnForward5,ui->btnBackward5,ui->btnZero5,ui->btnInit5},
        {ui->btnForward6,ui->btnBackward6,ui->btnZero6,ui->btnInit6},
        {ui->btnForward7,ui->btnBackward7,ui->btnZero7,ui->btnInit7}
    };
    QSignalMapper *signalMapper[5];//QSignalMapper 可以将这些信号映射到特定的参数
    //[0]on_btnForward_pressed
    //[1]on_btnBackward_pressed
    //[2]on_btnZero_clicked
    //[3]on_btnInit_clicked
    //[4]on_btn_released
    for(int i=0;i<5;i++)
        signalMapper[i] = new QSignalMapper(this); // 初始化 QSignalMapper

    for (int i = 0; i < ZMotionControl::AxisNum; i++) {
        // 映射按钮到对应的参数
        connect(buttonName[i][0], SIGNAL(pressed()), signalMapper[0], SLOT(map()));
        connect(buttonName[i][1], SIGNAL(pressed()), signalMapper[1], SLOT(map()));
        connect(buttonName[i][0], SIGNAL(released()), signalMapper[4], SLOT(map()));
        connect(buttonName[i][1], SIGNAL(released()), signalMapper[4], SLOT(map()));

        connect(buttonName[i][2], SIGNAL(pressed()), signalMapper[2], SLOT(map()));
        connect(buttonName[i][3], SIGNAL(pressed()), signalMapper[3], SLOT(map()));


        signalMapper[0]->setMapping(buttonName[i][0], i); // 映射到 i
        signalMapper[1]->setMapping(buttonName[i][1], i); // 映射到 i
        signalMapper[2]->setMapping(buttonName[i][2], i); // 映射到 i
        signalMapper[3]->setMapping(buttonName[i][3], i); // 映射到 i
        signalMapper[4]->setMapping(buttonName[i][0], i); // 映射到 i
        signalMapper[4]->setMapping(buttonName[i][1], i); // 映射到 i
    }

    // 连接 QSignalMapper 的信号到槽
    connect(signalMapper[0], SIGNAL(mapped(int)), this, SLOT(on_btnForward_pressed(int)));
    connect(signalMapper[1], SIGNAL(mapped(int)), this, SLOT(on_btnBackward_pressed(int)));
    connect(signalMapper[2], SIGNAL(mapped(int)), this, SLOT(on_btnZero_clicked(int)));
    connect(signalMapper[3], SIGNAL(mapped(int)), this, SLOT(on_btnInit_clicked(int)));
    connect(signalMapper[4], SIGNAL(mapped(int)), this, SLOT(on_btn_released(int)));


    //笛卡尔坐标系运动
    QPushButton *buttonNameXYZ[6][2]={
        {ui->btnXPlus,ui->btnXMinus},
        {ui->btnYPlus,ui->btnYMinus},
        {ui->btnZPlus,ui->btnZMinus},
        {ui->btnRXPlus,ui->btnRXMinus},
        {ui->btnRYPlus,ui->btnRYMinus},
        {ui->btnRZPlus,ui->btnRZMinus}
    };
    // 创建两个 QSignalMapper (局部变量，生命周期到析构)
    QSignalMapper *signalMapperPressed = new QSignalMapper(this);
    QSignalMapper *signalMapperReleased = new QSignalMapper(this);

    // 循环：预设映射 + 宏 connect
    for (int axis = 0; axis < 6; ++axis) {
        // 正方向 (e.g., X+ -> 1)
        signalMapperPressed->setMapping(buttonNameXYZ[axis][0], axis + 1);
        connect(buttonNameXYZ[axis][0], SIGNAL(pressed()), signalMapperPressed, SLOT(map()));

        // 负方向 (e.g., X- -> -1)
        signalMapperPressed->setMapping(buttonNameXYZ[axis][1], -(axis + 1));
        connect(buttonNameXYZ[axis][1], SIGNAL(pressed()), signalMapperPressed, SLOT(map()));

        // 释放：统一 0
        signalMapperReleased->setMapping(buttonNameXYZ[axis][0], 0);
        connect(buttonNameXYZ[axis][0], SIGNAL(released()), signalMapperReleased, SLOT(map()));

        signalMapperReleased->setMapping(buttonNameXYZ[axis][1], 0);
        connect(buttonNameXYZ[axis][1], SIGNAL(released()), signalMapperReleased, SLOT(map()));
    }

    // 连接 mapper 到槽：用宏（安全）
    connect(signalMapperPressed, SIGNAL(mapped(int)), this, SLOT(on_btnXYZ_Pressed(int)));
    connect(signalMapperReleased, SIGNAL(mapped(int)), this, SLOT(on_btnXYZ_Released(int)));
}

void MainWindow::iniInit()
{
    ui->txtEditConnectContent->setText(iniRead->value("/IP/ip").toString());
    ui->cBoxConnectType->setCurrentIndex(iniRead->value("/IP/com").toInt()-1);
}

void MainWindow::timerInit()
{
    connect(this, SIGNAL(UpdateUI(QString)), this, SLOT(UpdateUILog(QString)));
    connect(ui->UpdateUI,SIGNAL(textChanged()),SLOT(UpdateUIChanged()));
    mvGetTimer = new QTimer();
    mvGetTimer->setInterval(1000);
    connect(mvGetTimer, SIGNAL(timeout()), this, SLOT(onMvGetTimerOut()));
    jogTimer=new QTimer();
    connect(jogTimer, &QTimer::timeout, this, &MainWindow::onJogTimerTimeout);
    setLED(ui->ledMotion, 1, 32);
}

void MainWindow::robotInit()
{
    arm=new RobotArm(262);//187.14or127//218.4tcp相机末端//232力末端不带牛眼轴承！！！！！！+20+30。
    collision_system=new CollisionSystem(*arm);

    // RGB 颜色设定（均为 0~1 范围浮点数）
    const auto BLUE = std::array<float, 3>{0.4, 0.6, 1.0};  // 淡蓝色
    // 获取并打印当前路径
    QString currentPath = QDir::currentPath();
    // 推荐路径拼接方式
    QString baseQFile = QDir(currentPath).filePath("lib/data/trajectory/base.csv");
    QString robotQFile = QDir(currentPath).filePath("lib/data/trajectory/robot.csv");
    std::string baseFile = baseQFile.toStdString();
    std::string robotFile = robotQFile.toStdString();
    // 4. 加载动态模型 (假设前 8点属于关节 0，后 8点属于关节 1)
    collision_system->LoadRobotOBBFromCSV(
        robotFile
    );
    collision_system->LoadAABBFromCSV(
        baseFile,
        BLUE                      // 颜色
    );

    compensator=new PlaneAlignmentCompensator(); // 启用调试

    OptimizeParams ga_params;
    GAOptimizer = TrajectoryOptimizer::create("GA", *arm, *collision_system, ga_params);
}

///////////////////////////串口////////////////////////////////////////
void MainWindow::SerialPortInit()
{
    // 创建串口管理器
    m_serialManager = new LanserSerialPortManager(this);

    // 连接信号

    connect(m_serialManager, &BaseSerialPortManager::dataReceived,
            this, &MainWindow::onDataReceived);
    connect(m_serialManager, &BaseSerialPortManager::errorOccurred,
            this, &MainWindow::onErrorReceived);
    connect(m_serialManager, &BaseSerialPortManager::processedDataReady,
            this, &MainWindow::onDataUpdated);


    // 初始化串口列表
    ui->comboBoxPortName->addItems(m_serialManager->getAvailablePorts());

    QStringList bautRatesList;
    bautRatesList << "1200" << "2400" << "4800" << "9600" << "19200" << "57600" << "115200";
    ui->comboBoxBaudRate->addItems(bautRatesList);
    ui->comboBoxBaudRate->setCurrentIndex(6);

    ui->txtSend->setText("010301f400080402");

    //六维力传感器初始化
    forceForm=new ForceForm(ui->ForceFormWidget);
    QVBoxLayout* layout = new QVBoxLayout(ui->ForceFormWidget);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(forceForm);

}


void MainWindow::on_btnRefreshPortsbtn_clicked()
{
    QStringList availablePorts = m_serialManager->getAvailablePorts();
    ui->comboBoxPortName->clear();
    ui->comboBoxPortName->addItems(availablePorts);

    qDebug() << "Refreshed ports:" << availablePorts;
}

void MainWindow::on_btnOpenCOM_clicked()
{
    if (ui->btnOpenCOM->text()=="打开串口")
    {
        QString portName = ui->comboBoxPortName->currentText();
        int baudRate = ui->comboBoxBaudRate->currentText().toInt();
        if (m_serialManager->openPort(portName, baudRate)) {
            ui->btnOpenCOM->setText("关闭串口");
        }

    } else
    {
        m_serialManager->closePort();
        ui->btnOpenCOM->setText("打开串口");
    }
}

void MainWindow::onDataReceived(QString hexData)
{
    ui->txtReceiveData->append(hexData);
}

void MainWindow::onErrorReceived(const QString &error)
{
    UpdateUI(error);
}

void MainWindow::onDataUpdated(const QVector<double> value)
{
    qDebug() << "onDataUpdated called with size:" << value.size();

    if (value.isEmpty()) {
        qDebug() << "Warning: received empty data!";
        return;
    }

    for(int i=0; i<value.size(); i++)
    {
        qDebug() << i << ":" << value[i];
    }

    // 确保至少有3个值
    if(value.size() < 3) {
        qDebug() << "Warning: not enough data, size =" << value.size();
        return;
    }

    ui->Sensor1->setText(QString::number(value[0], 'f', 2));
    ui->Sensor2->setText(QString::number(value[1], 'f', 2));
    ui->Sensor3->setText(QString::number(value[2], 'f', 2));

    qDebug() << "UI updated successfully";
}

void MainWindow::AdmittanceControllerInit()
{
    //六维力传感器初始化
    admittanceControllerForm=new AdmittanceControllerForm(ui->AdmittanceControllerWidget);
    admittanceControllerForm->setOptimizer(GAOptimizer.get());

    QVBoxLayout* layout = new QVBoxLayout(ui->AdmittanceControllerWidget);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(admittanceControllerForm);

    // 连接子窗口请求刷新信号 -> 主窗口调用getPosition
    connect(admittanceControllerForm, &AdmittanceControllerForm::requestPosition,
            this, &MainWindow::getPosition);

    // 连接主窗口发出数据更新信号 -> 子窗口处理
    connect(this, &MainWindow::positionUpdated,
            admittanceControllerForm, &AdmittanceControllerForm::onPositionUpdated);
    //连接六维力传感器窗口
    connect(forceForm, &ForceForm::forceDataReady,
            admittanceControllerForm, &AdmittanceControllerForm::onForceDataReady);
    //连接UI
    connect(admittanceControllerForm, &AdmittanceControllerForm::UpdateUI,this, &MainWindow::UpdateUILog);

    connect(admittanceControllerForm, &AdmittanceControllerForm::MoveJointTraj,this, &MainWindow::onMoveJointTraj);
    connect(admittanceControllerForm, &AdmittanceControllerForm::requestIdle,this, &MainWindow::getIdle);
    connect(this, &MainWindow::isIdle, admittanceControllerForm, &AdmittanceControllerForm::receiveIdle);

    // 连接子页面请求信号到主页面请求槽
    connect(admittanceControllerForm, &AdmittanceControllerForm::requestTrajectory, this, &MainWindow::onRequestTrajectory);

    // 连接主页面轨迹信号到子页面轨迹槽
    connect(this, &MainWindow::sendTrajectory, admittanceControllerForm, &AdmittanceControllerForm::receiveTrajectory);

    connect(admittanceControllerForm, &AdmittanceControllerForm::requestCurrentEndMoveJoint,this, &MainWindow::getCurrentEndMoveJoint);
    connect(this, &MainWindow::sendCurrentEndMoveJoint, admittanceControllerForm, &AdmittanceControllerForm::receiveCurrentEndMoveJoint);

}

void MainWindow::on_btnAutoSendData_clicked()
{
    if (ui->btnAutoSendData->text() == "自动发送关闭") {
        m_serialManager->stopAutoPolling();
        ui->btnAutoSendData->setText("自动发送开启");
    } else {
        QVector<quint8> stations = {0x01, 0x02, 0x03};
        //QVector<quint8> stations = {0x01};

        m_serialManager->startAutoPolling(stations, 50);
        ui->btnAutoSendData->setText("自动发送关闭");
    }
}

void MainWindow::on_btnSendData_clicked()
{
    QString m_strSendData = ui->txtSend->text().trimmed();

    if (m_strSendData.isEmpty()) {
        qDebug() << "发送数据为空！";
        return;
    }

    QByteArray sendBuf;

    if (ui->checkBoxHexSend->isChecked()) {
        // Hex模式发送
        sendBuf = BaseSerialPortManager::convertHexStringToByteArray(m_strSendData);
        if (sendBuf.isEmpty()) {
            qDebug() << "Hex数据转换失败，未发送";
            return;
        }
    } else {
        // 文本模式发送
        sendBuf = m_strSendData.toLocal8Bit();
    }

    // 实际发送数据
    m_serialManager->sendData(sendBuf);
}

//////////////////////////////////////////
void MainWindow::MDHInit()
{

    QLineEdit* MDH[8][4] = {
        {ui->lineEditAlpha0,ui->lineEditA0,ui->lineEditTheta0,ui->lineEditD0},
        {ui->lineEditAlpha1,ui->lineEditA1,ui->lineEditTheta1,ui->lineEditD1},
        {ui->lineEditAlpha2,ui->lineEditA2,ui->lineEditTheta2,ui->lineEditD2},
        {ui->lineEditAlpha3,ui->lineEditA3,ui->lineEditTheta3,ui->lineEditD3},
        {ui->lineEditAlpha4,ui->lineEditA4,ui->lineEditTheta4,ui->lineEditD4},
        {ui->lineEditAlpha5,ui->lineEditA5,ui->lineEditTheta5,ui->lineEditD5},
        {ui->lineEditAlpha6,ui->lineEditA6,ui->lineEditTheta6,ui->lineEditD6},
        {ui->lineEditAlpha7,ui->lineEditA7,ui->lineEditTheta7,ui->lineEditD7}
    };
    for (int i = 0; i < 8; i++)
    {
        MDH[i][0]->setText(iniRead->value(QString::number(0) + "/MDH" + QString::number(i) + "/Alpha").toString());
        MDH[i][1]->setText(iniRead->value(QString::number(0) + "/MDH" + QString::number(i) + "/A").toString());
        MDH[i][2]->setText(iniRead->value(QString::number(0) + "/MDH" + QString::number(i) + "/Theta").toString());
        MDH[i][3]->setText(iniRead->value(QString::number(0) + "/MDH" + QString::number(i) + "/D").toString());

    }
    for (int i = 0; i < 8; i++)
    {
        arm->modifyMDHParam(i, MDH[i][1]->text().toFloat(), MDH[i][0]->text().toFloat(), MDH[i][3]->text().toFloat(), MDH[i][2]->text().toFloat());
    }

    QLabel* Block[4][2] = {
        {ui->lblBlockUpperPosi0,ui->lblBlockLowerPosi0},
        {ui->lblBlockUpperPosi1,ui->lblBlockLowerPosi1},
        {ui->lblBlockUpperPosi2,ui->lblBlockLowerPosi2},
        {ui->lblBlockUpperPosi3,ui->lblBlockLowerPosi3}


    };
    for (int i = 0; i < 4; i++)
    {
        Block[i][0]->setText(iniRead->value(QString::number(0) + "/Block" + QString::number(i) + "/UpperPosi").toString());
        Block[i][1]->setText(iniRead->value(QString::number(0) + "/Block" + QString::number(i) + "/LowerPosi").toString());

    }
    ui->BlockDescription->setText(iniRead->value(QString::number(0) + "/Block" + "/Descripition").toString());
    UpdateUI("读取第"+ QString::number(0) +"个Block数据");

}



void MainWindow::handleDataFetched(ZmotionStatus *packet)
{
    if(!zm->GetConnectStatus())
        return;

    //qDebug()<<QString::number(packet[0].fslimit, 'f', 3);
    for(int i=0;i<ZMotionControl::AxisNum;i++)
    {


        allAxisStatus[i].posi=packet->allAxisStatus[i].posi;
        allAxisStatus[i].fslimit=packet->allAxisStatus[i].fslimit;
        allAxisStatus[i].rslimit=packet->allAxisStatus[i].rslimit;
        allAxisStatus[i].status=packet->allAxisStatus[i].status;
        allAxisStatus[i].mtype=packet->allAxisStatus[i].mtype;

    }
    statusIOEmergencyStop=packet->allIOStatus->IOInputEmergencyStop;
}

void MainWindow::cameraInit()
{

    // 内参标定
    chessboardParams.board_size=cv::Size(8, 11);
    chessboardParams.square_size=6.0f;
    cameraParamsCalibrator =new CameraParamsCalibrator(chessboardParams);
    cameraParamsCalibrator->setFixedDepth(118);
    if(!cameraParamsCalibrator->loadParams("lib/data/calibration/CameraParamsCalibration/cameraParamsCalibrator.txt"))
    {
        UpdateUI("相机内参导入失败，需要重新标定");
    }
    else
    {
        UpdateUI("相机内参导入成功");
    }
    //tcp标定
    tCPCalibrator =new TCPCalibrator();
    if(!tCPCalibrator->loadTCPResult("lib/data/calibration/TCPCalibration/TCPCalibration.txt"))
    {
        UpdateUI("TCP导入失败，需要重新标定");
    }
    else
    {
        UpdateUI("TCP导入成功");
    }
    //手眼标定
    cameraParams = cameraParamsCalibrator->getCameraParams();

    // handEyeCalibration=new HandEyeCalibrator(chessboardParams);
    // calibrationResult=handEyeCalibration->loadCalibrationResult("lib/data/calibration/EyeInHandCalibration/EyeInHandCalibration.txt");
    // if(!calibrationResult.success)
    // {
    //     UpdateUI("手眼标定导入失败，需要重新标定");
    // }
    // else
    // {
    //     UpdateUI("手眼标定导入成功");
    // }

    //特征检测
    featureDetector=new FeatureDetector(cameraParams,calibrationResult);
    // 步骤 4: 配置检测参数 (DetectionParams)
    detectionParams.visualize = true;                       // 启用可视化
    detectionParams.circle_z_depth_mm = 150.0;              // 圆心 Z 深度假设 150mm
    detectionParams.circle_min_radius = 30;                 // 最小半径 30 像素
    detectionParams.circle_max_radius = 80;                 // 最大半径 80 像素
    detectionParams.compute_base_coords = true;             // 计算基坐标

    // 示例机器人当前位姿 (4x4 单位矩阵；实际从机器人 API 获取)
    detectionParams.current_robot_pose = cv::Mat::eye(4, 4, CV_64F);
    detectionParams.current_robot_pose.at<double>(0, 3) = 200.0;  // 示例平移 X=200mm

    // 棋盘格参数 (仅棋盘检测使用)
    detectionParams.chess_params=chessboardParams;


    // calibrator=new CameraCalibration() ;
    // // 设置棋盘格参数
    // CameraCalibration::ChessboardParams params;
    // params.boardSize = cv::Size(8, 11);
    // params.squareSize = 6.0f;
    // calibrator->setChessboardParams(params);
    // QString currentPath = QDir::currentPath();
    // QString dataQFile = QDir(currentPath).filePath("lib/data/calibration/");
    // std::string xmlPath=dataQFile.toStdString()+"camera_params.xml";
    // calibrator->loadCalibrationData(xmlPath);
    // calibrator->loadHandEyeData("lib/data/calibration/hand_eye_calibration.xml");

    dahengForm = new dahengTwoCams_qt_vs(ui->dahengWidget);
    QVBoxLayout* layout = new QVBoxLayout(ui->dahengWidget);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(dahengForm->centralWidget());
}

void MainWindow::onCaculateTrajectoryPoints(const std::vector<Matrix4d> &trajPoints)
{
    if (trajPoints.empty()) {
        UpdateUI("Trajectory is empty. Please load data first.");
        return;
    }

    if (asyncOptimizeTask) {
        asyncOptimizeTask.reset();  // 清理旧任务
    }
    jointTrajectory.clear();
    isOptimizing = true;

    try {
        auto task = GAOptimizer->optimizeTrajectoryAsync(
            trajPoints,
            [this](const std::string& progress) {
                QString msg = QString::fromStdString(progress);
                // QMetaObject::invokeMethod(this, [this, msg]() { UpdateUI(msg); }, Qt::QueuedConnection);
                UpdateUI(msg);
            }
            );

        asyncOptimizeTask = std::make_unique<AsyncOptimizeTask>(std::move(task));
        // 无需检查 isValid()：启动后自动有效

    } catch (const std::exception& e) {
        isOptimizing = false;
        UpdateUI(QString("Failed to start optimization: %1").arg(e.what()));
    }
}

void MainWindow::onCancelCaculateTrajectory()
{
    // 简化保护：指针 + 有效性
    if (!asyncOptimizeTask || !asyncOptimizeTask->isValid()) {
        UpdateUI("No valid task to cancel.");
        return;
    }
    if (!isOptimizing) {
        UpdateUI("No running task.");
        return;
    }

    try {
        // 直接调用 cancel()（内部自保护：无效时静默返回）
        asyncOptimizeTask->cancel();
        UpdateUI("Cancellation requested. Waiting for task to stop...");

        // 检查是否完成，并清理（get() 内部自保护）
        if (asyncOptimizeTask->isReady()) {
            try {
                auto result = asyncOptimizeTask->get();  // 捕获取消异常，忽略结果
                (void)result;
            } catch (const std::exception& e) {
                UpdateUI(QString("Task cancelled: %1").arg(e.what()));
            }
        }

        // 重置
        isOptimizing = false;
        asyncOptimizeTask.reset();

    } catch (const std::invalid_argument& e) {
        // 特定捕获：无效任务（虽 cancel 已自保护，但 get 可能触发）
        isOptimizing = false;
        asyncOptimizeTask.reset();
        UpdateUI(QString("Invalid task during cancel: %1").arg(e.what()));
    } catch (const std::exception& e) {
        // 其他异常
        isOptimizing = false;
        asyncOptimizeTask.reset();
        UpdateUI(QString("Error during cancellation: %1").arg(e.what()));
    }
}

// 返回是否成功
bool MainWindow::onTrajectoryPlanningFinished(std::vector<std::array<double, 8>>& outJointTrajectory)
{
    if (!asyncOptimizeTask || !asyncOptimizeTask->isValid()) {
        UpdateUI("No valid optimization task found. Please start calculation first.");
        return false;
    }
    if (!asyncOptimizeTask->isReady()) {
        UpdateUI("Optimization not completed yet. Please wait.");
        return false;
    }

    try {
        outJointTrajectory = asyncOptimizeTask->get();
        if (outJointTrajectory.empty()) {
            UpdateUI("Empty trajectory result");
            return false;
        }

        UpdateUI(QString("Loaded trajectory with %1 points").arg(outJointTrajectory.size()));

        // 清理状态
        isOptimizing = false;
        asyncOptimizeTask.reset();

        return true;
    } catch (const std::invalid_argument& e) {
        isOptimizing = false;
        UpdateUI(QString("Invalid task: %1").arg(e.what()));
        return false;
    } catch (const std::exception& e) {
        isOptimizing = false;
        UpdateUI(QString("Failed to load/export trajectory: %1").arg(e.what()));
        // 不重置任务，允许重试
        return false;
    }
}

void MainWindow::onMoveJointTraj(const std::vector<std::array<double, 8> > &jointTraj)
{
    if(!zm->GetConnectStatus())
    {
        UpdateUI("未连接控制器");
        qInfo()<<"未连接控制器";
        return;
    }
    if (jointTraj.empty()) {
        UpdateUI("Empty trajectory result");
    }
    zm->MoveAbsTrajectoryThetas(jointTraj);
}

void MainWindow::getIdle()
{
    if(!zm->GetConnectStatus())
    {
        UpdateUI("未连接控制器");
        qInfo()<<"未连接控制器";
        emit isIdle(false);
        return;
    }
    // return zm->IsAllAxisIdle();
    for(int i=0;i<ZMotionControl::AxisNum;i++)
        if(allAxisStatus[i].mtype!=0)
        {
            emit isIdle(false);
            return;
        }
    emit isIdle(true);
}

void MainWindow::getCurrentEndMoveJoint()
{
    if(!zm->GetConnectStatus())
    {
        UpdateUI("未连接控制器");
        qInfo()<<"未连接控制器";
        emit isIdle(false);
        return;
    }
    emit sendCurrentEndMoveJoint(zm->GetAllAxesFinalPositionsPhysical());
}



void MainWindow::onRequestTrajectory()
{
    emit sendTrajectory(trajectory);
}

void MainWindow::on_btnInitBus_clicked()
{
    if(!zm->GetConnectStatus())
    {
        UpdateUI("未连接控制器");
        qInfo()<<"未连接控制器";
        return;
    }
    zm->SetBrakeDisenable();
    zm->EnableMergeChange();
    zm->SetAllAxisSramp(1000);
    zm->SetCornerMode();

    if(1==zm->EcatInit(0,0,0,0))
    {
        qDebug()<<"总线初始化完成";
        qInfo()<<"总线初始化完成";
    }
}


void MainWindow::on_btnConnectSimulation_clicked()
{
    //系统连接控制器
    if(!zm->ConnectSimulation())
    {
        setLED(ui->ledMotion, 2, 32);
        emit UpdateUI("仿真器连接成功");
        qInfo()<<"仿真器连接成功";
        mvGetTimer->start(500);
        if(allAxisStatus[2].posi==0)
        {
            ui->robot3D_virtual->mRobotConfig.JVars[3]=-allAxisStatus[2].posi+180;
            ui->robot3D_virtual->updateGL();
        }
    }
    else
    {
        emit UpdateUI("仿真器连接失败");
        qWarning()<<"仿真器连接失败";

    }
}


void MainWindow::on_btnConnect_clicked()
{
    std::string str = ui->txtEditConnectContent->toPlainText().toStdString();//QString转换为string
    const char* ch = str.c_str();
    if(!zm->Connect(ui->cBoxConnectType->currentIndex()+1, ch))
    {
        setLED(ui->ledMotion, 2, 32);

        //startTimer(50);
        emit UpdateUI("控制器连接成功");
        qInfo()<<"控制器连接成功";
        mvGetTimer->start(1000);
        iniRead->setValue("/IP/ip",ui->txtEditConnectContent->toPlainText());
        iniRead->setValue("/IP/com",ui->cBoxConnectType->currentIndex()+1);
        if(allAxisStatus[2].posi==0)
        {
            ui->robot3D_virtual->mRobotConfig.JVars[3]=-allAxisStatus[2].posi+180;
            ui->robot3D_virtual->updateGL();
        }
        // 启动回调线程，并传入主线程的回调函数
        // 使用 lambda 表达式包装成员函数指针
        zm->startCallbackThread([this](ZmotionStatus* status) {
            handleDataFetched(status);
        });
    }
    else
    {
        emit UpdateUI("控制器连接失败");
        qWarning()<<"控制器连接失败";

    }
}

void MainWindow::on_btnEnableAxis_clicked()
{
    if(!zm->GetConnectStatus())
    {
        emit UpdateUI("未连接控制器");
        qInfo()<<"未连接控制器";
        return;
    }
    int* err=new int[ZMotionControl::AxisNum];
    bool hasFailure = false;

    err=zm->SetAxisEnable();
    zm->EnableMergeChange();
    zm->SetAllAxisSramp(1000);
    zm->SetCornerMode();
    for(int i=0;i<ZMotionControl::AxisNum;i++)
    {
        if (err[i] == 1) {
            hasFailure = true;
            emit UpdateUI("失败使能轴" + QString::number(i));
            qWarning()<<"失败使能轴" + QString::number(i);

        }
    }
    if(!hasFailure)
    {
        emit UpdateUI("使能所有轴");
        qInfo()<<"使能所有轴";
    }
    zm->SetBrakeEnable();


}


void MainWindow::on_btnDisableAxis_clicked()
{
    if(!zm->GetConnectStatus())
    {
        emit UpdateUI("未连接控制器");
        qInfo()<<"未连接控制器";
        return;
    }

    int* err=new int[ZMotionControl::AxisNum];
    bool hasFailure = false;
    zm->SetBrakeDisenable();
    zm->DisenableMergeChange();
    err=zm->SetAxisDisable();
    for(int i=0;i<ZMotionControl::AxisNum;i++)
    {
        if (err[i] == 1) {
            hasFailure = true;
            emit UpdateUI("失败取消使能轴" + QString::number(i));
            qWarning()<<"失败取消使能轴" + QString::number(i);

        }


    }
    if(!hasFailure)
    {
        emit UpdateUI("取消使能所有轴");
        qInfo()<<"取消使能所有轴";
    }

}


void MainWindow::on_btnDisconnect_clicked()
{
    if(!zm->GetConnectStatus())
    {
        emit UpdateUI("未连接控制器");
        qInfo()<<"未连接控制器";
        return;
    }
    mvGetTimer->stop();

    zm->Disconnect();
    emit UpdateUI("控制器断开连接");
    qInfo()<<"控制器断开连接";
    setLED(ui->ledMotion, 1, 32);
}
///
/// \brief MainWindow::UpdateUILog
/// 在UI提示框显示信息
/// \param str 显示的字符串
///
void MainWindow::UpdateUILog(QString str)
{

    ui->UpdateUI->appendPlainText(QTime::currentTime().toString("hh:mm:ss:zzz")+" "+str);
    qInfo()<<(QTime::currentTime().toString("hh:mm:ss:zzz")+" "+str);
}
///
/// \brief MainWindow::UpdateUIChanged
/// 每次出现文字都刷新到最后一行
///
void MainWindow::UpdateUIChanged()
{
    ui->UpdateUI->moveCursor(QTextCursor::End);
}

void MainWindow::on_btnAxisConfirm_clicked()
{
    if(!zm->GetConnectStatus())
    {
        emit UpdateUI("未连接控制器");
        qInfo()<<"未连接控制器";
        return;
    }
    int i=ui->cBoxAxisNum->currentIndex();
    zm->SetAxisUnits(i,ui->cBoxAxisUnits->currentText().toFloat());
    zm->SetAxisSpeed(i,ui->cBoxAxisSpeed->currentText().toFloat());
    zm->SetAxisAccel(i,ui->cBoxAxisAccel->currentText().toFloat());
    zm->SetAxisDecel(i,ui->cBoxAxisDecel->currentText().toFloat());
    zm->SetAxisFSLimit(i,ui->cBoxAxisFSLimit->currentText().toFloat());
    zm->SetAxisRSLimit(i,ui->cBoxAxisRSLimit->currentText().toFloat());
    zm->myZmotionStatus->allAxisStatus[i].init=ui->cBoxAxisInit->currentText().toFloat();
    emit UpdateUI("设置轴参数");
    qInfo()<<"设置轴参数";
}


void MainWindow::on_btnAxisSave_clicked()
{
    int i=ui->cBoxAxisNum->currentIndex();
    iniRead->setValue("/Axis"+QString::number(i)+"/Speed",ui->cBoxAxisSpeed->currentText());
    iniRead->setValue("/Axis"+QString::number(i)+"/Units",ui->cBoxAxisUnits->currentText());
    iniRead->setValue("/Axis"+QString::number(i)+"/Accel",ui->cBoxAxisAccel->currentText());
    iniRead->setValue("/Axis"+QString::number(i)+"/Decel",ui->cBoxAxisDecel->currentText());
    iniRead->setValue("/Axis"+QString::number(i)+"/FSLimit",ui->cBoxAxisFSLimit->currentText());
    iniRead->setValue("/Axis"+QString::number(i)+"/RSLimit",ui->cBoxAxisRSLimit->currentText());
    iniRead->setValue("/Axis"+QString::number(i)+"/Init",ui->cBoxAxisInit->currentText());
    emit UpdateUI("保存轴参数");
    qInfo()<<"保存轴参数";
}


void MainWindow::on_cBoxAxisNum_editTextChanged(const QString &arg1)
{
    int i=ui->cBoxAxisNum->currentIndex();
    //如果已经连接控制器从控制器获得数据
    if(zm->GetConnectStatus())
    {
        ui->cBoxAxisUnits->setCurrentText(QString::number(zm->GetAxisUnits(i)));
        ui->cBoxAxisSpeed->setCurrentText(QString::number(zm->GetAxisSpeed(i)));
        ui->cBoxAxisAccel->setCurrentText(QString::number(zm->GetAxisAccel(i)));
        ui->cBoxAxisDecel->setCurrentText(QString::number(zm->GetAxisDecel(i)));
        ui->cBoxAxisFSLimit->setCurrentText(QString::number(zm->GetAxisFSLimit(i)));
        ui->cBoxAxisRSLimit->setCurrentText(QString::number(zm->GetAxisRSLimit(i)));
        ui->cBoxAxisInit->setCurrentText(QString::number(zm->myZmotionStatus->allAxisStatus[i].init));

    }
    else//如果没链接控制器从ini文件获得数据
    {
        ui->cBoxAxisSpeed->setCurrentText(iniRead->value("/Axis"+QString::number(i)+"/Speed").toString());
        ui->cBoxAxisUnits->setCurrentText(iniRead->value("/Axis"+QString::number(i)+"/Units").toString());
        ui->cBoxAxisAccel->setCurrentText(iniRead->value("/Axis"+QString::number(i)+"/Accel").toString());
        ui->cBoxAxisDecel->setCurrentText(iniRead->value("/Axis"+QString::number(i)+"/Decel").toString());
        ui->cBoxAxisFSLimit->setCurrentText(iniRead->value("/Axis"+QString::number(i)+"/FSLimit").toString());
        ui->cBoxAxisRSLimit->setCurrentText(iniRead->value("/Axis"+QString::number(i)+"/RSLimit").toString());
        ui->cBoxAxisInit->setCurrentText(iniRead->value("/Axis"+QString::number(i)+"/Init").toString());
    }
}

void MainWindow::onMvGetTimerOut()
{
    if(!zm->GetConnectStatus())
    {
        return;
    }
    if(statusIOEmergencyStop==0)
    {
        zm->EmergencyStop();
        UpdateUI("急停！！！");
        qInfo()<<"急停！！！";
    }
    bool posichanged=false;
    ///轴状态显示页面设置
    QLabel* lblBsLimits[8][5] = {
        {ui->lblPosi0, ui->lblAxisStatus0, ui->lblFsLimit0, ui->lblBsLimit0},
        {ui->lblPosi1, ui->lblAxisStatus1, ui->lblFsLimit1, ui->lblBsLimit1},
        {ui->lblPosi2, ui->lblAxisStatus2, ui->lblFsLimit2, ui->lblBsLimit2},
        {ui->lblPosi3, ui->lblAxisStatus3, ui->lblFsLimit3, ui->lblBsLimit3},
        {ui->lblPosi4, ui->lblAxisStatus4, ui->lblFsLimit4, ui->lblBsLimit4},
        {ui->lblPosi5, ui->lblAxisStatus5, ui->lblFsLimit5, ui->lblBsLimit5},
        {ui->lblPosi6, ui->lblAxisStatus6, ui->lblFsLimit6, ui->lblBsLimit6},
        {ui->lblPosi7, ui->lblAxisStatus7, ui->lblFsLimit7, ui->lblBsLimit7}
    };
    //每当数据发生变化时才修改ui界面
    for(int i=0;i<ZMotionControl::AxisNum;i++)
    {
        if(pastallAxisStatus[i].posi!=allAxisStatus[i].posi)
        {
            posichanged=true;
            pastallAxisStatus[i].posi=allAxisStatus[i].posi;
            lblBsLimits[i][0]->setText(QString::number(allAxisStatus[i].posi, 'f', 3));
        }
        if(pastallAxisStatus[i].status!=allAxisStatus[i].status)
        {
            QString statusname;
            pastallAxisStatus[i].status=allAxisStatus[i].status;
            //qDebug()<<allAxisStatus[i].status;
            if (allAxisStatus[i].status ==NoError)
            {
                statusname= "NoError";
            }
            else if (allAxisStatus[i].status  == FlimitError)
            {
                statusname= "FlimitError";
            }
            else if (allAxisStatus[i].status == BlimitError)
            {
                statusname= "BlimitError";
            }
            else if (allAxisStatus[i].status ==ServoError)
            {
                statusname= "ServoError";
            }
            lblBsLimits[i][1]->setText(statusname);
        }
        if(pastallAxisStatus[i].fslimit!=allAxisStatus[i].fslimit)
        {
            pastallAxisStatus[i].fslimit=allAxisStatus[i].fslimit;
            lblBsLimits[i][2]->setText(QString::number(allAxisStatus[i].fslimit, 'f', 3));
        }
        if(pastallAxisStatus[i].rslimit!=allAxisStatus[i].rslimit)
        {
            pastallAxisStatus[i].rslimit=allAxisStatus[i].rslimit;
            lblBsLimits[i][3]->setText(QString::number(allAxisStatus[i].rslimit, 'f', 3));
        }
        if(pastallAxisStatus[i].mtype!=allAxisStatus[i].mtype)
        {
            if(allAxisStatus[i].mtype==0)
                emit UpdateUI("轴"+QString::number(i)+"停止运动");
            else if(pastallAxisStatus[i].mtype==0)
                emit UpdateUI("轴"+QString::number(i)+"开始运动");
            pastallAxisStatus[i].mtype=allAxisStatus[i].mtype;

        }
    }

    if(posichanged)
    {

        ui->robot3D_virtual->updateGL();

        getPosition();

        for(int i=0;i<5;i++)
        {

            if(i==0||i==2||i==3)
                ui->robot3D_virtual->mRobotConfig.JVars[i+1]=-allAxisStatus[i+3].posi;
            else
                ui->robot3D_virtual->mRobotConfig.JVars[i+1]=allAxisStatus[i+3].posi;
        }


    }

}


void MainWindow::on_btnInitAxis_clicked()
{
    if(!zm->GetConnectStatus())
    {
        UpdateUI("未连接控制器");
        qInfo()<<"未连接控制器";
        return;
    }
    for(int i=0;i<zm->AxisNum;i++)
    {
        zm->SetAxisUnits(i,iniRead->value("/Axis"+QString::number(i)+"/Units").toFloat());
        zm->SetAxisSpeed(i,iniRead->value("/Axis"+QString::number(i)+"/Speed").toFloat());
        zm->SetAxisAccel(i,iniRead->value("/Axis"+QString::number(i)+"/Accel").toFloat());
        zm->SetAxisDecel(i,iniRead->value("/Axis"+QString::number(i)+"/Decel").toFloat());
        zm->SetAxisFSLimit(i,iniRead->value("/Axis"+QString::number(i)+"/FSLimit").toFloat());
        zm->SetAxisRSLimit(i,iniRead->value("/Axis"+QString::number(i)+"/RSLimit").toFloat());
        zm->myZmotionStatus->allAxisStatus[i].init=iniRead->value("/Axis"+QString::number(i)+"/Init").toFloat();
    }
    zm->SetDecelAngle(0);
    zm->SetStopAngle(360);
    zm->SetForceSpeed(5);
    UpdateUI("初始化轴参数");
    qInfo()<<"初始化轴参数";
}


void MainWindow::on_btnType0_clicked()
{
    if(!zm->GetConnectStatus())
    {
        UpdateUI("未连接控制器");
        qInfo()<<"未连接控制器";
        return;
    }
    zm->SetAxisType(0);
    UpdateUI("设置虚拟轴");
    qInfo()<<"设置虚拟轴";
}


void MainWindow::on_btnType65_clicked()
{
    if(!zm->GetConnectStatus())
    {
        UpdateUI("未连接控制器");
        qInfo()<<"未连接控制器";
        return;
    }
    zm->SetAxisType(65);
    UpdateUI("设置实际轴");
    qInfo()<<"设置实际轴";
}

void MainWindow::on_btnXYZ_Pressed(int i)
{
    qDebug() << "Pressed direction:" << i;
    if (jogTimer->isActive()) {
        jogTimer->stop();
    }
    currentDirection = i;
    isWaitingForArrival = false;

    // 步骤1: 读当前实际位姿作为起点
    getPosition();  // 确保最新
    Point6D initialPose = robotPose.tcp_pose;  // 当前实际

    // 步骤2: 计算第一个目标 (初始 + 第一步偏移)
    double step = 0;
    int axis = std::abs(currentDirection) - 1;
    bool isPositive = (currentDirection > 0);
    bool isRotation = (axis >= 3);
    double rotationStep = degToRad(ui->doubleSpinBoxMoveAngle->value());
    if (!isRotation) {
        step = isPositive ? ui->spinBoxMoveDistance->value() : -ui->spinBoxMoveDistance->value();
    } else {
        step = isPositive ? rotationStep : -rotationStep;
    }

    // 应用偏移到初始
    Point6D firstTargetPose = initialPose;  // 复制
    switch(axis) {
    case 0: firstTargetPose.x += step; break;
    case 1: firstTargetPose.y += step; break;
    case 2: firstTargetPose.z += step; break;
    case 3: firstTargetPose.rx += step; break;
    case 4: firstTargetPose.ry += step; break;
    case 5: firstTargetPose.rz += step; break;
    }

    // IK第一个目标
    Eigen::Matrix4d firstTargetMatrix = toEigenMatrix(firstTargetPose);
    std::array<double, 8> firstThetas = GAOptimizer->optimizeSinglePoint(firstTargetMatrix);
    if (!areThetasEqual(firstThetas, robotPose.joint_positions)) {
        zm->MoveAbsTrajectoryThetas(firstThetas);
        pendingThetas = firstThetas;
        lastTargetPose = firstTargetPose;  // 记录第一个目标作为“上个”
        isWaitingForArrival = true;
        qDebug() << "Sent first target based on initial pose";
    }

    // 启动检查定时器
    jogTimer->start(100);

}

void MainWindow::on_btnXYZ_Released(int i)
{
    qDebug() << "Released; stopping";
    jogTimer->stop();
    currentDirection = 0;
    isWaitingForArrival = false;
    getPosition();  // 最终同步实际
    // 可选：发送停止命令到机器人，确保平滑停
    // e.g., MoveAbsTrajectoryThetas(robotPose.joint_positions); // 保持当前关节
}
void MainWindow::onJogTimerTimeout() {
    if (currentDirection == 0) return;
    // 现在计算下一个：基于上个目标位姿 + 步 (非当前实际)
    Point6D basePose = lastTargetPose;  // 关键：用上个目标作为基点
    double step = 0;
    int axis = std::abs(currentDirection) - 1;
    bool isPositive = (currentDirection > 0);
    bool isRotation = (axis >= 3);
    double rotationStep = degToRad(ui->doubleSpinBoxMoveAngle->value());
    if (!isRotation) {
        step = isPositive ? ui->spinBoxMoveDistance->value() : -ui->spinBoxMoveDistance->value();
    } else {
        step = isPositive ? rotationStep : -rotationStep;
    }
    // 新增：等待上个目标到达？
    if (isWaitingForArrival) {
        getPosition();  // 刷新实际
        if (areThetasEqual(robotPose.joint_positions, pendingThetas, step*0.1)) {
            qDebug() << "Arrived at last target; calculating next from lastTargetPose";
            isWaitingForArrival = false;
        } else {
            return;  // 等到达再算下一个
        }
    }



    // 应用偏移到上个目标
    Point6D nextTargetPose = basePose;  // 复制上个目标
    switch(axis) {
    case 0: nextTargetPose.x += step; break;
    case 1: nextTargetPose.y += step; break;
    case 2: nextTargetPose.z += step; break;
    case 3: nextTargetPose.rx += step; break;
    case 4: nextTargetPose.ry += step; break;
    case 5: nextTargetPose.rz += step; break;
    }
    Eigen::Matrix4d targetMatrix = toEigenMatrix(nextTargetPose);

    // IK
    std::array<double, 8> targetThetas = GAOptimizer->optimizeSinglePoint(targetMatrix);

    // 一致检查 (基于当前实际，防微动)
    if (areThetasEqual(targetThetas, robotPose.joint_positions)) {
        isWaitingForArrival = false;  // 允许重试
        return;
    }

    // 执行 + 更新
    zm->MoveAbsTrajectoryThetas(targetThetas);
    pendingThetas = targetThetas;
    lastTargetPose = nextTargetPose;  // 更新上个目标为这个
    isWaitingForArrival = true;
    qDebug() << "Sent next target from lastTargetPose (axis:" << axis << ", step:" << step << ")";
}
void MainWindow::on_btnEmergencyStop_clicked()
{

    zm->EmergencyStop();
    UpdateUI("急停！！！");
    qInfo()<<"急停！！！";
    // if(isIdle())
    //     UpdateUI("停止");
}

void MainWindow::on_btnSaveDH_clicked()
{
    QLineEdit* MDH[8][4] = {
    {ui->lineEditAlpha0,ui->lineEditA0,ui->lineEditTheta0,ui->lineEditD0},
    {ui->lineEditAlpha1,ui->lineEditA1,ui->lineEditTheta1,ui->lineEditD1},
    {ui->lineEditAlpha2,ui->lineEditA2,ui->lineEditTheta2,ui->lineEditD2},
    {ui->lineEditAlpha3,ui->lineEditA3,ui->lineEditTheta3,ui->lineEditD3},
    {ui->lineEditAlpha4,ui->lineEditA4,ui->lineEditTheta4,ui->lineEditD4},
    {ui->lineEditAlpha5,ui->lineEditA5,ui->lineEditTheta5,ui->lineEditD5},
    {ui->lineEditAlpha6,ui->lineEditA6,ui->lineEditTheta6,ui->lineEditD6},
    {ui->lineEditAlpha7,ui->lineEditA7,ui->lineEditTheta7,ui->lineEditD7}
    };
    for (int i = 0; i < 8; i++)
    {
        iniRead->setValue(QString::number(ui->spbDHNum->value()) + "/MDH" + QString::number(i) + "/Alpha", MDH[i][0]->text());
        iniRead->setValue(QString::number(ui->spbDHNum->value()) + "/MDH" + QString::number(i) + "/A", MDH[i][1]->text());
        iniRead->setValue(QString::number(ui->spbDHNum->value()) + "/MDH" + QString::number(i) + "/Theta", MDH[i][2]->text());
        iniRead->setValue(QString::number(ui->spbDHNum->value()) + "/MDH" + QString::number(i) + "/D", MDH[i][3]->text());
    }
    iniRead->setValue(QString::number(ui->spbDHNum->value()) + "/MDH" + "/Descripition", ui->DHDescription->text());
    UpdateUI("保存第" + QString::number(ui->spbDHNum->value()) + "个DH表数据");

}

void MainWindow::on_btnSetDH_clicked()
{
    arm->resetMDHParam();
    QLineEdit* MDH[8][4] = {
    {ui->lineEditAlpha0,ui->lineEditA0,ui->lineEditTheta0,ui->lineEditD0},
    {ui->lineEditAlpha1,ui->lineEditA1,ui->lineEditTheta1,ui->lineEditD1},
    {ui->lineEditAlpha2,ui->lineEditA2,ui->lineEditTheta2,ui->lineEditD2},
    {ui->lineEditAlpha3,ui->lineEditA3,ui->lineEditTheta3,ui->lineEditD3},
    {ui->lineEditAlpha4,ui->lineEditA4,ui->lineEditTheta4,ui->lineEditD4},
    {ui->lineEditAlpha5,ui->lineEditA5,ui->lineEditTheta5,ui->lineEditD5},
    {ui->lineEditAlpha6,ui->lineEditA6,ui->lineEditTheta6,ui->lineEditD6},
    {ui->lineEditAlpha7,ui->lineEditA7,ui->lineEditTheta7,ui->lineEditD7}
     };
    for (int i = 0; i < 8; i++)
    {
        arm->modifyMDHParam(i, MDH[i][1]->text().toFloat(), MDH[i][0]->text().toFloat(), MDH[i][3]->text().toFloat(), MDH[i][2]->text().toFloat());
    }
    UpdateUI("设置DH表");
    getPosition();
}


void MainWindow::on_btnLoadData_clicked()
{

    QString dataQFile =ui->LineEditDataAddress->text();
    ifstream file(dataQFile.toStdString());
    std::string line;
    // ����������
    std::getline(file, line);
    excel_xyz.clear();
    while (std::getline(file, line)) {
        // �滻�����Ʊ���Ϊ�ո�
        replace(line.begin(), line.end(), ',', ' ');

        std::istringstream iss(line);
        double x, y, z, qx, qy, qz, qw;

        // �Ľ��Ĵ�����
        if (iss >> x >> y >> z >> qx >> qy >> qz >> qw) {
            excel_xyz.push_back({ x * 1000, y * 1000, z * 1000 - 2358 });
        }

    }
    ui->lblCurrentPosiX->setText(QString::number(excel_xyz[ui->spbCurrentPosi->value()][0], 'f', 3));
    ui->lblCurrentPosiY->setText(QString::number(excel_xyz[ui->spbCurrentPosi->value()][1], 'f', 3));
    ui->lblCurrentPosiZ->setText(QString::number(excel_xyz[ui->spbCurrentPosi->value()][2], 'f', 3));
    trajectory.clear();
    trajectory = loadTrajectoryFromCSV(dataQFile.toStdString());

}


void MainWindow::on_btnCurrentPosition_clicked()
{
    if(!zm->GetConnectStatus())
    {
        UpdateUI("未连接控制器");
        qInfo()<<"未连接控制器";
        return;
    }
    if (jointTrajectory.empty())
        return;
    if (ui->spbCurrentPosi->value() < 0)
        ui->spbCurrentPosi->setValue(0);
    else if (ui->spbCurrentPosi->value() > jointTrajectory.size())
        ui->spbCurrentPosi->setValue(jointTrajectory.size());
    std::vector<std::array<double, 8>> joint(1);

    joint[0]=jointTrajectory[ui->spbCurrentPosi->value()];
    zm->MoveAbsTrajectoryThetas(joint);
    UpdateUI("运动到第" + QString::number(ui->spbCurrentPosi->value()) + "个轨迹");
    int currentIndex = ui->spbCurrentPosi->value();
    if (currentIndex >= 0 && currentIndex < excel_xyz.size()) {
        if (!excel_xyz[currentIndex].empty()) {
            ui->lblCurrentPosiX->setText(QString::number(excel_xyz[ui->spbCurrentPosi->value()][0], 'f', 3));
            ui->lblCurrentPosiY->setText(QString::number(excel_xyz[ui->spbCurrentPosi->value()][1], 'f', 3));
            ui->lblCurrentPosiZ->setText(QString::number(excel_xyz[ui->spbCurrentPosi->value()][2], 'f', 3));
        }
    }



}


void MainWindow::on_btnLastPosition_clicked()
{
    if(!zm->GetConnectStatus())
    {
        UpdateUI("未连接控制器");
        qInfo()<<"未连接控制器";
        return;
    }
    if (jointTrajectory.empty())
        return;
    std::vector<std::array<double, 8>> joint(1);
    if(ui->spbCurrentPosi->value()-1>=0)
    {
        joint[0]=jointTrajectory[ui->spbCurrentPosi->value()-1];
        ui->spbCurrentPosi->setValue(ui->spbCurrentPosi->value()-1);
    }
    else
    {
        joint[0]=jointTrajectory[ui->spbCurrentPosi->value()];
    }

    zm->MoveAbsTrajectoryThetas(joint);
    int currentIndex = ui->spbCurrentPosi->value();
    if (currentIndex >= 0 && currentIndex < excel_xyz.size()) {
        if (!excel_xyz[currentIndex].empty()) {
            ui->lblCurrentPosiX->setText(QString::number(excel_xyz[ui->spbCurrentPosi->value()][0], 'f', 3));
            ui->lblCurrentPosiY->setText(QString::number(excel_xyz[ui->spbCurrentPosi->value()][1], 'f', 3));
            ui->lblCurrentPosiZ->setText(QString::number(excel_xyz[ui->spbCurrentPosi->value()][2], 'f', 3));
        }
    }

    UpdateUI("运动到第" + QString::number(ui->spbCurrentPosi->value()) + "个轨迹");

}


void MainWindow::on_btnNextPosi_clicked()
{
    if(!zm->GetConnectStatus())
    {
        UpdateUI("未连接控制器");
        qInfo()<<"未连接控制器";
        return;
    }
    if (jointTrajectory.empty())
        return;
    std::vector<std::array<double, 8>> joint(1);
    if(ui->spbCurrentPosi->value()+1<=jointTrajectory.size())
    {
        joint[0]=jointTrajectory[ui->spbCurrentPosi->value()+1];
        ui->spbCurrentPosi->setValue(ui->spbCurrentPosi->value()+1);
    }
    else
    {
        joint[0]=jointTrajectory[ui->spbCurrentPosi->value()];
    }
    zm->MoveAbsTrajectoryThetas(joint);
    int currentIndex = ui->spbCurrentPosi->value();
    if (currentIndex >= 0 && currentIndex < excel_xyz.size()) {
        if (!excel_xyz[currentIndex].empty()) {
            ui->lblCurrentPosiX->setText(QString::number(excel_xyz[ui->spbCurrentPosi->value()][0], 'f', 3));
            ui->lblCurrentPosiY->setText(QString::number(excel_xyz[ui->spbCurrentPosi->value()][1], 'f', 3));
            ui->lblCurrentPosiZ->setText(QString::number(excel_xyz[ui->spbCurrentPosi->value()][2], 'f', 3));
        }
    }

    UpdateUI("运动到第"+QString::number(ui->spbCurrentPosi->value()) + "个轨迹");

}


void MainWindow::on_btnClearData_clicked()
{
    jointTrajectory.clear();
    excel_xyz.clear();
    ui->lblCurrentPosiX->setText("0");
    ui->lblCurrentPosiY->setText("0");
    ui->lblCurrentPosiZ->setText("0");
    UpdateUI("清空导入轨迹数据");

}


void MainWindow::on_btnClearDH_clicked()
{
    arm->resetMDHParam();
    QLineEdit* MDH[8][4] = {
    {ui->lineEditAlpha0,ui->lineEditA0,ui->lineEditTheta0,ui->lineEditD0},
    {ui->lineEditAlpha1,ui->lineEditA1,ui->lineEditTheta1,ui->lineEditD1},
    {ui->lineEditAlpha2,ui->lineEditA2,ui->lineEditTheta2,ui->lineEditD2},
    {ui->lineEditAlpha3,ui->lineEditA3,ui->lineEditTheta3,ui->lineEditD3},
    {ui->lineEditAlpha4,ui->lineEditA4,ui->lineEditTheta4,ui->lineEditD4},
    {ui->lineEditAlpha5,ui->lineEditA5,ui->lineEditTheta5,ui->lineEditD5},
    {ui->lineEditAlpha6,ui->lineEditA6,ui->lineEditTheta6,ui->lineEditD6},
    {ui->lineEditAlpha7,ui->lineEditA7,ui->lineEditTheta7,ui->lineEditD7}
    };
    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 4; j++)
            MDH[i][j]->setText("0");
    }
    ui->DHDescription->setText("描述");
    getPosition();
    UpdateUI("清空DH表数据");

}


void MainWindow::on_spbDHNum_editingFinished()
{
    QLineEdit* MDH[8][4] = {
    {ui->lineEditAlpha0,ui->lineEditA0,ui->lineEditTheta0,ui->lineEditD0},
    {ui->lineEditAlpha1,ui->lineEditA1,ui->lineEditTheta1,ui->lineEditD1},
    {ui->lineEditAlpha2,ui->lineEditA2,ui->lineEditTheta2,ui->lineEditD2},
    {ui->lineEditAlpha3,ui->lineEditA3,ui->lineEditTheta3,ui->lineEditD3},
    {ui->lineEditAlpha4,ui->lineEditA4,ui->lineEditTheta4,ui->lineEditD4},
    {ui->lineEditAlpha5,ui->lineEditA5,ui->lineEditTheta5,ui->lineEditD5},
    {ui->lineEditAlpha6,ui->lineEditA6,ui->lineEditTheta6,ui->lineEditD6},
    {ui->lineEditAlpha7,ui->lineEditA7,ui->lineEditTheta7,ui->lineEditD7}
    };
    for (int i = 0; i < 8; i++)
    {
        MDH[i][0]->setText(iniRead->value(QString::number(ui->spbDHNum->value()) + "/MDH" + QString::number(i) + "/Alpha").toString());
        MDH[i][1]->setText(iniRead->value(QString::number(ui->spbDHNum->value()) + "/MDH" + QString::number(i) + "/A").toString());
        MDH[i][2]->setText(iniRead->value(QString::number(ui->spbDHNum->value()) + "/MDH" + QString::number(i) + "/Theta").toString());
        MDH[i][3]->setText(iniRead->value(QString::number(ui->spbDHNum->value()) + "/MDH" + QString::number(i) + "/D").toString());

    }
    ui->DHDescription->setText(iniRead->value(QString::number(ui->spbDHNum->value()) + "/MDH" + "/Descripition").toString());
    UpdateUI("读取第"+ QString::number(ui->spbDHNum->value()) +"个DH表数据");
}


void MainWindow::on_btnOpenData_clicked()
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


void MainWindow::on_btnSetAxisContinuity_clicked()
{
    auto tra=processor->generateMoveItTrajectory();
    // 获取文件名（包括扩展名）
    ////导出轨迹
    QDateTime curDateTime = QDateTime::currentDateTime();
    QString dataQFile = ui->LineEditDataAddress->text();
    QFileInfo fileInfo(dataQFile);
    QString fileNameWithExtension = fileInfo.fileName();
    QString desktopPath = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation);
    QString file(desktopPath + '/' + curDateTime.toString("yyyy-MM-dd-hh-mm-") + fileNameWithExtension);

    processor->savePosesToCSV(file.toStdString(),tra);
}


void MainWindow::on_btnSetAxisNotContinuity_clicked()
{
    GAOptimizer->setNotContinuity();
}


void MainWindow::on_btnExecuteTrajectory_clicked()
{
    QString dataQFile =ui->LineEditDataAddress->text();
    jointTrajectory.clear();
    jointTrajectory = arm->readJointTrajectoryFromCSV(dataQFile.toStdString());

    // // 打印读取的数据进行验证
    // for (size_t i = 0; i < jointTrajectory.size(); ++i) {
    //     std::cout << "Point " << i << ": ";
    //     for (size_t j = 0; j < 8; ++j) {
    //         std::cout << jointTrajectory[i][j] << " ";
    //     }
    //     std::cout << std::endl;
    // }
}


void MainWindow::on_btnRecordUpperBlock_clicked()
{
    if(!zm->GetConnectStatus())
    {
        UpdateUI("未连接控制器");
        qInfo()<<"未连接控制器";
        return;
    }
    QLabel* Block[4][2] = {
        {ui->lblBlockUpperPosi0,ui->lblBlockLowerPosi0},
        {ui->lblBlockUpperPosi1,ui->lblBlockLowerPosi1},
        {ui->lblBlockUpperPosi2,ui->lblBlockLowerPosi2},
        {ui->lblBlockUpperPosi3,ui->lblBlockLowerPosi3}
    };
    Block[ui->spbBlockNum->value()][0]->setText(QString::number((robotPose.tcp_pose.x)/1000.0,'f',3)+","
                                                +QString::number((robotPose.tcp_pose.y)/1000.0,'f',3)+","
                                                +QString::number((robotPose.tcp_pose.z+2358)/1000.0,'f',3));
}


void MainWindow::on_btnRecordLowerBlock_clicked()
{
    if(!zm->GetConnectStatus())
    {
        UpdateUI("未连接控制器");
        qInfo()<<"未连接控制器";
        return;
    }
    QLabel* Block[4][2] = {
        {ui->lblBlockUpperPosi0,ui->lblBlockLowerPosi0},
        {ui->lblBlockUpperPosi1,ui->lblBlockLowerPosi1},
        {ui->lblBlockUpperPosi2,ui->lblBlockLowerPosi2},
        {ui->lblBlockUpperPosi3,ui->lblBlockLowerPosi3}
    };
    Block[ui->spbBlockNum->value()][1]->setText(QString::number((robotPose.tcp_pose.x)/1000.0,'f',3)+","
                                                +QString::number((robotPose.tcp_pose.y)/1000.0,'f',3)+","
                                                +QString::number((robotPose.tcp_pose.z+2358)/1000.0,'f',3));


}


void MainWindow::on_btnSaveBlock_clicked()
{
    QLabel* Block[4][2] = {
                           {ui->lblBlockUpperPosi0,ui->lblBlockLowerPosi0},
                           {ui->lblBlockUpperPosi1,ui->lblBlockLowerPosi1},
                           {ui->lblBlockUpperPosi2,ui->lblBlockLowerPosi2},
                           {ui->lblBlockUpperPosi3,ui->lblBlockLowerPosi3}
    };
    for (int i = 0; i < 4; i++)
    {
        iniRead->setValue(QString::number(ui->spbBlockGroupNum->value()) + "/Block" + QString::number(i) + "/UpperPosi", Block[i][0]->text());
        iniRead->setValue(QString::number(ui->spbBlockGroupNum->value()) + "/Block" + QString::number(i) + "/LowerPosi", Block[i][1]->text());

    }
    iniRead->setValue(QString::number(ui->spbBlockGroupNum->value()) + "/Block" + "/Descripition", ui->BlockDescription->text());
    UpdateUI("保存第" + QString::number(ui->spbBlockGroupNum->value()) + "个Block数据");
}


void MainWindow::on_btnSetRealBlock_clicked()
{
    if(processor==nullptr)
    {
        UpdateUI("请先处理SW曲线");
        return;
    }
    vector<Vector3d> real_point;
    QLabel* Block[4][2] = {
        {ui->lblBlockUpperPosi0,ui->lblBlockLowerPosi0},
        {ui->lblBlockUpperPosi1,ui->lblBlockLowerPosi1},
        {ui->lblBlockUpperPosi2,ui->lblBlockLowerPosi2},
        {ui->lblBlockUpperPosi3,ui->lblBlockLowerPosi3}
    };
    // vector<Vector3d> real_point={
    //     {1.63912,-1.93073,0.70022},
    //     {1.64912,-1.9836,0.44139},
    //     {1.63912,-0.6592,0.70025},
    //     {1.64912,-0.60648,0.44139},
    //     {0.45912,-0.85232,0.66432},
    //     {0.44912,-0.80319,0.40463},
    //     {0.45912,-1.73761,0.6643},
    //     {0.44912,-1.78691,0.40459}

    // };
    // 假设 inputString 是从 Block[...]->text() 获取的字符串
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<2;j++)
        {
            // 分割字符串
            QString inputString = Block[i][j]->text();
            QStringList parts = inputString.split(",");
            if (parts.size() == 3) {
                Vector3d point;
                point.x() = parts[0].toDouble();
                point.y() = parts[1].toDouble();
                point.z() = parts[2].toDouble();

                real_point.push_back(point);
            }
        }

    }
    // for(int i=0;i<real_point.size();i++)
    // {

    //     UpdateUI(QString::number(real_point[i].x()));
    //     UpdateUI(QString::number(real_point[i].y()));
    //     UpdateUI(QString::number(real_point[i].z()));

    // }


    // 使用 QFileInfo 提取文件信息
    QString dataQFile = ui->LineEditDataAddress->text();
    QFileInfo fileInfo(dataQFile);
    QString fileNameWithExtension = fileInfo.fileName();
    qDebug() << "Selected file:" << fileNameWithExtension;

    // 获取文件名（包括扩展名）
    ////导出轨迹
    QDateTime curDateTime = QDateTime::currentDateTime();

    QString desktopPath = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation);
    QString file(desktopPath + '/' + curDateTime.toString("yyyy-MM-dd-hh-mm-") + fileNameWithExtension);

    processor->setRealCorners(real_point);
    trajectory.clear();

    //trajectory=processor->generateOrientInpIKTrajectory(file.toStdString());
    // trajectory = processor->generateSideFaceIKTrajectory();


}


void MainWindow::on_spbBlockGroupNum_editingFinished()
{
    QLabel* Block[4][2] = {
        {ui->lblBlockUpperPosi0,ui->lblBlockLowerPosi0},
        {ui->lblBlockUpperPosi1,ui->lblBlockLowerPosi1},
        {ui->lblBlockUpperPosi2,ui->lblBlockLowerPosi2},
        {ui->lblBlockUpperPosi3,ui->lblBlockLowerPosi3}


    };
    for (int i = 0; i < 4; i++)
    {
        Block[i][0]->setText(iniRead->value(QString::number(ui->spbBlockGroupNum->value()) + "/Block" + QString::number(i) + "/UpperPosi").toString());
        Block[i][1]->setText(iniRead->value(QString::number(ui->spbBlockGroupNum->value()) + "/Block" + QString::number(i) + "/LowerPosi").toString());

    }
    ui->BlockDescription->setText(iniRead->value(QString::number(ui->spbBlockGroupNum->value()) + "/Block" + "/Descripition").toString());
    UpdateUI("读取第"+ QString::number(ui->spbBlockGroupNum->value()) +"个Block数据");

}


void MainWindow::on_btnLoadSWCurve_clicked()
{
    QString dataQFile =ui->LineEditDataAddress->text();
    if (processor != nullptr) {
        delete processor;
        processor = nullptr;  // 避免悬空指针
    }

    processor=new CurveProcessor(dataQFile.toStdString());
    trajectory.clear();
}



void MainWindow::on_btnCalcTrajectory_clicked()
{
    onCaculateTrajectoryPoints(trajectory);
    // if (trajectory.empty()) {
    //     UpdateUI("Trajectory is empty. Please load data first.");
    //     return;
    // }

    // if (asyncOptimizeTask) {
    //     asyncOptimizeTask.reset();  // 清理旧任务
    // }
    // jointTrajectory.clear();
    // isOptimizing = true;

    // try {
    //     auto task = GAOptimizer->optimizeTrajectoryAsync(
    //         trajectory,
    //         [this](const std::string& progress) {
    //             QString msg = QString::fromStdString(progress);
    //             QMetaObject::invokeMethod(this, [this, msg]() { UpdateUI(msg); }, Qt::QueuedConnection);
    //         }
    //         );

    //     asyncOptimizeTask = std::make_unique<AsyncOptimizeTask>(std::move(task));
    //     // 无需检查 isValid()：启动后自动有效

    // } catch (const std::exception& e) {
    //     isOptimizing = false;
    //     UpdateUI(QString("Failed to start optimization: %1").arg(e.what()));
    // }

}
void MainWindow::on_btnLoadCalcTrajectory_clicked()
{
    // 简化保护：指针 + 有效性（isValid() 内部检查 future）
    if (!asyncOptimizeTask || !asyncOptimizeTask->isValid()) {
        UpdateUI("No valid optimization task found. Please start calculation first.");
        return;
    }
    if (!asyncOptimizeTask->isReady()) {
        UpdateUI("Optimization not completed yet. Please wait.");
        return;
    }

    try {
        // 直接调用 get()（内部自保护：如果无效会抛异常）
        jointTrajectory = asyncOptimizeTask->get();

        if (jointTrajectory.empty()) {
            throw std::runtime_error("Empty trajectory result");
        }

        UpdateUI(QString("Loaded trajectory with %1 points").arg(jointTrajectory.size()));

        // 导出轨迹（路径验证和目录创建）
        QDateTime curDateTime = QDateTime::currentDateTime();
        QString dataQFile = ui->LineEditDataAddress->text();
        QFileInfo fileInfo(dataQFile);
        if (!fileInfo.exists()) {
            throw std::runtime_error("Input file does not exist: " + dataQFile.toStdString());
        }

        QString fileNameWithExtension = fileInfo.fileName();
        // 路径构建（QString 方式）
        QString qPath = QDir::currentPath() + QDir::separator() +
                        "lib/data/csv/" +
                        curDateTime.toString("yyyy-MM-dd-hh-mm-") +
                        fileNameWithExtension;
        qPath = QDir::cleanPath(qPath);
        std::string file = qPath.toStdString();

        // 检查/创建目录
        QDir exportDir(QDir::currentPath() + QDir::separator() + "lib/data/csv");
        if (!exportDir.exists()) {
            if (!exportDir.mkpath(".")) {
                throw std::runtime_error("Failed to create export directory");
            }
        }

        GAOptimizer->exportJointTrajectory(jointTrajectory, file);
        UpdateUI(QString("Trajectory exported to: %1").arg(qPath));

        // 成功重置
        isOptimizing = false;
        asyncOptimizeTask.reset();  // 清理，future 自动析构

    } catch (const std::invalid_argument& e) {
        // 特定捕获：无效任务异常（来自 get() 内部）
        isOptimizing = false;
        UpdateUI(QString("Invalid task: %1").arg(e.what()));
    } catch (const std::exception& e) {
        // 其他异常（如取消、导出失败）
        isOptimizing = false;
        UpdateUI(QString("Failed to load/export trajectory: %1").arg(e.what()));
        // 不重置任务，让用户重试（如果适用）
    }
}

void MainWindow::on_pushButton_clicked()
{
    onCancelCaculateTrajectory();
    // // 简化保护：指针 + 有效性
    // if (!asyncOptimizeTask || !asyncOptimizeTask->isValid()) {
    //     UpdateUI("No valid task to cancel.");
    //     return;
    // }
    // if (!isOptimizing) {
    //     UpdateUI("No running task.");
    //     return;
    // }

    // try {
    //     // 直接调用 cancel()（内部自保护：无效时静默返回）
    //     asyncOptimizeTask->cancel();
    //     UpdateUI("Cancellation requested. Waiting for task to stop...");

    //     // 检查是否完成，并清理（get() 内部自保护）
    //     if (asyncOptimizeTask->isReady()) {
    //         try {
    //             auto result = asyncOptimizeTask->get();  // 捕获取消异常，忽略结果
    //             (void)result;
    //         } catch (const std::exception& e) {
    //             UpdateUI(QString("Task cancelled: %1").arg(e.what()));
    //         }
    //     }

    //     // 重置
    //     isOptimizing = false;
    //     asyncOptimizeTask.reset();

    // } catch (const std::invalid_argument& e) {
    //     // 特定捕获：无效任务（虽 cancel 已自保护，但 get 可能触发）
    //     isOptimizing = false;
    //     asyncOptimizeTask.reset();
    //     UpdateUI(QString("Invalid task during cancel: %1").arg(e.what()));
    // } catch (const std::exception& e) {
    //     // 其他异常
    //     isOptimizing = false;
    //     asyncOptimizeTask.reset();
    //     UpdateUI(QString("Error during cancellation: %1").arg(e.what()));
    // }

}

void MainWindow::on_btnMoveReverseTrajectory_clicked()
{
    if(!zm->GetConnectStatus())
    {
        UpdateUI("未连接控制器");
        qInfo()<<"未连接控制器";
        return;
    }
    if (jointTrajectory.empty()) {
        UpdateUI("Empty trajectory result");
    }
    zm->MoveAbsTrajectoryThetas(std::vector<std::array<double, 8>>(jointTrajectory.rbegin(), jointTrajectory.rend()));

}

void MainWindow::on_btnMoveTrajectory_clicked()
{
    // if(!zm->GetConnectStatus())
    // {
    //     UpdateUI("未连接控制器");
    //     qInfo()<<"未连接控制器";
    //     return;
    // }
    // if (jointTrajectory.empty()) {
    //     UpdateUI("Empty trajectory result");
    // }
    // zm->MoveAbsTrajectoryThetas(jointTrajectory);
    onMoveJointTraj(jointTrajectory);
}


void MainWindow::on_btnTestChess_clicked()
{
    // QString currentPath = QDir::currentPath();
    // QString dataQFile = QDir(currentPath).filePath("lib/data/photos/calibration/");
    // // 2. 对特定图像检测角点
    // cv::Mat image = cv::imread(dataQFile.toStdString()+"13.jpg");

    cv::Mat chess_img = cv::imread(ui->LineEditPhotoAddress->text().toStdString());
    if (chess_img.empty()) {
        UpdateUI("没有棋盘格照片");
        return;
    }

    std::vector<cv::Point2f> img_chess_points;       // 图像点输出
    std::vector<cv::Point3f> world_chess_coords;     // 世界坐标输出
    std::optional<Eigen::Matrix4d> cam_chess_pose;   // 相机位姿输出
    std::optional<Eigen::Matrix4d> base_chess_pose;  // 基位姿输出

    bool chess_success = featureDetector->detectAndComputeChessboardPoints(chess_img, img_chess_points, world_chess_coords,
                                                                   cam_chess_pose, base_chess_pose, detectionParams);

    if (chess_success) {
        std::cout << "棋盘格检测成功！检测到 " << img_chess_points.size() << " 个角点" << std::endl;
        std::cout << "重投影误差: " << std::fixed << std::setprecision(2)
                  << featureDetector->getLastResult().reproj_error_mm << " 像素" << std::endl;

        // 打印世界坐标 (前几个示例)
        std::cout << "世界坐标 (x, y, z mm，前 5 个):" << std::endl;
        for (size_t i = 0; i < std::min<size_t>(5, world_chess_coords.size()); ++i) {
            const auto& pt = world_chess_coords[i];
            std::cout << "  点 " << i << ": (" << pt.x << ", " << pt.y << ", " << pt.z << ")" << std::endl;
        }

        // 打印相机位姿平移
        if (cam_chess_pose) {
            Eigen::Vector3d trans = cam_chess_pose->col(3).head<3>();
            std::cout << "棋盘格相机位姿平移: (" << std::fixed << std::setprecision(1)
                      << trans.x() << ", " << trans.y() << ", " << trans.z() << ") mm" << std::endl;
        }

        // 打印基位姿 (如果计算)
        if (base_chess_pose) {
            Eigen::Vector3d trans = base_chess_pose->col(3).head<3>();
            std::cout << "棋盘格基位姿平移: (" << std::fixed << std::setprecision(1)
                      << trans.x() << ", " << trans.y() << ", " << trans.z() << ") mm" << std::endl;
        }
    }
    else {
        std::cerr << "棋盘格检测失败: " << featureDetector->getLastResult().error_message << std::endl;
    }

    // std::vector<cv::Point2f> imagePoints;
    // bool found = calibrator->detectFeaturePoints(image, imagePoints, "CHESSBOARD");

    // if (found) {
    //     // 3. 将图像坐标转换到相机坐标系
    //     std::vector<cv::Point3f> cameraPoints = calibrator->pixel2Camera(imagePoints, 115);
    //     getPosition();
    //     Eigen::Vector3d camera(80.05, -7.73, -115);          // 相机位置

    //     // 构建相机相对于末端执行器的变换矩阵
    //     Eigen::Matrix4d T_end_camera = Eigen::Matrix4d::Identity();
    //     T_end_camera.block<3, 1>(0, 3) = camera;  // 设置平移部分

    //     // 右乘得到相机相对于基座的变换
    //     Eigen::Matrix4d T_base_camera_eigen = robotPose.flange_matrix4d * T_end_camera;

    //     // 转换为 cv::Mat

    //     cv::Mat T_base_camera = cv::Mat::zeros(4, 4, CV_64F);
    //     for (int i = 0; i < 4; i++) {
    //         for (int j = 0; j < 4; j++) {
    //             T_base_camera.at<double>(i, j) = T_base_camera_eigen(i, j);
    //         }
    //     }
    //     // 5. 转换到机械臂基座标系
    //     std::vector<cv::Point3f> basePoints = calibrator->camera2Base(cameraPoints, T_base_camera_eigen);

    //     // 6. 可视化结果
    //     calibrator->visualizeResults(image, imagePoints, basePoints, T_base_camera);
    // }
}


void MainWindow::on_btnTestCircle_clicked()
{
    if(ui->LineEditPhotoAddress->text()=="")
        return;

    cv::Mat circle_img = cv::imread(ui->LineEditPhotoAddress->text().toStdString());
    if (circle_img.empty()) {
        UpdateUI("没有圆形照片");
    }

    std::vector<cv::Point3f> cam_circle_coords;  // 相机坐标输出
    std::optional<std::vector<cv::Point3f>> base_circle_coords;  // 基坐标输出 (可选)

    bool circle_success = featureDetector->detectAndComputeCircleCenters(circle_img, cam_circle_coords, base_circle_coords, detectionParams);

    if (circle_success) {
        std::cout << "圆心检测成功！检测到 " << cam_circle_coords.size() << " 个点" << std::endl;
        std::cout << "重投影误差: " << std::fixed << std::setprecision(2)
                  << featureDetector->getLastResult().reproj_error_mm << " 像素" << std::endl;

        // 打印相机坐标
        std::cout << "相机坐标 (x, y, z mm):" << std::endl;
        for (size_t i = 0; i < cam_circle_coords.size(); ++i) {
            const auto& pt = cam_circle_coords[i];
            std::cout << "  点 " << i << ": (" << pt.x << ", " << pt.y << ", " << pt.z << ")" << std::endl;
        }

        // 打印基坐标 (如果计算)
        if (base_circle_coords) {
            std::cout << "基坐标 (x, y, z mm):" << std::endl;
            for (size_t i = 0; i < base_circle_coords->size(); ++i) {
                const auto& pt = (*base_circle_coords)[i];
                std::cout << "  点 " << i << ": (" << pt.x << ", " << pt.y << ", " << pt.z << ")" << std::endl;
            }
        }
    }
    else {
        std::cerr << "圆心检测失败: " << featureDetector->getLastResult().error_message << std::endl;
    }
    // QString currentPath = QDir::currentPath();
    // QString dataQFile = QDir(currentPath).filePath("lib/data/photos/calibration/");
    // cv::Mat image = cv::imread(dataQFile.toStdString()+"C1.jpg");


    // std::vector<cv::Point2f> imagePoints;
    // bool found = calibrator->detectFeaturePoints(image, imagePoints, "CIRCLE");

    // if (found) {
    //     // 3. 将图像坐标转换到相机坐标系
    //     std::vector<cv::Point3f> cameraPoints = calibrator->pixel2Camera(imagePoints,115);
    //     getPosition();
    //     Eigen::Vector3d camera(80.05, -7.73, -115);          // 相机位置

    //     //// 构建相机相对于末端执行器的变换矩阵
    //     //Eigen::Matrix4d T_end_camera = Eigen::Matrix4d::Identity();
    //     //T_end_camera.block<3, 1>(0, 3) = camera;  // 设置平移部分

    //     //// 右乘得到相机相对于基座的变换
    //     //Eigen::Matrix4d T_base_camera_eigen = current_pose * T_end_camera;

    //     //// 转换为 cv::Mat

    //     //cv::Mat T_base_camera = cv::Mat::zeros(4, 4, CV_64F);
    //     //for (int i = 0; i < 4; i++) {
    //     //    for (int j = 0; j < 4; j++) {
    //     //        T_base_camera.at<double>(i, j) = T_base_camera_eigen(i, j);
    //     //    }
    //     //}
    //     // 5. 转换到机械臂基座标系
    //     // std::vector<cv::Point3f> basePoints = calibrator->camera2Base(cameraPoints, T_base_camera_eigen);
    //     auto basePoints = calibrator->pixelToRobotBase(imagePoints, eigenToCvMat(robotPose.flange_matrix4d));

    //     // 6. 可视化结果
    //     //calibrator->visualizeResults(image, imagePoints, basePoints, T_base_camera);
    //     std::stringstream ss;
    //     ss << "Base Points (" << basePoints.size() << " points):\n";
    //     ss << std::fixed << std::setprecision(3);

    //     for (size_t i = 0; i < basePoints.size(); ++i) {
    //         ss << "[" << i << "] ("
    //            << basePoints[i].x << ", "
    //            << basePoints[i].y << ", "
    //            << basePoints[i].z << ")\n";
    //     }

    //     // 直接在调用时转换
    //     UpdateUI(QString::fromStdString(ss.str()));
    // }

}


void MainWindow::on_btnOpenPhoto_clicked()
{
    QFileDialog dlg(this);
    QString currentPath = QDir::currentPath();
    QString dataQFile = QDir(currentPath).filePath("lib/data/photos");

    // 打开文件对话框，让用户选择一个 CSV 文件
    QString fileName = dlg.getOpenFileName(
        nullptr,                    // 父窗口指针，这里为 nullptr 表示没有父窗口
        tr("Open Photo File"),       // 对话框标题
        dataQFile,                    // 默认目录
        tr("JPG Files (*.jpg);;All Files (*)")  // 文件过滤器，只显示 CSV 文件和所有文件
        );

    // 检查是否选择了文件
    if (!fileName.isEmpty()) {
        qDebug() << "Selected file:" << fileName;
        // 将文件路径赋值给变量
        // QString csvfilePath = fileName;
        ui->LineEditPhotoAddress->setText(fileName);
    } else {
        qDebug() << "No file selected.";
    }
}


void MainWindow::on_btnRecordCircleUpperBlock_clicked()
{
    if(!zm->GetConnectStatus())
    {
        UpdateUI("未连接控制器");
        qInfo()<<"未连接控制器";
        return;
    }
    QLabel* Block[4][2] = {
        {ui->lblBlockPosi00,ui->lblBlockPosi00},
        {ui->lblBlockPosi10,ui->lblBlockPosi11},
        {ui->lblBlockPosi20,ui->lblBlockPosi20},
        {ui->lblBlockPosi30,ui->lblBlockPosi31}
    };
    getPosition();
    Block[ui->spbBlockCircleNum->value()][0]->setText(QString::number((robotPose.world_pose.x)/1000.0,'f',3)+","
                                                +QString::number((robotPose.world_pose.y)/1000.0,'f',3)+","
                                                +QString::number((robotPose.world_pose.z+2358)/1000.0,'f',3));
}


void MainWindow::on_btnRecordCircleLowerBlock_clicked()
{
    if(!zm->GetConnectStatus())
    {
        UpdateUI("未连接控制器");
        qInfo()<<"未连接控制器";
        return;
    }
    QLabel* Block[4][2] = {
        {ui->lblBlockPosi00,ui->lblBlockPosi00},
        {ui->lblBlockPosi10,ui->lblBlockPosi11},
        {ui->lblBlockPosi20,ui->lblBlockPosi20},
        {ui->lblBlockPosi30,ui->lblBlockPosi31}


    };
    getPosition();
    Block[ui->spbBlockCircleNum->value()][1]->setText(QString::number(robotPose.world_pose.x,'f',3)+","
                                                +QString::number(robotPose.world_pose.y,'f',3)+","
                                                +QString::number(robotPose.world_pose.z,'f',3));
}


void MainWindow::on_btnSaveCircleBlock_clicked()
{

        QLabel* Block[4][2] = {
            {ui->lblBlockPosi00,ui->lblBlockPosi00},
            {ui->lblBlockPosi10,ui->lblBlockPosi11},
            {ui->lblBlockPosi20,ui->lblBlockPosi20},
            {ui->lblBlockPosi30,ui->lblBlockPosi31}


        };
        for (int i = 0; i < 4; i++)
        {
            iniRead->setValue(QString::number(ui->spbBlockGroupCircleNum->value()) + "/Block" + QString::number(i) + "/CirclePosi0", Block[i][0]->text());
            iniRead->setValue(QString::number(ui->spbBlockGroupCircleNum->value()) + "/Block" + QString::number(i) + "/CirclePosi1", Block[i][1]->text());

        }
        iniRead->setValue(QString::number(ui->spbBlockGroupCircleNum->value()) + "/Block" + "/Circle/Descripition", ui->BlockCircleDescription->text());
        UpdateUI("保存第" + QString::number(ui->spbBlockGroupCircleNum->value()) + "个Block数据");
}


void MainWindow::on_btnSetRealCircleBlock_clicked()
{
    if(processor==nullptr)
    {
        UpdateUI("请先处理SW曲线");
        return;
    }
    vector<Vector3d> real_point;
    QLabel* Block[4][2] = {
        {ui->lblBlockPosi00,ui->lblBlockPosi00},
        {ui->lblBlockPosi10,ui->lblBlockPosi11},
        {ui->lblBlockPosi20,ui->lblBlockPosi20},
        {ui->lblBlockPosi30,ui->lblBlockPosi31}
    };
    // 假设 inputString 是从 Block[...]->text() 获取的字符串
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<2;j++)
        {
            if (j==1&&(i == 0 || i == 2))
                continue;
            // 分割字符串
            QString inputString = Block[i][j]->text();
            QStringList parts = inputString.split(",");
            if (parts.size() == 3) {
                Vector3d point;
                point.x() = parts[0].toDouble();
                point.y() = parts[1].toDouble();
                point.z() = parts[2].toDouble();
                if(point.x()!=0&& point.y() != 0 && point.z() != 0 )
                real_point.push_back(point);
            }
        }

    }
    // for(int i=0;i<real_point.size();i++)
    // {

    //     UpdateUI(QString::number(real_point[i].x()));
    //     UpdateUI(QString::number(real_point[i].y()));
    //     UpdateUI(QString::number(real_point[i].z()));

    // }
    // 使用 QFileInfo 提取文件信息
    QString dataQFile = ui->LineEditDataAddress->text();
    QFileInfo fileInfo(dataQFile);
    QString fileNameWithExtension = fileInfo.fileName();
    qDebug() << "Selected file:" << fileNameWithExtension;

    // 获取文件名（包括扩展名）
    ////导出轨迹
    QDateTime curDateTime = QDateTime::currentDateTime();

    QString desktopPath = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation);
    QString file(desktopPath + '/' + curDateTime.toString("yyyy-MM-dd-hh-mm-") + fileNameWithExtension);

    processor->setRealCircles(real_point);
    trajectory.clear();
    trajectory=processor->generateOrientInpIKTrajectory(file.toStdString());

}


void MainWindow::on_spbBlockGroupCircleNum_editingFinished()
{
    QLabel* Block[4][2] = {
        {ui->lblBlockPosi00,ui->lblBlockPosi00},
        {ui->lblBlockPosi10,ui->lblBlockPosi11},
        {ui->lblBlockPosi20,ui->lblBlockPosi20},
        {ui->lblBlockPosi30,ui->lblBlockPosi31}


    };
    for (int i = 0; i < 4; i++)
    {
        Block[i][0]->setText(iniRead->value(QString::number(ui->spbBlockGroupCircleNum->value()) + "/Block" + QString::number(i) + "/CirclePosi0").toString());
        Block[i][1]->setText(iniRead->value(QString::number(ui->spbBlockGroupCircleNum->value()) + "/Block" + QString::number(i) + "/CirclePosi1").toString());

    }
    ui->BlockCircleDescription->setText(iniRead->value(QString::number(ui->spbBlockGroupCircleNum->value()) + "/Block" + "/Circle/Descripition").toString());
    UpdateUI("读取第"+ QString::number(ui->spbBlockGroupCircleNum->value()) +"个Block数据");
}

void MainWindow::on_btnCalculatePlaneAlignmentTrajectory_clicked()
{
    if(adjustedPose.isApprox(Eigen::Matrix4d::Identity(), 1e-6))//判断是否为单位矩阵
    {
        UpdateUI("传感器数值错误，未计算出结果");
        return;
    }
    auto adjustedTrajectory= GAOptimizer->optimizeSinglePoint(adjustedPose);
    std::vector<std::array<double, 8>> adjustedTrajectoryVector;
    adjustedTrajectoryVector.push_back(adjustedTrajectory);
    zm->MoveAbsTrajectoryThetas(adjustedTrajectoryVector);

}


void MainWindow::on_btnPlaneAlignment_clicked()
{
    adjustedPose=Matrix4d::Identity();
    compensator->setDebugMode(true);
    // 临时存储三个传感器值
    Eigen::Vector3d sensorValues(3);
    bool hasInvalidValue = false;

    // 读取并转换传感器数据
    sensorValues[0] = ui->Sensor1->text().toDouble();
    sensorValues[1] = ui->Sensor2->text().toDouble();
    sensorValues[2] = ui->Sensor3->text().toDouble();

    // 检查是否有无效值 (999.99)
    const double INVALID_VALUE = 999.99;
    for (int i = 0; i < 3; ++i) {
        if (std::abs(sensorValues[i] - INVALID_VALUE) < 1) {  // 浮点数比较容差
            hasInvalidValue = true;
            break;
        }
    }

    // 如果有无效值则返回默认值
    if (hasInvalidValue) {
        return;
    }
    getPosition();
    // 4. 计算修正后的位姿
    adjustedPose = compensator->computeAdjustedPose(
        toEigenMatrix(robotPose.tcp_pose), sensorValues);
}

void MainWindow::on_btnCameraParamsCalibrator_clicked()
{
    std::filesystem::path folder_path("lib/data/calibration/CameraParamsCalibration");
    std::thread([this, folder_path]() {
        std::vector<std::string> calibration_images;

        // 步骤1: 检查文件夹是否存在
        if (!std::filesystem::exists(folder_path) || !std::filesystem::is_directory(folder_path)) {
            QString errorMsg = QString("错误：标定文件夹 '%1' 不存在或不是目录！").arg(QString::fromStdString(folder_path.string()));
            UpdateUI(errorMsg);  // UI更新错误
            return;
        }

        // 步骤2: 遍历文件夹，收集 .jpg 文件
        for (const auto& entry : std::filesystem::directory_iterator(folder_path)) {
            if (entry.is_regular_file()) {  // 只处理普通文件
                std::string ext = entry.path().extension().string();
                // 忽略大小写检查扩展名（.jpg 或 .JPG）
                std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
                if (ext == ".jpg") {
                    calibration_images.push_back(entry.path().string());  // 完整路径
                }
            }
        }

        // 步骤3: 如果没有图像，报错
        if (calibration_images.empty()) {
            QString warnMsg = "警告：标定文件夹中没有找到 JPG 图像！请检查路径和文件。";
            UpdateUI(warnMsg);
        }

        // 步骤4: 排序
        std::sort(calibration_images.begin(), calibration_images.end(),
                  [](const std::string& a, const std::string& b) { return a < b; });

        // 步骤5: 更新UI - 显示数量和路径列表
        QString countMsg = QString("找到 %1 张标定图像。").arg(calibration_images.size());
        UpdateUI(countMsg);  // 第一条消息：数量

        // 步骤6: 执行标定
        if (!cameraParamsCalibrator->calibrateCamera(calibration_images)) {
            QString failMsg = "相机内参标定失败";
            UpdateUI(failMsg);
            return;
        }
        cameraParamsCalibrator->saveParams(folder_path.string()+"/cameraParamsCalibrator.txt");

        QString successMsg = "标定成功！";
        UpdateUI(successMsg);
    }).detach();  // 启动并detach（独立跑，不join阻塞）

}


void MainWindow::on_btnTCPCalibration_clicked()
{
    // 1. 设置理论TCP偏移（比如根据工具的CAD尺寸）
    tCPCalibrator->setTheoreticalTCP(0, 0, 342); // 工具长度200mm，沿Z轴342

    tCPCalibrator->loadCalibrationPosesFromCSV("lib/data/calibration/TCPCalibration/robot_poses.csv");

    std::thread([this]() {
        // 4. 执行标定
        if (auto result = tCPCalibrator->calibrateTCP()) {
            UpdateUI("TCP标定成功!");
            tCPCalibrator->printCalibrationInfo();

            // 保存结果
            tCPCalibrator->saveTCPResult("lib/data/calibration/TCPCalibration/TCPCalibration.txt");

            // 5. 验证标定结果
            tCPCalibrator->validateTCP();
        }
        else {
            UpdateUI("TCP标定失败!");
        }
    }).detach();  // 启动并detach（独立跑，不join阻塞）

}


void MainWindow::on_btnEyeInHandCalibration_clicked()
{
    // // 准备手眼标定数据
    // std::vector<cv::Mat> handEyeImages;
    // std::vector<cv::Mat> robotPoses;
    // std::vector<Eigen::Matrix4d> eigenPoses = CameraCalibration::readPosesFromCSV("lib/data/calibration/robot_poses.csv");

    // // 加载图像和对应的机器人位姿
    // for(int i = 0; i < 17; i++) {
    //     std::cout << "lib/data/calibration/00" + std::to_string(i) + ".jpg";
    //     cv::Mat img = cv::imread("lib/data/calibration/00" + std::to_string(i) + ".jpg");
    //     handEyeImages.push_back(img);

    //     // 机器人位姿 (4x4变换矩阵)
    //     cv::Mat pose = eigenToCvMat(eigenPoses[i]);
    //     robotPoses.push_back(pose);
    // }

    // // 执行手眼标定
    // bool success = calibrator->performHandEyeCalibration(handEyeImages, robotPoses);

    // if(success) {
    //     // 保存结果
    //     calibrator->saveHandEyeData("lib/data/calibration/hand_eye_calibration.xml");

    //     // 使用结果进行坐标变换
    //     std::vector<cv::Point2f> pixelPoints = {cv::Point2f(320, 240)};
    //     cv::Mat currentPose = eigenToCvMat(eigenPoses[0]); // 当前机器人位姿

    //     auto basePoints = calibrator->pixelToRobotBase(pixelPoints, currentPose);

    //     std::cout << "机器人基座坐标: " << basePoints[0] << std::endl;
    // }
    // 设置参考图像
    // cv::Mat reference_image = cv::imread("lib/data/calibration/CameraParamsCalibration/003.jpg");
    // if (!handEyeCalibration->setReferenceImage(reference_image)) {
    //     UpdateUI("无法检测棋盘格" );
    //     return;
    // }
    // // 设置参考图像
    // handEyeCalibration->setReferenceImage(reference_image);

    // // 显示棋盘格和角点序号
    // std::vector<int> default_corners = handEyeCalibration->getDefaultCornerIndices();
    // handEyeCalibration->showBoardWithIndices("Chessboard", default_corners);

    // // 添加触碰点（默认四个角点）
    // std::vector<RobotPose> poses = { }; // 你的位姿数据
    // int added = handEyeCalibration->addTouchPointsBatch(default_corners, poses);

    // // 或者手动指定角点序号
    // // std::vector<int> custom_corners = {0, 8, 15, 35}; // 自定义序号
    // // calibration.addTouchPointsBatch(custom_corners, poses);
    // std::thread([this]() {
    // // 执行标定
    // auto result = handEyeCalibration->calibrate();
    // handEyeCalibration->saveCalibrationResult("lib/data/calibration/EyeInHandCalibration/EyeInHandCalibration.txt",result);
    // }).detach();  // 启动并detach（独立跑，不join阻塞）

}


void MainWindow::on_btnSaveCalibrationData_clicked()
{
    getPosition();
    savePoseToCSV("lib/data/calibration/robot_poses.csv", toEigenMatrix(robotPose.tcp_pose), true);

}


void MainWindow::on_btnGetShuaTraj_clicked()
{
    trajectory=processor->generateSideFaceIKTrajectory();
}


void MainWindow::on_btnGetTieTraj_clicked()
{
    //trajectory=processor->generateOrientInpIKTrajectory();

    if (processor == nullptr)
    {
        UpdateUI("请先选择文件并点击“处理SW曲线”");
        return;
    }

    const QString desktopPath =
        QStandardPaths::writableLocation(QStandardPaths::DesktopLocation);

    const QString fileName =
        QDateTime::currentDateTime().toString("yyyy-MM-dd-hh-mm-ss-")
        + "tie_pose_trajectory.csv";

    const QString outputPath =
        QDir(desktopPath).filePath(fileName);

    trajectory = processor->generateOrientInpIKTrajectory(
        outputPath.toStdString()
        );

    if (trajectory.empty())
    {
        UpdateUI("贴胶位姿轨迹生成失败：没有得到轨迹点");
        return;
    }

    UpdateUI(
        QString("贴胶位姿CSV生成成功：%1个点，保存位置：%2")
            .arg(trajectory.size())
            .arg(outputPath)
        );
}

