#include "forceform.h"
#include "ui_forceform.h"

ForceForm::ForceForm(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::ForceForm)
{
    ui->setupUi(this);
    SerialPortInit();
}

ForceForm::~ForceForm()
{
    delete ui;
}

void ForceForm::on_btnRefreshPortsbtn_Force_clicked()
{
    QStringList availablePorts = m_serialManager->getAvailablePorts();
    ui->comboBoxPortName_Force->clear();
    ui->comboBoxPortName_Force->addItems(availablePorts);

    qDebug() << "Refreshed ports:" << availablePorts;
}


void ForceForm::on_btnOpenCOM_Force_clicked()
{
    if (ui->btnOpenCOM_Force->text()=="OpenCOM")
    {
        QString portName = ui->comboBoxPortName_Force->currentText();
        int baudRate = ui->comboBoxBaudRate_Force->currentText().toInt();
        if (m_serialManager->openPort(portName, baudRate)) {
            ui->btnOpenCOM_Force->setText("CloseCOM");
        }

    }
    else
    {
        m_serialManager->closePort();
        ui->btnOpenCOM_Force->setText("OpenCOM");
    }

}

void ForceForm::on_btnSendData_Force_clicked()
{
    QString m_strSendData = ui->txtSend_Force->text().trimmed();

    if (m_strSendData.isEmpty()) {
        qDebug() << "发送数据为空！";
        return;
    }

    m_serialManager->sendCustomCommand(m_strSendData);

}

void ForceForm::SerialPortInit()
{
    // 创建串口管理器
    m_serialManager = new ForceSerialPortManager(this);

    // 连接信号
    connect(m_serialManager, &BaseSerialPortManager::dataReceived,
            this, &ForceForm::onDataReceived);
    connect(m_serialManager, &BaseSerialPortManager::errorOccurred,
            this, &ForceForm::onErrorReceived);
    connect(m_serialManager, &BaseSerialPortManager::processedDataReady,
            this, &ForceForm::onForceDataUpdated);
    // 初始化串口列表
    ui->comboBoxPortName_Force->addItems(m_serialManager->getAvailablePorts());

    QStringList bautRatesList;
    bautRatesList << "1200" << "2400" << "4800" << "9600" << "19200" << "57600" << "115200";
    ui->comboBoxBaudRate_Force->addItems(bautRatesList);
    ui->comboBoxBaudRate_Force->setCurrentIndex(6);

    ui->txtSend_Force->setText("AT+GOD");

}

void ForceForm::onDataReceived(QString hexData)
{
    if(ui->btnStartAutoSendData_Force->text()=="StopContinuous")
        return;
        ui->txtReceiveData_Force->append(hexData);

}

void ForceForm::onErrorReceived(const QString &error)
{
    ui->txtReceiveData_Force->append(error);

}


void ForceForm::on_btnOnceSendData_Force_clicked()
{
    m_serialManager->sendSingleDataRequest();
}


void ForceForm::on_btnStartAutoSendData_Force_clicked()
{
    if(ui->btnStartAutoSendData_Force->text()=="StartContinuous")
    {

        m_serialManager->startContinuousData();
        ui->btnStartAutoSendData_Force->setText("StopContinuous");
    }
    else
    {
        m_serialManager->stopContinuousData();
        ui->btnStartAutoSendData_Force->setText("StartContinuous");
    }

}

void ForceForm::onForceDataUpdated(const QVector<double> &forceData)
{
    if(forceData.size()==3)
    {
        ui->Sensor_XForce->setText(QString::number(forceData[0], 'f', 2));
        ui->Sensor_YForce->setText(QString::number(forceData[1], 'f', 2));
        ui->Sensor_ZForce->setText(QString::number(forceData[2], 'f', 2));
        emit forceDataReady(forceData);
    }
    else if(forceData.size()==6)
    {
        ui->Sensor_XForce->setText(QString::number(forceData[0], 'f', 2));
        ui->Sensor_YForce->setText(QString::number(forceData[1], 'f', 2));
        ui->Sensor_ZForce->setText(QString::number(forceData[2], 'f', 2));
        ui->Sensor_XMForce->setText(QString::number(forceData[3], 'f', 2));
        ui->Sensor_YMForce->setText(QString::number(forceData[4], 'f', 2));
        ui->Sensor_ZMForce->setText(QString::number(forceData[5], 'f', 2));
        emit forceDataReady(forceData);
    }
    else
        qWarning()<<"ForceDataError!!!";

}

void ForceForm::on_btnSetHz_clicked()
{
    float hz=ui->lineEditHZ->text().toFloat();
    m_serialManager->setContinuousHZ(hz);
}


void ForceForm::on_btnInitSensor_clicked()
{
    m_serialManager->initSensor();

}


void ForceForm::on_btnSaveData_clicked()
{
    if(ui->btnSaveData->text()=="StartSaveData")
    {

        m_serialManager->setSaveDataEnabled(true);
        ui->btnSaveData->setText("StopSaveData");
    }
    else
    {
        m_serialManager->setSaveDataEnabled(false);
        ui->btnSaveData->setText("StartSaveData");
    }
}


void ForceForm::on_btnExportData_clicked()
{
    bool exportSuccess = m_serialManager->exportSavedDataToCsv();
    if (exportSuccess) {
        qDebug() << "数据导出成功";
    } else {
        qDebug() << "数据导出失败";
    }
}

