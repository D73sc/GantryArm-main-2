#include "BaseSerialPortManager.h"
#include <QSerialPortInfo>
#include <QDebug>

BaseSerialPortManager::BaseSerialPortManager(QObject *parent)
    : QObject(parent)
    , m_serialPort(nullptr)
    , m_pollingTimer(nullptr)
    , m_currentStationIndex(0)
    , m_workerThread(nullptr)
    , m_dataWorker(nullptr)
{
    initSerialPort();
    static bool registered = false;
    if (!registered) {
        qRegisterMetaType<QVector<double>>("QVector<double>");
        registered = true;
    }
    connect(this, &BaseSerialPortManager::processedDataReady,
            this, &BaseSerialPortManager::onProcessedDataReady);
}

BaseSerialPortManager::~BaseSerialPortManager()
{

    stopDataProcessing();
    stopCollectData();
    closePort();

    if (m_pollingTimer) {
        delete m_pollingTimer;
    }
    if (m_serialPort) {
        delete m_serialPort;
    }
}

void BaseSerialPortManager::initSerialPort()
{
    m_serialPort = new QSerialPort(this);
    m_pollingTimer = new QTimer(this);
    connect(m_pollingTimer, &QTimer::timeout, this, &BaseSerialPortManager::handlePollingTimeout);
    m_frameTimer.start();
}

QStringList BaseSerialPortManager::getAvailablePorts()
{
    QStringList portNames;
    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
    {
        portNames << info.portName();
        qDebug() << "Available port:" << info.portName();
    }
    return portNames;
}

bool BaseSerialPortManager::openPort(const QString &portName, int baudRate)
{
    if (m_serialPort->isOpen()) {
        m_serialPort->clear();
        m_serialPort->close();
    }

    m_serialPort->setPortName(portName);
    if (!m_serialPort->open(QIODevice::ReadWrite)) {
        emit errorOccurred(QString("Failed to open serial port %1").arg(portName));
        return false;
    }

    m_serialPort->setBaudRate(baudRate, QSerialPort::AllDirections);
    m_serialPort->setDataBits(QSerialPort::Data8);
    m_serialPort->setFlowControl(QSerialPort::NoFlowControl);
    m_serialPort->setParity(QSerialPort::NoParity);
    m_serialPort->setStopBits(QSerialPort::OneStop);

    connect(m_serialPort, &QSerialPort::readyRead, this, &BaseSerialPortManager::handleReadyRead);

    // 打开串口后自动启动数据处理线程
    startDataProcessing(50);  // 默认50ms处理一次

    return true;
}

void BaseSerialPortManager::closePort()
{
    stopAutoPolling();
    stopDataProcessing();

    if (m_serialPort && m_serialPort->isOpen()) {
        m_serialPort->close();
    }
}

bool BaseSerialPortManager::isPortOpen() const
{
    return m_serialPort && m_serialPort->isOpen();
}

void BaseSerialPortManager::sendData(const QByteArray &data)
{
    if (!isPortOpen()) {
        emit errorOccurred("Serial port not open");
        return;
    }

    qint64 bytesWritten = m_serialPort->write(data);
    if (bytesWritten == -1) {
        emit errorOccurred(QString("Failed to send data: %1").arg(m_serialPort->errorString()));
    }
}

void BaseSerialPortManager::sendHexString(const QString &hexStr)
{
    QByteArray data;
    QString tempStr = hexStr;
    tempStr.remove(' ');
    if (tempStr.length() % 2 != 0) {
        emit errorOccurred("Hex string length invalid (not even)");
        return;
    }
    bool ok;
    for (int i = 0; i < tempStr.length(); i += 2) {
        QString byteStr = tempStr.mid(i, 2);
        quint8 byte = byteStr.toUShort(&ok, 16);
        if (!ok) {
            emit errorOccurred(QString("Invalid hex character: %1").arg(byteStr));
            return;
        }
        data.append(byte);
    }
    sendData(data);
}

void BaseSerialPortManager::startAutoPolling(const QVector<quint8> &stationAddresses, int interval)
{
    if (!isPortOpen()) {
        emit errorOccurred("Serial port not open, cannot start auto polling");
        return;
    }
    m_stationAddresses = stationAddresses;
    m_stationValues.resize(stationAddresses.size());
    m_currentStationIndex = 0;
    m_pollingTimer->start(interval);
}

void BaseSerialPortManager::stopAutoPolling()
{
    if (m_pollingTimer) {
        m_pollingTimer->stop();
    }
}

bool BaseSerialPortManager::isAutoPolling() const
{
    return m_pollingTimer && m_pollingTimer->isActive();
}

void BaseSerialPortManager::startDataProcessing(int intervalMs)
{
    if (!m_workerThread) {
        m_workerThread = new QThread(this);
        m_dataWorker = new DataProcessWorker(this);
        m_dataWorker->moveToThread(m_workerThread);

        connect(m_workerThread, &QThread::finished, m_dataWorker, &QObject::deleteLater);
        connect(m_dataWorker, &DataProcessWorker::dataProcessed,
                this, &BaseSerialPortManager::processedDataReady);
        // connect(this, &BaseSerialPortManager::processedDataReady,
        //         this, [](const QVector<double> &data){
        //             qDebug() << "processedDataReady received data with size:" << data.size();
        //         });
        m_workerThread->start();
    }

    QMetaObject::invokeMethod(m_dataWorker, "startProcessing", Qt::QueuedConnection,
                              Q_ARG(int, intervalMs));
}

void BaseSerialPortManager::stopDataProcessing()
{
    if (m_workerThread) {
        QMetaObject::invokeMethod(m_dataWorker, "stopProcessing", Qt::QueuedConnection);
        m_workerThread->quit();
        m_workerThread->wait();
        m_workerThread->deleteLater();
        m_workerThread = nullptr;
        m_dataWorker = nullptr;
    }
}

void BaseSerialPortManager::setMaxBufferFrames(int maxFrames)
{
    if (m_dataWorker) {
        QMetaObject::invokeMethod(m_dataWorker, "setMaxBufferFrames", Qt::QueuedConnection,
                                  Q_ARG(int, maxFrames));
    }
}

QByteArray BaseSerialPortManager::convertHexStringToByteArray(const QString &hexStr)
{
    QByteArray byteArray;
    QString tempStr = hexStr;
    tempStr.remove(' ');

    if (tempStr.length() % 2 != 0) {
        qDebug() << "Hex string length invalid (not even)";
        return QByteArray();
    }

    bool ok;
    for (int i = 0; i < tempStr.length(); i += 2) {
        QString byteStr = tempStr.mid(i, 2);
        quint8 byte = byteStr.toUShort(&ok, 16);
        if (!ok) {
            qDebug() << "Invalid hex character:" << byteStr;
            return QByteArray();
        }
        byteArray.append(byte);
    }

    return byteArray;
}

bool BaseSerialPortManager::parseRawData(const QByteArray &buffer)
{
    QMutexLocker locker(&m_bufferMutex);
    m_receiveBuffer.append(buffer);
    // 调用子类具体解析逻辑，processDataFrame中要负责更新m_stationValues
    processDataFrame();
    // 简单判断：如果m_stationValues非空，则解析成功
    return !m_stationValues.isEmpty();
}

void BaseSerialPortManager::handleReadyRead()
{
    if (m_frameTimer.hasExpired(FRAME_TIMEOUT_MS)) {
        QMutexLocker locker(&m_bufferMutex);
        m_rawDataBuffer.clear();
    }
    m_frameTimer.start();

    QByteArray newData = m_serialPort->readAll();

    {
        QMutexLocker locker(&m_bufferMutex);
        m_rawDataBuffer.append(newData);
    }

    emit dataReceived(newData.toHex().toUpper());
    // processDataFrame();  // 子类可选实现实时处理
}

void BaseSerialPortManager::handlePollingTimeout()
{
    if (!isPortOpen() || m_stationAddresses.isEmpty()) {
        stopAutoPolling();
        return;
    }
    quint8 stationAddress = m_stationAddresses[m_currentStationIndex];
    sendPollingRequest(stationAddress);

    m_currentStationIndex = (m_currentStationIndex + 1) % m_stationAddresses.size();
}

void BaseSerialPortManager::setSaveDataEnabled(bool enabled)
{
    if (m_saveDataEnabled != enabled) {
        m_saveDataEnabled = enabled;
        emit saveDataStatusChanged(enabled);
    }
}

bool BaseSerialPortManager::isSaveDataEnabled() const
{
    return m_saveDataEnabled;
}

// 当有新的平均数据时，如果保存开关打开，则保存数据
void BaseSerialPortManager::onProcessedDataReady(const QVector<double> &data)
{
    if (!m_saveDataEnabled) return;

    DataRecord record;
    record.timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz");
    record.values = data;

    QMutexLocker locker(&m_dataMutex);
    m_savedData.push_back(std::move(record));
}

// 导出保存的数据到csv，文件名使用当前时间
bool BaseSerialPortManager::exportSavedDataToCsv()
{
    QMutexLocker locker(&m_dataMutex);

    if (m_savedData.empty()) {
        emit errorOccurred("No data to export");
        return false;
    }
    // 目标文件夹路径（相对于程序运行目录）
    QString dirPath = "lib/data/sensor";
    QDir dir;
    if (!dir.exists(dirPath)) {
        if (!dir.mkpath(dirPath)) {     // 递归创建目录
            emit errorOccurred(QString("Failed to create directory: %1").arg(dirPath));
            return false;
        }
    }

    // 文件名用当前时间命名，防止覆盖
    QString fileName = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss") + ".csv";
    QString fullFilePath = dir.filePath(dirPath + "/" + fileName);

    QFile file(fullFilePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        emit errorOccurred(QString("Failed to open file '%1' for writing").arg(fullFilePath));
        return false;
    }

    QTextStream out(&file);
    out << "Timestamp";

    // 写表头，假设所有行长度相同
    if (!m_savedData.first().values.isEmpty()) {
        for (int i = 0; i < m_savedData.first().values.size(); ++i) {
            out << QString(",Value%1").arg(i + 1);
        }
    }
    out << "\n";

    for (const auto &rec : m_savedData) {
        out << rec.timestamp;
        for (float v : rec.values) {
            out << "," << v;
        }
        out << "\n";
    }

    file.close();
    return true;
}

// ==================== DataProcessWorker 实现 ====================

DataProcessWorker::DataProcessWorker(BaseSerialPortManager *manager, QObject *parent)
    : QObject(parent)
    , m_manager(manager)
    , m_processTimer(new QTimer(this))
    , m_maxBufferFrames(100)
{
    connect(m_processTimer, &QTimer::timeout, this, &DataProcessWorker::processDataBuffer);
}

DataProcessWorker::~DataProcessWorker()
{
    stopProcessing();
}

void DataProcessWorker::startProcessing(int intervalMs)
{
    m_processTimer->start(intervalMs);
    qDebug() << "Data processing started with interval:" << intervalMs << "ms";
}

void DataProcessWorker::stopProcessing()
{
    m_processTimer->stop();
    m_frameBuffer.clear();
    qDebug() << "Data processing stopped";
}

void DataProcessWorker::setMaxBufferFrames(int maxFrames)
{
    m_maxBufferFrames = maxFrames;
}

void DataProcessWorker::processDataBuffer()
{
    QByteArray localBuffer;
    {
        QMutexLocker locker(&m_manager->m_bufferMutex);
        if (m_manager->m_rawDataBuffer.isEmpty()) {
            return;
        }
        localBuffer = m_manager->m_rawDataBuffer;
        m_manager->m_rawDataBuffer.clear();
    }

    bool success = m_manager->parseRawData(localBuffer);
    if (!success) return;

    QVector<double> currentFrame;
    {
        QMutexLocker locker(&m_manager->m_bufferMutex);
        currentFrame = m_manager->m_stationValues;
    }

    if (currentFrame.isEmpty()) return;

    // 直接返回当前帧数据作为“平均值”
    emit dataProcessed(currentFrame);
}
