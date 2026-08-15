#ifndef BASESERIALPORTMANAGER_H
#define BASESERIALPORTMANAGER_H

#include <QObject>
#include <QSerialPort>
#include <QTimer>
#include <QElapsedTimer>
#include <QByteArray>
#include <QStringList>
#include <QVector>
#include <QMutex>
#include <QThread>
#include <limits>
#include <QDateTime>
#include <QFile>
#include <QTextStream>
#include <QMutex>
#include <QDir>
#include <QMutexLocker>

class DataProcessWorker;  // 前置声明
struct DataRecord {
    QString timestamp;     // 时间字符串
    QVector<double> values; // 对应的平均值数据
};
class BaseSerialPortManager : public QObject
{
    Q_OBJECT
public:
    explicit BaseSerialPortManager(QObject *parent = nullptr);
    virtual ~BaseSerialPortManager();

    void initSerialPort();
    QStringList getAvailablePorts();
    bool openPort(const QString &portName, int baudRate);
    void closePort();
    bool isPortOpen() const;

    void sendData(const QByteArray &data);
    void sendHexString(const QString &hexStr);

    void startAutoPolling(const QVector<quint8> &stationAddresses, int interval = 20);
    void stopAutoPolling();
    bool isAutoPolling() const;

    // 数据处理线程控制
    void startDataProcessing(int intervalMs = 50);
    void stopDataProcessing();
    void setMaxBufferFrames(int maxFrames);

    static QByteArray convertHexStringToByteArray(const QString &hexStr);
    // 多线程安全接口，用于在工作线程调用数据解析，返回是否成功解析帧
    bool parseRawData(const QByteArray &buffer);
    // 数据保存
    void setSaveDataEnabled(bool enabled);
    bool isSaveDataEnabled() const;

    bool exportSavedDataToCsv();
signals:
    void dataReceived(const QString &hexData);
    void errorOccurred(const QString &error);
    void processedDataReady(const QVector<double> &data);  // 处理后的平均数据
    void saveDataStatusChanged(bool enabled);
protected slots:
    void handleReadyRead();
    void handlePollingTimeout();
    void onProcessedDataReady(const QVector<double> &data);

protected:
    virtual void processDataFrame() = 0;
    virtual void sendPollingRequest(quint8 stationAddress) = 0;
    virtual void stopCollectData(){}
    QSerialPort *m_serialPort;
    QTimer *m_pollingTimer;
    QElapsedTimer m_frameTimer;
    QByteArray m_receiveBuffer;
    QVector<quint8> m_stationAddresses;
    QVector<double> m_stationValues;
    int m_currentStationIndex;

    // 线程安全的数据缓存
    QByteArray m_rawDataBuffer;
    QMutex m_bufferMutex;

    static const int FRAME_TIMEOUT_MS = 50;

private:
    QThread *m_workerThread;
    DataProcessWorker *m_dataWorker;
    friend class DataProcessWorker;
    bool m_saveDataEnabled = false;
    QVector<DataRecord> m_savedData;
    QMutex m_dataMutex;  // 保护m_savedData
};

// 数据处理工作线程类
class DataProcessWorker : public QObject
{
    Q_OBJECT
public:
    explicit DataProcessWorker(BaseSerialPortManager *manager, QObject *parent = nullptr);
    ~DataProcessWorker();

public slots:
    void startProcessing(int intervalMs);
    void stopProcessing();
    void setMaxBufferFrames(int maxFrames);

private slots:
    void processDataBuffer();

signals:
    void dataProcessed(const QVector<double> &avgData);

private:
    BaseSerialPortManager *m_manager;
    QTimer *m_processTimer;
    QVector<QVector<double>> m_frameBuffer;  // 多帧数据缓存
    int m_maxBufferFrames;
};

#endif // BASESERIALPORTMANAGER_H
