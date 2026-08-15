#include "ForceSerialPortManager.h"
#include <QDebug>
#include <cstring>

ForceSerialPortManager::ForceSerialPortManager(QObject *parent)
    : BaseSerialPortManager(parent)
{
}

// ==================== 指令集合 ====================

void ForceSerialPortManager::sendSingleDataRequest()
{
    QByteArray cmd = commandToAscii("AT+GOD");
    sendData(cmd);
    qDebug() << "Sent command: AT+GOD (Single data request)";
}

void ForceSerialPortManager::startContinuousData()
{
    QByteArray cmd = commandToAscii("AT+GSD");
    sendData(cmd);
    qDebug() << "Sent command: AT+GSD (Start continuous data)";
}

void ForceSerialPortManager::setContinuousHZ(float hz)
{
    // 先把频率转成字符串，比如保留一位小数
    QString cmdStr = QString("AT+SMPF=%1").arg(hz, 0, 'f', 1);
    QByteArray cmd = commandToAscii(cmdStr);

    sendData(cmd);

    // 打印完整命令，方便调试
    qDebug() << "Sent command:" << cmdStr;
}

void ForceSerialPortManager::initSensor()
{

    QByteArray cmd = commandToAscii("AT+SGDM=(A01,A02,A03,A04,A05,A06);E;1;(WMA:1)");
    sendData(cmd);
    qDebug() << "Sent command: AT+GSD (Start continuous data)";
}

void ForceSerialPortManager::stopContinuousData()
{
    QByteArray cmd = commandToAscii("AT+GSD=STOP");
    sendData(cmd);
    qDebug() << "Sent command: AT+GSD=STOP (Stop continuous data)";
}

void ForceSerialPortManager::sendCustomCommand(const QString &cmd)
{
    QByteArray asciiCmd = commandToAscii(cmd);
    sendData(asciiCmd);
    qDebug() << "Sent custom command:" << cmd;
}

// ==================== 数据处理 ====================

void ForceSerialPortManager::processDataFrame()
{
    const int MAX_PACKET_SIZE = 1024;  // 根据实际最大包长度调整
    const int MAX_BUFFER_SIZE = 4096;  // 防止缓冲区无限增长

    // 防止缓冲区过大
    if (m_receiveBuffer.size() > MAX_BUFFER_SIZE) {
        qWarning() << "Buffer overflow detected, clearing buffer";
        m_receiveBuffer.clear();
        return;
    }

    while (true)
    {
        // 查找包头 AA 55
        int headerIndex = -1;
        for (int i = 0; i < m_receiveBuffer.size() - 1; ++i) {
            if (static_cast<quint8>(m_receiveBuffer[i]) == PACKET_HEADER_1 &&
                static_cast<quint8>(m_receiveBuffer[i + 1]) == PACKET_HEADER_2) {
                headerIndex = i;
                break;
            }
        }

        if (headerIndex < 0) {
            // 没找到包头，保留最后一个字节（可能是下一个包头的第一个字节）
            if (m_receiveBuffer.size() > 1) {
                m_receiveBuffer.remove(0, m_receiveBuffer.size() - 1);
            }
            break;
        }

        if (headerIndex > 0) {
            // 丢弃包头之前的无效数据
            qWarning() << "Discarding" << headerIndex << "bytes before header";
            m_receiveBuffer.remove(0, headerIndex);
        }

        // 检查是否有足够的数据读取长度字段
        if (m_receiveBuffer.size() < HEADER_LENGTH) {
            break; // 等待更多数据
        }

        quint8 lenHigh = static_cast<quint8>(m_receiveBuffer[2]);
        quint8 lenLow = static_cast<quint8>(m_receiveBuffer[3]);
        int packetLength = (lenHigh << 8) | lenLow;

        // 长度字段校验
        if (packetLength <= 0 || packetLength > MAX_PACKET_SIZE) {
            qWarning() << "Invalid packet length:" << packetLength << ", discarding one byte and resync";
            m_receiveBuffer.remove(0, 1);
            continue;
        }

        int fullPacketLength = HEADER_LENGTH + packetLength;

        if (m_receiveBuffer.size() < fullPacketLength) {
            break; // 数据包未接收完整，等待
        }

        QByteArray fullPacket = m_receiveBuffer.left(fullPacketLength);

        // 没有CRC，直接尝试解析
        if (!parseForceDataPacket(fullPacket)) {
            // 解析失败，丢弃当前包头，继续同步
            m_receiveBuffer.remove(0, 1);
            continue;
        }

        // 解析成功，丢弃已处理数据
        m_receiveBuffer.remove(0, fullPacketLength);
    }
}

bool ForceSerialPortManager::parseForceDataPacket(const QByteArray &packet)
{
    const int HEADER_LEN = 4;

    if (packet.size() < HEADER_LEN) {
        emit errorOccurred(QString("Packet too short for header: %1 bytes").arg(packet.size()));
        return false;
    }

    quint8 lenHigh = static_cast<quint8>(packet[2]);
    quint8 lenLow = static_cast<quint8>(packet[3]);
    int dataLen = (lenHigh << 8) | lenLow;

    if (packet.size() != HEADER_LEN + dataLen) {
        emit errorOccurred(QString("Packet size mismatch: expected %1 bytes, got %2 bytes")
                               .arg(HEADER_LEN + dataLen).arg(packet.size()));
        return false;
    }

    if (dataLen < 2) {  // 至少要有包编号
        emit errorOccurred(QString("Data length too short: %1").arg(dataLen));
        return false;
    }

    // 包编号
    quint8 seqHigh = static_cast<quint8>(packet[4]);
    quint8 seqLow = static_cast<quint8>(packet[5]);
    quint16 sequenceNumber = (seqHigh << 8) | seqLow;

    // 通道数据长度 = dataLen - 2(包编号)
    int channelDataLen = dataLen - 2;
    if (channelDataLen % 4 != 1) {
        emit errorOccurred(QString("Channel data length %1 is not multiple of 4").arg(channelDataLen));
        return false;
    }

    int channelCount = channelDataLen / 4;

    const int MAX_CHANNELS = 32;
    if (channelCount > MAX_CHANNELS) {
        emit errorOccurred(QString("Too many channels: %1").arg(channelCount));
        return false;
    }

    QVector<double> forceData;
    forceData.reserve(channelCount);

    int dataStartIndex = 6;

    for (int i = 0; i < channelCount; ++i) {
        int offset = dataStartIndex + i * 4;
        if (offset + 4 > packet.size()) {
            emit errorOccurred(QString("Channel %1 data incomplete").arg(i));
            return false;
        }

        QByteArray channelBytes = packet.mid(offset, 4);
        float value = bytesToFloat(channelBytes);
        forceData.append(value);
    }

    m_stationValues = forceData;

    return true;
}

// ==================== 工具函数 ====================

QByteArray ForceSerialPortManager::commandToAscii(const QString &cmd)
{
    // 自动添加 \r\n 结尾
    QString fullCmd = cmd + "\r\n";
    return fullCmd.toLatin1();
}

float ForceSerialPortManager::bytesToFloat(const QByteArray &data)
{
    if (data.size() != 4) {
        qWarning() << "Invalid data size for float conversion:" << data.size();
        return std::numeric_limits<float>::quiet_NaN();
    }

    // 根据你的示例: 016AF4C0 需要转换成 C0F46A01
    // 即: 字节序需要完全反转 (小端序转大端序)
    // QByteArray reordered;
    // reordered.append(data[3]);
    // reordered.append(data[2]);
    // reordered.append(data[1]);
    // reordered.append(data[0]);

    // 使用 memcpy 转换为 float (IEEE 754)
    float value = 0.0f;
    std::memcpy(&value, data, sizeof(float));

    return value;
}

void ForceSerialPortManager::sendPollingRequest(quint8 stationAddress)
{
    // 六维力传感器不使用 Modbus 轮询，这里可以留空或发送单次请求
    Q_UNUSED(stationAddress);
    // 如果需要定时请求，可以调用:
    // sendSingleDataRequest();
}
