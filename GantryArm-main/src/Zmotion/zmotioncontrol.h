#ifndef ZMOTIONCONTROL_H
#define ZMOTIONCONTROL_H
#pragma execution_character_set("utf-8")
#include "zmotion.h"
#include "zmcaux.h"
#include "QDebug"
#include <QThread>
#include <QObject>
#include <QTimer>
#include <vector>
#include <array>

enum AxisMType : int
{
    Idle = 0,     //没有运动
    Move = 1,     //单轴直线或者插补直线运动
    MoveAbs = 2,  //绝对值单轴直线或者绝对插补直线运动
    MheliCal = 3, //圆心螺旋运动
    MoveCirc =4,  // 圆弧插补
    MoveModify = 5,// 修改运动位置
    MoveSp =6,      // SP速度的单轴直线或者SP速度的插补直线运动
    MoveAbsSp =7,   //SP速度的绝对值单轴直线/SP速度的绝对插补直线运动
    MoveCircSp =8,  //SP速度的圆弧插补
    MheliCalSp =9,  //SP速度的圆心螺旋运动
    ForwardVmove =10,//正向持续运动
    ReverseVmove =11, //负向持续运动
    DatuMing =12,    //回零运动中
    Cam =13,         //凸轮表运动
    FWD_Jog =14,     //映射正向JOG运动
    REV_Jog =15,     //映射负向JOG运动
    Cam_Box =20,     //跟随凸轮表运动
    Connect =21,     //同步运动
    MoveLink =22,     //自动凸轮运动
    ConnPath =23,      //同步运动2，矢量类型的
    MoveSLink =25,     //自动凸轮运动2
    MoveSpiRal =26,     //渐开线圆弧
    Meclpse =27,      //椭圆运动
    Move_Aout =28,    //缓冲IO/缓冲寄存器操作等
    Move_Delay =29, //  缓冲延时
    MspheriCal =31,  //空间圆弧
    Move_PT = 32,   //单位时间内运动距离
    ConnFRame =33,   // 机械手逆解运动
    ConnRefRame = 34,  //机械手正解运动
    EactCycle=65 //ECAT周期位置模式，需支持EtherCAT
};
enum AxisSatus : int
{
    /// <summary>
    /// 轴的状态正常
    /// </summary>
    NoError = 0,
    /// <summary>
    /// 轴正限位报警
    /// </summary>
    FlimitError = 1,
    /// <summary>
    /// 轴负限位报警
    /// </summary>
    BlimitError = 2,
    /// <summary>
    /// 轴伺服报警
    /// </summary>
    ServoError = 3
};
enum MoveMode : int
{
    /// <summary>
    /// 正向运行
    /// </summary>
    Forward = 1,
    /// <summary>
    /// 反向运动
    /// </summary>
    Backword = -1,
    /// <summary>
    /// 停止或取消运动
    /// </summary>
    Cancel = 2
};
class AxisStatus
{
public:
    AxisStatus();
    AxisStatus(int num_,float posi_=0, float init_=0, float fslimit_=0, float rslimit_=0,AxisSatus status_=NoError,AxisMType mtype_=Idle);
    int num;
    float posi;
    float init;
    float fslimit;
    float rslimit;
    AxisSatus status;
    AxisMType mtype;

};
class IOStatus
{
public:

    uint32 IOInputEmergencyStop=0;
    uint32 IOOutputBrake[5];
};

class ZmotionStatus
{
public:

    AxisStatus *allAxisStatus;
    IOStatus* allIOStatus;
};

class ZMotionControl:public QObject
{
    Q_OBJECT
private:
    ZMC_HANDLE g_handle;
    QTimer *SecurityCheck;
    ZMotionControl();
    void DataFetcher(std::function<void(ZmotionStatus*)> callback);//回调函数
    static ZMotionControl* instance;
    static std::mutex mtx;
    std::atomic<bool> running;
    std::thread workerThread;
    void onSecuityChecked();

public:
    enum{AxisNum=8};//轴数量
    int AxisList[AxisNum]={5,6,7,4,3,0,2,1};//轴映射
    int IOInputEmergencyStop=0;
    int IOOutputBrake[6]={0,2,4,6,8,10};
    ///
    /// \brief instance单例模式
    /// \return
    ///
    static ZMotionControl* getinstance() {
        if (instance == nullptr) {
            std::lock_guard<std::mutex> lock(mtx);
            if (instance == nullptr) {
                instance = new ZMotionControl();
            }
        }
        return instance;
    }
    // 禁用拷贝构造和赋值操作符，确保实例唯一
    ZMotionControl(const ZMotionControl&) = delete;
    ZMotionControl& operator=(const ZMotionControl&) = delete;
    ~ZMotionControl();
    // 清理单例实例
    static void cleanup() {
        delete instance;
    }

    // 启动回调线程
    void startCallbackThread(std::function<void(ZmotionStatus*)> callback);

    // 停止回调线程
    void stopCallbackThread();


    ZmotionStatus *myZmotionStatus;
    //AxisStatus *allAxisStatus;
    bool IsAllAxisIdle();
    bool IsSingleIdle(int i);
    int EcatInit(int SlotId,int LocalAxisNum,int BusAxisStartId,int ProFileMode);
    int WaitContrFunctComple(const char *cmdbuff, int TimeOut);
    int ConnectSimulation();
    int Connect(int type,const char str[20]);
    void Disconnect();
    bool GetConnectStatus();
    int* SetAxisEnable();
    int* SetAxisDisable();
    void SetAxisUnits(int i,float units);
    void SetAxisSpeed(int i,float speed);
    void SetAxisAccel(int i,float accel);
    void SetAxisDecel(int i,float decel);
    void SetAxisFSLimit(int i,float fslimit);
    void SetAxisRSLimit(int i,float rslimit);
    void SetAxisSramp(int i, float sramp);
    float GetAxisUnits(int i);
    float GetAxisSpeed(int i);
    float GetAxisAccel(int i);
    float GetAxisDecel(int i);
    float GetAxisFSLimit(int i);
    float GetAxisRSLimit(int i);
    float GetAxisDpos(int i);
    int EmergencyStop();
    AxisSatus GetAxisStatus(int i);
    void MoveZero(int i);
    void MoveInit(int i);
    void MoveSingleAbs(int i,float position);
    void SingleVMove(int i,MoveMode movemod);
    void SetAxisType(int type);
    void EnableMergeChange();//打开连续插补
    void DisenableMergeChange();//打开连续插补

    AxisMType GetAxisMType(int n);
    void MoveAbsTrajectoryThetas(std::array<double, ZMotionControl::AxisNum>  thetas);
    void MoveAbsTrajectoryThetas(std::vector<std::array<double, ZMotionControl::AxisNum> > thetas);
    /// \brief 获取指定 I/O 编号的输入值
    /// \param IONum I/O 编号
    /// \param value 输入值（返回参数）
    void GetIOInput(int IONum, uint32& value);
    /// \brief 设置指定 I/O 编号的输出值
    /// \param IONum I/O 编号
    /// \param value 输出值
    void SetIOOutput(int IONum,const uint32 value);
    /// \brief 获取指定 I/O 编号的输出值
    /// \param IONum I/O 编号
    /// \param value 输出值
    void GetIOOutput(int IONum, uint32 &value);
    ///
    /// \brief JudgeEmergencyStopIO 判断急停开关是否按下
    /// \return 常闭开关，未按下为1，按下为0
    ///
    bool IsPressedEmergencyStopIO();
    ///
    /// \brief SetBrakeEnable打开刹车IO，让刹车上电
    ///
    void SetBrakeEnable();
    ///
    /// \brief SetBrakeDisenable关闭刹车IO，刹车下电
    ///
    void SetBrakeDisenable();
    void SetAllAxisSramp(float sramp);
    //设置拐角模式
    void SetCornerMode();
    void ResetCornerMode();
    void SetDecelAngle(float angle);
    void SetStopAngle(float angle);
    void SetForceSpeed(float speed);

    float GetAxisFinalPositionPhysical(int i);
    std::array<double, AxisNum> GetAllAxesFinalPositionsPhysical();
signals:
    void dataFetched(AxisStatus* status); // 定义信号

};


#endif // ZMOTIONCONTROL_H
