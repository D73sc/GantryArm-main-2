#include "zmotioncontrol.h"
;
ZMotionControl* ZMotionControl::instance = nullptr;
std::mutex ZMotionControl::mtx;
ZMotionControl::ZMotionControl()
{

    running=false;
    g_handle=NULL;
    myZmotionStatus=new ZmotionStatus;
    myZmotionStatus->allAxisStatus=new AxisStatus[AxisNum];
    for(int i=0;i<AxisNum;i++)
    {
        myZmotionStatus->allAxisStatus[i]=AxisStatus(AxisList[i]);
    }
    myZmotionStatus->allIOStatus = new IOStatus;

}



ZMotionControl::~ZMotionControl() 
{
    // 1. 停止所有活动操作
    stopCallbackThread();

    // 2. 安全停止定时器

    // 3. 释放硬件资源
     Disconnect();
    


    // 4. 释放动态内存
    //delete myZmotionStatus;

    //cleanup();
    // 不需要设置为nullptr，因为对象即将销毁
    //SecurityCheck->stop();

    // 5. 清理单例引用
    //if (instance == this) {
    //    instance = nullptr;
    //}
}

void ZMotionControl::startCallbackThread(std::function<void (ZmotionStatus*)> callback)
{
    if (!running) {
        running = true;
        workerThread = std::thread(&ZMotionControl::DataFetcher, this, callback);
    }
}

void ZMotionControl::stopCallbackThread() {
    running = false;

    if (workerThread.joinable()) {
        workerThread.join();
    }
}

bool ZMotionControl::IsAllAxisIdle()
{
    if(!GetConnectStatus())
        return false;
    for(int i=0;i<AxisNum;i++)
    {
        // if(allAxisStatus[i].mtype!=0)
        if(!IsSingleIdle(i))
            return false;
    }
    return true;

}

bool ZMotionControl::IsSingleIdle(int i)
{
    int Idle=0;
    ZAux_Direct_GetIfIdle(g_handle,AxisList[i],&Idle);
    if(Idle!=-1)
        return false;
    return true;

}

void ZMotionControl::DataFetcher(std::function<void(ZmotionStatus*)> callback)
{
    while(running)
    {

        std::this_thread::sleep_for(std::chrono::milliseconds(50));  // 每隔 50ms 执行一次
        if(!GetConnectStatus())
            continue;
        for(int i=0;i<AxisNum;i++)
        {
            myZmotionStatus->allAxisStatus[i].posi=GetAxisDpos(i);
            myZmotionStatus->allAxisStatus[i].fslimit=GetAxisFSLimit(i);
            myZmotionStatus->allAxisStatus[i].rslimit=GetAxisRSLimit(i);
            myZmotionStatus->allAxisStatus[i].status=GetAxisStatus(i);
            myZmotionStatus->allAxisStatus[i].mtype=GetAxisMType(i);
        }
        GetIOInput(IOInputEmergencyStop,myZmotionStatus->allIOStatus->IOInputEmergencyStop);


       //qDebug()<<myZmotionStatus->allIOStatus->IOInputEmergencyStop;
        callback(myZmotionStatus);
        //emit dataFetched(allAxisStatus); // 发出数据获取信号
    }
}

void ZMotionControl::onSecuityChecked()
{
    if(!GetConnectStatus())
        return;
    if(IsPressedEmergencyStopIO())
    {
        EmergencyStop();
    }
}

/**
*@brief Qt总线初始化
*@param[in] 槽位号,本地轴数目,总线轴起始轴号，PDO模式
*@return 0：执行正常、    非0：执行异常
*@ingroup
*@see
*@note
*/
int ZMotionControl::EcatInit(int SlotId,int LocalAxisNum,int BusAxisStartId,int ProFileMode)
{
    char CmdBuff[64]={0},ReceBuff[256];
    float VirtualAxises=0,RealAxises=0; //虚拟轴和实轴数目
    int EcatInitStatus=-1,Err=0;
    int TimeOut=3000,TimeOutFlag=3000;
    float NodeNum=0;
    //延迟5秒，等待驱动器上电，不同驱动器自身上电时间不同，具体根据驱动器调整延时
    QThread::sleep(3);
    //获取总线通讯周期
    Err = ZAux_Execute(g_handle,"?SERVO_PERIOD", ReceBuff,256 );
    qDebug()<<"总线通讯周期(us)："<<ReceBuff;
    //初始化还原轴类型
    Err = ZAux_Direct_Rapidstop(g_handle,2);
    Err = ZAux_Execute(g_handle,"Table(0)=SYS_ZFEATURE(0)", ReceBuff,256 );
    Err = ZAux_Direct_GetTable(g_handle, 0, 1, &VirtualAxises);
    if(VirtualAxises>=64)
        VirtualAxises=63;
    for(int i=0;i<VirtualAxises;i++)
    {
        Err = ZAux_Direct_SetAxisAddress(g_handle,i,0);
        Err = ZAux_Direct_SetAxisEnable(g_handle,i,0);
        Err = ZAux_Direct_SetAtype(g_handle,i,0);
        //等待轴停止
        int Idle=0;
        TimeOutFlag =TimeOut;
        while(TimeOutFlag>0)
        {
            Err = ZAux_Direct_GetIfIdle(g_handle,i,&Idle);
            if(Idle)
            {
                break;
            }
            QThread::msleep(10);
            TimeOutFlag = TimeOutFlag-10;
        }
    }
    //本地轴重新映射
    for(int i=0;i<LocalAxisNum;i++)
    {
        Err = ZAux_Direct_SetAxisAddress(g_handle,i,(-1<<16)+i);
        Err = ZAux_Direct_SetAtype(g_handle,i,1);
    }
    //停止总线
    sprintf(CmdBuff,"SLOT_STOP(%d)",SlotId);
    Err = ZAux_Execute(g_handle,CmdBuff, ReceBuff,256 );
    QThread::msleep(200);
    //总线时钟优化
    Err = ZAux_Execute(g_handle,"SYSTEM_ZSET = SYSTEM_ZSET OR 128", ReceBuff,256 );
    //扫描总线
    sprintf(CmdBuff,"SLOT_SCAN(%d) ?return",SlotId);
    Err = ZAux_Execute(g_handle,CmdBuff, ReceBuff,256 );
    QThread::sleep(3);
    ReceBuff[2]=0;
    //等待总线扫描完成
    if(0 == strcmp("-1",ReceBuff))
    {

        TimeOutFlag = TimeOut;
        while(TimeOutFlag>0)
        {
            sprintf(CmdBuff,"Table(0)=NODE_COUNT(%d)",SlotId);
            Err = ZAux_Execute(g_handle,CmdBuff, ReceBuff,256 );
            Err = ZAux_Direct_GetTable(g_handle, 0, 1, &NodeNum);
            if(NodeNum>0)
            {
                break;
            }
            QThread::msleep(10);
            TimeOutFlag = TimeOutFlag-10;
        }
        qDebug()<<"总线扫描成功，连接设备数："<<NodeNum;
        //初始化节点
        if(NodeNum>=0)
        {
            int BusAxis_Num=0; //总线轴总数，从0开始计数
            for(int i=0; i<NodeNum;i++)
            {
                //读取节点轴数
                float NodeAxisNum=0;
                sprintf(CmdBuff,"Table(0)=NODE_AXIS_COUNT(%d,%d)",SlotId,i);
                Err = ZAux_Execute(g_handle,CmdBuff, ReceBuff,256 );
                Err = ZAux_Direct_GetTable(g_handle, 0, 1, &NodeAxisNum);
                for(int j=0;j<NodeAxisNum;j++)
                {
                    Err = ZAux_Direct_SetAxisAddress(g_handle,BusAxisStartId+BusAxis_Num,BusAxis_Num+1);
                    Err = ZAux_Direct_SetAtype(g_handle,BusAxisStartId+BusAxis_Num,65);
                    sprintf(CmdBuff,"DRIVE_PROFILE(%d) = %d",BusAxisStartId+BusAxis_Num,ProFileMode);
                    Err = ZAux_Execute(g_handle,CmdBuff, ReceBuff,256 );
                    sprintf(CmdBuff,"DISABLE_GROUP(%d)",BusAxisStartId+BusAxis_Num);
                    Err = ZAux_Execute(g_handle,CmdBuff, ReceBuff,256 );
                    BusAxis_Num = BusAxis_Num+1;
                }
            }
            EcatInitStatus = 0;//总线扫描完成
            qDebug()<<"轴扫描映射完成，连接总线轴数："<<BusAxis_Num;
            //开启总线
            QThread::msleep(100);
            sprintf(CmdBuff,"SLOT_START(%d) ?return",SlotId);
            Err = ZAux_Execute(g_handle,CmdBuff, ReceBuff,256 );
            ReceBuff[2]=0;
            if(0 == strcmp("-1",ReceBuff))
            {
                qDebug()<<"总线开启正常";
                QThread::sleep(3);//延迟3秒，等待驱动器时钟同步
                int AxisList[1];
                for(int i=BusAxisStartId;i<BusAxisStartId+BusAxis_Num;i++)
                {
                    AxisList[0]=i;
                    Err = ZAux_Direct_Base(g_handle, 1,AxisList);
                    Err = ZAux_Execute(g_handle,"DRIVE_CLEAR(0)", ReceBuff,256 );
                    QThread::msleep(10);
                    sprintf(CmdBuff,"DRIVE_CONTROLWORD(%d) = 128 ",i);
                    Err = ZAux_Execute(g_handle,CmdBuff, ReceBuff,256 );
                    QThread::msleep(10);
                    sprintf(CmdBuff,"DRIVE_CONTROLWORD(%d)=6 ",i);
                    Err = ZAux_Execute(g_handle,CmdBuff, ReceBuff,256 );
                    QThread::msleep(10);
                    sprintf(CmdBuff,"DRIVE_CONTROLWORD(%d)=15 ",i);
                    Err = ZAux_Execute(g_handle,CmdBuff, ReceBuff,256 );
                    QThread::msleep(10);
                }
                QThread::msleep(100);
                Err = ZAux_Execute(g_handle,"DATUM(0)", ReceBuff,256 );
                qDebug()<<"控制器报警清除完成";
                QThread::msleep(1000);
                Err = ZAux_Execute(g_handle,"WDOG = 1", ReceBuff,256 );
                for(int i=BusAxisStartId;i<BusAxisStartId+BusAxis_Num;i++)
                {
                    ZAux_Direct_SetAxisEnable(g_handle,i,1);
                    QThread::msleep(10);
                }
                qDebug()<<"伺服使能完成";
                EcatInitStatus = 1;//总线初始化完成
            }
            else
            {
                EcatInitStatus=-1;
                qDebug()<<"总线开启异常";
            }
        }
    }
    return EcatInitStatus;
}
/**
*@brief 等待控制器函数执行完成
*@param[in] 函数执行完成的标志位变量名字符串（该变量在Basic定义）
*@return 0：执行正常、    非0：执行异常
*@ingroup
*@see
*@note
*/
int ZMotionControl::WaitContrFunctComple(const char *cmdbuff, int TimeOut)
{
    int FunctCompleFlag=0;  //函数执行标志 1：执行完成 0：执行未完成
    int Count=0;            //计时器
    int Err=0;              //Zmotion函数返回值
    //等待控制器函数执行完成
    while(0 == FunctCompleFlag)
    {
        Err = ZAux_Direct_GetVariableInt(g_handle,cmdbuff,&FunctCompleFlag);
        QThread::msleep(10);
        Count++;
        if((TimeOut/10) < Count)
        {
            //等待时间大于5s退出循环
            Err=1;
            break;
        }
    }
    //如果FunctCompleFlag 不等于1不是Basic那边函数执行有错误
    if(FunctCompleFlag != 1 )
    {
        Err = FunctCompleFlag;
    }
    return Err;
}
///
/// \brief ZMotionControl::ConnectSimulation
/// 连接仿真器
/// \return 0为成功1为失败
///
int ZMotionControl::ConnectSimulation()
{
    int rint =ZAux_OpenEth((char*)"127.0.0.1", &g_handle);
    qDebug()<<g_handle;
    return rint;
}
///
/// \brief ZMotionControl::Connect
/// \param type
///    ZMC_CONNECTION_COM = 1,
///    ZMC_CONNECTION_ETH = 2,
///    ZMC_CONNECTION_USB = 3,
///    ZMC_CONNECTION_PCI = 4,
///    ZMC_CONNECTION_LOCAL = 5,
/// \param str
/// 连接字符串
/// \return 0为成功1为失败
///
int ZMotionControl::Connect(int type,const char  str[20])
{
    ZMC_CONNECTION_TYPE zmtype;
    switch(type)
    {
    case 1:
        zmtype=ZMC_CONNECTION_COM;
        break;
    case 2:
        zmtype=ZMC_CONNECTION_ETH;
        break;
    case 3:
        zmtype=ZMC_CONNECTION_USB;
        break;
    case 4:
        zmtype=ZMC_CONNECTION_PCI;
        break;
    case 5:
        zmtype=ZMC_CONNECTION_LOCAL;
        break;

    }
    char str1[20];
    strcpy(str1,str);


    int rint = ZMC_Open(zmtype,str1,&g_handle);
    return rint;
}

void ZMotionControl::Disconnect()
{

    g_handle = NULL;

}
///检测是否连接控制器
///true为成功false为失败
bool ZMotionControl::GetConnectStatus()
{
    if (g_handle != NULL)
    {
        return true;
    }
    return false;
}
///
/// \brief ZMotionControl::SetAxisEnable
/// 使能所有轴
/// \return 错误码
///
int* ZMotionControl::SetAxisEnable()
{
    int* err = new int[AxisNum];
    for(int i=0;i<AxisNum;i++)
        err[i]=ZAux_Direct_SetAxisEnable(g_handle,AxisList[i],1);
    return err;
}
///
/// \brief ZMotionControl::SetAxisDisable
/// 取消使能所有轴
/// \return 错误码
///
int* ZMotionControl::SetAxisDisable()
{
    int* err = new int[AxisNum];
    for(int i=0;i<AxisNum;i++)
        err[i]=ZAux_Direct_SetAxisEnable(g_handle,AxisList[i],0);
    return err;

}

void ZMotionControl::SetAxisUnits(int i,float units)
{
    ZAux_Direct_SetUnits(g_handle,AxisList[i],units);
}

void ZMotionControl::SetAxisSpeed(int i, float speed)
{
    ZAux_Direct_SetSpeed(g_handle,AxisList[i],speed);

}

void ZMotionControl::SetAxisAccel(int i, float accel)
{
    ZAux_Direct_SetAccel(g_handle,AxisList[i],accel);

}

void ZMotionControl::SetAxisDecel(int i, float decel)
{
    ZAux_Direct_SetDecel(g_handle,AxisList[i],decel);

}

void ZMotionControl::SetAxisFSLimit(int i, float fslimit)
{
    ZAux_Direct_SetFsLimit(g_handle,AxisList[i],fslimit);

}

void ZMotionControl::SetAxisRSLimit(int i, float rslimit)
{
    ZAux_Direct_SetRsLimit(g_handle,AxisList[i],rslimit);

}

void ZMotionControl::SetAxisSramp(int i, float sramp)
{
    ZAux_Direct_SetSramp(g_handle, AxisList[i], sramp);
}

float ZMotionControl::GetAxisUnits(int i)
{
    float a;
    ZAux_Direct_GetUnits(g_handle,AxisList[i],&a);
    return a;

}

float ZMotionControl::GetAxisSpeed(int i)
{
    float a;
    ZAux_Direct_GetSpeed(g_handle,AxisList[i],&a);
    return a;
}

float ZMotionControl::GetAxisAccel(int i)
{
    float a;
    ZAux_Direct_GetAccel(g_handle,AxisList[i],&a);
    return a;
}

float ZMotionControl::GetAxisDecel(int i)
{
    float a;
    ZAux_Direct_GetDecel(g_handle,AxisList[i],&a);
    return a;
}

float ZMotionControl::GetAxisFSLimit(int i)
{
    float a;
    ZAux_Direct_GetFsLimit(g_handle,AxisList[i],&a);
    return a;
}

float ZMotionControl::GetAxisRSLimit(int i)
{
    float a;
    ZAux_Direct_GetRsLimit(g_handle,AxisList[i],&a);
    return a;
}

float ZMotionControl::GetAxisDpos(int i)
{
    float a;
    ZAux_Direct_GetDpos(g_handle,AxisList[i],&a);
    return a;
}

float ZMotionControl::GetAxisFinalPositionPhysical(int i) {
    float finalPosUnits = 0.0f;
    if (ZAux_Direct_GetEndMove(g_handle, AxisList[i], &finalPosUnits) != 0) {
        return 0.0f;
    }
    // 如果unit表示"1物理单位=unit个控制器单位"，则用除法
    return finalPosUnits ;
}

int ZMotionControl::EmergencyStop()
{
    if (0 == ZAux_Direct_Rapidstop(g_handle, 2))
    {
        return true;
    }

    return false;
}

AxisSatus ZMotionControl::GetAxisStatus(int i)
{
    int axisstate=0;
    ZAux_Direct_GetAxisStatus(g_handle, AxisList[i], &axisstate);
    //return static_cast<AxisSatus>(axisstate);
        if (axisstate == 0)
        {
            //return NoError;
            return NoError;
        }
        else if (((axisstate >> 9) & 1) == 1)
        {
            return FlimitError;
        }
        else if (((axisstate >> 10) & 1) == 1)
        {
            return BlimitError;
        }
        else if (((axisstate >> 3) & 1) == 1)
        {
            return ServoError;
        }

}

void ZMotionControl::MoveZero(int i)
{
    float poslist[1]{0};
    int axislist[1]{AxisList[i]};

    ZAux_Direct_MoveAbs(g_handle,1,axislist,poslist);
}

void ZMotionControl::MoveInit(int i)
{
    float poslist[1]{myZmotionStatus->allAxisStatus[i].init};
    int axislist[1]{AxisList[i]};

    ZAux_Direct_MoveAbs(g_handle,1,axislist,poslist);

}

void ZMotionControl::MoveSingleAbs(int i, float position)
{
    int axislist[1]{AxisList[i]};
    float poslist[1]{position};
    ZAux_Direct_MoveAbs(g_handle,1,axislist,poslist);

}

void ZMotionControl::SingleVMove(int i, MoveMode movemod)
{
    if(movemod==Cancel)
    {

        ZAux_Direct_Single_Cancel(g_handle, AxisList[i], 2);
    }
    else
        ZAux_Direct_Single_Vmove(g_handle, AxisList[i], (int)movemod);
}

void ZMotionControl::SetAxisType(int type)
{
    for(int i=0;i<AxisNum;i++)
    {
        ZAux_Direct_SetAtype(g_handle,AxisList[i],type);
    }
}

void ZMotionControl::EnableMergeChange()
{
    for(int i=0;i<AxisNum;i++)
    {
        ZAux_Direct_SetMerge(g_handle,AxisList[i],1);
    }
}

void ZMotionControl::DisenableMergeChange()
{
    for(int i=0;i<AxisNum;i++)
    {
        ZAux_Direct_SetMerge(g_handle,AxisList[i],0);
    }
}

AxisMType ZMotionControl::GetAxisMType(int n)
{
    int mt;
    ZAux_Direct_GetMtype(g_handle,AxisList[n],&mt);
    AxisMType amt=(AxisMType)mt;
    return amt;
}

void ZMotionControl::MoveAbsTrajectoryThetas(std::vector<std::array<double, ZMotionControl::AxisNum> > thetas)
{

    float poslist[8];
    int axislist[8] = {
        AxisList[0], AxisList[1], AxisList[2], AxisList[3],
        AxisList[4], AxisList[5], AxisList[6], AxisList[7]
    };

        for (const auto& pos : thetas) {
            // 处理多个元素
            for (int i = 0; i < 8; ++i) {

                if (i < 3)
                    poslist[i] = static_cast<float>(pos[i])/10.0;
                else
                    poslist[i] = static_cast<float>(pos[i] * 180 / 3.1415926);
                if (i == 1)//X轴实际零点为预设的200
                    poslist[i] -= 20;
            }
            ZAux_Direct_MoveAbs(g_handle, 8, axislist, poslist);
        }
}

void ZMotionControl::MoveAbsTrajectoryThetas(std::array<double, ZMotionControl::AxisNum>  thetas)
{

    float poslist[8];
    int axislist[8] = {
        AxisList[0], AxisList[1], AxisList[2], AxisList[3],
        AxisList[4], AxisList[5], AxisList[6], AxisList[7]
    };

        // 处理多个元素
        for (int i = 0; i < 8; ++i) {
            if (i < 3)
                poslist[i] = static_cast<float>(thetas[i])/10.0;
            else
                poslist[i] = static_cast<float>(thetas[i] * 180 / 3.1415926);
            if (i == 1)//X轴实际零点为预设的200
                poslist[i] -= 20;
        }
        ZAux_Direct_MoveAbs(g_handle, 8, axislist, poslist);

}

std::array<double, ZMotionControl::AxisNum> ZMotionControl::GetAllAxesFinalPositionsPhysical() {
    std::array<double, AxisNum> positions{};
    for (int i = 0; i < AxisNum; ++i) {
        positions[i] = static_cast<double>(GetAxisFinalPositionPhysical(i));
    }
    return positions;
}

void ZMotionControl::GetIOInput(int IONum, uint32 &value)
{
    ZAux_Direct_GetIn(g_handle,IONum,&value);
}

void ZMotionControl::SetIOOutput(int IONum, const uint32 value)
{
    ZAux_Direct_SetOp(g_handle,IONum,value);
}

void ZMotionControl::GetIOOutput(int IONum,  uint32 &value)
{
    ZAux_Direct_GetOp(g_handle,IONum,&value);

}

///
/// \brief JudgeEmergencyStopIO 判断急停开关是否按下
/// \return 常闭开关，未按下为1，按下为0
///
bool ZMotionControl::IsPressedEmergencyStopIO()
{
    // uint32 emergencyStopValue=2;
    //GetIOInput(IOInputEmergencyStop,emergencyStopValue);

    if(myZmotionStatus->allIOStatus->IOInputEmergencyStop==1)
        return false;
    else
        return true;
}

void ZMotionControl::SetBrakeEnable()
{
    for(int i=0;i< (sizeof(IOOutputBrake) / sizeof(IOOutputBrake[0]));i++)
        SetIOOutput(IOOutputBrake[i],1);
}

void ZMotionControl::SetBrakeDisenable()
{
    for(int i=0;i<(sizeof(IOOutputBrake) / sizeof(IOOutputBrake[0]));i++)
        SetIOOutput(IOOutputBrake[i],0);
}

void ZMotionControl::SetAllAxisSramp(float sramp)
{
    for (int i = 0; i < AxisNum; i++)
    {
        ZAux_Direct_SetSramp(g_handle, AxisList[i], sramp);
    }
}

void ZMotionControl::SetCornerMode()
{
    ZAux_Direct_SetCornerMode(g_handle, AxisList[0],2);
}

void ZMotionControl::ResetCornerMode()
{
    ZAux_Direct_SetCornerMode(g_handle, AxisList[0],0);

}

void ZMotionControl::SetDecelAngle(float angle)
{
    ZAux_Direct_SetDecelAngle(g_handle, AxisList[0],angle);
}

void ZMotionControl::SetStopAngle(float angle)
{
    ZAux_Direct_SetStopAngle(g_handle, AxisList[0],angle);

}

void ZMotionControl::SetForceSpeed(float speed)
{
    ZAux_Direct_SetForceSpeed(g_handle, AxisList[0],speed);

}

AxisStatus::AxisStatus()
{

}

AxisStatus::AxisStatus(int num_, float posi_, float init_, float fslimit_, float rslimit_, AxisSatus status_, AxisMType mtype_)
{
    num=num_;
    posi=posi_;
    init=init_;
    status=status_;
    mtype=mtype_;
    fslimit=fslimit_;
    rslimit=rslimit_;
}



