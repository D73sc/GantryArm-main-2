#include "mainwindow.h"
#include <QApplication>
#include "loghandler.h"
// 在项目的主头文件中（例如在 main.cpp 或公共头文件）
#include <QMetaType>
#include <Eigen/Dense>

// 注册 Eigen::Matrix4d 为 Qt 元类型
Q_DECLARE_METATYPE(Eigen::Matrix4d)


int main(int argc, char *argv[])
{
    // 在 main 函数开始处或应用程序初始化时添加
    qRegisterMetaType<Eigen::Matrix4d>("Eigen::Matrix4d");
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    LogHandler::Get().installMessageHandler();

    return a.exec();
}
