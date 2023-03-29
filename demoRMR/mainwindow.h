#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#ifdef _WIN32
#include<windows.h>
#endif
#include<iostream>
//#include<arpa/inet.h>
//#include<unistd.h>
//#include<sys/socket.h>
#include<sys/types.h>
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include<vector>
//#include "ckobuki.h"
//#include "rplidar.h"

#include "robot.h"
#include "p_controller_rotation.h"
#include "p_controller_movement.h"


namespace Ui {
class MainWindow;
}

///toto je trieda s oknom.. ktora sa spusti ked sa spusti aplikacia.. su tu vsetky gombiky a spustania...
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    bool useCamera1;
  //  cv::VideoCapture cap;

    int actIndex;
    //    cv::Mat frame[3];


    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    int processThisLidar(LaserMeasurement laserData);

    int processThisRobot(TKobukiData robotdata);



private slots:
    void on_pushButton_9_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_6_clicked();

    void on_pushButton_5_clicked();

    void on_pushButton_4_clicked();

    void on_pushButton_clicked();
    void getNewFrame();

    void movementForward(int speedForward);
    void movementToRight();
    void movementToLeft();
    void movementBackwards();
    void robotSlowdown();
    void stopRobot();

    double calculateShortestRotation(double correctRotation);

    void robotMovement(TKobukiData robotdata);
    double getRightOrientation();
    double getDistanceToEnd();
    void calculateXY(TKobukiData robotdata);
    void initData(TKobukiData robotdata);

private:

    //--skuste tu nic nevymazat... pridavajte co chcete, ale pri odoberani by sa mohol stat nejaky drobny problem, co bude vyhadzovat chyby
    Ui::MainWindow *ui;
     void paintEvent(QPaintEvent *event);// Q_DECL_OVERRIDE;
     int updateLaserPicture;
     LaserMeasurement copyOfLaserData;
     std::string ipaddress;
     Robot robot;
     TKobukiData robotdata;
     int datacounter;
     QTimer *timer;



     double forwardspeed;//mm/s
     double rotationspeed;//omega/s

     double distanceLW;
     double distanceRW;
     short delta_leftWheel;
     short delta_rightWheel;
     unsigned short encLeftWheel;
     unsigned short encRightWheel;
     double x;
     double x_destination;
     double y;
     double y_destination;
     double d;
     double alfa;
     double tickToMeter;
     int speed;
     bool init;
     bool isCorrectRotation;
     bool isStop;
     bool isRobotMove;
     bool isRobotRotate;
     bool isConvertAngleRight;
     bool isConvertAngleLeft;

     double rightRotationAngle;
     double leftRotationAngle;

     double deadbandRotation;
     int xArray[5];
     int yArray[5];

     int pointReached;

     double distance;

     double gyroStart;
     double gyro;
     double gyroRad;

    PControllerRotation controllerRotation;
    PControllerMovement controllerMove;


public slots:
     void setUiValues(double robotX,double robotY,double robotFi);
signals:
     void uiValuesChanged(double newrobotX,double newrobotY,double newrobotFi); ///toto nema telo


};

#endif // MAINWINDOW_H
