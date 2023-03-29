#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <math.h>
#include <iostream>
#include <windows.h>
#include <cmath>
#include "p_controller_rotation.h"
#include "p_controller_movement.h"
#include <fstream>
#include <string>
using namespace std;
///Jozef Kosecky, Peter Dobias


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    controllerRotation(7.5, 0.01),
    controllerMove(7.5, 0.01, 500)
{
    //tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna
//    ipaddress="192.168.1.11"; //192.168.1.11 127.0.0.1
    ipaddress="127.0.0.1";
  //  cap.open("http://192.168.1.11:8000/stream.mjpg");
    ui->setupUi(this);
    datacounter=0;
  //  timer = new QTimer(this);
//    connect(timer, SIGNAL(timeout()), this, SLOT(getNewFrame()));
    actIndex=-1;
    useCamera1=false;

    datacounter=0;

    init = true;
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    ///prekreslujem obrazovku len vtedy, ked viem ze mam nove data. paintevent sa
    /// moze pochopitelne zavolat aj z inych dovodov, napriklad zmena velkosti okna
    painter.setBrush(Qt::black);//cierna farba pozadia(pouziva sa ako fill pre napriklad funkciu drawRect)
    QPen pero;
    pero.setStyle(Qt::SolidLine);//styl pera - plna ciara
    pero.setWidth(3);//hrubka pera -3pixely
    pero.setColor(Qt::green);//farba je zelena
    QRect rect;
    rect= ui->frame->geometry();//ziskate porametre stvorca,do ktoreho chcete kreslit
    rect.translate(0,15);
    painter.drawRect(rect);

    if(useCamera1==true && actIndex>-1)/// ak zobrazujem data z kamery a aspon niektory frame vo vectore je naplneny
    {
        std::cout<<actIndex<<std::endl;

    }
    else
    {
        if(updateLaserPicture==1) ///ak mam nove data z lidaru
        {
            updateLaserPicture=0;

            painter.setPen(pero);
            //teraz tu kreslime random udaje... vykreslite to co treba... t.j. data z lidaru
         //   std::cout<<copyOfLaserData.numberOfScans<<std::endl;
            for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++)
            {
                int dist=copyOfLaserData.Data[k].scanDistance/20; ///vzdialenost nahodne predelena 20 aby to nejako vyzeralo v okne.. zmen podla uvazenia
                int xp=rect.width()-(rect.width()/2+dist*2*sin((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().x(); //prepocet do obrazovky
                int yp=rect.height()-(rect.height()/2+dist*2*cos((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().y();//prepocet do obrazovky
                if(rect.contains(xp,yp))//ak je bod vo vnutri nasho obdlznika tak iba vtedy budem chciet kreslit
                    painter.drawEllipse(QPoint(xp, yp),2,2);
            }

            if(!isRobotRotate){

                updateMap();
            }

        }
    }
}

void MainWindow::updateMap(){
    cout << "zaciatok" << endl;

//    copyOfLaserData.numberOfScans
    for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++)
    {
        if(copyOfLaserData.Data[k].scanQuality != 0){
            continue;
        }
        int grid_size = 10; // Grid size in pixels
        int grid_offset = 60; // Grid offset in pixels

        int dist=copyOfLaserData.Data[k].scanDistance/10;

        if(dist <= 15){
            continue;
        }
        double angleRad = (((360-copyOfLaserData.Data[k].scanAngle)*PI)/180.0);

//        cout << "x robot: " << x << " y robot: " << y << " angleRad: " << angleRad << " gyroRadMap: " << gyroRadMap << endl;

        double xr = x + dist*cos(gyroRadMap + angleRad);
        double yr = y + dist*sin(gyroRadMap + angleRad);

        int x_wall = xr/grid_size + grid_offset;
        int y_wall = yr/grid_size + grid_offset;


        if((y_wall > 0 && y_wall < numberOfSqareInMap) && (x_wall > 0 && x_wall < numberOfSqareInMap)){
             map[y_wall][x_wall] = "*";
        }

    }
}


/// toto je slot. niekde v kode existuje signal, ktory je prepojeny. pouziva sa napriklad (v tomto pripade) ak chcete dostat data z jedneho vlakna (robot) do ineho (ui)
/// prepojenie signal slot je vo funkcii  on_pushButton_9_clicked
void  MainWindow::setUiValues(double robotX,double robotY,double robotFi)
{
     ui->lineEdit_2->setText(QString::number(robotX));
     ui->lineEdit_3->setText(QString::number(robotY));
     ui->lineEdit_4->setText(QString::number(robotFi));
}

///toto je calback na data z robota, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa vzdy ked dojdu nove data z robota. nemusite nic riesit, proste sa to stane
int MainWindow::processThisRobot(TKobukiData robotdata)
{
    if(init){
        initData(robotdata);
    }

    if(starMovement){
        robotMovement(robotdata);
    }

    ///tu mozete robit s datami z robota
    /// ale nic vypoctovo narocne - to iste vlakno ktore cita data z robota
    ///teraz tu posielam rychlosti na zaklade toho co setne joystick a vypisujeme data z robota(kazdy 5ty krat. ale mozete skusit aj castejsie). vyratajte si polohu. a vypiste spravnu
    /// tuto joystick cast mozete vklude vymazat,alebo znasilnit na vas regulator alebo ake mate pohnutky
/*    if(forwardspeed==0 && rotationspeed!=0)
        robot.setRotationSpeed(rotationspeed);
    else if(forwardspeed!=0 && rotationspeed==0)
        robot.setTranslationSpeed(forwardspeed);
    else if((forwardspeed!=0 && rotationspeed!=0))
        robot.setArcSpeed(forwardspeed,forwardspeed/rotationspeed);
    else
        robot.setTranslationSpeed(0);*/

///TU PISTE KOD... TOTO JE TO MIESTO KED NEVIETE KDE ZACAT,TAK JE TO NAOZAJ TU. AK AJ TAK NEVIETE, SPYTAJTE SA CVICIACEHO MA TU NATO STRING KTORY DA DO HLADANIA XXX

    if(datacounter%5)
    {

        ///ak nastavite hodnoty priamo do prvkov okna,ako je to na tychto zakomentovanych riadkoch tak sa moze stat ze vam program padne
                // ui->lineEdit_2->setText(QString::number(robotdata.EncoderRight));
                //ui->lineEdit_3->setText(QString::number(robotdata.EncoderLeft));
                //ui->lineEdit_4->setText(QString::number(robotdata.GyroAngle));
                /// lepsi pristup je nastavit len nejaku premennu, a poslat signal oknu na prekreslenie
                /// okno pocuva vo svojom slote a vasu premennu nastavi tak ako chcete. prikaz emit to presne takto spravi
                /// viac o signal slotoch tu: https://doc.qt.io/qt-5/signalsandslots.html
        ///posielame sem nezmysli.. pohrajte sa nech sem idu zmysluplne veci
        emit uiValuesChanged(x,y,robotdata.GyroAngle/100.0);
        ///toto neodporucam na nejake komplikovane struktury.signal slot robi kopiu dat. radsej vtedy posielajte
        /// prazdny signal a slot bude vykreslovat strukturu (vtedy ju musite mat samozrejme ako member premmennu v mainwindow.ak u niekoho najdem globalnu premennu,tak bude cistit bludisko zubnou kefkou.. kefku dodam)
        /// vtedy ale odporucam pouzit mutex, aby sa vam nestalo ze budete pocas vypisovania prepisovat niekde inde

    }
    datacounter++;

    return 0;

}



void MainWindow::robotSlowdown(){
    speed -= 5;
    movementForward(speed);
    if(speed < 0){
        cout << "zastavujem" << endl;
        stopRobot();
    }
}

void MainWindow::stopRobot(){
    movementForward(0);
    speed = 0;
    controllerMove.UpdateOutputToZero();
    isRobotMove = false;
}

double MainWindow::calculateShortestRotation(double correctRotation){
    //get difference between two angles
    if(gyroRad < correctRotation){
        rightRotationAngle = (2*PI) - correctRotation + gyroRad;
        leftRotationAngle = correctRotation - gyroRad;

        if(isConvertAngleLeft){
            isConvertAngleLeft = false;
        }

        if(rightRotationAngle < leftRotationAngle && !isCorrectRotation){
            isConvertAngleRight = true;
        }
    }
    else{
        leftRotationAngle = (2*PI) - gyroRad + correctRotation;
        rightRotationAngle = gyroRad - correctRotation;

        if(isConvertAngleRight){
            isConvertAngleRight = false;
        }

        if(leftRotationAngle < rightRotationAngle && !isCorrectRotation){
            isConvertAngleLeft = true;
        }
    }

    //We work with 360 degrees, but when we want use shortest path and this path pass through 0 degrees, we must convert circle from <0,360> to <0,180> and <-180,0>
    if(isConvertAngleRight){
        correctRotation -= (2*PI);
        if(gyroRad > PI){
            gyroRad -= (2*PI);
        }
        cout << "prepocet do prava: " << "correctRotation: " << correctRotation << "gyroRad: "<< gyroRad << endl;
    }

    if(isConvertAngleLeft){
        if(gyroRad > PI){
            gyroRad -= (2*PI);
        }
        cout << "prepocet do lava: " << "correctRotation: " << correctRotation << "gyroRad: "<< gyroRad << endl;
    }

    return correctRotation;
}


void MainWindow::robotMovement(TKobukiData robotdata){

    calculateXY(robotdata);
    double correctRotation = getRightOrientation();
    double distanceToEnd = getDistanceToEnd();

    cout << "correctRotation: " << correctRotation << "gyroRad: "<< gyroRad << endl;
    cout << "x_destination: " << x_destination << "y_destination: "<< y_destination << endl;

    correctRotation = calculateShortestRotation(correctRotation);

    // Update the control output based on the measured value
    double output = controllerRotation.Update(correctRotation, gyroRad);

    // Convert the control output to motor speed
    double rotationSpeed = max(-(3.14159/2), min((3.14159/2), output));

    cout << "rotationSpeed: " << rotationSpeed << endl;

    if(rightRotationAngle <= deadbandRotation || leftRotationAngle <= deadbandRotation){
        cout << "zelane otocenie" << endl;
        deadbandRotation = 0.2;
        isCorrectRotation = true;
        isConvertAngleRight = false;
        isConvertAngleLeft = false;
    }
    else{
        cout << "nezelane otocenie" << endl;
        deadbandRotation = 0.02;
        isCorrectRotation = false;
    }

    if(!isStop){
        if(!isCorrectRotation){
            if(isRobotMove){
                cout << "zastavujem pohyb pred rotaciou spomalenie" << endl;
                robotSlowdown();
            }

            if((rotationSpeed >= 0.0) && !isRobotMove){
                cout << "tocim dolava" << endl;
                robot.setRotationSpeed(rotationSpeed);
                isRobotRotate = true;
            }
            else if((rotationSpeed < 0.0) && !isRobotMove){
                cout << "tocim doprava" << endl;
                robot.setRotationSpeed(rotationSpeed);
                isRobotRotate = true;
            }
        }
        else{
            if((x_destination >= x - 2.5) && (x_destination <= x + 2.5) &&
                    (y_destination >= y - 2.5) && (y_destination <= y + 2.5)){
                cout << "zastavujem pohyb" << endl;
                stopRobot();

                pointReached++;
                if(!manualNavigation){
                    if(pointReached < 9){
                        x_destination = xArray[pointReached];
                        y_destination = yArray[pointReached];
                        distance = getDistanceToEnd();
                    }
                    else{
                       isStop = true;
                    }
                }
                else{
                    starMovement = false;
                }

                isRobotRotate = false;
            }
            else{
                isRobotRotate = false;
                double outputMove = controllerMove.Update(0, distanceToEnd);
                cout << "Zrychlujem" << endl;
                cout << "distance: " << distance << "distanceToEnd: "<< distanceToEnd << endl;

                speed = max((50), min((500), std::abs(outputMove)));
                cout << "speed: " << speed << endl;
                movementForward(speed);
                isRobotMove = true;
            }
        }
    }
    cout << "--------------------\n" << endl;
}

double MainWindow::getDistanceToEnd(){
    double distance = std::sqrt(std::pow(x_destination - x, 2) + std::pow(y_destination - y, 2));
    return distance;
}

double MainWindow::getRightOrientation(){
    double deltaX = x_destination - x; // change in x-coordinate
    double deltaY = y_destination - y; // change in y-coordinate
    double angle = atan2(deltaY, deltaX) * 180 / PI; // angle in degrees
    if(angle < 0){
        angle += 360;
    }

    int rounded = std::round(angle);

    if(rounded == 360){
        angle = 0;
    }

    cout << "correctRotation: " << angle << "gyro_angle: "<< gyro << endl;
    return (((angle)*PI)/180.0);
}


void MainWindow::calculateXY(TKobukiData robotdata){
    // pretecenie enkoder
    delta_leftWheel = robotdata.EncoderLeft - encLeftWheel;
    delta_rightWheel = robotdata.EncoderRight - encRightWheel;

    distanceLW = tickToMeter*(delta_leftWheel);
    distanceRW = tickToMeter*(delta_rightWheel);

    cout << "robotdata.GyroAngle/100: " << robotdata.GyroAngle/100 << " gyroStart: "<< gyroStart << endl;

    gyro = robotdata.GyroAngle/100 - gyroStart;
    double delta_distance  = (distanceLW + distanceRW) / 2.0;
    gyroRad = (((gyro)*PI)/180.0);
    gyroRadMap = gyroRad;

    cout << "gyro: " << gyroRad << endl;

    if(gyro < 0){
        gyro += 360;
    }
    // pretecenie gyro
    if(gyroRad < 0){
        gyroRad += 2*PI;
        gyroRadMap = gyroRad;
    }
    x = x + (delta_distance  * cos(gyroRad))*100;
    y = y + (delta_distance  * sin(gyroRad))*100;


    cout << "gyro: " << gyroRad << endl;
    cout << "gyroRad: " << gyroRad << endl;
    cout << "robotdata.GyroAngle: " << robotdata.GyroAngle/100 << endl;


    encLeftWheel = robotdata.EncoderLeft;
    encRightWheel = robotdata.EncoderRight;

}

void MainWindow::initData(TKobukiData robotdata){
    encLeftWheel = robotdata.EncoderLeft;
    encRightWheel = robotdata.EncoderRight;

    distanceLW = 0;
    distanceRW = 0;
    delta_leftWheel = 0;
    delta_rightWheel = 0;
    x = 0;
    y = 0;
    tickToMeter = 0.000085292090497737556558;
    d = 0.23; //m
    alfa = 0;

    speed = 0;

    //stvorec
//    xArray[0] = 0;
//    xArray[1] = 200;
//    xArray[2] = 0;
//    xArray[3] = 0;
//    xArray[4] = 150;

//    yArray[0] = 300;
//    yArray[1] = 325;
//    yArray[2] = -1;
//    yArray[3] = 0;
//    yArray[4] = 350;

    //trojuholnik
//    xArray[0] = 0;
//    xArray[1] = 0;
//    xArray[2] = 0;
//    xArray[3] = 0;
//    xArray[4] = 150;

//    yArray[0] = -20;
//    yArray[1] = 250;
//    yArray[2] = 0;
//    yArray[3] = 0;
//    yArray[4] = 350;

    //Prehladavanie mapy
    xArray[0] = 0;
    xArray[1] = 430;
    xArray[2] = 260;
    xArray[3] = 260;
    xArray[4] = 120;
    xArray[5] = 260;
    xArray[6] = 260;
    xArray[7] = 480;
    xArray[8] = 480;

    yArray[0] = 300;
    yArray[1] = 370;
    yArray[2] = 370;
    yArray[3] = 150;
    yArray[4] = 150;
    yArray[5] = 150;
    yArray[6] = 50;
    yArray[7] = 50;
    yArray[8] = 175;

    pointReached = 0;
    x_destination = xArray[pointReached];
    y_destination = yArray[pointReached];
    distance = getDistanceToEnd();
    deadbandRotation = 0.02;

    gyroStart = robotdata.GyroAngle/100;
    gyro = 0;
    gyroRad = 0;
    gyroRadMap = 0;

    isConvertAngleRight = false;
    isConvertAngleLeft = false;
    isCorrectRotation = false;
    isStop = false;
    isRobotMove = false;
    isRobotRotate = false;

    numberOfSqareInMap = 120;

    //MAP
        for (int i = 0; i < numberOfSqareInMap; i++) {
            for (int j = 0; j < numberOfSqareInMap; j++) {
                map[i][j] = " ";
            }
        }

    //    for (int i = 0; i < 120; i++) {
    //        for (int j = 0; j < 120; j++) {
    //            std::cout << map[i][j] << " ";
    //        }
    //        std::cout << std::endl;
    //    }

    starMovement = false;
    manualNavigation = false;

    init = false;
}

///toto je calback na data z lidaru, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa ked dojdu nove data z lidaru
int MainWindow::processThisLidar(LaserMeasurement laserData)
{
    memcpy( &copyOfLaserData,&laserData,sizeof(LaserMeasurement));
    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
    updateLaserPicture=1;
    update();//tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia

    return 0;
}


void MainWindow::on_pushButton_9_clicked() //start button
{

    forwardspeed=0;
    rotationspeed=0;
    //tu sa nastartuju vlakna ktore citaju data z lidaru a robota
    connect(this,SIGNAL(uiValuesChanged(double,double,double)),this,SLOT(setUiValues(double,double,double)));

    ///setovanie veci na komunikaciu s robotom/lidarom/kamerou.. su tam adresa porty a callback.. laser ma ze sa da dat callback aj ako lambda.
    /// lambdy su super, setria miesto a ak su rozumnej dlzky,tak aj prehladnost... ak ste o nich nic nepoculi poradte sa s vasim doktorom alebo lekarnikom...
    robot.setLaserParameters(ipaddress,52999,5299,/*[](LaserMeasurement dat)->int{std::cout<<"som z lambdy callback"<<std::endl;return 0;}*/std::bind(&MainWindow::processThisLidar,this,std::placeholders::_1));
    robot.setRobotParameters(ipaddress,53000,5300,std::bind(&MainWindow::processThisRobot,this,std::placeholders::_1));

    ///ked je vsetko nasetovane tak to tento prikaz spusti (ak nieco nieje setnute,tak to normalne nenastavi.cize ak napr nechcete kameru,vklude vsetky info o nej vymazte)
    robot.robotStart();

//    this_thread::sleep_for(chrono::milliseconds(500));

}

void MainWindow::movementForward(int speedForward){
    robot.setTranslationSpeed(speedForward);
}

void MainWindow::movementBackwards(){

}

void MainWindow::movementToRight(){

}

void MainWindow::movementToLeft(){

}

void MainWindow::on_pushButton_2_clicked() //forward
{
    //pohyb dopredu
    robot.setTranslationSpeed(500);
}

void MainWindow::on_pushButton_3_clicked() //back
{
    robot.setTranslationSpeed(-250);

}

void MainWindow::on_pushButton_6_clicked() //left
{
    isRobotRotate = true;
robot.setRotationSpeed(3.14159/2);

}

void MainWindow::on_pushButton_5_clicked()//right
{
    isRobotRotate = true;
robot.setRotationSpeed(-3.14159/2);
}

void MainWindow::on_pushButton_10_clicked()//right
{
    starMovement = true;
    manualNavigation = false;
}

void MainWindow::on_pushButton_11_clicked()//right
{
    cout << "nova mapa" << endl;

    std::ofstream outfile("C:\\Users\\jkose\\Documents\\vysoka skola\\4.rocnik\\2.semester\\RMR\\mapa2.txt");

//    // iterate through the rows of the array and swap them
//    for (int i = 0; i < numberOfSqareInMap-1; i++) {
//        for (int j = 0; j < numberOfSqareInMap; j++) {
//            tempMap[j] = map[i][j];
//            map[i][j] = map[numberOfSqareInMap-1-i][j];
//            map[numberOfSqareInMap-1-i][j] = tempMap[j];
//        }
//    }



    // Write the array contents to the file
    for (int i = numberOfSqareInMap; i > 0; i--) {
        for (int j = 0; j < numberOfSqareInMap; j++) {
            outfile << map[i][j] << " ";
        }
        outfile << std::endl;
    }

    // Close the output file
    outfile.close();

    cout<<"zapisal som"<<endl;
}

void MainWindow::on_pushButton_12_clicked() //forward
{

    //pohyb dopredu
    pointReached = 0;
    QString x = ui->lineEdit_6->text();
    QString y = ui->lineEdit_5->text();
    x_destination = x.toInt();
    y_destination = y.toInt();
    isStop = false;
    starMovement = true;
    manualNavigation = true;
}

void MainWindow::on_pushButton_4_clicked() //stop
{
    robot.setTranslationSpeed(0);
    isStop = true;
    isRobotRotate = false;
}



void MainWindow::on_pushButton_clicked()
{
    if(useCamera1==true)
    {
        useCamera1=false;

        ui->pushButton->setText("use camera");
    }
    else
    {
        useCamera1=true;

        ui->pushButton->setText("use laser");
    }
}

void MainWindow::getNewFrame()
{

}
