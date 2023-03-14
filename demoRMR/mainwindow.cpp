#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <math.h>
#include <iostream>
#include <windows.h>
///Jozef Kosecky


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{

    //tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna
    ipaddress="127.0.0.1"; //192.168.1.11 127.0.0.1
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

    robotMovement(robotdata);

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
        emit uiValuesChanged(robotdata.EncoderLeft,11,12);
        ///toto neodporucam na nejake komplikovane struktury.signal slot robi kopiu dat. radsej vtedy posielajte
        /// prazdny signal a slot bude vykreslovat strukturu (vtedy ju musite mat samozrejme ako member premmennu v mainwindow.ak u niekoho najdem globalnu premennu,tak bude cistit bludisko zubnou kefkou.. kefku dodam)
        /// vtedy ale odporucam pouzit mutex, aby sa vam nestalo ze budete pocas vypisovania prepisovat niekde inde

    }
    datacounter++;

    return 0;

}

void MainWindow::robotSlowdown(){
    speed -= 20;
    movementForward(speed);
    if(speed <= 20){
        cout << "zastavujem" << endl;
        movementForward(0);
        speed = 0;
        isRobotMove = false;
    }
}

void MainWindow::robotAcceleration(){
    if(speed < 500){
        cout << "Zrychlujem" << endl;
        speed += 50;
        movementForward(speed);
        isRobotMove = true;
    }
}

void MainWindow::robotMovement(TKobukiData robotdata){

    calculateXY(robotdata);
    double correctRotation = getRightOrientation();
    double distanceToEnd = getDistanceToEnd();

    int rounded_correctRotation = std::floor(correctRotation);
    cout << "correctRotation: " << correctRotation << " rounded_correctRotation: " << rounded_correctRotation << endl;
    cout << "--------------------\n\n" << endl;

    if((robotdata.GyroAngle/100 >= correctRotation - 2) && (robotdata.GyroAngle/100 <= correctRotation + 2)){
        isCorrectRotation = true;
    }
    else if(isRobotMove){
        isCorrectRotation = false;
    }

    if(!isStop){
        if(!isCorrectRotation){
            if(isRobotMove){
                cout << "zastavujem pohyb pred rotaciou spomalenie" << endl;
                robotSlowdown();
            }

            if((robotdata.GyroAngle/100 < correctRotation) && !isRobotRotate && !isRobotMove){
                cout << "tocim dolava" << endl;
                robot.setRotationSpeed(0.25);
                isRobotRotate = true;
            }
            else if((robotdata.GyroAngle/100 > correctRotation) && !isRobotRotate && !isRobotMove){
                cout << "tocim doprava" << endl;
                robot.setRotationSpeed(-0.25);
                isRobotRotate = true;
            }
        }
        else{
            if(isRobotRotate){
                cout << "zastavujem rotaciu pred pohybom" << endl;
                movementForward(0);
                speed = 0;
                isRobotRotate = false;
            }

            if((x_destination >= x - 1) && (x_destination <= x + 1) &&
                    (y_destination >= y - 1) && (y_destination <= y + 1)){
                cout << "zastavujem pohyb" << endl;
                robotSlowdown();
            }
            else{
                if(distanceToEnd < 50 && speed > 50){
                    cout << "Spomalujem" << endl;
                    int distanceToEndTemo = std::floor(distanceToEnd);
                    speed = distanceToEndTemo * 3;
                    movementForward(speed);
                }
                else{
                    robotAcceleration();
                }

//                if(!isRobotRotate){
//                    robotAcceleration();
//                }

//                if(speed == 0){
//                    speed = 50;
//                    movementForward(speed);
//                    isRobotMove = true;
//                }
//                else{


//                }
            }
        }
    }

//    if(!isStop){
//        if(!isCorrectRotation){
//            if((robotdata.GyroAngle/100 < correctRotation) && !isRobotMoving){
//                cout << "tocim dolava" << endl;
//                robot.setRotationSpeed(0.25);
//                isRobotMoving = true;
//            }
//            else if((robotdata.GyroAngle/100 > correctRotation) && !isRobotMoving){
//                cout << "tocim doprava" << endl;
//                robot.setRotationSpeed(-0.25);
//                isRobotMoving = true;
//            }
//        }

//        if(isCorrectRotation){
//            int rounded_x = std::floor(x);
//            int rounded_y = std::floor(y);
//            cout << "x: " << x << " rounded_x: " << rounded_x << " x_destination: " << x_destination << endl;
//            cout << "y: " << y << " rounded_y: " << rounded_y << "y_destination: " << y_destination << endl;
//            cout << "--------------------\n\n" << endl;

//            if((x_destination >= rounded_x - 1) && (x_destination <= rounded_x + 1) &&
//                    (y_destination >= rounded_y - 1) && (y_destination <= rounded_y + 1)){
//                cout << "zastavujem" << endl;
//                robot.setTranslationSpeed(0);
//                isRobotMoving = false;
//            }
//            else if(!isRobotMoving){
//                cout << "idem rovno" << endl;
//                movementForward();
//                isRobotMoving = true;
//            }
//        }
//    }
}

double MainWindow::getDistanceToEnd(){
    double distance = std::sqrt(std::pow(x_destination - x, 2) + std::pow(y_destination - y, 2));
    return distance;
}

double MainWindow::getRightOrientation(){
    double deltaX = x_destination - x; // change in x-coordinate
    double deltaY = y_destination - y; // change in y-coordinate
    double angle = atan2(deltaY, deltaX) * 180 / PI; // angle in degrees
//    if(angle < 0){
//        angle += 360;
//    }
    return angle;
}


void MainWindow::calculateXY(TKobukiData robotdata){
    delta_leftWheel = robotdata.EncoderLeft - encLeftWheel;
    delta_rightWheel = robotdata.EncoderRight - encRightWheel;

//    if( encLeftWheel > 60000 && robotdata.EncoderLeft <10000  )
//            delta_leftWheel=tickToMeter*(robotdata.EncoderLeft + (65530 - encLeftWheel));

//        else if (encLeftWheel < 10000 && robotdata.EncoderLeft > 60000)
//            delta_leftWheel=tickToMeter*(encLeftWheel + (65530 - robotdata.EncoderLeft));

//        else
//            delta_leftWheel=tickToMeter*(robotdata.EncoderLeft - encLeftWheel);

//        if( encRightWheel > 60000 && robotdata.EncoderRight <10000  )
//            delta_rightWheel=tickToMeter*(robotdata.EncoderRight + (65530 - encRightWheel));

//        else if (encRightWheel < 10000 && robotdata.EncoderRight > 60000)
//            delta_rightWheel=tickToMeter*(encRightWheel + (65530 - robotdata.EncoderRight));

//        else
//            delta_rightWheel=tickToMeter*(robotdata.EncoderRight - encRightWheel);

    //delta_leftWheel = robotdata.EncoderLeft - encLeftWheel;
    //delta_rightWheel = robotdata.EncoderRight - encRightWheel;

    distanceLW = tickToMeter*(delta_leftWheel);
    distanceRW = tickToMeter*(delta_rightWheel);

    cout << "encLeftWheel: " << encLeftWheel << endl;
    cout << "robotdata.EncoderLeft: " << robotdata.EncoderLeft << endl;
    cout << "encRightWheel: " << encRightWheel << endl;
    cout << "robotdata.EncoderRight: " << robotdata.EncoderRight << endl;

//    cout << "left: " << leftWheel << endl;
//    cout << "right: " << rightWheel << endl;
//    cout << "robotdata.GyroAngle: " << (robotdata.GyroAngle/100) << endl;

//    double alfa_new = ((robotdata.GyroAngle/100) * 3.14)/180;
//    double temp = 0;
//    if(leftWheel != 0 && rightWheel != 0){
//        temp = ( (d*(distanceLW + distanceRW)) / (2*(distanceLW - distanceRW)));
//    }
//    cout << "temp: " << temp << endl;
//    cout << "alfa_new: " << alfa_new << endl;

//    cout << "sin(alfa_new): " << sin(alfa_new) << endl;
//    cout << "sin(alfa): " << sin(alfa) << endl;

//    cout << "distanceLW: " << distanceLW << endl;
//    cout << "distanceRW: " << distanceRW << endl;

//    x = x + temp *(sin(alfa_new) - sin(alfa));
//    y = y - temp*(cos(alfa_new) - cos(alfa));


    double delta_distance  = (distanceLW + distanceRW) / 2;
    double gyro_angle = (((robotdata.GyroAngle)*3.14)/180)/100;
    if(gyro_angle < (PI)){
        gyro_angle += 2*PI;
    }
    x = x + (delta_distance  * cos(gyro_angle))*100;
    y = y + (delta_distance  * sin(gyro_angle))*100;


    cout << "x: " << x << endl;
    cout << "y: " << y << endl;
    cout << "robotdata.GyroAngle: " << robotdata.GyroAngle/100 << endl;
    cout << "--------------------\n\n" << endl;

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

//    x_destination = 20;
//    y_destination = 300;

//    x_destination = 40;
//    y_destination = 20;
    x_destination = -20;
    y_destination = 300;

    isCorrectRotation = false;
    isStop = false;
    isRobotMove = false;
    isRobotRotate = false;
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
robot.setRotationSpeed(3.14159/2);

}

void MainWindow::on_pushButton_5_clicked()//right
{
robot.setRotationSpeed(-3.14159/2);
}

void MainWindow::on_pushButton_4_clicked() //stop
{
    robot.setTranslationSpeed(0);
    isStop = true;
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
