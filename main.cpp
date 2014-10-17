#include "Aria.h"
#include <iostream>


int main(void)
{
//the serial connection(robot)
ArSerialConnection serConn;

//tcp connection (sim)
ArTcpConnection tcpConn;

//robot
ArRobot robot;


Aria::init();



tcpConn.setPort("localhost",8101);
//tcpConn.setPort("10.0.126.12",8101);
//see if we can get to the simulator(true is sucess)
if(tcpConn.openSimple())
{
    printf("Connecting to simulator through tcp. \n");
    robot.setDeviceConnection(&tcpConn);
}
else{
    serConn.setPort(); //default "/dev/ttyS0" or "COM1" serial port
    printf("Could not connect to simulator, connecting to robot through serial. \n");
    robot.setDeviceConnection(&serConn);
}

Aria::exit(0);
return 0;
}
