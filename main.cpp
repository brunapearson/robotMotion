#include "Aria.h"
#include <iostream>
#include <stdio.h>




/* Main method */
int main(int argc, char** argv)
{

    // Initialize some global data
    Aria::init();


    // If you want ArLog to print "Verbose" level messages uncomment this:
    //ArLog::init(ArLog::StdOut, ArLog::Verbose);

    // This object parses program options from the command line
    ArArgumentParser parser(&argc, argv);

    // Load some default values for command line arguments from /etc/Aria.args
    // (Linux) or the ARIAARGS environment variable.
    parser.loadDefaultArguments();

    // Central object that is an interface to the robot and its integrated
    // devices, and which manages control of the robot by the rest of the program.
    ArRobot robot;

    // Object that connects to the robot or simulator using program options
    ArRobotConnector robotConnector(&parser, &robot);

    // If the robot has an Analog Gyro, this object will activate it, and
    // if the robot does not automatically use the gyro to correct heading,
    // this object reads data from it and corrects the pose in ArRobot
    ArAnalogGyro gyro(&robot);

    // Connect to the robot, get some initial data from it such as type and name,
    // and then load parameter files for this robot.
    if (!robotConnector.connectRobot())
    {
        // Error connecting:
        // if the user gave the -help argumentp, then just print out what happened,
        // and continue so options can be displayed later.
        if (!parser.checkHelpAndWarnUnparsed())
        {
            ArLog::log(ArLog::Terse, "Could not connect to robot, will not have parameter file so options displayed later may not include everything");
        }
        // otherwise abort
        else
        {
            ArLog::log(ArLog::Terse, "Error, could not connect to robot.");
            Aria::logOptions();
            Aria::exit(1);
        }
    }

    if(!robot.isConnected())
    {
        ArLog::log(ArLog::Terse, "Internal error: robot connector succeeded but ArRobot::isConnected() is false!");
    }

    /* Start the robot task loop running in a new background thread. The 'true' argument means if it loses
     connection the task loop stops and the thread exits. Note that after starting this thread, we must look and unlook the ArRobot object
     before and after accessing it. */

    robot.runAsync(false);//true

    /* Send the robot a series of motion commands directly, sleepig for a
    few seconds afterwards to give the robot time to execute them.*/
    robot.lock();
    robot.setRotVel(50);
    robot.unlock();
    ArUtil::sleep(3*1000);
    printf("Stopping \n");
    robot.lock();
    robot.setRotVel(0);
    robot.unlock();
    ArUtil::sleep(200);

    ArLog::log(ArLog::Terse, "Sending command to move forward 1 meter...");

    // turn on the motors
    robot.comInt(ArCommands::ENABLE, 1);

    robot.lock();

    robot.move(1000);

    robot.unlock();

    ArTime start;

    start.setToNow();

    while(1)
    {
        robot.lock();
        if(robot.isMoveDone())
        {
            printf("Finished distance \n");
            robot.unlock();
            break;
        }
        if(start.mSecSince() > 5000)
        {
            printf("Distance time out \n");
            robot.unlock();
            break;
        }
        robot.unlock();
        ArUtil::sleep(50);
    }

    ArLog::log(ArLog::Terse, "Sending command to move backwards 1 meter...");

    robot.lock();

    robot.move(-1000);

    robot.unlock();

    start.setToNow();

    while(1)
    {
        robot.lock();
        if(robot.isMoveDone())
        {
            printf("Finished distance \n");
            robot.unlock();
            break;
        }
        if(start.mSecSince() > 10000)
        {
            printf("Distance time out \n");
            robot.unlock();
            break;
        }
        robot.unlock();
        ArUtil::sleep(50);
    }

    ArLog::log(ArLog::Terse, "Sending command to rotate 30 degrees...");

    robot.lock();

    robot.setHeading(30);

    robot.unlock();

    start.setToNow();

    while(1)
    {
        robot.lock();
        if(robot.isHeadingDone(5))
        {
            printf("Finished distance \n");
            robot.unlock();
            break;
        }
        if(start.mSecSince() > 5000)
        {
            printf("Distance time out \n");
            robot.unlock();
            break;
        }
        robot.unlock();
        ArUtil::sleep(100);
    }
    ArLog::log(ArLog::Terse, "Sending command to rotate -30 degrees...");

    robot.lock();

    robot.setHeading(-30);

    robot.unlock();

    start.setToNow();

    while(1)
    {
        robot.lock();
        if(robot.isHeadingDone(5))
        {
            printf("Finished distance \n");
            robot.unlock();
            break;
        }
        if(start.mSecSince() > 5000)
        {
            printf("Distance time out \n");
            robot.unlock();
            break;
        }
        robot.unlock();
        ArUtil::sleep(100);
    }

    /*
     *----------------------------------------------------------------------------------
     * Print raw sonar data
     *----------------------------------------------------------------------------------
     */

    ArSonarDevice mySonar;
    robot.addRangeDevice(&mySonar);

    double value; //variable to hold the closest value from all the sonar readings

    //angleAtValue is passed as pointer to method to retrieve angle at closest value double angleAtValue;
    double angleAtValue;

    //A slice of the polar region (from -90 to 90)
    //value = mySonar.currentReadingPolar(-90,90, &angleAtValue);

    std::cout << "Sonar output " << value << std::endl;

    int total = robot.getNumSonar();

    std::cout << "Number of Sonars " << total << std::endl;

    ArSensorReading* values; //This class abstracts range and angle read from sonar

    int counter = 0;

    while(counter < 5)
    {
        for(int i=0; i < total; i++)
        {
            values = robot.getSonarReading(i);
            double range = values->getRange();
            double angle = values->getSensorTh();
            std::cout << "Sonar reading " << i << " = " << range
                      <<" Angle " << i << " = " <<
                      angle << "\n";
            if(range < 300)
            {
                std::cout << "Obstacle detected at " << angle << std::endl;
                //robot.setRotVel(30.0);
                break;
            }
        }

        counter += 1;

    }
    Aria::exit(0); //Exit Aria
    return 0;

}
