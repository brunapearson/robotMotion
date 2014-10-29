#include "Aria.h"
#include <iostream>
#include <stdio.h>

//using namespace cv;
using namespace std;

/*This class creates its own thread, and then runs in the thread, controlling the robot's moviments. */

class RobotMotion : public ArASyncTask
{
public:
    // constructor
    RobotMotion(ArRobot *robot);
    // empty destructor
    ~RobotMotion(void) {}

    // the function to run in the new thread, this just is called once, so
    // only return when you want th ethread to exit
    virtual void * runThread(void *arg);

protected:

    // robot pointer
    ArRobot *myRobot;
};

// a nice simple constructor
RobotMotion::RobotMotion(ArRobot *robot)
{
    //set the robot pointer
    myRobot = robot;

    //print out some information about the robot
    printf("rx %6.1f y %66.1f tth %6.1f vel %7.1f mpacs %3d ", myRobot->getX(), myRobot->getY(), myRobot->getTh(), myRobot->getVel(), myRobot->getMotorPacCount() );
    fflush(stdout);

    // this is what creates are own thread, its from the ArASyncTask
    create();
}

// this is the function called in the new thread
void *RobotMotion::runThread(void *arg)
{
    threadStarted();

    /*Send the robot a serie of motion commands directly, sleeping for a few seconds afterwards to give the robot time to execute them */
    while(myRunning)
    {
        //lock the robot before touching it
        myRobot->lock();
        if(!myRobot->isConnected())
        {
            myRobot->unlock();
            break;
        }
        //print out some information about the robot
        printf("rx %6.1f y %66.1f tth %6.1f vel %7.1f mpacs %3d ", myRobot->getX(), myRobot->getY(), myRobot->getTh(), myRobot->getVel(), myRobot->getMotorPacCount() );
        fflush(stdout);
        myRobot->setRotVel(50);
        myRobot->unlock();
        ArUtil::sleep(3*1000);
        cout << "Stopping" << endl;
        myRobot->lock();
        myRobot->setRotVel(0);
        myRobot->unlock();
        ArUtil::sleep(200);

        ArLog::log(ArLog::Terse, "Sending command to move forward 1 meter...");

        //turn on the motors
        myRobot->comInt(ArCommands::ENABLE,1);
        myRobot->lock();
        myRobot->move(1000);
        myRobot->unlock();
        ArTime start;
        start.setToNow();

        while(1)
        {
            myRobot->lock();
            if(myRobot->isMoveDone())
            {
                cout << "Finished distance" << endl;
                myRobot->unlock();
                break;
            }
            if(start.mSecSince() > 5000)
            {
                cout << "Distance time out" << endl;
                myRobot->unlock();
                break;
            }
            myRobot->unlock();
            ArUtil::sleep(50);
        }
    }


    // return out here, means the thread is done
    return NULL;
}

int main(int argc, char **argv)
{
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

    //RobotMotion rm(&robot);

    // init aria, which will make a dedicated signal handling thread
    Aria::init(Aria::SIGHANDLE_THREAD);

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

    RobotMotion rm(&robot);

    //have the robot connect asyncronously (so its loop is still running)
    //if this fails it means that the robot isn't running in its own thread
    /*if (!robot.asyncConnect())
    {
    printf(
    "asyncConnect failed because robot is not running in its own thread.\n");
    Aria::shutdown();
    return 1;
    }*/

    /*Used to perform actions when keyboard keys are pressed */
    ArKeyHandler keyHandler;
    Aria::setKeyHandler(&keyHandler);
    robot.attachKeyHandler(&keyHandler);

    //now is just wait for the robot to be done running
    cout << "Waiting for the robot's finish executing the code to exit" << endl;
    robot.waitForRunExit();

    //the we exit
    cout << "Exiting main" << endl;
    Aria::exit(0);
    return 0;
}
