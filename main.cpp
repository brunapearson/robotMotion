#include "flycapture/FlyCapture2.h"
#include "Aria.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>

#include "PreProcessing.h"
#include "HistogramAnalysis.h"
#include "CreateCamera.h"
#include "DetectArea.h"
#include "TraversabilityMap.h"
#include "NormEdges.h"

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <unistd.h>

using namespace FlyCapture2;
using namespace cv;
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

    //create file and print out the robot position
        string position;
        ofstream robotPosition;
        robotPosition.open("robotPosition.txt");

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

        position = format("rx %6.1f - y %6.1f - tth %6.1f - vel %7.1f - mpacs %3d ", myRobot->getX(), myRobot->getY(), myRobot->getTh(), myRobot->getVel(), myRobot->getMotorPacCount() );

        //print position to file
        robotPosition << position << endl;


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

robotPosition.close();

    // return out here, means the thread is done
    return NULL;
}

/*Sonar */
class SonarThread : public ArASyncTask
{
public:
    //constructor
    SonarThread(ArRobot *robot, RobotMotion *rmotion);
    //destructor
    ~SonarThread(void) {}
    //to be called if the connection was made
    //void connected(void);

    double obstacleAtAngle = 40.0;

    virtual void * runThread(void *arg);


protected:
    //robot pointer
    ArRobot *myRobot;

    //pointer to robotmotion
    RobotMotion *myRobotMotion;

    //the functor callbacks
    // ArFunctorC<SonarThread> *myConnectedCB;



};
/* Sonar constructor */
SonarThread::SonarThread(ArRobot *robot, RobotMotion *rmotion)
{
    //set the pointers
    myRobot = robot;

    ArSonarDevice mySonar;

    myRobot->addRangeDevice(&mySonar);





    //now create the functor callbacks, than set them on the robot
    //myConnectedCB =  new ArFunctorC<SonarThread>(this, &SonarThread::connected);
    //myRobot->addConnectCB(myConnectedCB, ArListPos::FIRST);

    // this is what creates are own thread, its from the ArASyncTask
    create();
}


// this is the function called in the new thread
void *SonarThread::runThread(void *arg)
{
    threadStarted();

    double value; //variable to hold the closest value from all the sonar readings

//    double angleValue; // passed as pointer to the method in order to retrive the angle at closest value

    ArSensorReading* values; //This class abstracts range and angle read from sonar

//A slice of the polar region (from -90 to 90)
//value = mySonar.currentReadingPolar(-90,90, &angleAtValue);

    /*Send the robot a serie of motion commands directly, sleeping for a few seconds afterwards to give the robot time to execute them */
    while(myRunning)
    {
        ArUtil::sleep(1000);
        cout << "Sonar output " << value << endl;

        int total = myRobot->getNumSonar();

        cout << "Number of Sonar " << total << endl;

        for(int i=0; i < total; i++)
        {
            values = myRobot->getSonarReading(i);
            double range = values->getRange();
            double angle = values->getSensorTh();
            cout << "Sonar reading " << i << " = " << range
                 << " Angle " << i << " = " <<
                 angle << "\n";
            if(range < 300)
            {
                cout << "Obstacle detected at " << angle << endl;
                //myRobotMotion->stopRunning();
                break;
            }
        }

    }
}

int main(int argc, char **argv)
{
    PreProcessing pp;
    HistogramAnalysis ha;
    CreateCamera cc;
    DetectArea da;
    TraversabilityMap tm;
    NormEdges ne;

    Mat pathSize;
    Mat pathEdgesID;
    Mat pathEdgesy;
    Mat pathEdgesLeft;
    Mat pathEdgesRight;
    Mat pathEdgesxLeft;
    Mat pathEdgesxRight;
    int pathEdgesNumSegm;

    Mat obstacleSize;
    Mat obstacleEdgesID;
    Mat obstacleEdgesy;
    Mat obstacleEdgesxLeft;
    Mat obstacleEdgesxRight;

    int obstacleedgesNumSegm;


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

    //SonarThread st(&robot, &rm);

    //cout << "--------Testing " << st.obstacleAtAngle << endl;

    //rm.stopRunning();




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

    /* Camera */

    Error error;
    Camera camera; //capture object

    //Connect the camera
    error = camera.Connect(0);
    if(error != PGRERROR_OK)
    {
        cout << "Failed to connect to camera" << endl;
        Aria::exit(0);
        return false;
    }

    error = camera.StartCapture();

    if(error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED)
    {
        cout << "Bandwidth exceeded" << endl;
        Aria::exit(0);
        return false;
    }
    else if (error != PGRERROR_OK)
    {
        cout << "Failed to start image capture" << endl;
        Aria::exit(0);
        return false;
    }

    /*Record Video */

    VideoWriter videoOutput("output.avi", CV_FOURCC('M','J','P','G'), 25, cvSize(320,240),true);
    if(!videoOutput.isOpened())
    {
        cout << "error: could not open video file" << endl;
        exit(0);
    }

    /* Main loop */
    char key = 0;
    //while(true)
    while(key !='q')
    {

        // Get image
        Image rawImage;
        Error error = camera.RetrieveBuffer( &rawImage );
        if(error != PGRERROR_OK)
        {
            cout << "capture error" << endl;
            Aria::exit(0);
            continue;
        }

        /*Initialisation */
        cc.sCamera("Default");

        // convert to rgb
        Image rgbImage;
        rawImage.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage);

        // convert to OpenCV MAt
        unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();
        Mat image = Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(), rowBytes);
        //IplImage *bgr_frame=cvQueryFrame(rowBytes); //Init the video read

        // resize input image
        Size size(320,240); //the dst image size, e.g. 400/300
        Mat dst, salient;   //dst and salient image
        resize(image,dst,size);//resize image

        //****************************************************************************************************************
        // ------- 1 - camera image pre-processing
        //****************************************************************************************************************

        //Saturation (based on the S channel of the HSL colourspace)
        pp.Saturation(dst);
        imshow("Display Saturation", pp.hls_320x240);

        //Saturation-based texture
        pp.SaturationTexture(dst);
        imshow("Display Saturation-based texture", pp.hls2_320x240);

        //Mean Chroma
        pp.MeanChroma(dst);
        imshow("Display Mean Chroma", pp.meanChroma_320x240);

        //Chroma-based texture
        pp.ChromaTexture(dst);
        imshow("Display Chroma-based texture", pp.chromaTextureMap);

        //DIVoG
        pp.DiVoG(dst);
        imshow("DiVoG",pp.divog_320x240);


        //****************************************************************************************************************
        // ------- 2 - multi-dimensional segmentation by histogram analysis
        //****************************************************************************************************************
        //ha.CompositeImage(dst,pp.hls_320x240,pp.hls2_320x240,pp.meanChroma_320x240,pp.chromaTextureMap);
        ha.CompositeImage(dst,pp.divog_320x240,pp.hls2_320x240,pp.meanChroma_320x240,pp.chromaTextureMap);

        ha.HistoryArchiveIndex();
        ha.VertLinesEdges();
        ha.BlockElements();

        ha.Histogram(ha.imageComp, cc.YPixels, cc.XPixels);
        ha.ClassifySurface();

        ha.Remap();

        //****************************************************************************************************************
        // ------- 3 - temporal information processing
        //****************************************************************************************************************
        da.SurfaceObstacleDetection(ha.finalFrameFused, ha.numGridRows, ha.numGridCols, ha.path_Shade, ha.maxPathLines);
        pathSize = da.areaSize2.clone();
        pathEdgesID = da.areaEdges_ID2.clone();
        pathEdgesy = da.areaEdges_y.clone();
        pathEdgesxLeft = da.areaEdges_xLeft.clone();
        pathEdgesxRight =  da.areaEdges_xRight.clone();
        pathEdgesNumSegm = da.areaEdgesNumSegm;

        ha.Segmentation(ha.finalFrameFused,pathEdgesNumSegm,pathEdgesy,pathSize,pathEdgesID,pathEdgesxLeft,pathEdgesxRight);

        da.SurfaceObstacleDetection(ha.finalFrameFusedSeg2,ha.numGridRows, ha.numGridCols, ha.obstShade, ha.maxPathLines);

        obstacleSize = da.areaSize2.clone();
        obstacleEdgesID = da.areaEdges_ID2.clone();
        obstacleEdgesy = da.areaEdges_y.clone();
        obstacleEdgesxLeft = da.areaEdges_xLeft.clone();
        obstacleEdgesxRight = da.areaEdges_xRight.clone();
        obstacleedgesNumSegm = da.areaEdgesNumSegm;

        ha.DetectObstacles(ha.finalFrameFusedSeg2, obstacleedgesNumSegm, obstacleSize, obstacleEdgesID, obstacleEdgesxRight, obstacleEdgesxLeft, obstacleEdgesy);

        ha.ThresholdMap(ha.finalFrameFusedObs2);

        /*Detect Path Area - Iteration 2 */
        da.SurfaceObstacleDetection(ha.finalFrameFusedThres2, ha.numGridRows,ha.numGridCols,ha.path_Shade, ha.maxPathLines);

        pathEdgesy = da.areaEdges_y.clone();
        pathEdgesxLeft = da.areaEdges_xLeft.clone();
        pathEdgesxRight = da.areaEdges_xRight.clone();
        pathEdgesNumSegm = da.areaEdgesNumSegm;

        da.SurfaceObstacleDetection(ha.finalFrameFusedThres2,ha.numGridRows, ha.numGridCols, ha.obstShade, ha.maxPathLines);

        obstacleSize = da.areaSize2.clone();
        obstacleEdgesID = da.areaEdges_ID2.clone();
        obstacleEdgesy = da.areaEdges_y.clone();
        obstacleEdgesxLeft = da.areaEdges_xLeft.clone();
        obstacleEdgesxRight = da.areaEdges_xRight.clone();
        obstacleedgesNumSegm = da.areaEdgesNumSegm;

        ha.Coordinates(pathEdgesNumSegm,pathEdgesy,pathEdgesxLeft, pathEdgesxRight);

        ha.ObstacleCoord(obstacleedgesNumSegm, obstacleEdgesID, obstacleEdgesxRight, obstacleEdgesxLeft, obstacleEdgesy);

        //****************************************************************************************************************
        // ------- 4 - Draw map
        //****************************************************************************************************************

        Mat normLeftEdges;
        Mat normRightEdges;

        if(ha.obstacleData_NoOfObstacles > 0 && ha.pathDataNoOfLines > 0)
        {
        ne.NormaliseEdges(ha.pathData_LeftEdge2, ha.pathDataNoOfLines, 5);
        normLeftEdges = ne.finalArray.clone();

        ne.NormaliseEdges(ha.pathData_RightEdge, ha.pathDataNoOfLines, 5);
        normRightEdges = ne.finalArray.clone();

        tm.HorizonLine(dst, ha.X_resolution, ha.obstacleData_NoOfObstacles, ha.obstacleData_LeftEdge, ha.obstacleData_RightEdge, ha.obstacleData_yPixel, ha.pathData_yPixel, normLeftEdges, normRightEdges, ha.pathDataNoOfLines);

        //write text on video
        /*char TestStr[100];
        sprintf(TestStr,"waited: %1f and loop is %1f", 0.5,0.7);
        putText(dst,TestStr,Point(10,50),CV_FONT_NORMAL,0.45,Scalar(255,255,255),1,1);*/


        imshow("Camera Video", dst);
        videoOutput.write(dst); // save input image to output.avi file
        }





        //imshow("Camera Video", dst);
        //videoOutput.write(dst); // save input image to output.avi file
        key = waitKey(25);

        //while(waitKey(1) !=27); //27 = ascii value for ESC

    }

    error = camera.StopCapture();
    camera.Disconnect();

    //now is just wait for the robot to be done running
    cout << "Waiting for the robot's finish executing the code to exit" << endl;
    robot.waitForRunExit();

    //the we exit
    cout << "Exiting main" << endl;
    Aria::exit(0);
    return 0;
}
