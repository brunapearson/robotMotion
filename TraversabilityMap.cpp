//Include Files
#include "TraversabilityMap.h"
#include <fstream>

//namespace declaration
using namespace cv;
using namespace std;

//! Class constructor
TraversabilityMap::TraversabilityMap()
{

}

//! Class destructor
TraversabilityMap::~TraversabilityMap()
{

}


void TraversabilityMap::HorizonLine(Mat image,int X_resolution, int obstacleData_NoOfObstacles, Mat obstacleData_LeftEdge, Mat obstacleData_RightEdge, Mat obstacleData_yPixel, Mat pathData_yPixel, Mat pathData_LeftEdge, Mat pathData_RightEdge, int pathDataNoOfLines)
{
    int Horizon = 0;
    int lineType = 2;

    /* Draw obstacles */
    for(int i = 0; i < obstacleData_NoOfObstacles; i++)
    {
        cv::line(image, cv::Point(obstacleData_LeftEdge.at<int>(i), obstacleData_yPixel.at<int>(i)), cv::Point(obstacleData_RightEdge.at<int>(i), obstacleData_yPixel.at<int>(i)), cv::Scalar(0,0,255),2, 1);
    }

    /* Draw Left Edge */
    Point** path_points = NULL; //Pointer to int, initialize to nothing
    path_points = new Point*[pathDataNoOfLines]; //Allocate pathDataNoOfLines and save ptr in path_points

    path_points[0] = new Point[pathDataNoOfLines];

    int counter=0;

    //create file and print out the robot position
    /*string coordinatesPath;
    ofstream coordPathway;
    coordPathway.open("coordPathway.txt");*/

    for(int i = 0; i < pathDataNoOfLines; i++)
    {
        path_points[0][i] = Point(pathData_LeftEdge.at<int>(i),pathData_yPixel.at<int>(i));

        /*coordinatesPath = format("L %6.1f - Y %6.1f ", pathData_LeftEdge.at<int>(i), pathData_yPixel.at<int>(i));
        //print position to file
        coordPathway << coordinatesPath << endl;*/
    }




    const Point* ppt[1] = { path_points[0]};
    int npt[] = { pathDataNoOfLines };
    polylines (image, ppt, npt, 1, false, Scalar(225,0,0),
               lineType,CV_AA,0);

    delete[] path_points; //free memory pointed to by path_points.
    path_points = NULL; //clear path_points to prevent using invalid memory reference.

    /*Draw lines Right Edge */
    Point** path_points_right = NULL; //Pointer to int, initialize to nothing
    path_points_right = new Point*[pathDataNoOfLines]; //Allocate pathDataNoOfLines and save ptr in path_points

    path_points_right[0] = new Point[pathDataNoOfLines];

    for(int i = 0; i < pathDataNoOfLines; i++)
    {
        path_points_right[0][i] = Point(pathData_RightEdge.at<int>(i),pathData_yPixel.at<int>(i));

        //coordinatesPath = format("L %6.1f - Y %6.1f ", c_right, c_y);
        //print position to file
        //coordPathway << c_right << endl;
    }
    //coordPathway.close();

    const Point* ppt_right[1] = { path_points_right[0]};
    int npt_right[] = { pathDataNoOfLines };
    polylines (image, ppt_right, npt_right, 1, false, Scalar(225,0,0),
               lineType,CV_AA,0);

    delete[] path_points_right; //free memory pointed to by path_points.
    path_points_right = NULL; //clear path_points to prevent using invalid memory reference.

}
