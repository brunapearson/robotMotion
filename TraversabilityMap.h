#ifndef TRAVERSABILITYMAP_H_INCLUDED
#define TRAVERSABILITYMAP_H_INCLUDED



#endif // TRAVERSABILITYMAP_H_INCLUDED

#ifndef TRAVERSABILITYMAP_H_INCLUDED
#define TRAVERSABILITYMAP_H_INCLUDED



#endif // TRAVERSABILITYMAP_H_INCLUDED

// Pragmas
#pragma once

// Include Files
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <stdio.h>

// namespcaes declaration
using namespace cv;
using namespace std;


//! A class for Traversability Map
class TraversabilityMap{
private:

protected:

public:


// Constructor & Destructor
TraversabilityMap();
~TraversabilityMap();

// Public Function Prototypes
void HorizonLine(Mat image,int X_resolution, int obstacleData_NoOfObstacles, Mat obstacleData_LeftEdge, Mat obstacleData_RightEdge, Mat obstacleData_yPixel, Mat pathData_yPixel, Mat pathData_LeftEdge, Mat pathData_RightEdge, int pathDataNoOfLines);

};
