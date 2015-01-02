#ifndef DETECTAREA_H_INCLUDED
#define DETECTAREA_H_INCLUDED



#endif // DETECTAREA_H_INCLUDED

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


//! A class for histogram analysis
class DetectArea
{
private:

protected:

public:
    Mat areaEdgesTemp_ID;
    Mat areaEdgesTemp_y;
    Mat areaEdgesTemp_xLeft;
    Mat areaEdgesTemp_xRight;
    //int areaEdgesTempNumSegm;

    Mat areaSize;

    Mat areaSize2;
    Mat areaLeft2;
    Mat areaLeft;
    Mat areaRight2;
    Mat areaRight;
    Mat areaTop2;
    Mat areaTop;
    Mat areaBottom2;
    Mat areaBottom;
    Mat areaConf2;
    Mat areaConf;
    Mat areaWeight2;
    Mat areaWeight;
    int areaNumAreas2;

    Mat areaEdges_ID2;
    Mat areaEdges_y;
    Mat areaEdges_xLeft;
    Mat areaEdges_xRight;
    int areaEdgesNumSegm;

// Constructor & Destructor
    DetectArea();
    ~DetectArea();

// Public Function Prototypes
    void SurfaceObstacleDetection(Mat img, int imgRows, int imgCols, int shade, int maxEdges);
    void ReleaseMemory();

};
