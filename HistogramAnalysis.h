#ifndef HISTOGRAMANALYSIS_H_INCLUDED
#define HISTOGRAMANALYSIS_H_INCLUDED



#endif // HISTOGRAMANALYSIS_H_INCLUDED

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
class HistogramAnalysis
{
private:

protected:

public:
    Mat imageComp;
    int hIndex = 0;
    int historySamples = 5; //0-4 rather than 1-5 as in matlab

    int videoSizeFactor;
    int X_resolution = 320;
    int Y_resolution = 240;
    double cellWidth;
    int scanVertLineEdges[65] = {0};
    int numGridCols = 64; //80
    int numGridRows = 43; //60

    Mat scanBlockElements;
    int scanHorizLineEdges[44] = {1,6,11,16,21,26,31,36,41,46,51,56,61,66,71,76,81,86,91,96,101,106,111,116,121,126,131,136,141,146,151,156,161,166,171,176,181,186,191,200,209,219,229,239};
    int scanHorizLineEdges_2[43] = {4,9,14,19,24,29,34,39,44,49,54,59,64,69,74,79,84,89,94,99,104,109,114,119,124,129,134,139,144,149,154,159,164,169,174,179,184,189,198,207,217,227,237};

    int MinDetectionRange_Pixels;
    int safeWindowLeftInset = floor(0.234*X_resolution);
    int safeWindowBottomInset = floor(0.021*Y_resolution);
    int safeWindowRightInset = floor(0.234*X_resolution);
    int safeWindowTopInset = floor(0.725*Y_resolution);
    int XMax;
    int YMax;
    int numChans;
    Mat frameLowResolution;
    Mat inputFrame;
    Mat finalFrameFusedSeg2;
    Mat finalFrameFusedObs2;
    Mat finalFrameFusedThres2;
    Mat frameFused2;

    Mat mHistGrayscale;
    int Horizon = 1;
    int YMin = Horizon + safeWindowTopInset;
    int XMin = safeWindowLeftInset;


    Mat mHistLowRes;
    int histSize = 256;
    int histScaleFactor = 8;
    int histRes = histSize/histScaleFactor;
    double histMax, histLow, averageHistLevel, minHistThres, maxHistThres;
    bool segmentFound;
    int prevSegmentEdge;

    //int cellNum;
    double mSegmentLeftAvg, mSegmentRightAvg;

    double shade;
    bool flag = false;
    int shadeErrorMargin;
    int noSurfaces = 15;
    int maxPathConfidence = 30;
    int maxConf, pathAgeThres;
    int path_Shade = 150;
    int obstShade = 0;

    int tempSurface_HistHeight[4] = {0};
    int tempSurface_HistSegmLeft[4] = {0};
    int tempSurface_HistSegmRight[4] = {0};

    Mat finalFrameFused;
    int maxPathLines;
    int minObstacleSize;

    int minPathSize;
    int pathDataNoOfLines;
    Mat pathData_yPixel;
    Mat pathData_LeftEdge2;
    Mat pathData_RightEdge;

    int maxObstacles;
    int obstacleData_NoOfObstacles;
    Mat obstacleData_yPixel;
    Mat obstacleData_LeftEdge;
    Mat obstacleData_RightEdge;
    Mat obstacleData_ID;

    Mat surfaceType_Diff;

    //int finalFrameFusedArchive[5][43][64];

    Mat finalFrameFusedArchive[5];




// Constructor & Destructor
    HistogramAnalysis();
    ~HistogramAnalysis();

// Public Function Prototypes
    void CompositeImage(const Mat& image, Mat& hls_320x240, Mat& hls2_320x240, Mat& meanChroma_320x240, Mat& chromaTextureMap);
    void HistoryArchiveIndex();
    void VertLinesEdges();
    void BlockElements();
    void Histogram(const Mat& image,  int YPixels, int XPixels);
    void ReleaseMemory();
    void ClassifySurface();
    //void Remap(Mat inputFrameFused);
    void Remap();
    void Segmentation(Mat finalFrameFused, int pathEdges_NumSegm, Mat pathEdges_y, Mat path_Size, Mat pathEdges_ID, Mat pathEdges_xLeft, Mat pathEdges_xRight);
    void DetectObstacles(Mat finalFrameFusedObs, int obstacleEdges_NumSegm, Mat obstacle_Size, Mat obstacleEdges_ID, Mat obstacleEdges_xRight, Mat obstacleEdges_xLeft,Mat obstacleEdges_y);
    void Coordinates(int pathEdges_NumSegm, Mat pathEdges_y, Mat pathEdges_xLeft, Mat pathEdges_xRight);
    void ThresholdMap(Mat finalFrameFusedThres);
    void ObstacleCoord(int obstacleEdges_NumSegm, Mat obstacleEdges_ID, Mat obstacleEdges_xRight, Mat obstacleEdges_xLeft,Mat obstacleEdges_y);
};
