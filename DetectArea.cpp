//Include Files
#include "DetectArea.h"
#include <cmath>

//namespace declaration
using namespace cv;
using namespace std;

//! Class constructor
DetectArea::DetectArea()
{

}

//! Class destructor
DetectArea::~DetectArea()
{

}

//Mat areaSize = Mat::zeros(1,200,CV_32SC1);
//Mat areaLeft = Mat::zeros(1,200,CV_32SC1);
//Mat areaEdgesTemp_ID = Mat::zeros(1,200,CV_32SC1);
//Mat areaRight = Mat::zeros(1,200,CV_32SC1);
//Mat areaTop = Mat::zeros(1,200,CV_32SC1);
//Mat areaBottom = Mat::zeros(1,200,CV_32SC1);
//Mat areaConf = Mat::zeros(1,200,CV_32FC1);
//Mat areaWeight = Mat::zeros(1,200,CV_32SC1);
Mat areaEdges_ID;
int areaNumAreas = 0;
//Mat areaEdgesTemp_xLeft = Mat::zeros(1,200,CV_32SC1);
//Mat areaEdgesTemp_xRight = Mat::zeros(1,200,CV_32SC1);
//Mat areaEdgesTemp_y = Mat::zeros(1,200,CV_32SC1);
int areaEdgesTempNumSegm = 0;

void DetectArea::SurfaceObstacleDetection(Mat img, int imgRows, int imgCols, int shade, int maxEdges)
{
    //cout << "img= "<< endl << " " << img << endl << endl;
    areaEdgesTemp_ID = Mat::zeros(1,maxEdges,CV_32SC1);
    areaEdgesTemp_xLeft = Mat::zeros(1,maxEdges,CV_32SC1);
    areaEdgesTemp_xRight = Mat::zeros(1,maxEdges,CV_32SC1);
    areaEdgesTemp_y = Mat::zeros(1,maxEdges,CV_32SC1);
    //areaEdgesTempNumSegm = 0;

    areaSize = Mat::zeros(1,maxEdges,CV_32SC1);
    areaLeft = Mat::zeros(1,maxEdges,CV_32SC1);
    areaRight = Mat::zeros(1,maxEdges,CV_32SC1);
    areaTop = Mat::zeros(1,maxEdges,CV_32SC1);
    areaBottom = Mat::zeros(1,maxEdges,CV_32SC1);
    areaConf = Mat::zeros(1,maxEdges,CV_32FC1);
    areaWeight = Mat::zeros(1,maxEdges,CV_32SC1);
    //areaNumAreas = 0;

    int obstID = 0;
    bool leftEdgeFound;

    for( int i = 1; i <= imgRows; i++)
    {
        leftEdgeFound = false;
        for(int j = 1; j <= imgCols; j++)
        {
            //if the segmented frame pixel is black then that pixek is marked as a potential obstacle.
            // This part of the algorithm identifies the edges of each black segment in each row.
            if(img.at<uchar>(i-1,j-1) == shade && j < imgCols && leftEdgeFound == false)
            {
                if(obstID-1 < maxEdges)
                {
                    obstID = obstID + 1;
                    areaEdgesTemp_ID.at<int>(obstID-1) = obstID;
                    areaEdgesTemp_y.at<int>(obstID-1) = i;
                    areaEdgesTemp_xLeft.at<int>(obstID-1) = j;
                    leftEdgeFound = true;
                    //obstID = obstID + 1;

                }

            }
            else if((img.at<uchar>(i-1,j-1) != shade || j == imgCols) && leftEdgeFound == true)
            {
                if(j == imgCols)
                {
                    areaEdgesTemp_xRight.at<int>(obstID-1) = imgCols;
                }
                else
                {
                    areaEdgesTemp_xRight.at<int>(obstID-1) = j-1;
                }
                leftEdgeFound = false;
            }
        }
    }
    areaEdgesTempNumSegm = obstID;

    /* merge neighbouring eges under the same ID */
    for(int i = 1; i <= areaEdgesTempNumSegm; i++)
    {
        for(int j = 1; j <= i-1; j++)
        {
            if(areaEdgesTemp_y.at<int>(i-1) == areaEdgesTemp_y.at<int>(j-1) + 1)
            {
                if((areaEdgesTemp_xLeft.at<int>(i-1) >= areaEdgesTemp_xLeft.at<int>(j-1) && areaEdgesTemp_xLeft.at<int>(i-1) <= areaEdgesTemp_xRight.at<int>(j-1))
                   ||(areaEdgesTemp_xRight.at<int>(i-1) >= areaEdgesTemp_xLeft.at<int>(j-1) && areaEdgesTemp_xRight.at<int>(i-1) <= areaEdgesTemp_xRight.at<int>(j-1))
                   || (areaEdgesTemp_xLeft.at<int>(i-1) <= areaEdgesTemp_xLeft.at<int>(j-1) && areaEdgesTemp_xRight.at<int>(i-1) >= areaEdgesTemp_xRight.at<int>(j-1)))
                {
                    obstID = min(areaEdgesTemp_ID.at<int>(i-1), areaEdgesTemp_ID.at<int>(j-1));
                    areaEdgesTemp_ID.at<int>(areaEdgesTemp_ID.at<int>(i-1)-1) = obstID;
                    areaEdgesTemp_ID.at<int>(areaEdgesTemp_ID.at<int>(j-1)-1) = obstID;
                    areaEdgesTemp_ID.at<int>(i-1) = obstID;
                    areaEdgesTemp_ID.at<int>(j-1) = obstID;
                }
            }
        }
    }
//cout << "TempID_1= "<< endl << " " << areaEdgesTemp_ID << endl << endl;
// update structure
    for(int i = 1; i <= areaEdgesTempNumSegm; i++)
    {
        areaEdgesTemp_ID.at<int>(i-1)  = areaEdgesTemp_ID.at<int>(areaEdgesTemp_ID.at<int>(i-1)-1);

    }

// reassing IDs to incremental order
    int ID = 0;
    int temp = 0;

// the original code in matlab uses AreaEdges = AreaEdgesTemp;
    areaEdges_ID = areaEdgesTemp_ID.clone();
    areaEdges_y = areaEdgesTemp_y.clone();
    areaEdges_xLeft = areaEdgesTemp_xLeft.clone();
    areaEdges_xRight = areaEdgesTemp_xRight.clone();
    areaEdgesNumSegm = areaEdgesTempNumSegm;

//cout << "areaEdges_y= "<< endl << " " << areaEdges_y << endl << endl;

    for(int i = 1; i <= areaEdgesTempNumSegm; i++)
    {
        if(areaEdgesTemp_ID.at<int>(i-1) == temp || areaEdgesTemp_ID.at<int>(i-1) == 0)
        {
            continue;
        }
        else
        {
            ID = ID + 1;
            temp = areaEdgesTemp_ID.at<int>(i-1);
            areaEdges_ID.at<int>(i-1) = ID;
            areaEdgesTemp_ID.at<int>(i-1) = 0;
            areaSize.at<int>(ID-1) = (areaEdgesTemp_xRight.at<int>(i-1) - areaEdgesTemp_xLeft.at<int>(i-1))+1;
            areaLeft.at<int>(ID-1) = areaEdgesTemp_xLeft.at<int>(i-1);
            areaRight.at<int>(ID-1) = areaEdgesTemp_xRight.at<int>(i-1);
            areaTop.at<int>(ID-1) = areaEdgesTemp_y.at<int>(i-1);
            areaBottom.at<int>(ID-1) = areaEdgesTemp_y.at<int>(i-1);
            areaConf.at<float>(ID-1) = 0.0;
            areaWeight.at<int>(ID-1) = areaSize.at<int>(ID-1)*areaEdgesTemp_y.at<int>(i-1);

            for( int j = (i+1); j <= areaEdgesTempNumSegm; j++)
            {

                if(areaEdgesTemp_ID.at<int>(j-1) == temp)
                {
                    areaEdges_ID.at<int>(j-1) = ID;
                    areaEdgesTemp_ID.at<int>(j-1) = 0;
                    areaSize.at<int>(ID-1) = areaSize.at<int>(ID-1) + (areaEdgesTemp_xRight.at<int>(j-1) - areaEdgesTemp_xLeft.at<int>(j-1))+1;
                    areaWeight.at<int>(ID-1) = areaWeight.at<int>(ID-1) + (areaSize.at<int>(ID-1)*areaEdgesTemp_y.at<int>(j-1));

                    if(areaEdgesTemp_xLeft.at<int>(j-1) < areaLeft.at<int>(ID-1))
                    {
                        areaLeft.at<int>(ID-1) = areaEdgesTemp_xLeft.at<int>(j-1);
                    }
                    if(areaEdgesTemp_xRight.at<int>(j-1) > areaRight.at<int>(ID-1))
                    {
                        areaRight.at<int>(ID-1) =areaEdgesTemp_xRight.at<int>(j-1) ;
                    }
                    if(areaEdgesTemp_y.at<int>(j-1) < areaTop.at<int>(ID-1))
                    {
                        areaTop.at<int>(ID-1) = areaEdgesTemp_y.at<int>(j-1);
                    }
                    if(areaEdgesTemp_y.at<int>(j-1) > areaBottom.at<int>(ID-1))
                    {
                        areaBottom.at<int>(ID-1) = areaEdgesTemp_y.at<int>(j-1);
                    }
                }
            }
            /*Confidence is measured by adding the square root of size and the width, height of the area. */
            areaConf.at<float>(ID-1) = ((float) pow((double) areaWeight.at<int>(ID-1), 0.5) + (float) pow((double) areaSize.at<int>(ID-1), 0.5)) + (areaBottom.at<int>(ID-1)-areaTop.at<int>(ID-1)) + (areaRight.at<int>(ID-1) - areaLeft.at<int>(ID-1)) - ((imgRows-areaBottom.at<int>(ID-1))*imgCols);
            //ID = ID + 1;
        }
    }

    areaNumAreas = ID;
    areaSize2 = areaSize.clone();
    areaEdges_ID2 = areaEdges_ID.clone();
    areaLeft2 = areaLeft.clone();
    areaRight2 = areaRight.clone();
    areaTop2 = areaTop.clone();
    areaBottom2 = areaBottom.clone();
    areaConf2 = areaConf.clone();
    areaWeight2 = areaWeight.clone();
    areaNumAreas2 = areaNumAreas;
    //cout << "areaSize2= "<< endl << " " << areaSize2 << endl << endl;
    //cout << "areaSize2= "<< endl << " " << areaSize2 << endl << endl;



}
void DetectArea::ReleaseMemory()
{
    areaEdgesTemp_ID.release();
    areaEdgesTemp_y.release();
    areaEdgesTemp_xLeft.release();
    areaEdgesTemp_xRight.release();

    areaSize.release();
    areaLeft.release();
    areaRight.release();
    areaTop.release();
    areaBottom.release();
    areaConf.release();
    areaWeight.release();
}
