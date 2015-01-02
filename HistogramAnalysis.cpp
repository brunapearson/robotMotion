//Include Files
#include "HistogramAnalysis.h"
#include <string>
#include <iostream>
#include <fstream>
#include <vector>

//namespace declaration
using namespace cv;
using namespace std;


//! Class constructor
HistogramAnalysis::HistogramAnalysis()
{

}

//! Class destructor
HistogramAnalysis::~HistogramAnalysis()
{

}
Mat mSegmentLeft =  Mat::zeros(5,4, CV_16SC1);
Mat mSegmentRight =  Mat::zeros(5,4, CV_16SC1);

Mat pathData_LeftEdge = Mat::zeros(1,200,CV_32SC1);


int mSurface_pConf[15][4];
int mSurface_pShade[15][4];
int mSurface_pLeft[15][4];
int mSurface_pRight[15][4];
int mSurface_pStatus[15][4];

int q;

int frameNum = 0;
int row[43];
int counter = 0;


Mat filteredGrid = Mat::zeros(43, 64, CV_16SC4);
Mat frameProc = Mat::zeros(43,64, CV_32SC4);
Mat frameFused = Mat::zeros(43, 64, CV_16SC1);

//Mat finalFrameFusedArchive[5];
//vector<Mat> finalFrameFusedArchive;
//int finalFrameFusedArchive[5][43][64];

//Composite Image for processing
void HistogramAnalysis::CompositeImage(const Mat& image, Mat& hls_320x240, Mat& hls2_320x240, Mat& meanChroma_320x240, Mat& chromaTextureMap)
{
    /*ofstream myfile;
    myfile.open("finalFrameFusedObs.txt");
    myfile << image ;
    myfile.close();*/

    imageComp = Mat::zeros(image.size(), image.depth());
    vector<cv::Mat> channelsComp;

    channelsComp.push_back(hls_320x240);
    channelsComp.push_back(hls2_320x240);
    channelsComp.push_back(meanChroma_320x240);
    channelsComp.push_back(chromaTextureMap);

    merge(channelsComp, imageComp);

//build safe window for each colourspace
    //Rect region_of_interest = Rect(60,160,200,70);
    Rect region_of_interest = Rect(100,180,150,50);
    Mat imageCompROI = imageComp(region_of_interest);
    Mat hls_320x240ROI = hls_320x240(region_of_interest);
    Mat hls2_320x240ROI = hls2_320x240(region_of_interest);
    Mat meanChroma_320x240ROI = meanChroma_320x240(region_of_interest);
    Mat chromaTextureMapROI = chromaTextureMap(region_of_interest);
    imshow( "ROI" , hls2_320x240ROI);

//draw safe window for each colourspace
    rectangle( imageComp, Point( 100, 200 ), Point( 190, 235), Scalar( 255, 255, 255 ), +1, 4 );
    rectangle( hls_320x240, Point( 60, 160 ), Point( 258, 228), Scalar( 255, 255, 255 ), +1, 4 );
    rectangle( hls2_320x240, Point( 60, 160 ), Point( 258, 228), Scalar( 255, 255, 255 ), +1, 4 );
    rectangle( meanChroma_320x240, Point( 60, 160 ), Point( 258, 228), Scalar( 255, 255, 255 ), +1, 4 );
    rectangle( chromaTextureMap, Point( 60, 160 ), Point( 258, 228), Scalar( 255, 255, 255 ), +1, 4 );

    channelsComp[0].release();
    channelsComp[1].release();
    channelsComp[2].release();
    channelsComp[3].release();

    imageCompROI.release();
    hls_320x240ROI.release();
    hls2_320x240ROI.release();
    meanChroma_320x240ROI.release();
    chromaTextureMapROI.release();
//hls_320x240.release();
    hls2_320x240.release();
    meanChroma_320x240.release();
    chromaTextureMap.release();

imshow( "w" , imageComp);
//cout << "b3 = "<< endl << " " << imageComp << endl << endl;

}


void HistogramAnalysis::HistoryArchiveIndex()
{

    hIndex = hIndex + 1;
    if(hIndex > historySamples)
    {
        hIndex = 1;
    }
}

void HistogramAnalysis::VertLinesEdges()
{
    videoSizeFactor = (double)480/Y_resolution;
    cellWidth = (double)10/videoSizeFactor;

//populate scanVertLineEdges
    scanVertLineEdges[0] = 1;
    for(int i = 1; i < numGridCols; i++)
    {
        scanVertLineEdges[i] = scanVertLineEdges[i-1]+cellWidth;
    }
    scanVertLineEdges[numGridCols] = X_resolution+1;
}

void HistogramAnalysis::BlockElements()
{
    //populate scanBlockElements
    scanBlockElements = Mat::zeros(numGridRows, numGridCols, CV_16SC1);

    for(int i = 0; i < numGridRows; i++)
    {
        for(int j = 0; j < numGridCols; j++)
        {
            if(i > 41)
            {
                for(int k = scanHorizLineEdges[i-1]; k < scanHorizLineEdges[i]; k++)
                {
                    for(int l = scanVertLineEdges[j]; l <= scanVertLineEdges[j+1]-1; l++)
                    {
                        scanBlockElements.at<short>(i,j) = scanBlockElements.at<short>(i,j)+1;
                    }
                }
            }
            else
            {

                for(int k = scanHorizLineEdges[i]; k < scanHorizLineEdges[i+1]; k++)
                {
                    for(int l = scanVertLineEdges[j]; l <= scanVertLineEdges[j+1]-1; l++)
                    {
                        scanBlockElements.at<short>(i,j) = scanBlockElements.at<short>(i,j)+1;
                    }
                }
            }
        }
    }


}

void HistogramAnalysis::Histogram(const Mat& image, int YPixels, int XPixels)
{
    //imshow( "w" , image);
    MinDetectionRange_Pixels = YPixels - safeWindowBottomInset;
    XMax = XPixels - safeWindowRightInset;
    YMax = MinDetectionRange_Pixels;

    numChans = 4;//4

    frameLowResolution = Mat::zeros(numGridRows,numGridCols, CV_32SC4);

    vector<Mat> tempChannels(4);

//split composite image
    split(image,tempChannels);
    inputFrame = Mat::zeros(1,76800, CV_8UC1);
    int tempOffset = 0;

    int pathShades_Shade[15][4]= {0};
    int pathShades_Conf[15][4]= {0};
    int pathShades_Left[15][4]= {0};
    int pathShades_Right[15][4];

    for(int i = 0; i < 15; ++i)
    {
        for(int j = 0; j < 4; j++)
        {
            pathShades_Right[i][j] = 320;
        }
    }

    //for(int f = 0; f < numChans; f++)
    for(int f = 0; f < 2; f++)
    {
        // create inputFrame 76800x1 for each channel
        int counter = 0;
        Mat ch = tempChannels[f].clone(); //temporary value of "3-f" in order to change from RGB to BGR

         for(int i = 0; i < image.rows; i++)
         {
             for(int j=0; j < image.cols; j++)
             {
                 inputFrame.at<uchar>(0,counter) = ch.at<uchar>(i,j);
                 counter = counter+1;
             }
         }
        /*if(f==0)
        {
            for(int i=0; i < 76800; i++)
            {
                inputFrame.at<uchar>(i) = image.at<Vec4b>(i)[0];
            }
        }
        if(f==1)
        {
            for(int i=0; i < 76800; i++)
            {
                inputFrame.at<uchar>(i) = image.at<Vec4b>(i)[1];
            }
        }
        if(f==2)
        {
            for(int i=0; i < 76800; i++)
            {
                inputFrame.at<uchar>(i) = image.at<Vec4b>(i)[2];
            }
        }
        if(f==3)
        {
            for(int i=0; i < 76800; i++)
            {
                inputFrame.at<uchar>(i) = image.at<Vec4b>(i)[3];
            }
        }*/
        /*ofstream myfile;
        myfile.open("finalFrameFusedObs.txt");
        myfile << inputFrame ;
        myfile.close();*/

        /* Create low resolution image */
        for(int i = 0; i < numGridRows; i++)
        {
            for(int j = 0; j < numGridCols; j++)
            {
                double sumElements = 0.0;
                for(int k = scanHorizLineEdges[i]; k <= scanHorizLineEdges[i+1]-1; k++)
                {
                    tempOffset = (k-1)*X_resolution;
                    for(int l = scanVertLineEdges[j]; l <= scanVertLineEdges[j+1]-1; l++ )
                    {
                        sumElements = sumElements + inputFrame.at<uchar>((l-1) + tempOffset);
                    }
                }//end k loop
                frameLowResolution.at<Vec4i>(i,j)[f] = round((double)sumElements/scanBlockElements.at<short>(i,j));
            }
        }

        //Display frameLowResolution
        /*vector<Mat> frameLR(4);

        //split composite image
        split(frameLowResolution,frameLR);
        imshow("Channel 0", frameLR[0]);
        imshow("Channel 1", frameLR[1]);
        imshow("Channel 2", frameLR[2]);
        imshow("Channel 3", frameLR[3]);*/

// generate histogram of the safe window [c,r]
        mHistGrayscale = Mat::zeros(256,1, CV_32SC1); // the original matlab files is using uint32. However opencv does not support 32U - not sure what to do about it
        int tempOffset_2 = 0;

        //cout << "b1 = "<< endl << " " << inputFrame << endl << endl;

        for(int i = YMin; i < YMax; i++)
        {
            tempOffset_2 = (i-1)*(double)X_resolution;
            for(int j = XMin; j < XMax; j++)
            {
                //cout << "a " << j-1 << " b " << tempOffset_2 << " c " << int(inputFrame.at<uchar>((j-1)+tempOffset_2)) << " d " << (mHistGrayscale.at<int>(inputFrame.at<uchar>((j-1)+tempOffset_2))+1)  << endl;
                mHistGrayscale.at<int>(inputFrame.at<uchar>((j-1)+tempOffset_2)) = (mHistGrayscale.at<int>(inputFrame.at<uchar>((j-1)+tempOffset_2))+1);
                //cout << "j "<<j <<" d " << mHistGrayscale.at<int>(inputFrame.at<uchar>(j+tempOffset_2))+1 << endl;
               // cout << "b1 = "<< endl << " " << mHistGrayscale << endl << endl;
            }
        }
       // cout << "b1 = "<< endl << " " << mHistGrayscale << endl << endl;

        mHistLowRes =  Mat::zeros(histRes,1, CV_32SC1);//in the original matlab files this is uint32 as well

        for(int i = 1; i <= histRes; i++)
        {
            int sumElements_2 = 0;
            //for(int j = (((histScaleFactor*(i+1))-7)-1); j < (histScaleFactor*(i+1)); j++)
            for(int j = ((histScaleFactor*i)-7); j <= (histScaleFactor*i); j++)
            {
                sumElements_2 = sumElements_2 + mHistGrayscale.at<int>(j-1);
                //cout << sumElements_2 << endl;
            }
            mHistLowRes.at<int>(i-1) = round((double)sumElements_2/histScaleFactor);
        }

        //cout << "b1 = "<< endl << " " << mHistLowRes << endl << endl;

        /* get histMax */
        minMaxLoc(mHistLowRes, &histLow, &histMax, 0, 0);

        averageHistLevel = round((double)histMax/3);
        minHistThres = round((double)averageHistLevel/2);
        maxHistThres = round((double)averageHistLevel + ((double)averageHistLevel - minHistThres));

        //cout << "a " <<  histMax << " b " << averageHistLevel <<  " c " << minHistThres << " d " << maxHistThres << endl;

//Calculate Grayscale Segment. A segment is the area created by two
//points, where the histogram coincides with the average level line
//(noted with an 'o'). However, the edges of a segment are the troughs
//either side of the peaks (noted with a '*').
        //                       /\         /\
        //                      /  \       /  \
        //                 ----o----o-----o----o---- Average Level
        //                    /      \   /      \
        //                  */        \*/        \*

        segmentFound = false;

        prevSegmentEdge = 1;

        for(int i = 1; i <= histRes; i++)
        {
            int cellNum;

            if((mHistLowRes.at<int>(i-1) > maxHistThres) && segmentFound == false)
            {
                //cellNum = i+1;
                cellNum = i;

                for(int k = i; k --> 1;)
                {

                    if(mHistLowRes.at<int>(k-1) <= minHistThres || k == prevSegmentEdge || mHistLowRes.at<int>(k-1) > maxHistThres || k == 1)
                    {
                        cellNum = k;
                        break;
                    }
                }
                if(cellNum <= 1)
                {
                    mSegmentLeft.at<short>(hIndex-1,f) = 0;
                }
                else
                {
                    mSegmentLeft.at<short>(hIndex-1,f) = (cellNum * histScaleFactor);
                }
                mSegmentRight.at<short>(hIndex-1,f) = histSize - 1; //temporarily set the end of the segment to maximum

                segmentFound = true;

            }
            else if( mHistLowRes.at<int>(i-1) < minHistThres && segmentFound == true)
            {
                cellNum = i;
                mSegmentRight.at<short>(hIndex-1,f) = max(0,((cellNum*histScaleFactor)-1));
                prevSegmentEdge = cellNum;
                segmentFound = false;
            }
        }

        //cout << "b1 = "<< endl << " " << mSegmentRight << endl << endl;

        double couterSegmentLeft[4] = {0.0};
        double couterSegmentRight[4] = {0.0};

        for(int i = 0; i < mSegmentLeft.rows; i++ )
        {
            couterSegmentLeft[f] += mSegmentLeft.at<short>(i,f);
            couterSegmentRight[f] += mSegmentRight.at<short>(i,f);

        }
        mSegmentLeftAvg = ceil((double)couterSegmentLeft[f]/mSegmentLeft.rows)-1;
        mSegmentRightAvg = floor((double)couterSegmentRight[f]/mSegmentRight.rows)+1;

        int YPos = floor(4500/videoSizeFactor);
        YPos = YPos - 400;

        /*Display Histogram for each channel */
        /*if(f==0)
        {
            cout << "Saturation " << endl;
            cout << "L: " << mSegmentLeftAvg << " R: " << mSegmentRightAvg << endl;
            //cout << "CH 0 = "<< endl << " " << mHistLowRes << endl << endl;

        }
        if(f==1)
        {
            cout << "Saturation-based Texture Density " << endl;
            cout << "L: " << mSegmentLeftAvg << " R: " << mSegmentRightAvg << endl;
            //cout << "CH 1 = "<< endl << " " << mHistLowRes << endl << endl;
        }
        if(f==2)
        {
            cout << "Hue-Variance " << endl;
            cout << "L: " << mSegmentLeftAvg << " R: " << mSegmentRightAvg << endl;
            //cout << "CH 2 = "<< endl << " " << mHistLowRes << endl << endl;
        }
        if(f==3)
        {
            cout << "Variance-based Texture Density " << endl;
            cout << "L: " << mSegmentLeftAvg << " R: " << mSegmentRightAvg << endl;
            //cout << "CH 3 = "<< endl << " " << mHistLowRes << endl << endl;
        }*/


        /*
        Track Surface Shades
        Segmentation of the image results into a set of dominant shades.
        Eventually, one of these shades will be part of the pathway Surface.
        However, some of these shades may also be instanteneous due to noise or
        other factors. This algorithm tracks the behaviour of each shade over time.
        If the shade appears consistently then the algorithm registers it as a possible surface shade.
        */

//Reset status of all surfaces to false

        for(int i = 0; i < 15; i++)
        {
            mSurface_pStatus[i][f] = 0;
        }

        shade = round((double)(mSegmentLeftAvg+mSegmentRightAvg)/2);
        flag = false;

        shadeErrorMargin = 24;

        /*Search for current shade in historical data */
        for(int j = 0; j < noSurfaces; j++)
        {
            //cout << "a " << shade << " b " << mSurface_pShade[j][f] <<  " c " << shadeErrorMargin << endl;
            if(shade > (mSurface_pShade[j][f] - shadeErrorMargin) && shade < (mSurface_pShade[j][f] + shadeErrorMargin))
            {
                //cout << "a " << mSurface_pRight[j][f] << " b " << j+1 << " c " << f+1 << endl;
                //mSurface_pShade[j][f] = floor((double)(mSurface_pShade[j][f]+shade)/2);
                mSurface_pShade[j][f] = ceil((double)(mSurface_pShade[j][f]+shade)/2.0);
                mSurface_pConf[j][f] = min(maxPathConfidence,(mSurface_pConf[j][f]+1));
                mSurface_pLeft[j][f] = floor((mSurface_pLeft[j][f]+mSegmentLeftAvg)/2);
                mSurface_pRight[j][f] = floor((mSurface_pRight[j][f]+mSegmentRightAvg)/2);
                mSurface_pStatus[j][f] = 1;
                flag = true;
                break;
            }
        }



        /* If the current shade is seen for the first time, register it with age = 1 */
        if(flag == false)
        {
            for(int index = 0; index < noSurfaces; index++)
            {
                /*find next free slot to store the new surface (free slots have zero confidence. */
                if(mSurface_pConf[index][f] == 0)
                {
                    mSurface_pConf[index][f] = 1;
                    mSurface_pShade[index][f] = shade;
                    mSurface_pLeft[index][f] = mSegmentLeftAvg;
                    mSurface_pRight[index][f] = mSegmentRightAvg;
                    mSurface_pStatus[index][f] = 1;
                    break;
                }
            }
        }

        /*If the confidence of the surface has not increased then it shall get decreased by 2 points. */
        maxConf = 0;

        for(int i = 0; i < noSurfaces ; i++)
        {
            //Look for active
            if(mSurface_pConf[i][f]>0)
            {
                if(mSurface_pStatus[i][f] == 0)
                {
                    mSurface_pConf[i][f] = max(0,((mSurface_pConf[i][f])-2));
                }
                //Delete Surfaces with zero confidence
                if(mSurface_pConf[i][f] == 0)
                {
                    mSurface_pConf[i][f] = 0;
                    mSurface_pShade[i][f] = 0;
                    mSurface_pLeft[i][f] = 0;
                    mSurface_pRight[i][f] = 0;
                }
                if(mSurface_pConf[i][f] > maxConf)
                {
                    maxConf = mSurface_pConf[i][f];
                }
            }
        }

        /*for(int i = 0; i < 15; i++)
         {
             for(int j =0; j < 4; j++)
             {
                 cout << mSurface_pConf[i][j] <<" " << mSurface_pShade[i][j] << " "<< mSurface_pLeft[i][j] << " " << mSurface_pRight[i][j] << " " <<mSurface_pStatus[i][j] << endl;
             }
             cout << " " << endl;
         }*/


//Print surface information
        /*for(int i = 0; i < noSurfaces; i++)
        {
            if(mSurface_pShade[i][f] > 0)
            {
                YPos = YPos - 400;
                cout <<"Channel 1 - " <<"Shade: " << mSurface_pShade[i][0] << " Age: " << mSurface_pConf[i][0] << endl;
                cout <<"Channel 2 - " <<"Shade: " << mSurface_pShade[i][1] << " Age: " << mSurface_pConf[i][1] << endl;
                cout <<"Channel 3 - "  <<"Shade: " << mSurface_pShade[i][2] << " Age: " << mSurface_pConf[i][2] << endl;
                cout <<"Channel 4 - "  <<"Shade: " << mSurface_pShade[i][3] << " Age: " << mSurface_pConf[i][3] << endl;
                YPos = YPos - 400;
            }
        }*/
        /*Calculate maximum path age */
        q = 0;
        int thres;
        pathAgeThres = 10;

        thres = min(pathAgeThres, maxConf);

        for(int i = 0; i < noSurfaces; i++)
        {
            if(mSurface_pConf[i][f] >=thres)
            {
                pathShades_Shade[q][f] = mSurface_pShade[i][f];
                pathShades_Conf[q][f] = mSurface_pConf[i][f];
                pathShades_Left[q][f] = mSurface_pLeft[i][f];
                pathShades_Right[q][f] = mSurface_pRight[i][f];
                q = q+1;
            }
        }

        /*If no surface found then search again with lower threshold */
        if(q==0)
        {
            thres = (pathAgeThres/2);
            for(int i = 0; i < noSurfaces; i++)
            {
                if(mSurface_pConf[i][f] >= thres)
                {
                    pathShades_Shade[q][f] = mSurface_pShade[i][f];
                    pathShades_Conf[q][f] = mSurface_pConf[i][f];
                    pathShades_Left[q][f] = mSurface_pLeft[i][f];
                    pathShades_Right[q][f] = mSurface_pRight[i][f];
                    q = q+1;
                }
            }
        }

        /* for(int i = 0; i < 15; i++)
         {
             for(int j =0; j < 4; j++)
             {
                 cout << pathShades_Right[i][j] << " ";
             }
             cout << endl;
         }*/

        for(int i = 0; i < numGridRows; i++)
        {
            for(int j = 0; j < numGridCols; j++)
            {
                frameProc.at<Vec4i>(i,j)[f] = frameLowResolution.at<Vec4i>(i,j)[f];
            }
        }

        /*Black out all grid cells with grayscale shade outside the candidate path
        shades calculated above. Here the filteredGrid is also introduced. Each cell
        in this grid corresponds to a cell in the frameSegmLoReM. Only if a cell is satured then it is
        marked as path or no-path. Otherwise, it is set to an VideoParameter UNKNOWN state. */

        //cout << "f ----------------------- " << f << endl;

        for(int i = 0; i < numGridRows; i++)
        {
            for(int j = 0; j < numGridCols; j++)
            {
                flag = false;
                for(int k = 0; k < q; k++)
                {
                    if(frameProc.at<Vec4i>(i,j)[f] >= pathShades_Left[k][f] && frameProc.at<Vec4i>(i,j)[f] < pathShades_Right[k][f])
                    {
                        filteredGrid.at<Vec4s>(i,j)[f] = max(-3, (filteredGrid.at<Vec4s>(i,j)[f]-1));
                        if(filteredGrid.at<Vec4s>(i,j)[f] <= 1)
                        {
                            frameProc.at<Vec4i>(i,j)[f] = path_Shade;
                            frameFused.at<short>(i,j) = min(3, (frameFused.at<short>(i,j)+1));
                        }
                        else
                        {
                            frameProc.at<Vec4i>(i,j)[f] = obstShade;
                            frameFused.at<short>(i,j) = max(-3, (frameFused.at<short>(i,j)-1));
                        }
                        flag = true;
                        break;
                    }
                }
                if(flag == false)
                {
                    filteredGrid.at<Vec4s>(i,j)[f] = min(3,(filteredGrid.at<Vec4s>(i,j)[f]+1));
                    if(filteredGrid.at<Vec4s>(i,j)[f] > 1)
                    {
                        frameProc.at<Vec4i>(i,j)[f] = obstShade;
                        frameFused.at<short>(i,j) = max(-10, (frameFused.at<short>(i,j)-1));
                    }
                    else
                    {
                        frameProc.at<Vec4i>(i,j)[f] = path_Shade;
                        frameFused.at<short>(i,j) = min(10,(frameFused.at<short>(i,j)+1));
                    }
                }
            }
        }

        //cout << "b1 = "<< endl << " " << frameFused << endl << endl;

        double safeWindowSize = (safeWindowTopInset-safeWindowBottomInset) * (X_resolution - safeWindowRightInset - safeWindowLeftInset);

        tempSurface_HistHeight[f] = floor((double(histMax)/safeWindowSize)*255)*10;
        tempSurface_HistSegmLeft[f] = pathShades_Left[0][f];
        tempSurface_HistSegmRight[f] = pathShades_Right[noSurfaces-1][f];

        //Size size(100,100);//the dst image size,e.g.100x100
        //Mat dst;//dst image

        /*if(f==0)
        {
            //resize(frameProc,dst,size);//resize image
            imshow("frameProc 0", frameProc);
            //cout << "b0 = "<< endl << " " << frameFused << endl << endl;
        }
        else if(f==1)
        {
            //resize(frameProc,dst,size);//resize image
            imshow("frameProc 1", frameProc);
            //cout << "b1 = "<< endl << " " << frameFused << endl << endl;
        }
        else if(f==2)
        {
            //resize(frameProc,dst,size);//resize image
            imshow("frameProc 2", frameProc);
            //cout << "b2 = "<< endl << " " << frameFused << endl << endl;
        }
        else if(f==3)
        {
            //resize(frameProc,dst,size);//resize image
            imshow("frameProc 3", frameProc);
            //cout << "b3 = "<< endl << " " << frameFused << endl << endl;
        }*/

    } //end for loop

} //end Histogram

void HistogramAnalysis::ReleaseMemory()
{
    scanBlockElements.release();
    frameLowResolution.release();
    inputFrame.release();
    mHistGrayscale.release();
    mHistLowRes.release();
    mSegmentLeft.release();
    mSegmentRight.release();
}

void HistogramAnalysis::ClassifySurface()
{


    /* Classify Surface */
    flag = false;
    string currLabel[1] = {"Unknown"}; //ths is never used anywhere else in the code.
    int minDiff = 100000;
    int curIndex = 1;
    int noSurfaceTrainingSamples = 100;
    Mat surfaceType_HistHeight = Mat::zeros(noSurfaceTrainingSamples,numChans, CV_32SC1);
    Mat surfaceType_HistSegmLeft = Mat::zeros(noSurfaceTrainingSamples,numChans, CV_32SC1);
    Mat surfaceType_HistSegmRight = Mat::zeros(noSurfaceTrainingSamples,numChans, CV_32SC1);
    surfaceType_Diff = Mat::zeros(noSurfaceTrainingSamples,1, CV_32SC1);
    Mat surfaceType_Status = Mat::zeros(noSurfaceTrainingSamples,1, CV_32SC1);
    Mat surfaceType_Label = Mat::zeros(1,noSurfaceTrainingSamples, CV_32SC1);

    for(int i = 0; i < noSurfaceTrainingSamples; i++)
    {
        surfaceType_Diff.at<int>(i) = 0;
        for(int j = 0; j < numChans; j++)
        {
            surfaceType_Diff.at<int>(i) = surfaceType_Diff.at<int>(i) + abs(surfaceType_HistHeight.at<int>(i,j) - tempSurface_HistHeight[j]) + abs(surfaceType_HistSegmLeft.at<int>(i,j) - tempSurface_HistSegmLeft[j]) + abs(surfaceType_HistSegmRight.at<int>(i,j) - tempSurface_HistSegmRight[j]);
        }
        if(surfaceType_Diff.at<int>(i) < minDiff)
        {
            minDiff = surfaceType_Diff.at<int>(i);
            /*the block bellow is not used by any other function. */
            std::ostringstream ostr;
            int number = surfaceType_Label.at<int>(0,i); //output string stream
            ostr << number; //use the string stream just like cout,
            //except the stream prints not to stdout but to a string.

            std::string theNumberString = ostr.str(); //the str() function of the stream
            //returns the string.
            currLabel[0] = theNumberString;
            curIndex = i;
            /* end of block */
        }
    }
    //cout << "b1 = "<< endl << " " << surfaceType_Diff << endl << endl;
//Remap Fused frame values T
//Remove logical indexing
}

//void HistogramAnalysis::Remap(Mat inputFrameFused)
void HistogramAnalysis::Remap()
{
    //cout << "b1 = "<< endl << " " << inputFrameFused << endl << endl;
    finalFrameFused = Mat::zeros(numGridRows, numGridCols, CV_8UC1);
    //finalFrameFused = Mat::zeros(numGridCols, numGridRows, CV_8UC1);

    /* in the original code the loop is different. Check line 654. */
    //cout << "value " << inputFrameFused.size()<< endl;

    int counter= 0;
    int counter2= 0;
    /*for(int i = 0; i < inputFrameFused.rows; i++)
        {
                     for(int j = 0; j < inputFrameFused.cols; j++)
                     {
                         //cout << "i " << i << " j " << j << " a " << a << endl;
                         //inputFrameFused.at<int>(i,j) = a;
                         cout <<counter2 <<  " i " << i << " j " << j  << " " << inputFrameFused.at<int>(counter2) << endl ;
                         counter2 = counter2+1;
                     }
        }*/



    for(int i = 0; i < frameFused.rows; i++)
    {
        for(int j=0; j < frameFused.cols; j++)
        {
            //cout << counter << " -------   " <<inputFrameFused.at<short>(counter) << endl;
            if(frameFused.at<short>(counter) > 0)
            {
                finalFrameFused.at<uchar>(i,j) = path_Shade;
            }
            if(frameFused.at<short>(counter) <= 0)
            {
                finalFrameFused.at<uchar>(i,j) = obstShade;
            }
            counter = counter+1;
        }
    }
    //cout << "-------   " <<inputFrameFused.at<int>(1441) << endl;
    //cout << "b1 = "<< endl << " " << finalFrameFused << endl << endl;

    /*ofstream myfile;
    myfile.open("finalFrameFusedObs.txt");
    myfile << finalFrameFused ;
    myfile.close();*/

    /* Detect Paht Area - Ieration 1 */
    maxPathLines=200;
    //imshow("final frame ", frameFused);
    //frameFused.release();

}

void HistogramAnalysis::Segmentation(Mat finalFrameFusedSeg, int pathEdges_NumSegm, Mat pathEdges_y, Mat path_Size, Mat pathEdges_ID, Mat pathEdges_xLeft, Mat pathEdges_xRight)
{

    /*ofstream myfile;
    myfile.open("finalFrameFusedObs.txt");
    myfile << finalFrameFusedSeg ;
    myfile.close();*/

    /*filter out path segments that are cleary out of the path (i.e. too short, very small gap between obstacles) */
    finalFrameFusedSeg2 = Mat::zeros(numGridRows, numGridCols, CV_8UC1);
    int y;
    minPathSize = 125;

    //finalFrameFusedSeg = finalFrameFused.clone();
    // cout << "pathEdges_xLeft= "<< endl << " " << pathEdges_xLeft << endl << endl;

    for(int i = pathEdges_NumSegm; i --> 0;)
    {
        //cout << "a " << i << " b " << pathEdges_ID.at<int>(i)-1 << " c " << path_Size.at<int>(pathEdges_ID.at<int>(i)-1) << endl;
        if(path_Size.at<int>(pathEdges_ID.at<int>(i)-1) < minPathSize)
        {
            //cout << "a " << i << " b " << pathEdges_xLeft.at<int>(i) << endl;
            for(int j = pathEdges_xLeft.at<int>(i); j <= pathEdges_xRight.at<int>(i); j++)
            {
                //cout << "a "<< j-1 << " b " << pathEdges_y.at<int>(i)-1 << endl;
                finalFrameFusedSeg.at<uchar>(pathEdges_y.at<int>(i)-1,j-1) = obstShade;

            }
        }
    }
    //cout << "finalFrameFusedSeg= "<< endl << " " << finalFrameFusedSeg << endl << endl;
    /*ofstream myfile;
    myfile.open("finalFrameFusedObs.txt");
    myfile << finalFrameFusedSeg ;
    myfile.close();*/

    double widthsum;
    double gapsum;
    int minPathWidth = 4;
    double minRatio = 0.4;

    /*filter out path segments that are cleary out of the path */
    for(int i = pathEdges_NumSegm; i --> 0;)
    {

        y = pathEdges_y.at<int>(i);
        //cout << y << endl;

        if( y < numGridRows) //discard the bottom row
        {
            widthsum = 0.0;
            gapsum = 0.0;
            for( int j = pathEdges_xLeft.at<int>(i); j <= pathEdges_xRight.at<int>(i); j++)
            {
                widthsum = widthsum + (double)finalFrameFusedSeg.at<uchar>(y-1,j-1);
                gapsum = gapsum + (double)finalFrameFusedSeg.at<uchar>(y,j-1);
            }
            double Width = (double)widthsum/path_Shade;
            double Gap = (double)gapsum/path_Shade;
            double Ratio = (double)Gap/Width;

            if( Gap < minPathWidth || Ratio < minRatio)
            {
                for(int j = pathEdges_xLeft.at<int>(i); j <= pathEdges_xRight.at<int>(i); j++)
                {
                    finalFrameFusedSeg.at<uchar>(y-1,j-1) = obstShade;
                    finalFrameFusedSeg.at<uchar>(y,j-1) = obstShade;
                }
            }
        }
    }
    finalFrameFusedSeg2 = finalFrameFusedSeg.clone();
    /*ofstream myfile;
    myfile.open("finalFrameFusedObs.txt");
    myfile << finalFrameFusedSeg2 ;
    myfile.close();*/

    //cout << "b3 = "<< endl << " " << finalFrameFusedSeg << endl << endl;

//frameFused.release();
}
/* need to revise this bit */
void HistogramAnalysis::DetectObstacles(Mat finalFrameFusedObs,int obstacleEdges_NumSegm, Mat obstacle_Size, Mat obstacleEdges_ID, Mat obstacleEdges_xRight, Mat obstacleEdges_xLeft, Mat obstacleEdges_y)
{
    /*ofstream myfile;
    myfile.open("finalFrameFusedObs.txt");
    myfile << finalFrameFusedObs ;
    myfile.close();*/

    finalFrameFusedObs2 = Mat::zeros(numGridRows, numGridCols, CV_8UC1);

    //cout << "b3 = "<< endl << " " << obstacle_Size << endl << endl;
    /*Detect Obstacles */
    minObstacleSize = 5;
    for(int i = obstacleEdges_NumSegm; i --> 0;)
    {
        //cout <<"a "<< obstacleEdges_ID.at<int>(i) <<" b " << obstacle_Size.at<int>(obstacleEdges_ID.at<int>(i)-1) << " c " << i << endl;
        if(obstacle_Size.at<int>(obstacleEdges_ID.at<int>(i)-1) < minObstacleSize || (obstacleEdges_xRight.at<int>(i) - obstacleEdges_xLeft.at<int>(i)) <= 1)
        {
            for(int j = obstacleEdges_xLeft.at<int>(i); j <= obstacleEdges_xRight.at<int>(i); j++)
            {

                //finalFrameFusedObs.at<uchar>(obstacleEdges_y.at<int>(i)-1,j-1) = obstShade;
                finalFrameFusedObs.at<uchar>(obstacleEdges_y.at<int>(i)-1,j-1) = path_Shade;
            }
        }
    }
    /*ofstream myfile;
    myfile.open("finalFrameFusedObs.txt");
    myfile << finalFrameFusedObs ;
    myfile.close();*/
    //cout << "b3 = "<< endl << " " << finalFrameFusedObs << endl << endl;

    /*Store Fused Segmented Frame into archive
    M.FinalFrameArchive(:,:,hIndex) = M.FinalFrameFused;*/
    finalFrameFusedArchive[hIndex-1] = finalFrameFusedObs.clone();

    /* Adding the values for each row in finalFrameFusedArchive */

    int row2[43] = {0};
    int rowK = 0;

    for(int i=0; i <43; i++)
    {
        for(int j=0; j < 64; j++)
        {
            for(int k=0; k < hIndex; k++)
            {
                rowK = rowK + finalFrameFusedArchive[k].at<uchar>(i,j);
            }
            finalFrameFusedObs.at<uchar>(i,j) = round(rowK/5);
            rowK = 0;

        }

    }

    /*ofstream myfile;
    myfile.open("finalFrameFusedObs.txt");
    myfile << finalFrameFusedObs ;
    myfile.close();*/
    // cout << "b3 = "<< endl << " " << finalFrameFusedObs2 << endl << endl;
    finalFrameFusedObs2 = finalFrameFusedObs.clone();

}
void HistogramAnalysis::ThresholdMap(Mat finalFrameFusedThres)
{
    //finalFrameFusedThres2 = Mat::zeros(numGridRows, numGridCols, CV_8UC1);

    /*Threshold Historical Map */
    for(int i = 0; i < finalFrameFusedThres.rows; i++)
    {
        for(int j = 0; j < finalFrameFusedThres.cols; j++)
        {
            if(finalFrameFusedThres.at<uchar>(i,j) > 100)
            {
                finalFrameFusedThres.at<uchar>(i,j) = path_Shade;
            }
            if(finalFrameFusedThres.at<uchar>(i,j) < 50)
            {
                finalFrameFusedThres.at<uchar>(i,j) = obstShade;
            }
        }
    }
    //cout << "b3 = "<< endl << " " << finalFrameFusedThres << endl << endl;
    finalFrameFusedThres2 = finalFrameFusedThres.clone();
}

void HistogramAnalysis::ObstacleCoord(int obstacleEdges_NumSegm, Mat obstacleEdges_ID, Mat obstacleEdges_xRight, Mat obstacleEdges_xLeft,Mat obstacleEdges_y)
{
    maxObstacles = 200;
    /*Convert Low-Res coords to Full-Res coords*/
    for(int i = 0; i < obstacleEdges_NumSegm; i++)
    {
        obstacleEdges_y.at<int>(i) = scanHorizLineEdges[obstacleEdges_y.at<int>(i)-1];
        if(obstacleEdges_xLeft.at<int>(0,i) == 1)
        {
            obstacleEdges_xLeft.at<int>(i) = scanVertLineEdges[obstacleEdges_xLeft.at<int>(i)-1];
        }
        else
        {
            obstacleEdges_xLeft.at<int>(i) = (scanVertLineEdges[obstacleEdges_xLeft.at<int>(i)-1] + scanVertLineEdges[obstacleEdges_xLeft.at<int>(i)]-1)/2;
        }
        if(obstacleEdges_xRight.at<int>(0,i) == numGridCols)
        {
            obstacleEdges_xRight.at<int>(0,i) = scanVertLineEdges[numGridCols]-1;
        }
        else
        {
            obstacleEdges_xRight.at<int>(0,i) = (scanVertLineEdges[obstacleEdges_xRight.at<int>(0,i)-1] + scanVertLineEdges[obstacleEdges_xRight.at<int>(0,i)]-1)/2;
        }
    }
    //cout << "b2 = "<< endl << " " << obstacleEdges_xRight << endl << endl;

    int finalIndex = max((obstacleEdges_NumSegm - (maxObstacles+1)), 1);

    obstacleData_NoOfObstacles = 0;
    obstacleData_yPixel = Mat::zeros(1,maxObstacles, CV_32SC1);
    obstacleData_LeftEdge = Mat::zeros(1, maxObstacles, CV_32SC1);
    obstacleData_RightEdge = Mat::zeros(1, maxObstacles, CV_32SC1);
    obstacleData_ID = Mat::zeros(1, maxObstacles, CV_32SC1);

    int counter = 0;

    for(int i = obstacleEdges_NumSegm; i --> 0;)
    {
        obstacleData_ID.at<int>(counter) = obstacleEdges_ID.at<int>(i);
        obstacleData_yPixel.at<int>(counter) = obstacleEdges_y.at<int>(i);
        obstacleData_LeftEdge.at<int>(counter) = obstacleEdges_xLeft.at<int>(i);
        obstacleData_RightEdge.at<int>(counter) = obstacleEdges_xRight.at<int>(i);
        counter = counter + 1;
    }
    obstacleData_NoOfObstacles = (obstacleEdges_NumSegm-finalIndex) + 1;

    //cout << "b2 = "<< endl << " " << obstacleData_NoOfObstacles << endl << endl;
}


void HistogramAnalysis::Coordinates(int pathEdges_NumSegm, Mat pathEdges_y, Mat pathEdges_xLeft, Mat pathEdges_xRight)
{
    //cout << "b2 = "<< endl << " " << pathEdges_y << endl << endl;
    //cout << "test "<< pathEdges_NumSegm << endl;

    /*Convert Low-Res coords to Full-Res coords */
    for(int i = 0; i < pathEdges_NumSegm; i++)
    {

        pathEdges_y.at<int>(i) = scanHorizLineEdges[pathEdges_y.at<int>(i)-1];
        //cout << "a "<< pathEdges_y.at<int>(i) << " b "<< i << " c " << scanHorizLineEdges[pathEdges_y.at<int>(i)-1] << endl;
        /*Path start/end is assumed to be in the middle of the block (i.e not at the edge) */
        if(pathEdges_xLeft.at<int>(i) == 1)
        {
            pathEdges_xLeft.at<int>(i) = scanVertLineEdges[pathEdges_xLeft.at<int>(i)-1];
        }
        else
        {
            //cout << "a " << pathEdges_xLeft.at<int>(i) << " b " << scanVertLineEdges[pathEdges_xLeft.at<int>(i)-1] << " c " << scanVertLineEdges[pathEdges_xLeft.at<int>(i)] << endl;
            pathEdges_xLeft.at<int>(i) = (scanVertLineEdges[pathEdges_xLeft.at<int>(i)-1] + scanVertLineEdges[pathEdges_xLeft.at<int>(i)] - 1)/2;
            //cout << "d " << pathEdges_xLeft.at<int>(i) << endl;
        }
        if(pathEdges_xRight.at<int>(i) == numGridCols)
        {
            pathEdges_xRight.at<int>(i) = scanVertLineEdges[numGridCols] - 1;
        }
        else
        {
            pathEdges_xRight.at<int>(i) = (scanVertLineEdges[pathEdges_xRight.at<int>(i)-1] + scanVertLineEdges[pathEdges_xRight.at<int>(i)]-1)/2;
            //cout << pathEdges_xRight.at<int>(i) << endl;
        }
    }
    //cout << "b3 = "<< endl << " " << pathEdges_xLeft << endl << endl;

    /*Store path edges ready for output */
    int index = 0;
    int ii = 0;
    int y = 0;
    pathDataNoOfLines = 0;
    pathData_yPixel = Mat::zeros(1,maxPathLines,CV_32SC1);
    //pathData_LeftEdge = Mat::zeros(1,maxPathLines,CV_32SC1);
    pathData_RightEdge = Mat::zeros(1,maxPathLines, CV_32SC1);
    int counter_j;


    while(ii < pathEdges_NumSegm)
    {

        index = index + 1;
        y = pathEdges_y.at<int>(ii);
        //cout << "i "<<ii << " y " << y << endl;
        pathData_yPixel.at<int>(index-1) = y;
        pathData_LeftEdge.at<int>(index-1) = pathEdges_xLeft.at<int>(ii);
        pathData_RightEdge.at<int>(index-1) = pathEdges_xRight.at<int>(ii);
        counter_j = ii;
        //for(int j = (counter_j+1); j < pathEdges_NumSegm; j++)
        for(int j = ii+1; j < pathEdges_NumSegm; j++)
        {
            //cout << "a " << j << endl;
            if(pathEdges_y.at<int>(j) == y)
            {
                pathData_RightEdge.at<int>(index-1) = pathEdges_xRight.at<int>(j);
                counter_j = j;
                ii = j;
                //break;
            }
            else
            {
                // ii++;
                break;
            }
            /*if(ii < j)
            {
              ii++;
            }*/
        }
        //index = index + 1;
        ii++;

    }

    pathDataNoOfLines = index;
    pathData_LeftEdge2 = pathData_LeftEdge.clone();
    //cout << "b3 = "<< endl << " " << pathDataNoOfLines << endl << endl;
}

