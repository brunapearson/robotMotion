//Include Files
#include "NormEdges.h"

//namespace declaration
using namespace cv;
using namespace std;

//! Class constructor
NormEdges::NormEdges()
{

}

//! Class destructor
NormEdges::~NormEdges()
{

}

void NormEdges::NormaliseEdges(Mat array1D, int arrayLength, int windowSize)
{
    //cout << "b4 = "<< endl << " " << array1D << endl << endl;
    offset = floor(windowSize/2);

    //cout << offset << endl;
    int addValues = 0;
    subArray = Mat::zeros(1,windowSize,CV_32SC1);
    finalArray = Mat::zeros(1,arrayLength,CV_32SC1);

    for(int i = (offset+1); i < (arrayLength-offset)+1; i++)
    {
        int startSelection = (i-offset)-1;
        int endSelection = (i+offset)-1;
        int sizeSubArray = (endSelection - startSelection)+1;

        //cout << "a " << startSelection << " b " << endSelection << endl;
        /*Populate subArray */
        for(int j = startSelection; j <= endSelection; j++)
        {
        //cout << j << endl;
           addValues = addValues + array1D.at<int>(j);
        }
        //array1D.at<int>(i-1) = round(addValues/sizeSubArray); //matlab 1-3 :: opencv 0-2
        array1D.at<int>(i-1) = round((double)addValues/windowSize);
        addValues = 0;
    }
    finalArray = array1D.clone();
    //cout << "b4 = "<< endl << " " << finalArray << endl << endl;
}
