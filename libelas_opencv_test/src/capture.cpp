#include <opencv2\opencv.hpp>
#include <iostream>
#include <omp.h> 
using namespace cv;
using namespace std;

Mat dualCap (int* camid, string camcalibfile)
{
	Mat img[2], imggray[2];
	VideoCapture cap[2];
	cap[1].set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
	cap[1].set(CV_CAP_PROP_FRAME_WIDTH, 1920);
	cap[1].set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
	cap[2].set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
	cap[2].set(CV_CAP_PROP_FRAME_WIDTH, 1920);
	cap[2].set(CV_CAP_PROP_FRAME_HEIGHT, 1080);

	for (int i = 1; i <= sizeof(cap); i++)
	{
		cap[i].open(camid[i]);
		cap[i]>>img[i];
		cvtColor(img[i], imggray[i], COLOR_RGB2GRAY);
	}
	return imggray[1];
}