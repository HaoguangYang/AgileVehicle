#include <opencv2\opencv.hpp>
#include <iostream>
#include <string>
using namespace cv;
using namespace std;
void main()
{
	for (int i = 0;; i++)
	{
		string filename;
		cout << "please input files you want to open..." << endl;
		cin >> filename;
		Mat img = imread(filename,CV_LOAD_IMAGE_COLOR);
		if (img.empty())
		{
			cout << "error -- "<<filename<<", bad filename or directory"<<endl;
		}
		else
			imshow(filename, img);
		waitKey();
		//cout << img.size << img.data << endl;
	}
}