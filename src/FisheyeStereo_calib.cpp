#include "opencv2/core/core.hpp" 
#include "opencv2/imgproc/imgproc.hpp"  
#include "opencv2/calib3d/calib3d.hpp"  
#include "opencv2/highgui/highgui.hpp"  
#include <vector>  
#include <string>  
#include <algorithm>  
#include <iostream>  
#include <iterator>  
#include <stdio.h>  
#include <stdlib.h>  
#include <ctype.h>   
#include <opencv2/opencv.hpp>  
#include "cv.h"  
#include <cv.hpp>  
using namespace std;
using namespace cv;                                        
const int imageWidth = 640;                             
const int imageHeight = 480;
const int boardWidth = 8;
const int boardHeight = 6;
const int boardCorner = boardWidth * boardHeight;
                            

const float squareSize = 0.051; //actual square size in m
const int frameNumber =21;

string folder_ = "/home/shinan/Project/Calibration/Henry_stereo_30cm/";
string format_R = "right";
string format_L = "left";
const Size boardSize = Size(8, 6);
Size imageSize = Size(640, 480);
Mat R, T, E, F;
vector<Mat> rvecs;
vector<Mat> tvecs;                                        
 
vector<vector<Point2f>> imagePointL;                    
vector<vector<Point2f>> imagePointR;                    
vector<vector<Point3f>> objRealPoint;                  
 
vector<Point2f> cornerL;                              
vector<Point2f> cornerR;                               
Mat rgbImageL, grayImageL;
Mat rgbImageR, grayImageR;
Mat Rl, Rr, Pl, Pr, Q;                                   
Mat mapLx, mapLy, mapRx, mapRy;                         
Rect validROIL, validROIR;                              
/*fx 0 cx
0 fy cy
0 0  1*/
Mat cameraMatrixL = (Mat_<double>(3, 3) << 329.6097279444598, 0, 331.175152524404,
0, 328.1697074161112, 236.8864680415888,
0, 0, 1);

Mat distCoeffL = (Mat_<double>(5, 1) << -0.2754059638752426, 0.0585763687394319, 0.0006185554148720049, 1.117975415713642e-05,0);
/*
fx 0 cx
0 fy cy
0 0  1*/
Mat cameraMatrixR = (Mat_<double>(3, 3) << 329.6097279444598, 0, 330.6758941468899,
0, 328.1697074161112, 246.0393771650396,
0, 0, 1);
Mat distCoeffR = (Mat_<double>(5, 1) << -0.2673692728595115, 0.05249778508056966, -0.0004142975139702069, -5.86915650585915e-05, 0);

void calRealPoint(vector<vector<Point3f>>& obj, int boardwidth, int boardheight, int imgNumber, int squaresize)
{ 	
		vector<Point3f> imgpoint;	
		for (int rowIndex = 0; rowIndex < boardheight; rowIndex++)	
			{		
				for (int colIndex = 0; colIndex < boardwidth; colIndex++)		
				{			
					imgpoint.push_back(Point3f(rowIndex * squaresize, colIndex * squaresize, 0));		
				}	
		}	
		for (int imgIndex = 0; imgIndex < imgNumber; imgIndex++)	
		{		
			obj.push_back(imgpoint);	
		}
}

void outputCameraParam(void)                   
{
	FileStorage fs("/home/shinan/Project/Calibration/Henry_stereo_30cm/Fish_stereo_intrinsics.yml", FileStorage::WRITE);

	if (fs.isOpened())	
	{		
		fs << "cameraMatrixL" << cameraMatrixL << "cameraDistcoeffL" << distCoeffL << "cameraMatrixR" << cameraMatrixR << "cameraDistcoeffR" << distCoeffR;		
		fs.release();		
		cout << "cameraMatrixL=:" << cameraMatrixL << endl << "cameraDistcoeffL=:" << distCoeffL << endl << "cameraMatrixR=:" << cameraMatrixR << endl << "cameraDistcoeffR=:" << distCoeffR << endl;
	}	
	else	
	{		
		cout << "Error: can not save the intrinsics!!!!!" << endl;	
	}	
	fs.open("/home/shinan/Project/Calibration/Henry_stereo_30cm/Fish_stereo_extrinsics.yml", FileStorage::WRITE);
	if (fs.isOpened())	
	{		
		fs << "R" << R << "T" << T << "Rl" << Rl << "Rr" << Rr << "Pl" << Pl << "Pr" << Pr << "Q" << Q;
		cout << "R=" << R << endl << "T=" << T << endl << "Rl=" << Rl << endl << "Rr=" << Rr << endl << "Pl=" << Pl << endl << "Pr=" << Pr << endl << "Q=" << Q << endl;
		fs.release();	
	}	
	else		
		cout << "Error: can not save the extrinsic parameters\n";
}

int main(int argc, char* argv[])
{	
	Mat img;	
	int goodFrameCount = 1;
	cout << "Total Images is" << frameNumber << endl;
	while (goodFrameCount < frameNumber)	
	{
		std::string tmp_str = to_string(goodFrameCount);
		if (goodFrameCount < 10)
			tmp_str = '0' + to_string(goodFrameCount);
		cout <<"Current image is " << goodFrameCount << endl;

		string 	filenamel,filenamer;
        filenamel = folder_ + format_L+ tmp_str +".jpg";
        filenamer = folder_ + format_R+ tmp_str +".jpg";
		rgbImageL = imread(filenamel, -1);
        rgbImageR = imread(filenamer, CV_LOAD_IMAGE_COLOR);
        if(rgbImageL.channels()>=3)
        {
            cvtColor(rgbImageL, grayImageL, CV_BGR2GRAY);
            cvtColor(rgbImageR, grayImageR, CV_BGR2GRAY);
        }


		bool isFindL, isFindR;		
		isFindL = findChessboardCorners(rgbImageL, boardSize, cornerL);		
		isFindR = findChessboardCorners(rgbImageR, boardSize, cornerR);		
		if (isFindL == true && isFindR == true)
		{						
			cornerSubPix(grayImageL, cornerL, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1));	
			drawChessboardCorners(rgbImageL, boardSize, cornerL, isFindL);
			//imshow("chessboardL", rgbImageL);
			imagePointL.push_back(cornerL);

			cornerSubPix(grayImageR, cornerR, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1));	
			drawChessboardCorners(rgbImageR, boardSize, cornerR, isFindR);		
			//imshow("chessboardR", rgbImageR);
			imagePointR.push_back(cornerR);						
			goodFrameCount++;			
			cout << "The image" << goodFrameCount << " is good" << endl;	
		}		
		else		
		{			
			cout << "The image "<< goodFrameCount <<"is bad please try again" << endl;
			goodFrameCount++;
		}		

		if (waitKey(10) == 'q')		
		{			
			break;		
		}	
	}	

	calRealPoint(objRealPoint, boardWidth, boardHeight, frameNumber-1, squareSize);	
	cout << "cal real successful" << endl;
	double rms = cv::stereoCalibrate(objRealPoint, imagePointL, imagePointR,
		cameraMatrixL, distCoeffL,
		cameraMatrixR, distCoeffR,
		Size(imageWidth, imageHeight), R, T, E, F,
		CV_CALIB_USE_INTRINSIC_GUESS+
		CV_CALIB_SAME_FOCAL_LENGTH +
		CV_CALIB_RATIONAL_MODEL  +
        CV_CALIB_FIX_K3+ CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5+ CV_CALIB_FIX_K6,
		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 80, 1e-5));	
	cout << "Stereo Calibration done with RMS error = " << rms << endl;

	initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pr, imageSize, CV_32FC1, mapLx, mapLy); 	
	initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);	

	Mat ImageL, ImageR;
    Mat rectifyImageL2, rectifyImageR2;
	cout << "debug"<<endl; 
        for (int num = 1; num < frameNumber;num++)
	{
		string 	filenamel,filenamer;
        std::string tmp_str = to_string(num);
        if (num < 10)
            tmp_str = '0' + to_string(num);
		filenamel = folder_ + format_L+ tmp_str +".jpg";
        filenamer = folder_ + format_R+	tmp_str+".jpg";
        ImageL = imread(filenamel,-1);
        ImageR = imread(filenamer,-1);
        if(ImageL.empty()||ImageR.empty())
        {
            std::cout<<"Failed to open file"<<filenamel<<std::endl<<
                      "Or " <<filenamer<<std::endl;
            return -1;
        }


		remap(ImageL, rectifyImageL2, mapLx, mapLy, INTER_LINEAR);
		remap(ImageR, rectifyImageR2, mapRx, mapRy, INTER_LINEAR);
		imshow("rectifyImageL", rectifyImageL2);	
		imshow("rectifyImageR", rectifyImageR2);	

		outputCameraParam();
		Mat canvas;	double sf;	
		int w, h;	
		sf = 600. / MAX(imageSize.width, imageSize.height);	
		w = cvRound(imageSize.width * sf);
		h = cvRound(imageSize.height * sf);	
		canvas.create(h, w * 2, CV_8UC3);	

		//draw the left part
		Mat canvasPart = canvas(Rect(w * 0, 0, w, h));
		resize(rectifyImageL2, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
		Rect vroiL(cvRound(validROIL.x*sf), cvRound(validROIL.y*sf),
			        cvRound(validROIL.width*sf), cvRound(validROIL.height*sf));
		rectangle(canvasPart, vroiL, Scalar(0, 0, 255), 3, 8);
		cout << "Painted ImageL" << endl;

		//draw the right part
		canvasPart = canvas(Rect(w, 0, w, h));
		resize(rectifyImageR2, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
		Rect vroiR(cvRound(validROIR.x * sf), cvRound(validROIR.y*sf),		
			cvRound(validROIR.width * sf), cvRound(validROIR.height * sf));	
		rectangle(canvasPart, vroiR, Scalar(0, 255, 0), 3, 8);
		cout << "Painted ImageR" << endl;

		for (int i = 0; i < canvas.rows; i += 16)		
			line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);
		imshow("rectified", canvas);
        char c = (char)waitKey();
        if (c == 27 || c == 'q' || c == 'Q')
            break;
	}
	return 0;
}

