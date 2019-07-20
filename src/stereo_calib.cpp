/* This is sample from the OpenCV book. The copyright notice is below */

/* *************** License:**************************
   Oct. 3, 2008
   Right to use this code in any way you want without warranty, support or any guarantee of it working.

   BOOK: It would be nice if you cited it:
   Learning OpenCV: Computer Vision with the OpenCV Library
     by Gary Bradski and Adrian Kaehler
     Published by O'Reilly Media, October 3, 2008

   AVAILABLE AT:
     http://www.amazon.com/Learning-OpenCV-Computer-Vision-Library/dp/0596516134
     Or: http://oreilly.com/catalog/9780596516130/
     ISBN-10: 0596516134 or: ISBN-13: 978-0596516130

   OPENCV WEBSITES:
     Homepage:      http://opencv.org
     Online docs:   http://docs.opencv.org
     Q&A forum:     http://answers.opencv.org
     Issue tracker: http://code.opencv.org
     GitHub:        https://github.com/Itseez/opencv/
   ************************************************** */

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

using namespace cv;
using namespace std;

/*cv::Mat K1, K2;
cv::Mat D1, D2;
cv::Mat R1, R2;
cv::Mat P1, P2;*/

Rect validRoi_cp[2];
static int print_help()
{
    cout <<
            " Given a list of chessboard images, the number of corners (nx, ny)\n"
            " on the chessboards, and a flag: useCalibrated for \n"
            "   calibrated (0) or\n"
            "   uncalibrated \n"
            "     (1: use cvStereoCalibrate(), 2: compute fundamental\n"
            "         matrix separately) stereo. \n"
            " Calibrate the cameras and display the\n"
            " rectified results along with the computed disparity images.   \n" << endl;
    cout << "Usage:\n ./stereo_calib -w=<board_width default=9> -h=<board_height default=6> <image list XML/YML file default=../data/stereo_calib.xml>\n" << endl;
    return 0;
}

int RectifyImages(const std::vector<std::string> goodImageList, const cv::Rect validROI[2], bool isVerticalStereo, bool useCalibrated,
	              cv::Mat K1, cv::Mat K2, cv::Mat D1, cv::Mat D2, cv::Mat R1, cv::Mat R2, cv::Mat P1, cv::Mat P2)
{
	std::cout << "K1: " << K1 << std::endl;
	std::cout << "K2: " << K2 << std::endl;
	std::cout << "D1: " << D1 << std::endl;
	std::cout << "D2: " << D2 << std::endl;
	std::cout << "R1: " << R1 << std::endl;
	std::cout << "R2: " << R2 << std::endl;
	std::cout << "P1: " << P1 << std::endl;
	std::cout << "P2: " << P2 << std::endl;
	cv::Mat rmap[2][2];
	cv::Size imageSize;
	imageSize.width = 640;
	imageSize.height = 480;
	size_t nimages = goodImageList.size()/2;
	cv::initUndistortRectifyMap(K1,D1, R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
	cv::initUndistortRectifyMap(K2, D2, R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);
	Mat canvas;
	double sf;
	int w, h;

	if (!isVerticalStereo)
	{
		sf = 600. / MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width*sf);
		h = cvRound(imageSize.height*sf);
		canvas.create(h, w * 2, CV_8UC3);
	}
	else
	{
		sf = 300. / MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width*sf);
		h = cvRound(imageSize.height*sf);
		canvas.create(h * 2, w, CV_8UC3);
	}
	



	for (size_t i = 0; i < nimages; i++)
	{
		for (int k = 0; k < 2; k++)
		{
			Mat img = imread(goodImageList[i * 2 + k], 0), rimg, cimg;
			remap(img, rimg, rmap[k][0], rmap[k][1], INTER_LINEAR);
			cvtColor(rimg, cimg, COLOR_GRAY2BGR);
			Mat canvasPart = !isVerticalStereo ? canvas(Rect(w*k, 0, w, h)) : canvas(Rect(0, h*k, w, h));
			resize(cimg, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
			if (useCalibrated)
			{
				Rect vroi(cvRound(validROI[k].x*sf), cvRound(validROI[k].y*sf),
					cvRound(validROI[k].width*sf), cvRound(validROI[k].height*sf));
				rectangle(canvasPart, vroi, Scalar(0, 0, 255), 3, 8);
			}
		}

		if (!isVerticalStereo)
			for (int j = 0; j < canvas.rows; j += 16)
				line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
		else
			for (int j = 0; j < canvas.cols; j += 16)
				line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
		cv::imshow("rectified", canvas);
		char c = (char)waitKey();
		if (c == 27 || c == 'q' || c == 'Q')
			break;
	}

	return 0;
}

static void
StereoCalib(const vector<string>& imagelist, Size boardSize,bool displayCorners = false, bool useCalibrated=true, bool showRectified=true)
{

    if( imagelist.size() % 2 != 0 )
    {
        cout << "Error: the image list contains odd (non-even) number of elements\n";
        return;
    }

    const int maxScale = 2;
    const float squareSize = 0.051;  // Set this to your actual square size

    vector<vector<Point2f> > imagePoints[2];
    vector<vector<Point3f> > objectPoints;
    Size imageSize;

    int i, j, k, nimages = (int)imagelist.size()/2;

    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    vector<string> goodImageList;

    for( i = j = 0; i < nimages; i++ )
    {
        for( k = 0; k < 2; k++ )
        {
            const string& filename = "/home/shinan/Project/Calibration/Henry_stereo_30cm/"+imagelist[i*2+k];
            Mat img = imread(filename, 0);
            if(img.empty())
                break;
            if( imageSize == Size() )
                imageSize = img.size();
            else if( img.size() != imageSize )
            {
                cout << "The image " << filename << " has the size different from the first image size. Skipping the pair\n";
                break;
            }
            bool found = false;
            vector<Point2f>& corners = imagePoints[k][j];
            for( int scale = 1; scale <= maxScale; scale++ )
            {
                Mat timg;
                if( scale == 1 )
                    timg = img;
                else
                    resize(img, timg, Size(), scale, scale);
                found = findChessboardCorners(timg, boardSize, corners,
                    CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
                if( found )
                {
                    if( scale > 1 )
                    {
                        Mat cornersMat(corners);
                        cornersMat *= 1./scale;
                    }
                    break;
                }
            }
            if( displayCorners )
            {
                cout << filename << endl;
                Mat cimg, cimg1;
                cvtColor(img, cimg, COLOR_GRAY2BGR);
                drawChessboardCorners(cimg, boardSize, corners, found);
                double sf = 640./MAX(img.rows, img.cols);
                resize(cimg, cimg1, Size(), sf, sf);
                imshow("corners", cimg1);
                char c = (char)waitKey(500);
                if( c == 27 || c == 'q' || c == 'Q' ) //Allow ESC to quit
                    exit(-1);
            }
            else
                putchar('.');
            if( !found )
                break;
            cornerSubPix(img, corners, Size(11,11), Size(-1,-1),
                         TermCriteria(TermCriteria::COUNT+TermCriteria::EPS,
                                      30, 0.01));
        }
        if( k == 2 )
        {
			std::string tmp_name = "/home/shinan/Project/Calibration/Henry_stereo_30cm/" + imagelist[i * 2];
            goodImageList.push_back(tmp_name);
			tmp_name = "/home/shinan/Project/Calibration/Henry_stereo_30cm/" + imagelist[i * 2 + 1];
            goodImageList.push_back(tmp_name);
            j++;
        }
    }
    cout << j << " pairs have been successfully detected.\n";
    nimages = j;
    if( nimages < 2 )
    {
        cout << "Error: too little pairs to run the calibration\n";
        return;
    }

    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    objectPoints.resize(nimages);

    for( i = 0; i < nimages; i++ )
    {
        for( j = 0; j < boardSize.height; j++ )
            for( k = 0; k < boardSize.width; k++ )
                objectPoints[i].push_back(Point3f(k*squareSize, j*squareSize, 0));
    }

    cout << "Running stereo calibration ...\n";

    Mat cameraMatrix[2], distCoeffs[2];
    cameraMatrix[0] = initCameraMatrix2D(objectPoints,imagePoints[0],imageSize,0);
    cameraMatrix[1] = initCameraMatrix2D(objectPoints,imagePoints[1],imageSize,0);
    Mat R, T, E, F;

    double rms = cv::stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
                    cameraMatrix[0], distCoeffs[0],
                    cameraMatrix[1], distCoeffs[1],
                    imageSize, R, T, E, F,
                    CV_CALIB_FIX_ASPECT_RATIO +
                    CALIB_ZERO_TANGENT_DIST +
                    CV_CALIB_USE_INTRINSIC_GUESS +
                    CV_CALIB_SAME_FOCAL_LENGTH +
                    CV_CALIB_RATIONAL_MODEL  +
                    CV_CALIB_FIX_K3+ CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5+ CV_CALIB_FIX_K6,
                    TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 100, 1e-5) );
    cout << "done with RMS error=" << rms << endl;

// CALIBRATION QUALITY CHECK 
	/*because the output fundamental matrix implicitly includes all the output information,
      we can check the quality of calibration using the epipolar geometry constraint: m2^t*F*m1=0*/
    double err = 0;
    int npoints = 0;
    vector<Vec3f> lines[2];
    for( i = 0; i < nimages; i++ )
    {
        int npt = (int)imagePoints[0][i].size();
        Mat imgpt[2];
        for( k = 0; k < 2; k++ )
        {
            imgpt[k] = Mat(imagePoints[k][i]);
            undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
            computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
        }
        for( j = 0; j < npt; j++ )
        {
            double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
                                imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
                           fabs(imagePoints[1][i][j].x*lines[0][j][0] +
                                imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
            err += errij;
        }
        npoints += npt;
    }
    cout << "average epipolar err = " <<  err/npoints << endl;
//SAVE THE RESULT
    // save intrinsic parameters
    FileStorage fs("/home/shinan/Project/Calibration/Henry_stereo_30cm/stereo_intrinsics.yml", FileStorage::WRITE);
    if( fs.isOpened() )
    {
        fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
            "M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
        fs.release();
    }
    else
        cout << "Error: can not save the intrinsic parameters\n";

    Mat R1, R2, P1, P2, Q;
    Rect validRoi[2];

    stereoRectify(cameraMatrix[0], distCoeffs[0],
                  cameraMatrix[1], distCoeffs[1],
                  imageSize, R, T, R1, R2, P1, P2, Q,
                  CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);

    fs.open("/home/shinan/Project/Calibration/Henry_stereo_30cm/stereo_extrinsics.yml", FileStorage::WRITE);
    if( fs.isOpened() )
    {
        fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
        fs.release();
    }
    else
        cout << "Error: can not save the extrinsic parameters\n";

    // OpenCV can handle left-right
    // or up-down camera arrangements
    bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

// COMPUTE AND DISPLAY RECTIFICATION
    if( !showRectified )
        return;

/*
// IF BY CALIBRATED (BOUGUET'S METHOD)
if( useCalibrated )
{
// we already computed everything
}
// OR ELSE HARTLEY'S METHOD
else
// use intrinsic parameters of each camera, but
// compute the rectification transformation directly
// from the fundamental matrix
{
vector<Point2f> allimgpt[2];
for( k = 0; k < 2; k++ )
{
for( i = 0; i < nimages; i++ )
std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
}
F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
Mat H1, H2;
stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);

R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
P1 = cameraMatrix[0];
P2 = cameraMatrix[1];
}
*/


	
	bool NewImgNeed = false;
	if (NewImgNeed)
	{
		int newimgNum = 10;
		for (int i = 1; i < newimgNum; i++)
		{
			std::string left_str = "E:/Project/CaptureVideo/BJT/Before_imleft" + std::to_string(i*100+1) + ".jpg";
			std::string right_str = "E:/Project/CaptureVideo/BJT/Before_imRight" + std::to_string(i*100+1) + ".jpg";
			goodImageList.push_back(left_str);
			goodImageList.push_back(right_str);
		}
	}


	RectifyImages(goodImageList, validRoi, isVerticalStereo, useCalibrated,
		          cameraMatrix[0],cameraMatrix[1],distCoeffs[0],distCoeffs[1],R1,R2,P1,P2);
	
}


static bool readStringList( const string& filename, vector<string>& l )
{
    l.resize(0);
    FileStorage fs(filename, FileStorage::READ);
    if( !fs.isOpened() )
        return false;
    FileNode n = fs.getFirstTopLevelNode();
    if( n.type() != FileNode::SEQ )
        return false;
    FileNodeIterator it = n.begin(), it_end = n.end();
    for( ; it != it_end; ++it )
        l.push_back((string)*it);
    return true;
}

int main(int argc, char** argv)
{
    Size boardSize;
    string imagelistfn;
    bool showRectified = true;
    /*cv::CommandLineParser parser(argc, argv, "{w|9|}{h|6|}{nr||}{help||}{@input|../data/stereo_calib.xml|}");
    if (parser.has("help"))
        return print_help();
    showRectified = !parser.has("nr");
    imagelistfn = parser.get<string>("@input");
    boardSize.width = parser.get<int>("w");
    boardSize.height = parser.get<int>("h");
    if (!parser.check())
    {
        parser.printErrors();
        return 1;
    }*/
	imagelistfn = "/home/shinan/Project/Calibration/Henry_stereo_30cm/stereo_calib.xml";
	boardSize.width = 8;
	boardSize.height = 6;
    vector<string> imagelist;
    bool ok = readStringList(imagelistfn, imagelist);
    if(!ok || imagelist.empty())
    {
        cout << "can not open " << imagelistfn << " or the string list is empty" << endl;
        return print_help();
    }

    StereoCalib(imagelist, boardSize,false, true, showRectified);
    return 0;
}
