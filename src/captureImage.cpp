#include<iostream>
#include<opencv2/opencv.hpp>
//#include<boost/filesystem.hpp>
//#include "calibration.hpp"
#include "opencv2/imgproc/types_c.h"
#include "opencv2/video.hpp"
#include<chrono>

//https://blog.csdn.net/aptx704610875/article/details/48914043
using namespace std;
using namespace cv;
int Width = 640;
int Height = 480;


void CaptureStereoImg(const int leftcamid, const int rightcamid,const std::string filepath)
{
	int img_num = 0;
	cv::Mat src_img1;	
	cv::Mat src_img2;
	cv::VideoCapture cap1(leftcamid);
	cv::VideoCapture cap2(rightcamid);
    cap1.set(cv::CAP_PROP_FRAME_HEIGHT,Height);
    cap1.set(cv::CAP_PROP_FRAME_WIDTH,Width);
    cap2.set(cv::CAP_PROP_FRAME_HEIGHT,Height);
    cap2.set(cv::CAP_PROP_FRAME_WIDTH,Width);
	while (cap1.read(src_img1) &&cap2.read(src_img2) )
	{
		std::string filename1;
		cv::imshow("left_img", src_img1);
		cv::imshow("right_img", src_img2);
		char c = cv::waitKey(1);
		if (c =='a')
		{
			img_num += 1;
			std::string tmpname = std::to_string(img_num);
			if (img_num < 10)
				tmpname = "0" + std::to_string(img_num);
			filename1 = filepath + "/left/" + tmpname +".jpg";
			std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
			imwrite(filename1, src_img1);
			filename1 = filepath+"/right/" + tmpname + ".jpg";
			std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
			double timeused = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
			std::cout << "time used to save image " << img_num << "is : " << timeused << std::endl;
			imwrite(filename1, src_img2);
			
		}
		if (c == 'q' || c == 'Q')
		{
			break;
		}
	}

	return;
}

void CaptureMonoImg(int camid, const std::string filepath)
{
	int img_num=0;
	cv::VideoCapture cap(camid);
	cap.set(cv::CAP_PROP_FRAME_WIDTH,Width);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT,Height);
	cv::Mat reading_img;
	while(cap.read(reading_img))
	{
		cv::imshow("current Image",reading_img);
		char c=cv::waitKey(1);
		if(c=='a'){
			std::string filename = filepath + "/" +std::to_string(img_num) + ".png";
			cv::imwrite(filename, reading_img);
			img_num++;
		}
		if(c=='q' || c=='Q')
		{
			break;
		}
	}

	return;
}

/**
 * @func: RealTimeUndistort() 实时去畸变函数 
 * @param： ori_img：原始灰度图
 * @param： new_img：新的图像
 * @note： Remember to change intrinsic and distortion params to your own result
 */
void RealTimeUndistort(cv::Mat &ori_img, cv::Mat &new_img)
{
	float fx = 214.9764957371081;
	float fy = 215.6817940573768;
	float cx = 334.804151915779;
	float cy = 238.0918046000016;
	float k1 = -0.100371;
	float k2 = 0.0257397;
	float p1 = -0.0211074;
	float p2 = 0.00568583;

  cv::Matx33f intrinsic_matrix = cv::Matx33f::eye();
  intrinsic_matrix(0,0) = fx;  intrinsic_matrix(1,1) = fy;
  intrinsic_matrix(0,2) = cx;  intrinsic_matrix(1,2) = cy;
  cv::Vec4f distortion_coeffs;
  distortion_coeffs(0) = k1;   distortion_coeffs(1) = k2;  
  distortion_coeffs(2) = p1;  distortion_coeffs(3) = p2;
  Mat mapx = Mat(Height,Width, CV_32FC1);
  Mat mapy = Mat(Height,Width, CV_32FC1);
  Mat R = Mat::eye(3, 3, CV_32F);

  fisheye::initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, R,intrinsic_matrix,
        cv::Size(Width,Height), CV_32FC1, mapx, mapy);
  cv::Mat GrayImg;
  cv::cvtColor(ori_img,GrayImg,CV_BGR2GRAY);
 
  cv::remap(GrayImg, new_img, mapx, mapy, INTER_LINEAR);
}

/**
 * MONO: 单目采集模式
 * STEREO：双目采集模式
 * UNDISTORTION： 实时去畸变模式
 */

int main(int argc, char **argv)
{
    if(!(argc == 4||argc==5)) 
    {
        std::cout<<"Usage: captureImage leftcamid [rightcamid] filepath mode[MONO/STEREO] \n " <<std::endl;
        return -1; 
    }

    std::string filepath = std::string(argv[argc-2]);
	//capture images	
    std::string mode = std::string(argv[argc-1]);
    if (mode == "MONO"){
		int camid = std::atoi(argv[1]);
		CaptureMonoImg(camid,filepath);
	}
        
    else if(mode == "STEREO"){
        int leftcamid = std::atoi(argv[1]);
        int rightcamid = std::atoi(argv[2]);
		CaptureStereoImg(leftcamid,rightcamid,filepath);
	}      
    else if(mode == "UNDISTORTION"){ 
        int camid = std::atoi(argv[1]);
		cv::Mat t(Height,Width,CV_8UC1);

		cv::VideoCapture cap(camid);
	    cap.set(cv::CAP_PROP_FRAME_WIDTH,Width);
	    cap.set(cv::CAP_PROP_FRAME_HEIGHT,Height);
	    cv::Mat reading_img;
	    while(cap.read(reading_img))
	    {
			RealTimeUndistort(reading_img,t);
	    	cv::imshow("current Image",t);
			char c=cv::waitKey(1);
			if(c=='q' || c=='Q')
		   {
				break;
		   }
	    }
	
	}
   	return 0;
}
