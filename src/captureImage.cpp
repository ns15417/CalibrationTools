#include<iostream>
#include<opencv2/opencv.hpp>
//#include<boost/filesystem.hpp>
//#include "calibration.hpp"
#include<chrono>

//https://blog.csdn.net/aptx704610875/article/details/48914043
using namespace std;
using namespace cv;

/*
int get_filenames(const std::string& dir, std::vector<std::string>& filenames)
{
	boost::filesystem::path path(dir);
	if (!boost::filesystem::exists(path))
	{
		return -1;
	}
	if (boost::filesystem::is_regular_file(path))
	{
		filenames.push_back(path.string());
		return 1;
	}
	boost::filesystem::directory_iterator end_iter;
	for (boost::filesystem::directory_iterator iter(path); iter != end_iter; ++iter)
	{
		if (boost::filesystem::is_regular_file(iter->status()) &&
			((iter->path().extension().string() == ".png") || (iter->path().extension().string() == ".jpg")))
		{
			filenames.push_back(iter->path().string());
		}
		if (boost::filesystem::is_directory(iter->status()))
		{
			get_filenames(iter->path().string(), filenames);
		}
	}
	return (int)filenames.size();
}*/


int main()
{

	cv::Mat src_img1;	
	cv::Mat src_img2;

	int img_num = 0;
	//默认的棋盘格标定板是横放，cols=8，rows=6

	//capture images	
	cv::VideoCapture cap1(1);
	cv::VideoCapture cap2(0);
	while (cap1.read(src_img1) &&cap2.read(src_img2) )
	{
		std::string filename1;
		cv::imshow("left_img", src_img1);
		cv::imshow("right_img", src_img2);
		char c = cv::waitKey(1);
		
		//
		if (c =='a')
		{
			img_num += 1;
			std::string tmpname = std::to_string(img_num);
			if (img_num < 10)
				tmpname = "0" + std::to_string(img_num);
			filename1 = "/home/shinan/Project/Calibration/Henry_stereo_30cm/left" + tmpname +".jpg";
			std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
			imwrite(filename1, src_img1);
			filename1 = "/home/shinan/Project/Calibration/Henry_stereo_30cm/right" + tmpname + ".jpg";
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
	
	
   	return 0;
}
