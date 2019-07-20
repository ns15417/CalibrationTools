#include<iostream>
#include<opencv2/opencv.hpp>
#include<boost/filesystem.hpp>
#include "calibration.hpp"

//参考https://blog.csdn.net/aptx704610875/article/details/48914043
using namespace std;
using namespace cv;

int fisheyeCalib(std::vector<std::vector<cv::Point2f>> imgPoints,
	std::vector<std::vector<cv::Point3f>> objectPoints,
	cv::Size img_size, cv::Mat &intrinsic_mat,
	cv::Vec4d &distortion_coeffs, std::vector<cv::Vec3d> &transform_vec, std::vector<cv::Vec3d> &rotation_vec)
{
	int flags = 0;
	flags |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
	flags |= cv::fisheye::CALIB_CHECK_COND;
	flags |= cv::fisheye::CALIB_FIX_SKEW;
	cv::fisheye::calibrate(objectPoints, imgPoints, img_size, intrinsic_mat, distortion_coeffs, 
		                  rotation_vec, transform_vec, flags);
	return 0;
}

std::vector<cv::Point2f> locationtransfer(std::vector<cv::Point2f> inputvec)
{
	std::vector<cv::Point2f> ret_vec;
	for (int i = 0; i < inputvec.size(); i++)
	{
		cv::Point2f tmppt;
		cv::Point2f curpt = inputvec[i];
		tmppt.x = curpt.x;
		tmppt.y = curpt.y;
		ret_vec.push_back(tmppt);
	}
	return ret_vec;
}
//获取每张图像的图像坐标
int GetImagePoint(cv::Mat curImg, cv::Size pattern_size, std::vector<cv::Point2f> &detected_corners)
{
	cv::Mat imggray;
	cv::cvtColor(curImg, imggray, CV_RGB2GRAY);
    //pattern size should be cv::size(cols,row)	
	bool patternfound = findChessboardCorners(imggray, pattern_size, detected_corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
	drawChessboardCorners(imggray, pattern_size, detected_corners, patternfound);
	//如果找到corner，对其进行亚像素优化
	if (patternfound)
	{
		cv::cornerSubPix(imggray, detected_corners, cv::Size(9, 9), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
		return 1;
	}
		
	return 0;
		 
}

//为每张图片生成世界坐标
int InitializeWorldPoints(cv::Size board_size,float square_size,std::vector<cv::Point3f> &worldPoints)
{
	for (int i = 0; i<board_size.height; i++) 
	{ 
		for (int j = 0; j<board_size.width; j++) {        
			/* 假设定标板放在世界坐标系中z=0的平面上 */   		
			Point3f tempPoint;		
			tempPoint.x = float(j)*square_size;		
			tempPoint.y = float(i)*square_size;			
			tempPoint.z = 0.0;			
			worldPoints.push_back(tempPoint);
		} 
	}	
	return 0;
}

float calculateReprojectionError(std::vector<cv::Point3f> objectPoints, std::vector<cv::Point2f> image_points,
	cv::Mat rotation_vec, cv::Mat trans_vec,
	cv::Mat intrinsic_matrix, cv::Mat distortion_coeffs)
{    
	std::vector<cv::Point2f> repro_imgpt;
	/****    通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点     ****/	
	projectPoints(objectPoints, rotation_vec, trans_vec, intrinsic_matrix, distortion_coeffs, repro_imgpt);
	/* 计算新的投影点和旧的投影点之间的误差*/  
		
	Mat tempImagePointMat = Mat(1, image_points.size(), CV_32FC2);
	Mat image_points2Mat = Mat(1, repro_imgpt.size(), CV_32FC2);
	for (int j = 0; j < image_points.size(); j++)
	{ 
		image_points2Mat.at<Vec2f>(0, j) = Vec2f(repro_imgpt[j].x, repro_imgpt[j].y);
		tempImagePointMat.at<Vec2f>(0, j) = Vec2f(image_points[j].x, image_points[j].y);
	}	
	float err = norm(image_points2Mat, tempImagePointMat, NORM_L2);

	return err;
}

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
}

int printImgandObject(std::vector<cv::Point2f> imgpts, std::vector<cv::Point3f> objpts)
{
	for (int i = 0; i < imgpts.size(); i++)
	{
		std::cout << "corner " << i << " on image is  (" << imgpts[i].x << " , " << imgpts[i].y << " )" << endl;
		std::cout << "corner " << i << " on world is  (" << objpts[i].x << " , " << objpts[i].y << " )" << endl;
	}
	return 0;
}

void ChangeFilenames(std::string &inputdir , std::string outputdir)
{
	std::vector<std::string> filenames;
	get_filenames(inputdir, filenames);
	if (filenames.empty())
		std::cout << "Failed to get images maybe path is wrong\n";
	for (size_t i = 0; i < filenames.size(); i++)
	{
		cv::Mat cur_img = cv::imread(filenames[i], -1);
		if (cur_img.empty())
			continue;
		std::string part_newname;
		if (i < 9)
			part_newname = '0' + std::to_string(i + 1);
		else
			part_newname = std::to_string(i + 1);
			
		std::string new_name = outputdir + "/left" + part_newname + ".jpg";
		cv::imwrite(new_name, cur_img);
	}

}

int main()
{
	std::string inputdir = "E:/Project/CaptureVideo/build/src/BJT_LEFT";
	std::string outdir = "E:/Project/CaptureVideo/build/src/BJT_stereo";

	
	ChangeFilenames(inputdir, outdir);
	cv::Mat src_img1,src_img2,src_img3;	

	int img_num = 0;
	int width,height;
	//默认的棋盘格标定板是横放，cols=8，rows=6
	bool horizonboard = true;
	cv::Size patternsize;
	if (horizonboard)
		patternsize = cv::Size(8, 6);
	else
		patternsize = cv::Size(6, 8);
	std::cout << patternsize << std::endl;
	std::vector<std::vector<cv::Point2f>> ImagePoints;
	std::vector < std::vector<cv::Point3f>> objectPoints;
	
	//capture images
	
	cv::VideoCapture cap1(0);
	bool retw = cap1.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	bool reth = cap1.set(CV_CAP_PROP_FRAME_HEIGHT, 480);  
	int ret1 = cap1.read(src_img1);
	
	std::cout << "The size of src_mg1: " << src_img1.size() << std::endl;
	while (cap1.read(src_img1))
	{
		//std::cout<< 
		char filename1[15];
		cv::imshow("src_img1", src_img1);		

		char c = cv::waitKey(1);
		
		//图像坐标
		if (c == 'a' && img_num <15)
		{
			cout << "current image num: " << img_num << endl;
			std::vector<cv::Point2f> cur_corners;
			int ret = GetImagePoint(src_img1, patternsize, cur_corners);
			if (ret)
			{
				ImagePoints.push_back(cur_corners);
				img_num++;
				sprintf(filename1, "img%d.jpg", img_num);
				imwrite(filename1, src_img1);
			}
		}
		if (c == 'q' || c == 'Q')
		{
			break;
		}
	}
	
	//using saved images
	/*
	std::string filedir = "E:/Project/CaptureVideo/CapturedImages/part3";
	std::vector<std::string> filenames;
	get_filenames(filedir,filenames);
	for (int i = 0; i < filenames.size(); i++)
	{
		cv::Mat cur_img = cv::imread(filenames[i], -1);
		width = cur_img.cols;
		height = cur_img.rows;
		std::vector<cv::Point2f> curimgcorner;
		int ret_val = GetImagePoint(cur_img, patternsize, curimgcorner);
		if (ret_val)
		{
			ImagePoints.push_back(curimgcorner);
			img_num = img_num + ret_val;
		}
	
	}
		cv::Mat tmp_img = cv::imread(filenames[0], -1);
	*/
	if (img_num < 6)
	{
		fprintf(stderr, "Not enough images, should be 10, only got %d", img_num);
		return 0;
	}
	//世界坐标
	for (int i = 0; i < img_num; i++)
	{ 
		vector<Point3f> tempPointSet;   
		InitializeWorldPoints(patternsize, 35, tempPointSet);
		objectPoints.push_back(tempPointSet);
	}
	
	printImgandObject(ImagePoints[0], objectPoints[0]);

	
	cv::Mat Intrinsic;
	cv::Mat distCoeff;
	std::vector<cv::Mat> rvec, tvec;
	//width and height are used to initialize the intrinsic of camera;
	//returns re-projection error
	cv::calibrateCamera(objectPoints, ImagePoints, cv::Size(height, width), Intrinsic, distCoeff, rvec, tvec);

	//计算重投影误差并写入文件
	float total_error=0;
	for (int i = 0; i < img_num; i++)
	{
		float cur_err= calculateReprojectionError(objectPoints[i],ImagePoints[i],rvec[i],tvec[i],Intrinsic,distCoeff);
		std::cout << "reprojection error for Image " << i << "is " << cur_err << std::endl;
		total_error += cur_err;
	}
	cout << "reprojection_error: " << total_error / img_num << endl;
	cout << "Intrinsic" << Intrinsic << endl;
	cout << "distCoeff " << distCoeff << endl;

	//再次利用BA算法进行最小化重投影误差的优化
	/*CalibrationAlgorithm secondopti;
	std::vector<cv::Mat> extrinsic_mat;
	for (int i = 0; i < img_num; i++)
	{
		cv::Mat new_extrinsic(6, 1, CV_64FC1);
		secondopti.getExtrinsicMat(rvec[i], tvec[i],new_extrinsic);
		extrinsic_mat.push_back(new_extrinsic);
	}

	
	double k1=
	secondopti.BundleAdjustment(Intrinsic, extrinsic_mat, objectPoints, ImagePoints, , k1, k2);*/
	return 0;
}