/*************************************************************************
        > File Name: fisheye_calibrate.cpp
        > Author: yujr
        > Mail: yujingrui@sjtu.edu.cn
        > Created Time: 2017年10月07日 星期六 20时07分37秒
 ************************************************************************/

#include <fstream>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
//#include <cv.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>

#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

Size image_size = Size(640,480); //图像尺寸
Size board_size = Size(8, 6);
Size square_size = Size(71.5, 71.5); //棋盘格上一个格子的size

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

int FindCheessboard(std::vector<std::string> &filenames, vector<std::string> &newFilenames,  vector<int> &point_counts,vector<vector<Point2f> > &corners_Seq)
{
  int successImgNUm = 0;
  std::vector<cv::Point2f> corners;
  for(int i =0;i < static_cast<int>(filenames.size()); i++)
  {
    std::string curImgName = filenames[i];
    cv::Mat image = imread(curImgName);
    cv::Mat imageGray;
    cvtColor(image, imageGray, cv::COLOR_BGR2GRAY);
  
    bool patternfound = findChessboardCorners(imageGray, board_size, corners,CALIB_CB_ADAPTIVE_THRESH);
    if (!patternfound) {
      std::cout << "can not find chessboard corners!\n";
    } else {
      Mat imageTemp = image.clone();
      for (int j = 0; j < static_cast<int>(corners.size()); j++) {
        circle(imageTemp, corners[j], 2, Scalar(0, 255, 0), 2, 8, 0);
      }

    //   cornerSubPix(imageGray, corners, Size(11, 11), Size(-1, -1),
    //                TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

    //  for (int j = 0; j < static_cast<int>(corners.size()); j++) {
    //     circle(imageTemp, corners[j], 2, Scalar(0, 0, 255), 2, 8, 0);
    //   }

      std::string imgname2 = curImgName +"_corner.jpg";
      //imwrite(imgname2, imageTemp);

      //count = count + corners.size();
      successImgNUm = successImgNUm + 1;
      corners_Seq.push_back(corners);
      cout << "current names pushback " << curImgName<< endl;
      newFilenames.push_back(curImgName);
    }
  }

  for (int i = 0; i < successImgNUm; i++) {
    point_counts.push_back(board_size.width * board_size.height);
  }
  std::cout << "总共有 " << successImgNUm << " 张图片，角点提取完成！\n ";
  return successImgNUm;
}
 
  /* 初始化定标板上角点的三维坐标 */
void GetObjectPoint(vector<vector<Point3f> > &object_Points, int number)
{
  for (int t = 0; t < number; t++) {
    vector<Point3f> tempPointSet;
    for (int i = 0; i < board_size.height; i++) {
      for (int j = 0; j < board_size.width; j++) {
        /* 假设定标板放在世界坐标系中z=0的平面上 */
        Point3f tempPoint;
        tempPoint.x = i * square_size.width;
        tempPoint.y = j * square_size.height;
        tempPoint.z = 1;
        tempPointSet.push_back(tempPoint);
      }
    }
    object_Points.push_back(tempPointSet);
  }
}
/* 每幅图像的平移向量 */   /* 每幅图像的旋转向量 */
void ValuateResult(std::vector<std::string> &filenames, vector<vector<Point3f>> object_Points,
  std::vector<cv::Vec3d> rotation_vectors,  
  std::vector<cv::Vec3d> translation_vectors,
  cv::Matx33d &intrinsic_matrix, cv::Vec4d &distortion_coeffs,
  std::vector<std::vector<cv::Point2f>> &oriCornerxy,
  std::vector<int> &point_counts)
{
  int num = filenames.size();
  double total_err = 0.0;        /* 所有图像的平均误差的总和 */
  double err = 0.0;              /* 每幅图像的平均误差 */
  vector<Point2f> image_points2; /****   保存重新计算得到的投影点    ****/

  cout << "每幅图像的定标误差：" << endl;
  cout << "每幅图像的定标误差：" << endl << endl;
  for (int i = 0; i < num; i++) {
    vector<Point3f> tempPointSet = object_Points[i];
    /****
     * 通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点
     * ****/
    fisheye::projectPoints(tempPointSet, image_points2, rotation_vectors[i],
                           translation_vectors[i], intrinsic_matrix,
                           distortion_coeffs);
    /* 计算新的投影点和旧的投影点之间的误差*/
    vector<Point2f> tempImagePoint = oriCornerxy[i];
    Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);
    Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);
    for (size_t j = 0; j != tempImagePoint.size(); j++) {
      image_points2Mat.at<Vec2f>(0, j) =
          Vec2f(image_points2[j].x, image_points2[j].y);
      tempImagePointMat.at<Vec2f>(0, j) =
          Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
    }
    err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
    total_err += err /= point_counts[i];
    cout << "图像" << filenames[i] << "  的平均误差：" << err << "像素" << endl;
  }
  cout << "总体平均误差：" << total_err / num << "像素" << endl;
  cout << "评价完成！" << endl;

}

cv::Mat GenerateNewFrame(cv::Size imgSize, cv::Mat &ori_img)
{
  cv::Mat newImg;
  ori_img.copyTo(newImg);
  if(newImg.channels() == 3){
    cv::cvtColor(newImg,newImg,cv::COLOR_BGR2GRAY);
  }
  cout << "newImg.rows: " << newImg.rows << ", newImg.cols " << newImg.cols << endl;
  for(int i = 160; i< 320; i++){
   //uchar* row_data = newImg.ptr<uchar>(i);
    for(int j = 120; j <240;j++)
    {
      newImg.at<uchar>(j,i) = 255;
    }
  }
  cout << "Generate new mask..." <<endl;
  cv::imwrite("mask.png",newImg);
  return newImg;
}

int main(int argc, char* argv[]) {
  ofstream fout("caliberation_result.txt");

  std::string image_path = string(argv[1]);
  std::vector<std::string> imagenames;
  get_filenames(image_path,imagenames);
  
  vector<vector<Point2f> > corners_Seq;
  vector<Mat> image_Seq;
  vector<int> point_counts;
  vector<std::string> sucFilenames;
  // // if you want to calibrate online
  // VideoCapture cap(1);
  // Mat image;

  int successImageNum = FindCheessboard(imagenames,sucFilenames,point_counts,corners_Seq);
    

  /************************************************************************
                                  摄像机定标
  *************************************************************************/
  cout << "开始定标" << endl;

  /****  保存定标板上角点的三维坐标   ****/
  vector<vector<Point3f> > object_Points; 
  GetObjectPoint(object_Points,successImageNum);

  for (int i = 0; i < successImageNum; i++) {
    point_counts.push_back(board_size.width * board_size.height);
  }

  cv::Matx33d intrinsic_matrix; /*****    摄像机内参数矩阵    ****/
  cv::Vec4d distortion_coeffs;  /* 摄像机的4个畸变系数：k1,k2,k3,k4*/
  std::vector<cv::Vec3d> rotation_vectors;    /* 每幅图像的旋转向量 */
  std::vector<cv::Vec3d> translation_vectors; /* 每幅图像的平移向量 */
  int flags = 0;
  flags |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
  flags |= cv::fisheye::CALIB_CHECK_COND;
  flags |= cv::fisheye::CALIB_FIX_SKEW;
  fisheye::calibrate(object_Points, corners_Seq, image_size, intrinsic_matrix,
                     distortion_coeffs, rotation_vectors, translation_vectors,
                     flags, cv::TermCriteria(3, 20, 1e-6));
  cout << "定标完成！\n";



  /************************************************************************
                              对定标结果进行评价
  *************************************************************************/
  cout << "开始评价定标结果………………" << endl;
  ValuateResult(sucFilenames, object_Points,rotation_vectors,  translation_vectors,intrinsic_matrix, distortion_coeffs, corners_Seq,point_counts);

  /************************************************************************
                              保存定标结果
  *************************************************************************/
  cout << "开始保存定标结果………………" << endl;
  Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* 保存每幅图像的旋转矩阵 */

  cout << "相机内参数矩阵：" << endl;
  cout << intrinsic_matrix << endl;
  cout << "畸变系数：\n";
  cout << distortion_coeffs << endl;
  for (int i = 0; i < static_cast<int>(sucFilenames.size()); i++) {
    /* 将旋转向量转换为相对应的旋转矩阵 */
    Rodrigues(rotation_vectors[i], rotation_matrix);
    cout << "第" << i + 1 << "幅图像的旋转矩阵：" << endl;
    cout << rotation_matrix << endl;
    cout << "第" << i + 1 << "幅图像的平移向量：" << endl;
    cout << translation_vectors[i] << endl;
  }
  cout << "完成保存" << endl;
  fout << endl;

  /************************************************************************
                              显示定标结果
  *************************************************************************/
  cout << "imgsize: " << image_size << endl;
  Mat mapx = Mat(image_size, CV_32FC1);
  Mat mapy = Mat(image_size, CV_32FC1);
  Mat R = Mat::eye(3, 3, CV_32F);
  //fisheye::initUndistortRectifyMap(intrinsic_matrix,distortion_coeffs,R,intrinsic_matrix,image_size,CV_32FC1,mapx,mapy);
  // cv::Mat new_intrinsic = getOptimalNewCameraMatrix(intrinsic_matrix, distortion_coeffs,
  //                                 image_size, 1, image_size, 0);
  fisheye::initUndistortRectifyMap(
        intrinsic_matrix, distortion_coeffs, R,
        // getOptimalNewCameraMatrix(intrinsic_matrix, distortion_coeffs,
        //                           image_size, 1, image_size, 0),
                                  intrinsic_matrix,
        image_size, CV_32FC1, mapx, mapy
  );
  
  //cout << "mapx: " << mapx << endl;
  cout << "保存矫正图像" << endl;
  for (int i = 0; i < static_cast<int>(imagenames.size()); i++) {
    cout << "Frame #" << i + 1 << "..." << endl;    
    cv::Mat ori_img = cv::imread(imagenames[i]);
    cv::Mat gray_img;
    cv::cvtColor(ori_img,gray_img,cv::COLOR_BGR2GRAY);
    cv::Mat t(gray_img.size(), gray_img.type());
    cv::remap(gray_img, t, mapx, mapy, INTER_LINEAR);
    string imageFileName = imagenames[i] + "_d.jpg";
    //imwrite(imageFileName, t);
  }

  cout << "保存结束" << endl;
  cv::Mat fisheyeImg = cv::imread(imagenames[0]);
  //cv::Mat fisheyeImg_gray;
  //cv::cvtColor(fisheyeImg,fisheyeImg_gray,cv::COLOR_BGR2GRAY);
  cv::Mat fisheyeImg_t(fisheyeImg.size(), fisheyeImg.type());
  cv::remap(fisheyeImg, fisheyeImg_t, mapx, mapy, INTER_LINEAR);

  // cv::Mat mask = GenerateNewFrame(fisheyeImg.size(),fisheyeImg_t);
  // for(int i = 120; i< 240; i++){
  //   for(int j = 160; j < 320;j++)
  //   {
  //     int value =  mask.at<uchar>(i,j);
  //     float loc_x = mapx.at<float>(i,j);
  //     float loc_y = mapy.at<float>(i,j);
  //     int new_x = std::floor(loc_x); //col
  //     int new_y = std::floor(loc_y); //row
  //     if(value == 255){
  //       fisheyeImg.at<Vec3b>(new_y,new_x)[0] = 255;
  //       fisheyeImg.at<Vec3b>(new_y,new_x)[1] = 0;
  //       fisheyeImg.at<Vec3b>(new_y,new_x)[2] = 0;
  //     }
  //   }
  // }


  cv::imwrite("fishwithmask.png",fisheyeImg_t);
  return 0;

}
