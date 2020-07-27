#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>

#include <algorithm>
#include <iostream>
#include <iterator>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
using namespace std;
using namespace cv;
const int imageWidth = 640;
const int imageHeight = 480;
const int boardWidth = 6;
const int boardHeight = 8;
const Size boardSize = Size(boardWidth, boardHeight);
Size imageSize = Size(imageWidth, imageHeight);

const int boardCorner = boardWidth * boardHeight;
const float squareSize = 71.5;  // actual square size in cm
const int frameNumber = 26;

string folder_ = "/home/shinan/Project/Calibration/180_stereo/";
string format_R = "right";
string format_L = "left";

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
Mat cameraMatrixL;  //= (Mat_<double>(3, 3) << 163.38454640186166 ,0.,
                    //316.93740846397657, 0.,
// 163.44478732969247, 220.67943473133187, 0., 0., 1);

Mat distCoeffL;  // = (Mat_<double>(5, 1) << -0.27, 0.05, 0, 0,0);
/*
fx 0 cx
0 fy cy
0 0  1*/
Mat cameraMatrixR;  // = (Mat_<double>(3, 3) <<164.41558284668497, 0.,
                    // 335.20449837071766, 0.,
// 165.28255528761292, 242.49520037582414, 0., 0., 1);
Mat distCoeffR;  // = (Mat_<double>(5, 1) << -0.26, 0.05, 0, 0, 0);

void calRealPoint(vector<vector<Point3f>>& obj, int boardwidth, int boardheight,
                  int imgNumber, int squaresize) 
{
  vector<Point3f> imgpoint;
  for (int rowIndex = 0; rowIndex < boardheight; rowIndex++) {
    for (int colIndex = 0; colIndex < boardwidth; colIndex++) {
      imgpoint.push_back(
          Point3f(colIndex * squaresize, rowIndex * squaresize, 0));
    }
  }
  for (int imgIndex = 0; imgIndex < imgNumber; imgIndex++) {
    obj.push_back(imgpoint);
  }
}

void outputCameraParam(void) {
  FileStorage fs(folder_ + "/Fish_stereo_intrinsics.yml", FileStorage::WRITE);

  if (fs.isOpened()) {
    fs << "cameraMatrixL" << cameraMatrixL << "cameraDistcoeffL" << distCoeffL
       << "cameraMatrixR" << cameraMatrixR << "cameraDistcoeffR" << distCoeffR;
    fs.release();
    cout << "cameraMatrixL=:" << cameraMatrixL << endl
         << "cameraDistcoeffL=:" << distCoeffL << endl
         << "cameraMatrixR=:" << cameraMatrixR << endl
         << "cameraDistcoeffR=:" << distCoeffR << endl;
  } else {
    cout << "Error: can not save the intrinsics!!!!!" << endl;
  }
  fs.open(folder_ + "/Fish_stereo_extrinsics.yml", FileStorage::WRITE);
  if (fs.isOpened()) {
    fs << "R" << R << "T" << T << "Rl" << Rl << "Rr" << Rr << "Pl" << Pl << "Pr"
       << Pr << "Q" << Q;
    cout << "R=" << R << endl
         << "T=" << T << endl
         << "Rl=" << Rl << endl
         << "Rr=" << Rr << endl
         << "Pl=" << Pl << endl
         << "Pr=" << Pr << endl
         << "Q=" << Q << endl;
    fs.release();
  } else
    cout << "Error: can not save the extrinsic parameters\n";
}

int FindCheessboard(std::vector<std::string>& filenames,
                    vector<std::string>& newFilenames,
                    vector<int>& point_counts,
                    vector<vector<Point2f>>& corners_Seq) {
  int successImgNUm = 0;
  std::vector<cv::Point2f> corners;
  for (int i = 0; i < static_cast<int>(filenames.size()); i++) {
    std::string curImgName = filenames[i];
    cv::Mat image = imread(curImgName);
    cv::Mat imageGray;
    cvtColor(image, imageGray, cv::COLOR_BGR2GRAY);

    bool patternfound = findChessboardCorners(imageGray, boardSize, corners,
                                              CALIB_CB_ADAPTIVE_THRESH);
    if (!patternfound) {
      std::cout << "can not find chessboard corners!\n";
    } else {
      Mat imageTemp = image.clone();
      for (int j = 0; j < static_cast<int>(corners.size()); j++) {
        circle(imageTemp, corners[j], 2, Scalar(0, 255, 0), 2, 8, 0);
      }

      //   cornerSubPix(imageGray, corners, Size(11, 11), Size(-1, -1),
      //                TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30,
      //                0.1));

      //  for (int j = 0; j < static_cast<int>(corners.size()); j++) {
      //     circle(imageTemp, corners[j], 2, Scalar(0, 0, 255), 2, 8, 0);
      //   }

      std::string imgname2 = curImgName + "_corner.jpg";
      // imwrite(imgname2, imageTemp);

      // count = count + corners.size();
      successImgNUm = successImgNUm + 1;
      corners_Seq.push_back(corners);
      cout << "current names pushback " << curImgName << endl;
      newFilenames.push_back(curImgName);
    }
  }

  for (int i = 0; i < successImgNUm; i++) {
    point_counts.push_back(boardSize.width * boardSize.height);
  }
  std::cout << "总共有 " << successImgNUm << " 张图片，角点提取完成！\n ";
  return successImgNUm;
}

int main(int argc, char* argv[]) {
  Mat img;
  int goodFrameCount = 1;
  int FrameCount = 1;
  cout << "Total Images is" << frameNumber << endl;
  cv::Size imgSize = cv::Size(640, 480);
  while (FrameCount < frameNumber) 
  {
    std::string tmp_str = to_string(FrameCount);
    if (FrameCount < 10) tmp_str = '0' + to_string(FrameCount);
    cout << "Current image is " << FrameCount << endl;

    string filenamel, filenamer;
    filenamel = folder_ + format_L + tmp_str + ".jpg";
    filenamer = folder_ + format_R + tmp_str + ".jpg";
    rgbImageL = imread(filenamel, -1);
    rgbImageR = imread(filenamer, -1);
    if (rgbImageL.channels() >= 3) {
      cvtColor(rgbImageL, grayImageL, cv::COLOR_BGR2GRAY);
      cvtColor(rgbImageR, grayImageR, cv::COLOR_BGR2GRAY);
    }

    bool isFindL, isFindR;
    isFindL = findChessboardCorners(rgbImageL, boardSize, cornerL);
    isFindR = findChessboardCorners(rgbImageR, boardSize, cornerR);
    if (isFindL == true && isFindR == true && 
	    cornerL.size() == boardHeight*boardWidth && 
		cornerR.size() == boardHeight*boardWidth) {
      // cornerSubPix(grayImageL, cornerL, Size(5, 5), Size(-1, -1),
      // TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1));
      drawChessboardCorners(rgbImageL, boardSize, cornerL, isFindL);
      // imshow("chessboardL", rgbImageL);
      imagePointL.push_back(cornerL);

      // cornerSubPix(grayImageR, cornerR, Size(5, 5), Size(-1, -1),
      // TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1));
      drawChessboardCorners(rgbImageR, boardSize, cornerR, isFindR);
      // imshow("chessboardR", rgbImageR);
      imagePointR.push_back(cornerR);
      FrameCount++;
      goodFrameCount++;
      cout << "The image" << FrameCount << " is good WITH " << cornerL.size()  << endl;
    } else {
      cout << "The image " << FrameCount << "is bad please try again" << endl;
      FrameCount++;
    }

    if (waitKey(10) == 'q') {
      break;
    }
  }

  calRealPoint(objRealPoint, boardWidth, boardHeight, goodFrameCount - 1,
               squareSize);
  cout << "cal real successful with size " << objRealPoint.size() << endl;
  cout << "img leftsize : " << imagePointL.size() << ",image rightsize: " << imagePointR.size() << endl;
  std::vector<cv::Vec3d> rotation_vectors;    /* 每幅图像的旋转向量 */
  std::vector<cv::Vec3d> translation_vectors; /* 每幅图像的平移向量 */
  int flags = 0;
  flags |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
  flags |= cv::fisheye::CALIB_CHECK_COND;
  flags |= cv::fisheye::CALIB_FIX_SKEW;
  fisheye::calibrate(objRealPoint, imagePointL, imgSize, cameraMatrixL, distCoeffL,
                     rotation_vectors, translation_vectors, flags,
                     cv::TermCriteria(3, 20, 1e-6));
  cout << "cameraMatrixL: " << cameraMatrixL << endl;
  cout << "distCoeffL: " << distCoeffL << endl;
  rotation_vectors.clear();
  translation_vectors.clear();
  fisheye::calibrate(objRealPoint, imagePointR, imgSize, cameraMatrixR, distCoeffR,
                     rotation_vectors, translation_vectors, flags,
                     cv::TermCriteria(3, 20, 1e-6));
  cout << "cameraMatrixR: " << cameraMatrixR << endl;
  cout << "distCoeffR: " << distCoeffR << endl;

  /*******双目标定开始***********/

  double rms = cv::fisheye::stereoCalibrate(
      objRealPoint, imagePointL, imagePointR, cameraMatrixL, distCoeffL,
      cameraMatrixR, distCoeffR, Size(imageWidth, imageHeight), R, T, 
      fisheye::CALIB_FIX_INTRINSIC + fisheye::CALIB_FIX_K4 + fisheye::CALIB_FIX_SKEW,
      TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 80, 1e-5)
	  );
  cout << "Stereo Calibration done with RMS error = " << rms << endl;
  cout << "R: " << R << endl;
  cout << "T: " << T << endl;

  cv::fisheye::stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize,
                R, T, Rl, Rr, Pl, Pr, Q, CALIB_ZERO_DISPARITY, imageSize);
  std::cout << "validRoi[0] :" << validROIL << std::endl;
  std::cout << "validRoi[1]: " << validROIR << std::endl;

  cv::fisheye::initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pl, imageSize,
                          CV_32FC1, mapLx, mapLy);
  cv::fisheye::initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize,
                          CV_32FC1, mapRx, mapRy);

  Mat ImageL, ImageR;
  Mat rectifyImageL2, rectifyImageR2;
  cout << "debug" << endl;
  for (int num = 1; num < frameNumber; num++) {
    string filenamel, filenamer;
    std::string tmp_str = to_string(num);
    if (num < 10) tmp_str = '0' + to_string(num);
    filenamel = folder_ + format_L + tmp_str + ".jpg";
    filenamer = folder_ + format_R + tmp_str + ".jpg";
    ImageL = imread(filenamel, -1);
    ImageR = imread(filenamer, -1);
    if (ImageL.empty() || ImageR.empty()) {
      std::cout << "Failed to open file" << filenamel << std::endl
                << "Or " << filenamer << std::endl;
      return -1;
    }

    remap(ImageL, rectifyImageL2, mapLx, mapLy, INTER_LINEAR);
    remap(ImageR, rectifyImageR2, mapRx, mapRy, INTER_LINEAR);
    imshow("rectifyImageL", rectifyImageL2);
    imshow("rectifyImageR", rectifyImageR2);

    outputCameraParam();
    /*Mat canvas;
    double sf;
    int w, h;
    sf = 600. / MAX(imageSize.width, imageSize.height);
    w = cvRound(imageSize.width * sf);
    h = cvRound(imageSize.height * sf);
    canvas.create(h, w * 2, CV_8UC3);

    // draw the left part
    Mat canvasPart = canvas(Rect(w * 0, 0, w, h));
    resize(rectifyImageL2, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
    Rect vroiL(cvRound(validROIL.x * sf), cvRound(validROIL.y * sf),
               cvRound(validROIL.width * sf), cvRound(validROIL.height * sf));
    rectangle(canvasPart, vroiL, Scalar(0, 0, 255), 3, 8);
    cout << "Painted ImageL" << endl;

    // draw the right part
    canvasPart = canvas(Rect(w, 0, w, h));
    resize(rectifyImageR2, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
    Rect vroiR(cvRound(validROIR.x * sf), cvRound(validROIR.y * sf),
               cvRound(validROIR.width * sf), cvRound(validROIR.height * sf));
    rectangle(canvasPart, vroiR, Scalar(0, 255, 0), 3, 8);
    cout << "Painted ImageR" << endl;

    for (int i = 0; i < canvas.rows; i += 16)
      line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);
    imshow("rectified", canvas);*/
    char c = (char)waitKey();
    if (c == 27 || c == 'q' || c == 'Q') break;
  }
  return 0;
}
