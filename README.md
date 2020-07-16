# FisheyeCalibration
## 第一步：采集图像
`./src/captureImage video_num path_to_save_images MONO`
其中videonum是摄像头在设备下的编号，如/dev/video1， 则video_num 为1
path_to_save_images: 用于保存采集来的标定图像的路径

窗口出来之后，按a键采集图像， 采集图像时的要求， 尽量保持相机不动，移动标定板，尽可能上下左右地多角度采集图像，保证最后采集出的图片集中，标定板在图像的各个角落都有出现(尽量全)，不同角度的图片总共25张左右即可；

具体的采集图像代码参见captureImage.cpp, 可根据自己的实际情况修改代码；

## 第二步： 相机标定
`./src/fisheyeCalib path_to_calibration_images`

path_to_calibration_images: 是第一步采集来的图像的保存路径，这里直接传入即可；

该程序运行后，会在终端有很多log输出，主要关注：
### 1. 有效的图片张数
   也就是log中的：总共有 ｎ 张图片，角点提取完成！ 这里ｎ最好大于 20
### 2. 标定完成后的重投影误差
   也就是log中的：总体平均误差：0.0931465像素，保证误差值小于0.5，小于0.1更好；
### 3. 标定结果：
**相机内参数矩阵：**内参矩阵(intrinsic Mat)为３＊３的矩阵，形式如下图：
　　
   ```
 fx , 0,  cx
     0 , fy, cy
     0 , 0,   1
```
其中fx,fy为焦距，以像素为单位，为实际的光学焦距除以CCD的显示参数dx (或者dy)所得；即fx = f/dx ;fy = f/dy

对同一厂家的相机而言，不同FOV的相机所对应的fx有一个规律，即FOV越大，fx越小，这个可以拿来做标定结果的一个参考； 

cx,cy为实际的图像中心，最理想的情况为cx = Width/2,cy = Height/2;但实际情况往往会在这个数值附近浮动；

**畸变系数：**畸变系数一般为1*4或者1*5的矩阵，这里为1*4,具体形式为：
    [k1,k2,p1,p2]

### 4. 检查标定结果：
   与采集来的图像同目录下，会将检测到的棋盘格角点和去畸变图像(校正后的图像)一同保存，_corner.jpg为角点图，_d.jpg为去畸变图像，检测这个图像可以检验自己的去畸变效果
具体代码参见： fisheye_calibrate.cpp

## 第三步：获取实时的去畸变图像
`./src/captureImage video_num 　path_to_save_images UNDISTORTION`

video_num 与第一步的意义一样

path_to_save_images： 这个的这个arg是无意义的，可以随意填入路径，不影响结果

UNDISTORTION： 表示要调用去畸变功能；

具体代码在captureImage.cpp中，这里可以根据自己实际的图像或视频采集条件修改，**最主要的函数是**：RealTimeUndistort()

这里需要手动修改k1,k2,p1,p2的值 以及fx,fy,cx,cy，也就是第二步的标定结果；

所使用模型论文参加：

Kannala J, Brandt S S. A generic camera model and calibration method for conventional, wide-angle, and fish-eye lenses[J]. Pattern Analysis and Machine Intelligence, IEEE Transactions on, 2006, 28(8): 1335-1340.