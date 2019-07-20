 #include <opencv2/opencv.hpp>
#include "Eigen/Core"
#include "ceres/ceres.h"
#include "ceres/rotation.h"
struct ReprojectionCost
{
	cv::Point2f observation;
	ReprojectionCost(cv::Point2f &) :observation(observation) {}

	//定义重投影误差函数
	// intrinsic 内参，注意这里不是内参矩阵A，而是包含内参中变量的矩阵intrinsic=[fx,fy,cx,cy,]
	// extrinsic 外餐,
	// pos3d     三维坐标点
	// k1,k2     畸变参数；
	//residuls   残差
	//对于const T* const intrinsic 的理解： 从后往前理解，第一个const表示传入的指针是常量，即地址不可更改，第二个const表示地址指向的内容不可更改
	//同理k1，k2也是如此，这里k1，k2传入的也是地址，因此在下面调用时需要值除是地址里面的第几个参数k1[0];

	template <typename T>
	bool operator()(const T* const intrinsic, const T* const extrinsic, const T* const pos3d, const T* const k1, const T* const k2, T* residuals) const
	{
		const T* r = extrinsic;
		const T* t = &extrinsic[3];

		//对世界坐标进行旋转和平移
		T projt3d[3];       //经过旋转矩阵后的坐标
		ceres::AngleAxisRotatePoint(r, pos3d, projt3d);
		projt3d[0] += t[0];
		projt3d[1] += t[1];
		projt3d[2] += t[2];

		//归一化后的世界坐标
		T x = projt3d[0] / projt3d[2];
		T y = projt3d[1] / projt3d[2];

		//乘以内参矩阵
		T fx = intrinsic[0];
		T fy = intrinsic[1];
		T cx = intrinsic[2];
		T cy = intrinsic[3];

		//计算出重投影后的像素坐标
		T tmp_u = fx*x + cx;
		T tmp_v = fy*y + cy;

		//计算重投影后的相机坐标系下的坐标，即物理坐标
		T new_x = (tmp_u - cx)*T(0.003);
		T new_y = (tmp_v - cy)*T(0.003);
		T r2 = pow(new_x, 2) + pow(new_y, 2);

		//加入畸变影响后再计算重投影误差
		T distort = T(1) + k1[0] * r2 + k2[0] * pow(r2, 2);
		T new_u = (tmp_u - cx)*distort + cx;
		T new_v = (tmp_v - cx)*distort + cy;

		residuals[0] = new_u - T(observation.x);
		residuals[1] = new_v - T(observation.y);

		return true;
	}
};

class CalibrationAlgorithm
{
public:
	CalibrationAlgorithm()
	{}
	~CalibrationAlgorithm() {}

	int getExtrinsicMat(cv::Mat rot,cv::Mat trns,cv::Mat &new_extrinsic)
	{
		//将3*3的旋转矩阵利用罗德里格公式转换为只有三个变量，与平移变量t组成只有6个变量的新外参
			cv::Mat rotaion_vec(3, 1, CV_64FC1);
			Rodrigues(rot, rotaion_vec);
			rotaion_vec.copyTo(new_extrinsic.colRange(0, 3));
			trns.copyTo(new_extrinsic.colRange(3, 6));
	}
/*
	int BundleAdjustment(cv::Mat &intrinsic, std::vector<cv::Mat> &extrinsic,
		std::vector<std::vector<cv::Point3f>> world_loc,
		const std::vector<std::vector<cv::Point2f>> &imgPoints,
		const std::vector<std::vector<int>> img_pt_idx,
		double &k1, double &k2,double &k3) 
	{
		ceres::Problem calibration;

		//加载外参
		for (size_t i = 0; i < extrinsic.size(); i++)
		{
			calibration.AddParameterBlock(extrinsic[i].ptr<double>(), 6);
		}

		//固定第一个相机，即将第一张图片所表示的坐标系看作世界坐标系
		// calibration.SetParameterBlockConstant(extrinsic[0].ptr<double>());  //标定过程中不需要固定第一张图像外参，这一步属于重建的步骤
		//加载内参
		calibration.AddParameterBlock(intrinsic.ptr<double>(), 4);
		//加载畸变参数
		calibration.AddParameterBlock(&k1, 1);        //??????AddParameterBlock 与cost function<>里面参数个数的顺序
		calibration.AddParameterBlock(&k2, 1);

		//加载点
		ceres::LossFunction* loss_function = new ceres::HuberLoss(4);
		//获取每张图像上每个点所对应的图像坐标和世界坐标，对世界坐标根据其内外参进行重投影，保存重投影误差
		for (size_t img_idx = 0; img_idx < img_pt_idx.size(); img_idx++)
		{
			std::vector<cv::Point3f> current_3dloc = world_loc[img_idx];
			std::vector<cv::Point2f> current_imgloc = imgPoints[img_idx];
			for (size_t point_idx = 0; point_idx < current_3dloc.size(); point_idx++)
			{
				cv::Point2f img_pt = current_imgloc[point_idx];

				//AutoDiffCostFunction模板类 <>中形参的顺序为 <重投影代价函数，残差项个数，模块1的参数个数，模块2...>
				//这里为<重投影代价函数，残差项2个，内参变量4个，外参变量6个，世界坐标3个>
				ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<ReprojectionCost, 2, 4, 6, 3, 1, 1>(new ReprojectionCost(img_pt));
				//添加残差块以及残差对应所需的变量，变量应该都以指针的形式输出，而不是vector的形式
				calibration.AddResidualBlock(cost_function, loss_function, intrinsic.ptr<double>(), extrinsic[img_idx].ptr<double>(), &(current_3dloc[point_idx].x), &k1, &k2);
			}
		}

		//求解BA
		ceres::Solver::Options ceres_config_options;
		ceres_config_options.minimizer_progress_to_stdout = false;
		ceres_config_options.logging_type = ceres::SILENT;
		ceres_config_options.num_threads = 1;
		ceres_config_options.preconditioner_type = ceres::JACOBI;             //线性迭代方法
		ceres_config_options.linear_solver_type = ceres::SPARSE_SCHUR;
		ceres_config_options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;     //加载稀疏线性代数库

		ceres::Solver::Summary summary;
		ceres::Solve(ceres_config_options, &calibration, &summary);

		if (!summary.IsSolutionUsable())
		{
			std::cout << "Bundle adjustment failed! \n" << std::endl;
		}
		else
		{
			std::cout << summary.FullReport() << "\n";
		}

		return 0;
	}*/
};