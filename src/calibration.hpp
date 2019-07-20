 #include <opencv2/opencv.hpp>
#include "Eigen/Core"
#include "ceres/ceres.h"
#include "ceres/rotation.h"
struct ReprojectionCost
{
	cv::Point2f observation;
	ReprojectionCost(cv::Point2f &) :observation(observation) {}

	//������ͶӰ����
	// intrinsic �ڲΣ�ע�����ﲻ���ڲξ���A�����ǰ����ڲ��б����ľ���intrinsic=[fx,fy,cx,cy,]
	// extrinsic ���,
	// pos3d     ��ά�����
	// k1,k2     ���������
	//residuls   �в�
	//����const T* const intrinsic ����⣺ �Ӻ���ǰ��⣬��һ��const��ʾ�����ָ���ǳ���������ַ���ɸ��ģ��ڶ���const��ʾ��ַָ������ݲ��ɸ���
	//ͬ��k1��k2Ҳ����ˣ�����k1��k2�����Ҳ�ǵ�ַ��������������ʱ��Ҫֵ���ǵ�ַ����ĵڼ�������k1[0];

	template <typename T>
	bool operator()(const T* const intrinsic, const T* const extrinsic, const T* const pos3d, const T* const k1, const T* const k2, T* residuals) const
	{
		const T* r = extrinsic;
		const T* t = &extrinsic[3];

		//���������������ת��ƽ��
		T projt3d[3];       //������ת����������
		ceres::AngleAxisRotatePoint(r, pos3d, projt3d);
		projt3d[0] += t[0];
		projt3d[1] += t[1];
		projt3d[2] += t[2];

		//��һ�������������
		T x = projt3d[0] / projt3d[2];
		T y = projt3d[1] / projt3d[2];

		//�����ڲξ���
		T fx = intrinsic[0];
		T fy = intrinsic[1];
		T cx = intrinsic[2];
		T cy = intrinsic[3];

		//�������ͶӰ�����������
		T tmp_u = fx*x + cx;
		T tmp_v = fy*y + cy;

		//������ͶӰ����������ϵ�µ����꣬����������
		T new_x = (tmp_u - cx)*T(0.003);
		T new_y = (tmp_v - cy)*T(0.003);
		T r2 = pow(new_x, 2) + pow(new_y, 2);

		//�������Ӱ����ټ�����ͶӰ���
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
		//��3*3����ת���������޵����ʽת��Ϊֻ��������������ƽ�Ʊ���t���ֻ��6�������������
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

		//�������
		for (size_t i = 0; i < extrinsic.size(); i++)
		{
			calibration.AddParameterBlock(extrinsic[i].ptr<double>(), 6);
		}

		//�̶���һ�������������һ��ͼƬ����ʾ������ϵ������������ϵ
		// calibration.SetParameterBlockConstant(extrinsic[0].ptr<double>());  //�궨�����в���Ҫ�̶���һ��ͼ����Σ���һ�������ؽ��Ĳ���
		//�����ڲ�
		calibration.AddParameterBlock(intrinsic.ptr<double>(), 4);
		//���ػ������
		calibration.AddParameterBlock(&k1, 1);        //??????AddParameterBlock ��cost function<>�������������˳��
		calibration.AddParameterBlock(&k2, 1);

		//���ص�
		ceres::LossFunction* loss_function = new ceres::HuberLoss(4);
		//��ȡÿ��ͼ����ÿ��������Ӧ��ͼ��������������꣬�������������������ν�����ͶӰ��������ͶӰ���
		for (size_t img_idx = 0; img_idx < img_pt_idx.size(); img_idx++)
		{
			std::vector<cv::Point3f> current_3dloc = world_loc[img_idx];
			std::vector<cv::Point2f> current_imgloc = imgPoints[img_idx];
			for (size_t point_idx = 0; point_idx < current_3dloc.size(); point_idx++)
			{
				cv::Point2f img_pt = current_imgloc[point_idx];

				//AutoDiffCostFunctionģ���� <>���βε�˳��Ϊ <��ͶӰ���ۺ������в��������ģ��1�Ĳ���������ģ��2...>
				//����Ϊ<��ͶӰ���ۺ������в���2�����ڲα���4������α���6������������3��>
				ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<ReprojectionCost, 2, 4, 6, 3, 1, 1>(new ReprojectionCost(img_pt));
				//��Ӳв���Լ��в��Ӧ����ı���������Ӧ�ö���ָ�����ʽ�����������vector����ʽ
				calibration.AddResidualBlock(cost_function, loss_function, intrinsic.ptr<double>(), extrinsic[img_idx].ptr<double>(), &(current_3dloc[point_idx].x), &k1, &k2);
			}
		}

		//���BA
		ceres::Solver::Options ceres_config_options;
		ceres_config_options.minimizer_progress_to_stdout = false;
		ceres_config_options.logging_type = ceres::SILENT;
		ceres_config_options.num_threads = 1;
		ceres_config_options.preconditioner_type = ceres::JACOBI;             //���Ե�������
		ceres_config_options.linear_solver_type = ceres::SPARSE_SCHUR;
		ceres_config_options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;     //����ϡ�����Դ�����

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