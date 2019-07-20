#include <opencv2/opencv.hpp>
#include "Eigen/Core"
#include "Eigen/Geometry"

int main()
{
	

	Eigen::Matrix3d RotationMatrix;
	/*RotationMatrix <<0.99806, -0.0592349, 0.0191656,
                     0.0597737, 0.997794, -0.0288826,
                     -0.0174124, 0.0299722, 0.999399;*/



	/*RotationMatrix << -0.481264 ,- 0.0126191, 0.876485, 
		0.0705732, 0.996093, 0.0530918 ,
		- 0.87373, 0.0874075 ,- 0.478493 ;*/
	RotationMatrix << -0.464389 ,0.013224, - 0.885532 ,
		0.0310061, 0.999518, - 0.00133391 ,
		0.885088, - 0.0280763, - 0.464576 ;
     Eigen::Vector3d euler_angles=RotationMatrix.eulerAngles(2,1,0);
	 std::cout << "yaw(Z) pitch(Y) roll(X)=\n" << euler_angles.transpose() << std::endl;
	 std::cout << "actural rotation around Z : " << euler_angles[0] / M_PI * 180 << std::endl
		 << "actural rotation around Y : " << euler_angles[1] / M_PI * 180 << std::endl
		 << "actural rotation around X : " << euler_angles[2] / M_PI * 180 << std::endl;
	 return 0;
}
