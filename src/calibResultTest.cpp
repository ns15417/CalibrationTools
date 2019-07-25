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

//////求出重心坐标系
/*camera1
1 0 0 -0
0 1 0 -0
0 0 1 -0

camera2
-0.515865 0.591116 0.620052 0.138369
-0.614747 0.248656 -0.748503 -0.15969
-0.596632 -0.767302 0.235114 -0.168594

camera3
-0.529815 -0.578947 -0.619771 -0.129092
0.612018 0.24491 -0.751966 -0.161506
0.587137 -0.777714 0.224569 -0.173803*/


	 Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
	 Eigen::AngleAxisd rotation_vector(M_PI/4, Eigen::Vector3d(1,0,0)); //重心坐标系cam0相对于cam1坐标系 绕x轴顺时针旋转45度
     rotation_matrix = rotation_vector.matrix();


     Eigen::Matrix3d RotationMatrix11,RotationMatrix12,RotationMatrix13;
     Eigen::Vector3d t1,t2,t3;
     RotationMatrix11 <<1,0,0,0,1,0,0,0,1;
     RotationMatrix12 << -0.515865, 0.591116, 0.620052,
                         -0.614747, 0.248656, -0.748503 ,
                          -0.596632, -0.767302, 0.235114;

    RotationMatrix13 << -0.529815, -0.578947, -0.619771 ,
    0.612018, 0.24491, -0.751966,
    0.587137, -0.777714, 0.224569;

    t1 <<0,0,0;
    t2 << 0.138369, -0.15969, -0.168594;
    t3 << -0.129092,-0.161506,-0.173803;


    Eigen::Matrix3d RotationMatrix10 = rotation_matrix;
     Eigen::Vector3d t01 ;
     t01(0) = (t1(0)+t2(0)+t3(0))/3;
     t01(1) = (t1(1)+t2(1)+t3(1))/3;
     t01(2) = (t1(2)+t2(2)+t3(2))/3;

     Eigen::Matrix3d RotationMatrix01 = RotationMatrix10.transpose();//转秩
     Eigen::Vector3d t10 = -RotationMatrix01*t01;

     Eigen::Matrix3d RotationMatrix02 = RotationMatrix12*RotationMatrix01; //先由0-1，再由1-2
     Eigen::Vector3d t20 = RotationMatrix12*t10 + t2;

     Eigen::Matrix3d ROtationMattix03 =  RotationMatrix13 *RotationMatrix01;
     Eigen::Vector3d t30 = RotationMatrix13*t10 + t3;

     std::cout << "RotationMatrix01: \n"<<RotationMatrix01<<std::endl;
     std::cout << "t10: \n"<<t10<<std::endl;

    std::cout << "RotationMatrix02: \n"<<RotationMatrix02<<std::endl;
    std::cout << "t20: \n"<<t20<<std::endl;

    std::cout << "ROtationMattix03: \n"<<ROtationMattix03<<std::endl;
    std::cout << "t30: \n"<<t30<<std::endl;






    return 0;

}
