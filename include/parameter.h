#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include "math.h"
#include <random>						//随机数
#include <Eigen/Dense>					//C++ 矩阵计算工具
#include <iomanip>
#include <map>
#include <cmath>

using namespace std;
using namespace Eigen;
using Eigen::Matrix3d;					//三维double矩阵
using Eigen::Vector3d;					//三维double向量
using Eigen::Vector4d;
using Eigen::Quaterniond;
using Eigen::MatrixXd;					//自定义维度向量

typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;
typedef Matrix<double, 9, 6> Matrix9_6d;
typedef Matrix<double, 9, 1> Vector9d;
typedef Matrix<double, 15, 1> Vector15d;
typedef Matrix<double, 15, 15> Matrix15d;
typedef Matrix<double, 15, 6> Matrix15_6d;
typedef Matrix<double, 15, 3> Matrix15_3d;
typedef Matrix<double, 6, 15> Matrix6_15d;
typedef Matrix<double, 3, 15> Matrix3_15d;


#define Omega_i_e	7.292115e-5			//地球自转角速度  unit: rad/s
#define g			9.8					//重力加速度  unit: m/s^2
#define a			6378137				//地球长半轴  unit: m 地球椭球模型WGS-84椭球
#define f			1/298.257223563		//地球曲率
#define lambda_0	100*RAD				//初始经度  unit:angle
#define L_0			30*RAD				//初始纬度  unit:angle
#define h_0			1000				//初始经度  unit:m
#define vN_0		1200				//初始速度  unit:m/s
#define PI			3.1415926535897932  //pi
#define RAD         0.0174532925199432  //角度转换为弧度
#define DEG         57.295779513082323  //弧度转换为角度
#define Flight_Time		100				//飞行时间
#define INS_UPDATE_TIME 0.01			//惯性器件更新时间 unit: s
#define GNSS_UPDATE_TIME 1				//GNSS数据更新时间 unit: s
#define N				 10000			//迭代次数，下面代码并没有用Flight/Update_Time表示此处

#define Acc_Markov_Variance			(10e-5)*g		//加计马尔科夫过程方差 unit:m/s^2
#define Acc_Markov_T_Constant		3600			//加计马尔科夫过程时间常数 unit: s
#define Acc_White_Variance			(10e-6)*g		//加计驱动白噪声
#define Gyro_Markov_Variance		1 * RAD /3600	//陀螺仪马尔科夫过程方差 unit: rad/s
#define Gyro_Markov_T_Constant		3600			//陀螺仪马尔科夫过程时间常数 unit: s
#define Gyro_White_Variance			0.1 * RAD /3600 //陀螺仪驱动白噪声

#define GNSS_Position_Variance		10				//定位系统位置方差 unit:m
#define GNSS_Velocity_Variance		0.1				//定位系统速度方差 unit:m/s
#define INS_Position_Variance		10				//惯性系统初始对准位置方差 unit:m
#define INS_Velocity_Variance		0.1				//惯性系统初始对准速度方差 unit:m/s
#define INS_Attitude_Variance		0.1*RAD			//惯性系统初始对准姿态方差 unit:°


typedef struct							//导航坐标系(NUE)下飞行状态量
{
	double t;									//飞行时间
	Vector3d position, velocity, attitude;		//位置、速度、姿态
	//Matrix3d Dcm;								//方向余弦矩阵
	//Quaterniond q;							//四元数

}FlightState;

typedef struct									//传感器数据
{
	double t;									//飞行时间
	Vector3d f_b;								//体系下比力
	Vector3d omega_b;							//体系下陀螺仪输出

	int	N_amount;								//迭代次数

}SensorData;



typedef struct							//体系下的惯性器件噪声
{
	double t;							//飞行时间
	Vector3d acc, gyro;					//加速度计 陀螺仪
	Vector3d GNSS_position;				//位置噪声
	Vector3d GNSS_velocity;				//速度噪声
	int N_amount;						//次数统计

}SensorNoise;

typedef struct							//GNSS系统信息
{
	double t;
	Vector3d position;					//位置
	Vector3d velocity;					//速度
	int N_amount;
}GNSS_INFO;

typedef struct							//卡尔曼滤波状态-误差
{
	double t;							//飞行时间
	Vector15d X;						//15个状态量
	Vector15d X_erro;					//15个状态误差量——为真实的在EKF中更新的量
	int N_amount;

}EKF_STATE;

