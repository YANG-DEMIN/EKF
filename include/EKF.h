#pragma once

#include "parameter.h"
#include "GNSS_INFO.h"
#include "Trajectory_Generator.hpp"
#include "Noise_Generator.h"
#include "INS_Calculate.h"


class EKF
{
public:
	EKF(void);													//构造函数
	~EKF(void);													//析构函数
	void Init(void);											//初始化
	void WriteData(void);										//写入数据
	void Add_Noise_EKF(SensorData* RawDa, SensorNoise* sensor_n, SensorData* NoiseDa);
	void INS_Output(SensorData* NoiseD, FlightState* SimS);		//INS输出
	void StateUpdate(void);										//EKF状态更新
	void Update(void);											//大循环更新



private:
	//私有结构体
	EKF_STATE* EKF_State;
	SensorData* NoiseData_EKF;
	FlightState* INS_State;

protected:
	//局部变量
	Vector3d Hp,att,omega;												//位置量测矩阵中的非零元素,姿态误差
	Vector4d q, delta_q;												//四元数
	Matrix15d F,I,Phi;
	Vector6d FM_vector;
	//Vector9d P0_vector;
	//Vector6d Z,R_vector,FM_vector,I_WANT_TO;							//量测数据矩阵
	Vector3d Z, R_vector, I_WANT_TO;
	//Matrix15_6d K;													//增益矩阵
	Matrix15_3d K;
	//Matrix6_15d H;													//量测矩阵
	Matrix3_15d H;
	Matrix15d P_k, Pk_k_1, Q_Constant, Q_k;											//系统干扰方差
	Matrix6d FM; //R													//测量噪声方差
	Matrix3d R;
	Vector15d X,Q_vector,X_INS,Xk_k_1, P0_vector;						//系统状态
	Matrix9_6d FS;
	double Rm, Rn;
	double lambda, L, h, vx, vy, vz, gamma, psi, phi;					//定义局部变量
	double dL, dlambda, dh, dvx, dxy, dvz;								//增量
	double a_x, a_y, a_z, omega_x, omega_y, omega_z;
	double fx, fy, fz;
	double Acc_Markov_driver, Gyro_Markov_driver;
	int i;
	Vector3d f_b, f_n;
	Vector3d g_n, omega_ie, omega_en, omega_in;
	Vector3d omega_in_b, omega_nb_b, omega_ib_b;
	Vector3d dV;
	Vector4d Q, dQ;
	Matrix3d C_nb, C_bn;

	//调用实例
	GNSS_Information* gi_EKF;
	TrajectoryGenerator* ty_EKF;
	Noise_Generator* ng_EKF;
	ofstream DataWrite,StateWrite, P_k_Write;												//写入函数

	string ExplainEKFContent[16] = { "1-time", "2-velocity_x", "3-velocity_y","4-velocity_z", 
		"4-滚转γ","5-偏航ψ","6-俯仰φ","7-纬度L", "8-经度λ","9-高度h",
		"10		","11	","12		","13	","14	","15	"};
	string ExplainFlightContent[20] = { "1-飞行时间t","2-纬度L","3-经度λ","4-高度h",
			"5-速度vx","6-速度vy","7-速度vz","8-滚转γ","9-偏航ψ","10-俯仰φ" };
};

