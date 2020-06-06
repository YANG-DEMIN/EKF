#pragma once

#include "Trajectory_Generator.hpp"
#include "Noise_Generator.h"

class INS_Calculate
{
public:
	INS_Calculate(void);
	~INS_Calculate(void);
	void INS_Update(void);
	void INS_Init(void);
	void Add_Noise(SensorData* RawData, SensorNoise* sensor_n, SensorData* NoiseData);
	void WriteData(void);
	static Matrix4d Vector2Matrix(Vector3d Vec3);
	static Matrix3d Q2DCM(Vector4d q);
	static Vector3d Q2Euler(Vector4d q);
	static Vector4d Euler2Q(Vector3d att);
	static Vector3d DCM2Euler(Matrix3d DCM);
	static Matrix3d Euler2DCM(Vector3d att);
	void StateUpdate(SensorData* NoiseD, FlightState* SimS);
	
	SensorData* NoiseData;
	FlightState* SimState;
	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
private:

protected:
	double Rm, Rn;
	double lambda, L, h, vx, vy, vz, gamma, psi, phi;				//定义局部变量
	double dL, dlambda,dh, dvx, dxy, dvz;								//增量
	double a_x, a_y, a_z, omega_x, omega_y, omega_z;
	Vector3d f_b, f_n; 
	Vector3d g_n, omega_ie, omega_en, omega_in;
	Vector3d omega_in_b, omega_nb_b, omega_ib_b;
	Vector3d dV;
	Vector4d Q, dQ;
	Matrix3d C_nb, C_bn;

	ofstream DataWrite, TraceWrite;										//写入函数

	TrajectoryGenerator* ty;
	Noise_Generator* ng;
	
	string ExplainFlightContent[20] = { "1-飞行时间t","2-纬度L","3-经度λ","4-高度h",
			"5-速度vx","6-速度vy","7-速度vz","8-滚转γ","9-偏航ψ","10-俯仰φ" };
	string ExplainSensorContent[14] = { "1-飞行时间t","2-acc_x","3-acc_y","4-acc_z",
			"5-gyro_x","6-gyro_y","7-gyro_z" };
};

