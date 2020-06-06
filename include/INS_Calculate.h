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
	double lambda, L, h, vx, vy, vz, gamma, psi, phi;				//����ֲ�����
	double dL, dlambda,dh, dvx, dxy, dvz;								//����
	double a_x, a_y, a_z, omega_x, omega_y, omega_z;
	Vector3d f_b, f_n; 
	Vector3d g_n, omega_ie, omega_en, omega_in;
	Vector3d omega_in_b, omega_nb_b, omega_ib_b;
	Vector3d dV;
	Vector4d Q, dQ;
	Matrix3d C_nb, C_bn;

	ofstream DataWrite, TraceWrite;										//д�뺯��

	TrajectoryGenerator* ty;
	Noise_Generator* ng;
	
	string ExplainFlightContent[20] = { "1-����ʱ��t","2-γ��L","3-���Ȧ�","4-�߶�h",
			"5-�ٶ�vx","6-�ٶ�vy","7-�ٶ�vz","8-��ת��","9-ƫ����","10-������" };
	string ExplainSensorContent[14] = { "1-����ʱ��t","2-acc_x","3-acc_y","4-acc_z",
			"5-gyro_x","6-gyro_y","7-gyro_z" };
};

