#pragma once

#include "parameter.h"
#include "GNSS_INFO.h"
#include "Trajectory_Generator.hpp"
#include "Noise_Generator.h"
#include "INS_Calculate.h"


class EKF
{
public:
	EKF(void);													//���캯��
	~EKF(void);													//��������
	void Init(void);											//��ʼ��
	void WriteData(void);										//д������
	void Add_Noise_EKF(SensorData* RawDa, SensorNoise* sensor_n, SensorData* NoiseDa);
	void INS_Output(SensorData* NoiseD, FlightState* SimS);		//INS���
	void StateUpdate(void);										//EKF״̬����
	void Update(void);											//��ѭ������



private:
	//˽�нṹ��
	EKF_STATE* EKF_State;
	SensorData* NoiseData_EKF;
	FlightState* INS_State;

protected:
	//�ֲ�����
	Vector3d Hp,att,omega;												//λ����������еķ���Ԫ��,��̬���
	Vector4d q, delta_q;												//��Ԫ��
	Matrix15d F,I,Phi;
	Vector6d FM_vector;
	//Vector9d P0_vector;
	//Vector6d Z,R_vector,FM_vector,I_WANT_TO;							//�������ݾ���
	Vector3d Z, R_vector, I_WANT_TO;
	//Matrix15_6d K;													//�������
	Matrix15_3d K;
	//Matrix6_15d H;													//�������
	Matrix3_15d H;
	Matrix15d P_k, Pk_k_1, Q_Constant, Q_k;											//ϵͳ���ŷ���
	Matrix6d FM; //R													//������������
	Matrix3d R;
	Vector15d X,Q_vector,X_INS,Xk_k_1, P0_vector;						//ϵͳ״̬
	Matrix9_6d FS;
	double Rm, Rn;
	double lambda, L, h, vx, vy, vz, gamma, psi, phi;					//����ֲ�����
	double dL, dlambda, dh, dvx, dxy, dvz;								//����
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

	//����ʵ��
	GNSS_Information* gi_EKF;
	TrajectoryGenerator* ty_EKF;
	Noise_Generator* ng_EKF;
	ofstream DataWrite,StateWrite, P_k_Write;												//д�뺯��

	string ExplainEKFContent[16] = { "1-time", "2-velocity_x", "3-velocity_y","4-velocity_z", 
		"4-��ת��","5-ƫ����","6-������","7-γ��L", "8-���Ȧ�","9-�߶�h",
		"10		","11	","12		","13	","14	","15	"};
	string ExplainFlightContent[20] = { "1-����ʱ��t","2-γ��L","3-���Ȧ�","4-�߶�h",
			"5-�ٶ�vx","6-�ٶ�vy","7-�ٶ�vz","8-��ת��","9-ƫ����","10-������" };
};

