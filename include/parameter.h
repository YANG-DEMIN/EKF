#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include "math.h"
#include <random>						//�����
#include <Eigen/Dense>					//C++ ������㹤��
#include <iomanip>
#include <map>
#include <cmath>

using namespace std;
using namespace Eigen;
using Eigen::Matrix3d;					//��άdouble����
using Eigen::Vector3d;					//��άdouble����
using Eigen::Vector4d;
using Eigen::Quaterniond;
using Eigen::MatrixXd;					//�Զ���ά������

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


#define Omega_i_e	7.292115e-5			//������ת���ٶ�  unit: rad/s
#define g			9.8					//�������ٶ�  unit: m/s^2
#define a			6378137				//���򳤰���  unit: m ��������ģ��WGS-84����
#define f			1/298.257223563		//��������
#define lambda_0	100*RAD				//��ʼ����  unit:angle
#define L_0			30*RAD				//��ʼγ��  unit:angle
#define h_0			1000				//��ʼ����  unit:m
#define vN_0		1200				//��ʼ�ٶ�  unit:m/s
#define PI			3.1415926535897932  //pi
#define RAD         0.0174532925199432  //�Ƕ�ת��Ϊ����
#define DEG         57.295779513082323  //����ת��Ϊ�Ƕ�
#define Flight_Time		100				//����ʱ��
#define INS_UPDATE_TIME 0.01			//������������ʱ�� unit: s
#define GNSS_UPDATE_TIME 1				//GNSS���ݸ���ʱ�� unit: s
#define N				 10000			//����������������벢û����Flight/Update_Time��ʾ�˴�

#define Acc_Markov_Variance			(10e-5)*g		//�Ӽ�����Ʒ���̷��� unit:m/s^2
#define Acc_Markov_T_Constant		3600			//�Ӽ�����Ʒ����ʱ�䳣�� unit: s
#define Acc_White_Variance			(10e-6)*g		//�Ӽ�����������
#define Gyro_Markov_Variance		1 * RAD /3600	//����������Ʒ���̷��� unit: rad/s
#define Gyro_Markov_T_Constant		3600			//����������Ʒ����ʱ�䳣�� unit: s
#define Gyro_White_Variance			0.1 * RAD /3600 //����������������

#define GNSS_Position_Variance		10				//��λϵͳλ�÷��� unit:m
#define GNSS_Velocity_Variance		0.1				//��λϵͳ�ٶȷ��� unit:m/s
#define INS_Position_Variance		10				//����ϵͳ��ʼ��׼λ�÷��� unit:m
#define INS_Velocity_Variance		0.1				//����ϵͳ��ʼ��׼�ٶȷ��� unit:m/s
#define INS_Attitude_Variance		0.1*RAD			//����ϵͳ��ʼ��׼��̬���� unit:��


typedef struct							//��������ϵ(NUE)�·���״̬��
{
	double t;									//����ʱ��
	Vector3d position, velocity, attitude;		//λ�á��ٶȡ���̬
	//Matrix3d Dcm;								//�������Ҿ���
	//Quaterniond q;							//��Ԫ��

}FlightState;

typedef struct									//����������
{
	double t;									//����ʱ��
	Vector3d f_b;								//��ϵ�±���
	Vector3d omega_b;							//��ϵ�����������

	int	N_amount;								//��������

}SensorData;



typedef struct							//��ϵ�µĹ�����������
{
	double t;							//����ʱ��
	Vector3d acc, gyro;					//���ٶȼ� ������
	Vector3d GNSS_position;				//λ������
	Vector3d GNSS_velocity;				//�ٶ�����
	int N_amount;						//����ͳ��

}SensorNoise;

typedef struct							//GNSSϵͳ��Ϣ
{
	double t;
	Vector3d position;					//λ��
	Vector3d velocity;					//�ٶ�
	int N_amount;
}GNSS_INFO;

typedef struct							//�������˲�״̬-���
{
	double t;							//����ʱ��
	Vector15d X;						//15��״̬��
	Vector15d X_erro;					//15��״̬���������Ϊ��ʵ����EKF�и��µ���
	int N_amount;

}EKF_STATE;

