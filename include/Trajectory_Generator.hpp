/*
 * Trajectory_Generator.hpp
 * Description: ��ʼ�������ɱ�׼�켣�����Ե�������
 *  Created on: 2020��5��4��
 *      Author: ydm
 * 
 */

#ifndef MODULES_TRAJECTORY_GENERATOR_TRAJECTORY_GENERATOR_HPP_
#define MODULES_TRAJECTORY_GENERATOR_TRAJECTORY_GENERATOR_HPP_
#include "parameter.h"
#include "Noise_Generator.h"



class TrajectoryGenerator
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	TrajectoryGenerator(void);						//���캯��
	~TrajectoryGenerator(void);					//��������
	void Init(void);						//��ʼ�����񼰲���
	void WriteData(void);					//д�����ݵ��ļ�
	double White_Generator(double std_div);						//���뷽�����������
	void Trajectory_Generator(FlightState* MissonS);		//���ɱ�׼�켣
	//void GNSS_Noise_Generator(void);
	void TrajectoryGeneratorUpdate(FlightState* MissonS);
	

	FlightState* MissionState;
	//FlightState* SimState;
	SensorData* RawData;
	//SensorData*	NoiseData;

private:



protected:

	double Rm, Rn;
	ofstream TraceWrite, DataWrite;
	Vector3d g_n, omega_ie, omega_en, omega_in;
	Vector3d omega_in_b, omega_nb_b;
	string ExplainFlightContent[20] = { "1-����ʱ��t","2-γ��L","3-���Ȧ�","4-�߶�h",
			"5-�ٶ�vx","6-�ٶ�vy","7-�ٶ�vz","8-��ת��","9-ƫ����","10-������" };
	string ExplainSensorontent[14] = { "1-����ʱ��t","2-fb_x","3-fb_y","4-fb_z",
			"5-wx","6-wy","7-wz" };

};
#endif /* MODULES_TRAJECTORY_GENERATOR_TRAJECTORY_GENERATOR_HPP_ */
