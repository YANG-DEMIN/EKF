#pragma once

#include "parameter.h"

class Noise_Generator
{

public:
	Noise_Generator(void);
	 ~Noise_Generator(void);
	void Init(void);
	void WriteData(void);										//д�����ݵ��ļ�
	static double White_Generator(double std_div);						//���뷽�����������
	//double Markov_Generator(double deltat, double T_constant,
	//	double mark_var, double white_var, double mark_noi_last);			//
	double Markov_Generator(double deltat, double T_constant,
		double mark_var, double mark_noi_last);	//����ʱ�� ʱ�䳣�� ����Ʒ򷽲� ��һ������ 
	void NoiseGeneratorUpdate(SensorNoise* sensor_n);

	SensorNoise* sensor_noise;									//�������ݽṹ��
											//��������

protected:
	
	double Markov_Noise_acc_x, Markov_Noise_acc_y, Markov_Noise_acc_z;
	double Markov_Noise_gyro_x, Markov_Noise_gyro_y, Markov_Noise_gyro_z;
	ofstream Noise_write;										//д�뺯��
	string ExplainSensorNoisecontent[14] = { "1-����ʱ��t","2-acc_x","3-acc_y","4-acc_z",
			"5-gyro_x","6-gyro_y","7-gyro_z" };
};
