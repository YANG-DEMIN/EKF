#pragma once

#include "parameter.h"

class Noise_Generator
{

public:
	Noise_Generator(void);
	 ~Noise_Generator(void);
	void Init(void);
	void WriteData(void);										//写入数据到文件
	static double White_Generator(double std_div);						//输入方差，输出随机噪声
	//double Markov_Generator(double deltat, double T_constant,
	//	double mark_var, double white_var, double mark_noi_last);			//
	double Markov_Generator(double deltat, double T_constant,
		double mark_var, double mark_noi_last);	//采样时间 时间常数 马尔科夫方差 上一步噪声 
	void NoiseGeneratorUpdate(SensorNoise* sensor_n);

	SensorNoise* sensor_noise;									//噪声数据结构体
											//噪声矩阵

protected:
	
	double Markov_Noise_acc_x, Markov_Noise_acc_y, Markov_Noise_acc_z;
	double Markov_Noise_gyro_x, Markov_Noise_gyro_y, Markov_Noise_gyro_z;
	ofstream Noise_write;										//写入函数
	string ExplainSensorNoisecontent[14] = { "1-飞行时间t","2-acc_x","3-acc_y","4-acc_z",
			"5-gyro_x","6-gyro_y","7-gyro_z" };
};
