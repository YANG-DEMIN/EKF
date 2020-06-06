
#include "Noise_Generator.h"


 Noise_Generator::Noise_Generator(void)									//����ռ估д���ļ�����
{
	 sensor_noise = new SensorNoise;

	 Noise_write.open("result\\Noisedata.txt", ios::out | ios::trunc);
	 Noise_write.setf(ios::fixed, ios::floatfield); Noise_write.precision(9);
};

 Noise_Generator::~Noise_Generator(void)								//��������
 {
	 delete sensor_noise;
	 Noise_write.close();
 }

void Noise_Generator::Init(void)										//��ʼ��
{
	sensor_noise->t				 = 0;
	sensor_noise->N_amount		 = 0;
	sensor_noise->acc			 = { 0, 0, 0 };
	sensor_noise->gyro				= { 0, 0, 0 };
	sensor_noise->GNSS_position = { 0, 0, 0 };
	sensor_noise->GNSS_velocity = { 0, 0, 0 };

	Markov_Noise_acc_x = 0;
	Markov_Noise_acc_y = 0;
	Markov_Noise_acc_z = 0;
	Markov_Noise_gyro_x = 0;
	Markov_Noise_gyro_y = 0;
	Markov_Noise_gyro_z = 0;
};

void Noise_Generator::WriteData(void)									//д����������
{
	int DimData = 7;
	double* Optr = (double*)sensor_noise;
	if (sensor_noise->N_amount == 0)
	{
		for (int i = 0; i < DimData; i++)
		{
			Noise_write << ExplainSensorNoisecontent[i] << "		";
		}
		Noise_write << endl;
	}
	for (int i = 0; i < DimData; i++)
	{
		Noise_write << Optr[i] << "		";
	}
	Noise_write << endl;
	sensor_noise->N_amount++;

};

double Noise_Generator::White_Generator(double std_dev)					//���������ɺ���
{
	random_device rd;
	mt19937 gen(rd());

	normal_distribution<> d(0, std_dev);

	return d(gen);
};

double Noise_Generator::Markov_Generator(double deltat, double T_constant,
	double mark_var, double mark_noi_last)									//������ɢ��ʽ
{
	double driver, phi_m, Mar_Noise;											//����ֲ�����

	/*Mar_Noise = mark_noi_last + (-mark_noi_last / T_constant + White_Generator(mark_var)) * deltat;*/
	driver = sqrt(mark_var * mark_var * 4 / T_constant);
	phi_m		= exp(-deltat / T_constant);							//Ȩֵ
	Mar_Noise	 = phi_m * mark_noi_last + White_Generator(driver);	//ǰһ������� * phi_m+ �������������İ�����

	return Mar_Noise;													//��������Ʒ�����
};


void Noise_Generator::NoiseGeneratorUpdate(SensorNoise* sensor_n)
{

	sensor_noise->t = sensor_noise->t + INS_UPDATE_TIME;
	Markov_Noise_acc_x = Markov_Generator(INS_UPDATE_TIME, Acc_Markov_T_Constant,
		Acc_Markov_Variance, Markov_Noise_acc_x);
	sensor_noise->acc(0) = White_Generator(Acc_White_Variance) + Markov_Noise_acc_x;

	Markov_Noise_acc_y = Markov_Generator(INS_UPDATE_TIME, Acc_Markov_T_Constant,
		Acc_Markov_Variance, Markov_Noise_acc_y);
	sensor_noise->acc(1) = White_Generator(Acc_White_Variance) + Markov_Noise_acc_y;

	Markov_Noise_acc_z = Markov_Generator(INS_UPDATE_TIME, Acc_Markov_T_Constant,
		Acc_Markov_Variance, Markov_Noise_acc_z);
	sensor_noise->acc(2) = White_Generator(Acc_White_Variance) + Markov_Noise_acc_z;

	Markov_Noise_gyro_x = Markov_Generator(INS_UPDATE_TIME, Gyro_Markov_T_Constant,
		Gyro_Markov_Variance, Markov_Noise_gyro_x);
	sensor_noise->gyro(0) = White_Generator(Gyro_White_Variance) + Markov_Noise_gyro_x;

	Markov_Noise_gyro_y = Markov_Generator(INS_UPDATE_TIME, Gyro_Markov_T_Constant,
		Gyro_Markov_Variance, Markov_Noise_gyro_y);
	sensor_noise->gyro(1) = White_Generator(Gyro_White_Variance) + Markov_Noise_gyro_y;

	Markov_Noise_gyro_z = Markov_Generator(INS_UPDATE_TIME, Gyro_Markov_T_Constant,
		Gyro_Markov_Variance, Markov_Noise_gyro_z);
	sensor_noise->gyro(2) = White_Generator(Gyro_White_Variance) + Markov_Noise_gyro_z;

	WriteData();
};