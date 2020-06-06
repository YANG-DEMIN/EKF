/*
 * Trajectory_Generator.cpp
 *
 *  Created on: 2020��5��4��
 *      Author: ydm
 */
#include "Trajectory_Generator.hpp"

TrajectoryGenerator::TrajectoryGenerator(void)
{
	MissionState 	= new FlightState;
	RawData			= new SensorData;
	TraceWrite.open("result\\Flightdata.txt", ios::out | ios::trunc);
	TraceWrite.setf(ios::fixed, ios::floatfield); TraceWrite.precision(9);
	DataWrite.open("result\\Rawdata.txt", ios::out | ios::trunc);
	DataWrite.setf(ios::fixed, ios::floatfield); DataWrite.precision(9);
	

}
TrajectoryGenerator::~TrajectoryGenerator(void) 
{
	delete MissionState;
	delete RawData;
	TraceWrite.close();
	DataWrite.close();
}

void TrajectoryGenerator::Init()		//��ʼ��������
{
	
	g_n = {0, -g, 0};										//����ϵ���������ٶ�
	Rm = 0;													//����Ȧ�뾶
	Rn = 0;													//î��Ȧ�뾶
	MissionState->t = 0;									//��ʼ������ʱ��
	MissionState->position = {L_0, lambda_0, h_0};			//��ʼ��γ��
	MissionState->velocity 	= {vN_0, 0, 0};					//�����ٶ�
	MissionState->attitude  = { 0, 0, 0 };					//��ʼ��̬
	RawData->t				= 0;							//������ʱ��
	RawData->N_amount		= 0;							//��������
	RawData->f_b			= { 0, 0, 0 };					//������ʼ��
	RawData->omega_b		= { 0, 0, 0 };					//�����ǳ�ʼ��


}

void TrajectoryGenerator::WriteData()				//д����м�����������
{
	int DimState = 10;
	int DimData  = 7;
	double* Optr1 = (double*)MissionState;
	double* Optr2 = (double*)RawData;
	if ( RawData->N_amount == 0)
	{
		for (int i = 0; i < DimState; i++)
		{
			TraceWrite << ExplainFlightContent[i] << "		";
		}
		TraceWrite << endl;
		for (int j = 0; j < DimData; j++)
		{
			DataWrite << ExplainSensorontent[j] << "		";
		}
		DataWrite << endl;
	}
	
	{
		for (int i = 0; i < DimState; i++)
		{
			TraceWrite << Optr1[i] << "		";
		}
		TraceWrite << endl;
		for (int j = 0; j < DimData; j++)
		{
			DataWrite << Optr2[j] << "	";
		}
		DataWrite << endl;
		//cout << Optr[1] << endl;
		RawData->N_amount++;
	}
	
}

double TrajectoryGenerator::White_Generator(double std_dev)					//���������ɺ���
{
	random_device rd;
	mt19937 gen(rd());

	normal_distribution<> d(0, std_dev);

	return d(gen);
};

void TrajectoryGenerator::Trajectory_Generator(FlightState* MissonS)
{
	
	double lambda, L, h, vx, vy, vz;				//����ֲ�����
	double dL, dlambda;								//����
	MissionState = MissonS;
	for (int i = 0; i < N ; i++)
	{
		//Ԫ�ظ�ֵ	��̬��Ϊ0 ���޸��� �ʲ���ֵ
		lambda = MissionState->position(0);
		L = MissionState->position(1);
		h = MissionState->position(2);
		vx = MissionState->velocity(0);
		vy = MissionState->velocity(1);
		vz = MissionState->velocity(2);
		

		Rm = a * (1 - 2 * f + 3 * f * sin(L) * sin(L));								//����Ȧ�뾶
		Rn = a * (1 + f * sin(L) * sin(L));											//î��Ȧ�뾶
		omega_ie = { Omega_i_e * cos(L), Omega_i_e * sin(L), 0 };								//����ϵ����ת���ٶ�
		omega_en = { vz / (Rn + h), vz * tan(L) / (Rn + h), - vx / (Rm + h)};		//����ϵ�µ���ϵ��Ե���ϵת��
		dL		 = vx / (Rm + h);													
		dlambda  = vz / ((Rm + h) * cos(L));
		L		 = L + INS_UPDATE_TIME * dL;										//����*����ʱ��
		lambda   = lambda + INS_UPDATE_TIME * dlambda;
		MissionState->t = MissionState->t + INS_UPDATE_TIME;

		MissionState->position(0) = lambda;											//��ֵ������
		MissionState->position(1) = L;
		MissionState->position(2) = h;

		RawData->f_b = (2 * omega_ie + omega_en).cross(MissionState->velocity) - g_n; //����
		RawData->omega_b = omega_ie + omega_en;

		//cout << MissionState->attitude(1) << endl;

		WriteData();
	}
	cout << "!!!Trajectory Generated!!!" << endl;
}

void TrajectoryGenerator::TrajectoryGeneratorUpdate(FlightState* MissonS)
{
	double lambda, L, h, vx, vy, vz;				//����ֲ�����
	double dL, dlambda;								//����
	MissionState = MissonS;

	//Ԫ�ظ�ֵ	��̬��Ϊ0 ���޸��� �ʲ���ֵ
	L = MissionState->position(0);
	lambda = MissionState->position(1);
	h = MissionState->position(2);
	vx = MissionState->velocity(0);
	vy = MissionState->velocity(1);
	vz = MissionState->velocity(2);


	Rm = a * (1 - 2 * f + 3 * f * sin(L) * sin(L));								//����Ȧ�뾶
	Rn = a * (1 + f * sin(L) * sin(L));											//î��Ȧ�뾶
	omega_ie = { Omega_i_e * cos(L), Omega_i_e * sin(L), 0 };								//����ϵ����ת���ٶ�
	omega_en = { vz / (Rn + h), vz * tan(L) / (Rn + h), -vx / (Rm + h) };		//����ϵ�µ���ϵ��Ե���ϵת��
	dL = vx / (Rm + h);
	dlambda = vz / ((Rm + h) * cos(L));
	L = L + INS_UPDATE_TIME * dL;										//����*����ʱ��
	lambda = lambda + INS_UPDATE_TIME * dlambda;
	MissionState->t = MissionState->t + INS_UPDATE_TIME;

	MissionState->position(0) = L;											//��ֵ������+ ����
	MissionState->position(1) = lambda;
	MissionState->position(2) = h;

	RawData->t = RawData->t + +INS_UPDATE_TIME;
	RawData->f_b = (2 * omega_ie + omega_en).cross(MissionState->velocity) - g_n; //����
	RawData->omega_b = omega_ie + omega_en;

	//cout << MissionState->attitude(1) << endl;

	WriteData();
};