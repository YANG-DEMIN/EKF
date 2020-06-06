/*
 * Trajectory_Generator.cpp
 *
 *  Created on: 2020年5月4日
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

void TrajectoryGenerator::Init()		//初始化各参数
{
	
	g_n = {0, -g, 0};										//导航系下重力加速度
	Rm = 0;													//子午圈半径
	Rn = 0;													//卯酉圈半径
	MissionState->t = 0;									//初始化飞行时间
	MissionState->position = {L_0, lambda_0, h_0};			//初始经纬度
	MissionState->velocity 	= {vN_0, 0, 0};					//飞行速度
	MissionState->attitude  = { 0, 0, 0 };					//初始姿态
	RawData->t				= 0;							//传感器时间
	RawData->N_amount		= 0;							//迭代次数
	RawData->f_b			= { 0, 0, 0 };					//比力初始化
	RawData->omega_b		= { 0, 0, 0 };					//陀螺仪初始化


}

void TrajectoryGenerator::WriteData()				//写入飞行及传感器数据
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

double TrajectoryGenerator::White_Generator(double std_dev)					//白噪声生成函数
{
	random_device rd;
	mt19937 gen(rd());

	normal_distribution<> d(0, std_dev);

	return d(gen);
};

void TrajectoryGenerator::Trajectory_Generator(FlightState* MissonS)
{
	
	double lambda, L, h, vx, vy, vz;				//定义局部变量
	double dL, dlambda;								//增量
	MissionState = MissonS;
	for (int i = 0; i < N ; i++)
	{
		//元素赋值	姿态角为0 且无更新 故不赋值
		lambda = MissionState->position(0);
		L = MissionState->position(1);
		h = MissionState->position(2);
		vx = MissionState->velocity(0);
		vy = MissionState->velocity(1);
		vz = MissionState->velocity(2);
		

		Rm = a * (1 - 2 * f + 3 * f * sin(L) * sin(L));								//子午圈半径
		Rn = a * (1 + f * sin(L) * sin(L));											//卯酉圈半径
		omega_ie = { Omega_i_e * cos(L), Omega_i_e * sin(L), 0 };								//导航系下自转角速度
		omega_en = { vz / (Rn + h), vz * tan(L) / (Rn + h), - vx / (Rm + h)};		//导航系下导航系相对地球系转动
		dL		 = vx / (Rm + h);													
		dlambda  = vz / ((Rm + h) * cos(L));
		L		 = L + INS_UPDATE_TIME * dL;										//增量*更新时间
		lambda   = lambda + INS_UPDATE_TIME * dlambda;
		MissionState->t = MissionState->t + INS_UPDATE_TIME;

		MissionState->position(0) = lambda;											//赋值回向量
		MissionState->position(1) = L;
		MissionState->position(2) = h;

		RawData->f_b = (2 * omega_ie + omega_en).cross(MissionState->velocity) - g_n; //比力
		RawData->omega_b = omega_ie + omega_en;

		//cout << MissionState->attitude(1) << endl;

		WriteData();
	}
	cout << "!!!Trajectory Generated!!!" << endl;
}

void TrajectoryGenerator::TrajectoryGeneratorUpdate(FlightState* MissonS)
{
	double lambda, L, h, vx, vy, vz;				//定义局部变量
	double dL, dlambda;								//增量
	MissionState = MissonS;

	//元素赋值	姿态角为0 且无更新 故不赋值
	L = MissionState->position(0);
	lambda = MissionState->position(1);
	h = MissionState->position(2);
	vx = MissionState->velocity(0);
	vy = MissionState->velocity(1);
	vz = MissionState->velocity(2);


	Rm = a * (1 - 2 * f + 3 * f * sin(L) * sin(L));								//子午圈半径
	Rn = a * (1 + f * sin(L) * sin(L));											//卯酉圈半径
	omega_ie = { Omega_i_e * cos(L), Omega_i_e * sin(L), 0 };								//导航系下自转角速度
	omega_en = { vz / (Rn + h), vz * tan(L) / (Rn + h), -vx / (Rm + h) };		//导航系下导航系相对地球系转动
	dL = vx / (Rm + h);
	dlambda = vz / ((Rm + h) * cos(L));
	L = L + INS_UPDATE_TIME * dL;										//增量*更新时间
	lambda = lambda + INS_UPDATE_TIME * dlambda;
	MissionState->t = MissionState->t + INS_UPDATE_TIME;

	MissionState->position(0) = L;											//赋值回向量+ 噪声
	MissionState->position(1) = lambda;
	MissionState->position(2) = h;

	RawData->t = RawData->t + +INS_UPDATE_TIME;
	RawData->f_b = (2 * omega_ie + omega_en).cross(MissionState->velocity) - g_n; //比力
	RawData->omega_b = omega_ie + omega_en;

	//cout << MissionState->attitude(1) << endl;

	WriteData();
};