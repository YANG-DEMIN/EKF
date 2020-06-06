#include "EKF.h"

EKF::EKF(void)
{
	//内部类
	EKF_State = new EKF_STATE;
	NoiseData_EKF = new SensorData;
	INS_State = new FlightState;
	//调用实例
	ty_EKF = new TrajectoryGenerator;
	ng_EKF = new Noise_Generator;
	gi_EKF = new GNSS_Information;
	//写入文件
	DataWrite.open("result\\EKF_State.txt", ios::out | ios::trunc);
	DataWrite.setf(ios::fixed, ios::floatfield); DataWrite.precision(9);
	StateWrite.open("result\\EKF_INS_State.txt", ios::out | ios::trunc);
	StateWrite.setf(ios::fixed, ios::floatfield); StateWrite.precision(9);
	P_k_Write.open("result\\P_k.txt", ios::out | ios::trunc);
	P_k_Write.setf(ios::fixed, ios::floatfield); P_k_Write.precision(9);

};

EKF::~EKF(void)
{
	delete EKF_State;
	delete NoiseData_EKF;
	delete gi_EKF;
	delete ty_EKF;
	delete ng_EKF;
	//delete ic_EKF;
	
	DataWrite.close();
	P_k_Write.close();
};

void EKF::WriteData()				//写入飞行及传感器数据
{

	int DimData = 16;
	int DimState = 10;
	long int k = 0;
	double* Optr1 = (double*)INS_State;
	double* Optr2 = (double*)EKF_State;
	//Matrix15d* Optr3 = &P_k;
	if (EKF_State->N_amount == 0)
	{
		for (int i = 0; i < DimState; i++)
		{
			StateWrite << ExplainFlightContent[i] << "		";
		}
		StateWrite << endl;
		for (int j = 0; j < DimData; j++)
		{
			DataWrite << ExplainEKFContent[j] << "		";
		}
		DataWrite << endl;
	}

	{
		for (int i = 0; i < DimState; i++)
		{
			StateWrite << Optr1[i] << "		";
		}
		StateWrite << endl;
		for (int j = 0; j < DimData; j++)
		{
			DataWrite << Optr2[j] << "		";
		}
		DataWrite << endl;
		for (long int i = 0, k = 0; i < DimState - 1; i++)
		{
			if (i == 6)
			{
				P_k_Write << P_k(i + k) * (Rm + h) * (Rm + h) << "		";
			}
			else if (i == 7)
			{
				P_k_Write << P_k(i + k) * (Rn + h) * cos(L) * (Rn + h) * cos(L) << "		";
			}
			else
			P_k_Write << P_k(i + k) << "		";
			k = k + 15;
		}
		P_k_Write << endl;
		
		EKF_State->N_amount++;
	}

}

void EKF::Init(void)
{
	gi_EKF->Init();
	ty_EKF->Init();
	ng_EKF->Init();

	i = 0;													//迭代初值
	g_n = { 0, -g, 0 };										//导航系下重力加速度
	L = L_0;												//纬度
	h = h_0;												//经度
	Rm = a * (1 - 2 * f + 3 * f * sin(L) * sin(L));			//子午圈半径
	Rn = a * (1 + f * sin(L) * sin(L));						//卯酉圈

	//类初始化
	INS_State->t = 0;
	INS_State->attitude << 0, 0, 0;
	INS_State->position << L_0, lambda_0, h_0;
	INS_State->velocity << vN_0, 0, 0;

	EKF_State->t = 0;										//状态初始化
	EKF_State->N_amount = 0;
	EKF_State->X << vN_0, 0, 0, 0, 0, 0, L_0, lambda_0, h_0, 0, 0, 0, 0, 0, 0;
	EKF_State->X_erro = MatrixXd::Zero(15, 1);


	//方差初始化 恒定量
	I = MatrixXd::Identity(15, 15);
	//Q由惯性器件决定，全是白噪声
	Q_vector << pow(Acc_White_Variance, 2), pow(Acc_White_Variance, 2), pow(Acc_White_Variance, 2),
		pow(Gyro_White_Variance, 2), pow(Gyro_White_Variance, 2), pow(Gyro_White_Variance, 2),
		0, 0, 0,
		pow(Gyro_White_Variance, 2), pow(Gyro_White_Variance, 2), pow(Gyro_White_Variance, 2),
		pow(Acc_White_Variance, 2), pow(Acc_White_Variance, 2), pow(Acc_White_Variance, 2);
	Q_Constant = Q_vector.asDiagonal();
	
	//P0初始值由初始对准精度决定
	P_k = MatrixXd::Zero(15, 15);
	Acc_Markov_driver = sqrt(2 * pow(Gyro_Markov_Variance, 2) / Gyro_Markov_T_Constant);
	Gyro_Markov_driver = sqrt(2 * pow(Acc_Markov_Variance, 2) / Acc_Markov_T_Constant);
	P0_vector << pow(INS_Velocity_Variance, 2), pow(INS_Velocity_Variance, 2), pow(INS_Velocity_Variance, 2),
		pow(INS_Attitude_Variance, 2), pow(INS_Attitude_Variance, 2), pow(INS_Attitude_Variance, 2),
		pow(INS_Position_Variance / (Rm + h), 2), pow(INS_Position_Variance / ((Rn + h) * cos(L)), 2), pow(INS_Position_Variance, 2),
		pow(Gyro_Markov_driver, 2), pow(Gyro_Markov_driver, 2), pow(Gyro_Markov_driver, 2),
		pow(Acc_Markov_driver, 2), pow(Acc_Markov_driver, 2), pow(Acc_Markov_driver, 2);
	P_k = P0_vector.asDiagonal();
	
	//EKF_State->X_erro << INS_Velocity_Variance, INS_Velocity_Variance, INS_Velocity_Variance,
	//	INS_Attitude_Variance, INS_Attitude_Variance, INS_Attitude_Variance,
	//	INS_Position_Variance / (Rm + h), INS_Position_Variance / ((Rn + h) * cos(L)), INS_Position_Variance,
	//	Gyro_Markov_driver, Gyro_Markov_driver, Gyro_Markov_driver,
	//	Acc_Markov_driver, Acc_Markov_driver, Acc_Markov_driver;

	cout << "!!!EKF Inited!!!" << endl;

};

void EKF::Add_Noise_EKF(SensorData* RawDa, SensorNoise* sensor_n, SensorData* NoiseDa)
{

	double* Optr1 = (double*)RawDa;
	double* Optr2 = (double*)sensor_n;
	double* Optr3 = (double*)NoiseDa;

	for (int i = 0; i < 7; i++)
	{
		Optr3[i] = Optr1[i] + Optr2[i];
	}

};

void EKF::INS_Output(SensorData* NoiseD, FlightState* SimS) 
{

	INS_State = SimS;
	NoiseData_EKF = NoiseD;
	//元素赋值
	L = INS_State->position(0);							//纬度
	lambda = INS_State->position(1);						//经度
	h = INS_State->position(2);							//高度
	vx = INS_State->velocity(0);							//北向速度
	vy = INS_State->velocity(1);							//天向速度
	vz = INS_State->velocity(2);							//东向速度
	Q = INS_Calculate::Euler2Q(INS_State->attitude);					//姿态角
	f_b = NoiseData_EKF->f_b;						//比力
	omega_ib_b = NoiseData_EKF->omega_b;					//陀螺仪

	//飞行物理信息更新
	Rm = a * (1 - 2 * f + 3 * f * sin(L) * sin(L));								//子午圈半径
	Rn = a * (1 + f * sin(L) * sin(L));											//卯酉圈半径
	omega_ie = { Omega_i_e * cos(L), Omega_i_e * sin(L), 0 };					//导航系下自转角速度
	omega_en = { vz / (Rn + h), vz * tan(L) / (Rn + h), -vx / (Rm + h) };		//导航系下导航系相对地球系转动
	omega_in = omega_ie + omega_en;												//导航系相对惯性系转动
	C_nb = INS_Calculate::Q2DCM(Q).transpose();											//导航系到弹体系 转换传感器数据用
	omega_in_b = C_nb * omega_in;												//弹体系下 导航系相对惯性系转动
	omega_nb_b = -omega_in_b + omega_ib_b;										//弹体系下 导航系相对弹体系转动

	//更新姿态
	dQ = 0.5 * INS_Calculate::Vector2Matrix(omega_nb_b) * Q;										//姿态增量
	Q = Q + INS_UPDATE_TIME * dQ;
	Q.normalize();																	//归一化
	C_bn = INS_Calculate::Q2DCM(Q);
	f_n = C_bn * f_b;																//比力 箭体坐标系转换到导航坐标系
	INS_State->attitude = INS_Calculate::Q2Euler(Q);												//写入

	//更新速度
	dV = f_n - (2 * omega_ie + omega_en).cross(INS_State->velocity) + g_n;			//速度增量
	INS_State->velocity = INS_State->velocity + INS_UPDATE_TIME * dV;					//写入

	dL = vx / (Rm + h);																//纬度
	dlambda = vz / ((Rm + h) * cos(L));												//精度
	dh = vy;																		//高度
	L = L + INS_UPDATE_TIME * dL;													//增量*更新时间
	lambda = lambda + INS_UPDATE_TIME * dlambda;
	h = h + INS_UPDATE_TIME * dh;
	INS_State->t = INS_State->t + INS_UPDATE_TIME;
	INS_State->position(0) = L;													//写入
	INS_State->position(1) = lambda;
	INS_State->position(2) = h;
	//WriteData();
};

void EKF::StateUpdate(void) 
{
	//将惯导数据输入
	EKF_State->X(0) = INS_State->velocity(0);
	EKF_State->X(1) = INS_State->velocity(1);
	EKF_State->X(2) = INS_State->velocity(2);
	EKF_State->X(3) = INS_State->attitude(0);
	EKF_State->X(4) = INS_State->attitude(1);
	EKF_State->X(5) = INS_State->attitude(2);
	EKF_State->X(6) = INS_State->position(0);
	EKF_State->X(7) = INS_State->position(1);
	EKF_State->X(8) = INS_State->position(2);

	//局部变量
	L = EKF_State->X(6);													//纬度
	lambda = EKF_State->X(7);												//经度
	h = EKF_State->X(8);													//高度

	Rm = a * (1 - 2 * f + 3 * f * sin(L) * sin(L));							//子午圈半径
	Rn = a * (1 + f * sin(L) * sin(L));										//卯酉圈半径
	
	vx = EKF_State->X(0);													//北向		
	vy = EKF_State->X(1);													//天向
	vz = EKF_State->X(2);													//东向

	fx = NoiseData_EKF->f_b(0);												//前
	fy = NoiseData_EKF->f_b(1);												//上
	fz = NoiseData_EKF->f_b(2);												//右

	F = MatrixXd::Zero(15, 15);
	F(1 - 1, 1 - 1) = -vy / (Rm + h);
	F(1 - 1, 2 - 1) = -vx / (Rm + h);
	F(1 - 1, 3 - 1) = -( 2 * vz * tan(L) / (Rn + h) + 2 * Omega_i_e * sin(L));
	F(1 - 1, 5 - 1) = -fz;
	F(1 - 1, 6 - 1) = fy;
	F(1 - 1, 7 - 1) = -(2 * Omega_i_e * vz * cos(L) + vz * vz / ((Rn + L) * cos(L) * cos(L)));
	
	F(2 - 1, 1 - 1) = 2 * vx / (Rm + h);
	F(2 - 1, 3 - 1) = (2 * vz / (Rn + h) + 2 * Omega_i_e * cos(L));
	F(2 - 1, 4 - 1) = fz;
	F(2 - 1, 6 - 1) = -fx;
	F(2 - 1, 7 - 1) = -2 * Omega_i_e * sin(L) * vz;
	F(2 - 1, 9 - 1) = 2 * g / Rm;
	
	F(3 - 1, 1 - 1) = 2 * Omega_i_e * sin(L) + vz / (Rn + h) * tan(L);
	F(3 - 1, 2 - 1) = -(2 * Omega_i_e * cos(L) + vz / (Rn + h));
	F(3 - 1, 3 - 1) = vx * tan(L) /(Rn + h) - vy / (Rn + h);
	F(3 - 1, 4 - 1) = -fy;
	F(3 - 1, 5 - 1) = fx;
	F(3 - 1, 7 - 1) = 2 * Omega_i_e * vx * cos(L) + vx * vz /((Rn + h) * cos(L) * cos(L)) + 2 * Omega_i_e * sin(L) * vy;

	F(4 - 1, 3 - 1) = 1 / (Rn + h);
	F(4 - 1, 5 - 1) = -vx / (Rm + h);
	F(4 - 1, 6 - 1) = -(vz * tan(L) / (Rn + h) + Omega_i_e * sin(L));
	F(4 - 1, 7 - 1) = -Omega_i_e * sin(L);
	
	F(5 - 1, 3 - 1) = tan(L) / (Rn + h);
	F(5 - 1, 4 - 1) = vx / (Rm + h);
	F(5 - 1, 6 - 1) = vz / (Rn + h) + Omega_i_e * cos(L);
	F(5 - 1, 7 - 1) = Omega_i_e * cos(L) + vz / ((Rn + h) * cos(L) * cos(L));

	F(6 - 1, 1 - 1) = -1 / (Rm + h);
	F(6 - 1, 4 - 1) = vz * tan(L) / (Rn + h) + Omega_i_e * sin(L);
	F(6 - 1, 5 - 1) = -(vz / (Rn + h) + Omega_i_e * cos(L));
	
	F(7 - 1, 1 - 1) = 1 / (Rm + h);
	F(8 - 1, 3 - 1) = 1/ ((Rn + h) * cos(L));
	F(8 - 1, 7 - 1) = vz * tan(L) / ((Rn + h) * cos(L));
	F(9 - 1, 2 - 1) = 1;

	FM_vector << -1.0 / Gyro_Markov_T_Constant, -1.0 / Gyro_Markov_T_Constant, -1.0 / Gyro_Markov_T_Constant,
		-1.0 / Acc_Markov_T_Constant, -1.0 / Acc_Markov_T_Constant, -1.0 / Acc_Markov_T_Constant;
	FM = FM_vector.asDiagonal();

	FS = MatrixXd::Zero(9, 6);
	att << EKF_State->X(3), EKF_State->X(4), EKF_State->X(5);
	FS.topRightCorner(3, 3) = INS_Calculate::Euler2DCM(att).transpose();
	FS.block<3, 3>(3, 0) = INS_Calculate::Euler2DCM(att).transpose();

	F.topRightCorner(9, 6) = FS;
	F.bottomRightCorner(6, 6) = FM;
	//更新状态转移矩阵 离散化
	Phi = I + F * INS_UPDATE_TIME + F * F * (INS_UPDATE_TIME * INS_UPDATE_TIME)/2;

	//Q离散化
	Q_k = Q_Constant + (F * Q_Constant + (F * Q_Constant).transpose())
		* (INS_UPDATE_TIME * INS_UPDATE_TIME);
	//cout << "Q = " << endl;
	//cout << Q_k << endl;

	//判断是有GPS数据更新
if (i % 100 == 0 && i != 0)		//
	{
		//测量信息:此处应该为INS-GNSS
		//速度组合
		Z(0) = EKF_State->X(0) - gi_EKF->GNSS_Info->velocity(0);
		Z(1) = EKF_State->X(1) - gi_EKF->GNSS_Info->velocity(1);
		Z(2) = EKF_State->X(2) - gi_EKF->GNSS_Info->velocity(2);
	
		H.topLeftCorner(3, 3) = MatrixXd::Identity(3, 3);
		H.topRightCorner(3, 12) = MatrixXd::Zero(3, 12);

		//GNSS速度精度
		R_vector << pow(GNSS_Velocity_Variance, 2), pow(GNSS_Velocity_Variance, 2), pow(GNSS_Velocity_Variance, 2);
		R = R_vector.asDiagonal();
		//位置组合
		if (i % 1000 == 0)
		{
			//位置组合
			Z(0) = EKF_State->X(6) - gi_EKF->GNSS_Info->position(0);
			Z(1) = EKF_State->X(7) - gi_EKF->GNSS_Info->position(1);
			Z(2) = EKF_State->X(8) - gi_EKF->GNSS_Info->position(2);

			H = MatrixXd::Zero(3, 15);
			Hp << Rm + h, (Rn + h)* cos(L), 1;
			H.block<3, 3>(0, 6) = Hp.asDiagonal();

			R_vector << pow(GNSS_Position_Variance / (Rm + h), 2), pow(GNSS_Position_Variance / ((Rn + h) * cos(L)), 2), pow(GNSS_Position_Variance, 2);
			R = R_vector.asDiagonal();
		}

		//初始误差为0
		Xk_k_1 = Phi * EKF_State->X_erro;												//一步预测
		Pk_k_1 = Phi * P_k * Phi.transpose() + Q_k;										//一步预测误差方差
		K = Pk_k_1 * H.transpose() * (H * Pk_k_1 * H.transpose() + R).inverse();		//计算增益矩阵
		
		I_WANT_TO = H * Xk_k_1;															//调试用
		EKF_State->X_erro = Xk_k_1 + K * (Z - H * Xk_k_1);								//滤波算法
		P_k = (I - K * H) * Pk_k_1 * (I - K * H).transpose() + K * R * K.transpose();	//计算估计均方差


		EKF_State->X(0) = INS_State->velocity(0) - EKF_State->X_erro(0);
		EKF_State->X(1) = INS_State->velocity(1) - EKF_State->X_erro(1);		//速度
		EKF_State->X(2) = INS_State->velocity(2) - EKF_State->X_erro(2);
		EKF_State->X(6) = INS_State->position(0) - EKF_State->X_erro(6);
		EKF_State->X(7) = INS_State->position(1) - EKF_State->X_erro(7);		//位置
		EKF_State->X(8) = INS_State->position(2) - EKF_State->X_erro(8);

		//姿态
		att << EKF_State->X(3), EKF_State->X(4), EKF_State->X(5);
		q = INS_Calculate::Euler2Q(att);
		omega << EKF_State->X_erro(3), EKF_State->X_erro(4), EKF_State->X_erro(5);
		q = q + 0.5 * INS_Calculate::Vector2Matrix(omega) * q * INS_UPDATE_TIME;
		q.normalize();
		att = INS_Calculate::Q2Euler(q);
		EKF_State->X(3) = att(0);
		EKF_State->X(4) = att(1);
		EKF_State->X(5) = att(2);

		//误差归零
		EKF_State->X_erro = MatrixXd::Zero(15, 1);

	}	
	else
	{																//没有GPS数据时，需要进行修正
		
		Xk_k_1 = Phi * EKF_State->X_erro;							//一步预测
		Pk_k_1 = Phi * P_k * Phi.transpose() + Q_k;					//一步预测误差方差
		EKF_State->X_erro = Xk_k_1;									//预测值即为估计值
		P_k = Pk_k_1;												//两步方差一致

		//cout << Pk_k_1 << endl;									//调试

	}
	
	//反馈
	EKF_State->t = EKF_State->t + INS_UPDATE_TIME;
	INS_State->velocity(0) = EKF_State->X(0);							
	INS_State->velocity(1) = EKF_State->X(1);
	INS_State->velocity(2) = EKF_State->X(2);
	INS_State->attitude(0) = EKF_State->X(3);
	INS_State->attitude(1) = EKF_State->X(4);
	INS_State->attitude(2) = EKF_State->X(5);
	INS_State->position(0) = EKF_State->X(6);
	INS_State->position(1) = EKF_State->X(7);
	INS_State->position(2) = EKF_State->X(8);

	WriteData();
};

void EKF::Update(void)
{

	for (i = 0; i < N; i++)
	{
		ty_EKF->TrajectoryGeneratorUpdate(ty_EKF->MissionState);				//生成标准轨迹
		ng_EKF->NoiseGeneratorUpdate(ng_EKF->sensor_noise);						//噪声生成器
		gi_EKF->GNSS_Info_Update(gi_EKF->MissionState_EKF);						//GNSS信息生成器
		Add_Noise_EKF(ty_EKF->RawData, ng_EKF->sensor_noise, NoiseData_EKF);	//噪声数据生成
		
		INS_Output(NoiseData_EKF, INS_State);									//惯导更新
		StateUpdate();															//EKF更新
		//WriteData();															//调试
	}

	cout << "!!!EKF GNSS DATA GENERATOED!!!" << endl;
	cout << "!!!EKF Succeed!!!" << endl;
};
