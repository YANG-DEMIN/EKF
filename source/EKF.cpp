#include "EKF.h"

EKF::EKF(void)
{
	//�ڲ���
	EKF_State = new EKF_STATE;
	NoiseData_EKF = new SensorData;
	INS_State = new FlightState;
	//����ʵ��
	ty_EKF = new TrajectoryGenerator;
	ng_EKF = new Noise_Generator;
	gi_EKF = new GNSS_Information;
	//д���ļ�
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

void EKF::WriteData()				//д����м�����������
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

	i = 0;													//������ֵ
	g_n = { 0, -g, 0 };										//����ϵ���������ٶ�
	L = L_0;												//γ��
	h = h_0;												//����
	Rm = a * (1 - 2 * f + 3 * f * sin(L) * sin(L));			//����Ȧ�뾶
	Rn = a * (1 + f * sin(L) * sin(L));						//î��Ȧ

	//���ʼ��
	INS_State->t = 0;
	INS_State->attitude << 0, 0, 0;
	INS_State->position << L_0, lambda_0, h_0;
	INS_State->velocity << vN_0, 0, 0;

	EKF_State->t = 0;										//״̬��ʼ��
	EKF_State->N_amount = 0;
	EKF_State->X << vN_0, 0, 0, 0, 0, 0, L_0, lambda_0, h_0, 0, 0, 0, 0, 0, 0;
	EKF_State->X_erro = MatrixXd::Zero(15, 1);


	//�����ʼ�� �㶨��
	I = MatrixXd::Identity(15, 15);
	//Q�ɹ�������������ȫ�ǰ�����
	Q_vector << pow(Acc_White_Variance, 2), pow(Acc_White_Variance, 2), pow(Acc_White_Variance, 2),
		pow(Gyro_White_Variance, 2), pow(Gyro_White_Variance, 2), pow(Gyro_White_Variance, 2),
		0, 0, 0,
		pow(Gyro_White_Variance, 2), pow(Gyro_White_Variance, 2), pow(Gyro_White_Variance, 2),
		pow(Acc_White_Variance, 2), pow(Acc_White_Variance, 2), pow(Acc_White_Variance, 2);
	Q_Constant = Q_vector.asDiagonal();
	
	//P0��ʼֵ�ɳ�ʼ��׼���Ⱦ���
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
	//Ԫ�ظ�ֵ
	L = INS_State->position(0);							//γ��
	lambda = INS_State->position(1);						//����
	h = INS_State->position(2);							//�߶�
	vx = INS_State->velocity(0);							//�����ٶ�
	vy = INS_State->velocity(1);							//�����ٶ�
	vz = INS_State->velocity(2);							//�����ٶ�
	Q = INS_Calculate::Euler2Q(INS_State->attitude);					//��̬��
	f_b = NoiseData_EKF->f_b;						//����
	omega_ib_b = NoiseData_EKF->omega_b;					//������

	//����������Ϣ����
	Rm = a * (1 - 2 * f + 3 * f * sin(L) * sin(L));								//����Ȧ�뾶
	Rn = a * (1 + f * sin(L) * sin(L));											//î��Ȧ�뾶
	omega_ie = { Omega_i_e * cos(L), Omega_i_e * sin(L), 0 };					//����ϵ����ת���ٶ�
	omega_en = { vz / (Rn + h), vz * tan(L) / (Rn + h), -vx / (Rm + h) };		//����ϵ�µ���ϵ��Ե���ϵת��
	omega_in = omega_ie + omega_en;												//����ϵ��Թ���ϵת��
	C_nb = INS_Calculate::Q2DCM(Q).transpose();											//����ϵ������ϵ ת��������������
	omega_in_b = C_nb * omega_in;												//����ϵ�� ����ϵ��Թ���ϵת��
	omega_nb_b = -omega_in_b + omega_ib_b;										//����ϵ�� ����ϵ��Ե���ϵת��

	//������̬
	dQ = 0.5 * INS_Calculate::Vector2Matrix(omega_nb_b) * Q;										//��̬����
	Q = Q + INS_UPDATE_TIME * dQ;
	Q.normalize();																	//��һ��
	C_bn = INS_Calculate::Q2DCM(Q);
	f_n = C_bn * f_b;																//���� ��������ϵת������������ϵ
	INS_State->attitude = INS_Calculate::Q2Euler(Q);												//д��

	//�����ٶ�
	dV = f_n - (2 * omega_ie + omega_en).cross(INS_State->velocity) + g_n;			//�ٶ�����
	INS_State->velocity = INS_State->velocity + INS_UPDATE_TIME * dV;					//д��

	dL = vx / (Rm + h);																//γ��
	dlambda = vz / ((Rm + h) * cos(L));												//����
	dh = vy;																		//�߶�
	L = L + INS_UPDATE_TIME * dL;													//����*����ʱ��
	lambda = lambda + INS_UPDATE_TIME * dlambda;
	h = h + INS_UPDATE_TIME * dh;
	INS_State->t = INS_State->t + INS_UPDATE_TIME;
	INS_State->position(0) = L;													//д��
	INS_State->position(1) = lambda;
	INS_State->position(2) = h;
	//WriteData();
};

void EKF::StateUpdate(void) 
{
	//���ߵ���������
	EKF_State->X(0) = INS_State->velocity(0);
	EKF_State->X(1) = INS_State->velocity(1);
	EKF_State->X(2) = INS_State->velocity(2);
	EKF_State->X(3) = INS_State->attitude(0);
	EKF_State->X(4) = INS_State->attitude(1);
	EKF_State->X(5) = INS_State->attitude(2);
	EKF_State->X(6) = INS_State->position(0);
	EKF_State->X(7) = INS_State->position(1);
	EKF_State->X(8) = INS_State->position(2);

	//�ֲ�����
	L = EKF_State->X(6);													//γ��
	lambda = EKF_State->X(7);												//����
	h = EKF_State->X(8);													//�߶�

	Rm = a * (1 - 2 * f + 3 * f * sin(L) * sin(L));							//����Ȧ�뾶
	Rn = a * (1 + f * sin(L) * sin(L));										//î��Ȧ�뾶
	
	vx = EKF_State->X(0);													//����		
	vy = EKF_State->X(1);													//����
	vz = EKF_State->X(2);													//����

	fx = NoiseData_EKF->f_b(0);												//ǰ
	fy = NoiseData_EKF->f_b(1);												//��
	fz = NoiseData_EKF->f_b(2);												//��

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
	//����״̬ת�ƾ��� ��ɢ��
	Phi = I + F * INS_UPDATE_TIME + F * F * (INS_UPDATE_TIME * INS_UPDATE_TIME)/2;

	//Q��ɢ��
	Q_k = Q_Constant + (F * Q_Constant + (F * Q_Constant).transpose())
		* (INS_UPDATE_TIME * INS_UPDATE_TIME);
	//cout << "Q = " << endl;
	//cout << Q_k << endl;

	//�ж�����GPS���ݸ���
if (i % 100 == 0 && i != 0)		//
	{
		//������Ϣ:�˴�Ӧ��ΪINS-GNSS
		//�ٶ����
		Z(0) = EKF_State->X(0) - gi_EKF->GNSS_Info->velocity(0);
		Z(1) = EKF_State->X(1) - gi_EKF->GNSS_Info->velocity(1);
		Z(2) = EKF_State->X(2) - gi_EKF->GNSS_Info->velocity(2);
	
		H.topLeftCorner(3, 3) = MatrixXd::Identity(3, 3);
		H.topRightCorner(3, 12) = MatrixXd::Zero(3, 12);

		//GNSS�ٶȾ���
		R_vector << pow(GNSS_Velocity_Variance, 2), pow(GNSS_Velocity_Variance, 2), pow(GNSS_Velocity_Variance, 2);
		R = R_vector.asDiagonal();
		//λ�����
		if (i % 1000 == 0)
		{
			//λ�����
			Z(0) = EKF_State->X(6) - gi_EKF->GNSS_Info->position(0);
			Z(1) = EKF_State->X(7) - gi_EKF->GNSS_Info->position(1);
			Z(2) = EKF_State->X(8) - gi_EKF->GNSS_Info->position(2);

			H = MatrixXd::Zero(3, 15);
			Hp << Rm + h, (Rn + h)* cos(L), 1;
			H.block<3, 3>(0, 6) = Hp.asDiagonal();

			R_vector << pow(GNSS_Position_Variance / (Rm + h), 2), pow(GNSS_Position_Variance / ((Rn + h) * cos(L)), 2), pow(GNSS_Position_Variance, 2);
			R = R_vector.asDiagonal();
		}

		//��ʼ���Ϊ0
		Xk_k_1 = Phi * EKF_State->X_erro;												//һ��Ԥ��
		Pk_k_1 = Phi * P_k * Phi.transpose() + Q_k;										//һ��Ԥ������
		K = Pk_k_1 * H.transpose() * (H * Pk_k_1 * H.transpose() + R).inverse();		//�����������
		
		I_WANT_TO = H * Xk_k_1;															//������
		EKF_State->X_erro = Xk_k_1 + K * (Z - H * Xk_k_1);								//�˲��㷨
		P_k = (I - K * H) * Pk_k_1 * (I - K * H).transpose() + K * R * K.transpose();	//������ƾ�����


		EKF_State->X(0) = INS_State->velocity(0) - EKF_State->X_erro(0);
		EKF_State->X(1) = INS_State->velocity(1) - EKF_State->X_erro(1);		//�ٶ�
		EKF_State->X(2) = INS_State->velocity(2) - EKF_State->X_erro(2);
		EKF_State->X(6) = INS_State->position(0) - EKF_State->X_erro(6);
		EKF_State->X(7) = INS_State->position(1) - EKF_State->X_erro(7);		//λ��
		EKF_State->X(8) = INS_State->position(2) - EKF_State->X_erro(8);

		//��̬
		att << EKF_State->X(3), EKF_State->X(4), EKF_State->X(5);
		q = INS_Calculate::Euler2Q(att);
		omega << EKF_State->X_erro(3), EKF_State->X_erro(4), EKF_State->X_erro(5);
		q = q + 0.5 * INS_Calculate::Vector2Matrix(omega) * q * INS_UPDATE_TIME;
		q.normalize();
		att = INS_Calculate::Q2Euler(q);
		EKF_State->X(3) = att(0);
		EKF_State->X(4) = att(1);
		EKF_State->X(5) = att(2);

		//������
		EKF_State->X_erro = MatrixXd::Zero(15, 1);

	}	
	else
	{																//û��GPS����ʱ����Ҫ��������
		
		Xk_k_1 = Phi * EKF_State->X_erro;							//һ��Ԥ��
		Pk_k_1 = Phi * P_k * Phi.transpose() + Q_k;					//һ��Ԥ������
		EKF_State->X_erro = Xk_k_1;									//Ԥ��ֵ��Ϊ����ֵ
		P_k = Pk_k_1;												//��������һ��

		//cout << Pk_k_1 << endl;									//����

	}
	
	//����
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
		ty_EKF->TrajectoryGeneratorUpdate(ty_EKF->MissionState);				//���ɱ�׼�켣
		ng_EKF->NoiseGeneratorUpdate(ng_EKF->sensor_noise);						//����������
		gi_EKF->GNSS_Info_Update(gi_EKF->MissionState_EKF);						//GNSS��Ϣ������
		Add_Noise_EKF(ty_EKF->RawData, ng_EKF->sensor_noise, NoiseData_EKF);	//������������
		
		INS_Output(NoiseData_EKF, INS_State);									//�ߵ�����
		StateUpdate();															//EKF����
		//WriteData();															//����
	}

	cout << "!!!EKF GNSS DATA GENERATOED!!!" << endl;
	cout << "!!!EKF Succeed!!!" << endl;
};
