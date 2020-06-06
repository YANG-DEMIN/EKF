#include "INS_Calculate.h"

INS_Calculate::INS_Calculate() 
{
	ty			= new TrajectoryGenerator;
	ng			= new Noise_Generator;
	NoiseData	= new SensorData;
	SimState	= new FlightState;

	TraceWrite.open("result\\SimuState.txt", ios::out | ios::trunc);
	TraceWrite.setf(ios::fixed, ios::floatfield); TraceWrite.precision(9);
	DataWrite.open("result\\NoiseCombinedData.txt", ios::out | ios::trunc);
	DataWrite.setf(ios::fixed, ios::floatfield); DataWrite.precision(9);
};

INS_Calculate::~INS_Calculate()
{
	delete ty;
	delete ng;
	delete NoiseData;
	delete SimState;
	DataWrite.close();
	TraceWrite.close();
};

void INS_Calculate::INS_Init()
{
	g_n = { 0, -g, 0 };										//����ϵ���������ٶ�
	Rm = 0;													//����Ȧ�뾶
	Rn = 0;													//î��Ȧ�뾶

	NoiseData->t = 0;
	NoiseData->N_amount = 0;
	NoiseData->f_b = { 0, 0, 0 };
	NoiseData->omega_b = { 0, 0, 0 };

	SimState->t = 0;
	SimState->attitude = { 0, 0, 0 };
	SimState->velocity = { vN_0, 0, 0 };
	SimState->position = { L_0, lambda_0, h_0 };
	ty->Init();
	ng->Init();

	cout << "!!!Inited!!!" << endl;
};

void INS_Calculate::WriteData(void)
{
	int DimData = 7;
	int DimState = 10;
	
	double* Optr1 = (double*)SimState;
	double* Optr2 = (double*)NoiseData;
	if (NoiseData->N_amount == 0)
	{
		for (int i = 0; i < DimState; i++)
		{
			TraceWrite << ExplainFlightContent[i] << "		";
		}
		TraceWrite << endl;
		for (int j = 0; j < DimData; j++)
		{
			DataWrite << ExplainSensorContent[j] << "		";
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
		NoiseData->N_amount++;
	}
};

void INS_Calculate::Add_Noise(SensorData* RawDa, SensorNoise* sensor_n, SensorData* NoiseDa)
{

	double* Optr1 = (double*)RawDa;
	double* Optr2 = (double*)sensor_n;
	double* Optr3 = (double*)NoiseDa;

	for (int i = 0; i < 7; i++)
	{
		Optr3[i] = Optr1[i] + Optr2[i];
	}

};

Matrix4d INS_Calculate::Vector2Matrix(Vector3d Vec3)		//������˾���
{
	double x, y, z;
	Matrix4d Mat4;
	x = Vec3(0);
	y = Vec3(1);
	z = Vec3(2);

	Mat4 << 0, -x, -y, -z,
				x, 0, z, -y,
				y, -z, 0, x,
				z, y, -x, 0;

	return Mat4;
};

Matrix3d INS_Calculate::Q2DCM(Vector4d q)
{
	double A, B, C, D;
	Matrix3d Dcm;
	A = q(0); B = q(1); C = q(2); D = q(3);
	Dcm << A * A + B * B - C * C - D * D,	2 * (B * C + A * D),			2 * (B * D - A * C),
		2 * (B * C - A * D),			 A* A - B * B + C * C - D * D,		2 * (C * D + A * B),
		2 * (B * D + A * C),			2 * (C * D - A * B),				 A*A - B*B - C*C + D*D;

	return Dcm;
};

Vector3d INS_Calculate::Q2Euler(Vector4d q) 
{
	double A, B, C, D;
	Vector3d Euler;
	A = q(0); B = q(1); C = q(2); D = q(3);

	//Euler(0) = atan2(-2 * (C * D - A * B), 1 - 2 * (B * B + D * D));		//�ý� -pi -> pi
	//Euler(1) = atan2(-2 * (B * C - A * D), 1 - 2 * (B * B + D * D));   //ƫ���� 0 -> 2pi
	//Euler(2) = asin(2 * (B * C + A * D));									//������ -pi/2 -> pi/2
	Euler(0) = atan2((-2 * (C * D - A * B)), (1 - 2 * (B * B + D * D)));		//�ý� -pi -> pi
	Euler(1) = atan2(-2 * (B * D - A * C), 1 - 2 * (C * C + D * D));		 //ƫ���� 0 -> 2pi
	Euler(2) = asin(2 * (B * C + A * D));									//������ -pi/2 -> pi/2

	return Euler;
};

Vector4d INS_Calculate::Euler2Q(Vector3d att)
{
	double A, B, C;
	Vector4d q;
	A = att(0); B = att(1); C = att(2);
	//q(0) = cos(B / 2) * cos(C / 2) * cos(A / 2) - sin(B / 2) * sin(C / 2) * sin(A / 2);
	//q(1) = -sin(B / 2) * sin(C / 2) * cos(A / 2) - cos(B / 2) * cos(C / 2) * sin(A / 2);
	//q(2) = -sin(B / 2) * cos(C / 2) * cos(A / 2) - cos(B / 2) * sin(C / 2) * sin(A / 2);
	//q(3) = -cos(B / 2) * sin(C / 2) * cos(A / 2) + sin(B / 2) * cos(C / 2) * sin(A / 2);

	q(0) = cos(B / 2) * cos(C / 2) * cos(A / 2) - sin(B / 2) * sin(C / 2) * sin(A / 2);
	q(1) = sin(B / 2) * sin(C / 2) * cos(A / 2) + cos(B / 2) * cos(C / 2) * sin(A / 2);
	q(2) = sin(B / 2) * cos(C / 2) * cos(A / 2) + cos(B / 2) * sin(C / 2) * sin(A / 2);
	q(3) = cos(B / 2) * sin(C / 2) * cos(A / 2) - sin(B / 2) * cos(C / 2) * sin(A / 2);

	return q;
};

Vector3d INS_Calculate::DCM2Euler(Matrix3d DCM) 
{
	Matrix3d dcm = DCM;
	Vector3d att;
	att(0) = -atan2(-dcm(1, 1), dcm(2, 1))-PI/2;			//��ת�Ǧ�  �ж���������ͷ���ź�PI/2������Ϊ�˴ս��������ȥ�ģ�����
	att(1) = atan2(-dcm(0, 2), dcm(0, 0));					//ƫ���Ǧ�
	att(2) = asin(dcm(0, 1));								//�����Ǧ�
	
	return att;
};

 Matrix3d INS_Calculate::Euler2DCM(Vector3d att) 
{
	Matrix3d Dcm;
	Dcm = Q2DCM(Euler2Q(att));

	return Dcm;
};

void INS_Calculate::StateUpdate(SensorData* NoiseD, FlightState* SimS)
{
	
	SimState = SimS;
	NoiseData = NoiseD;
	//�ֲ�������ֵ
	L = SimState->position(0);							//γ��
	lambda = SimState->position(1);						//����
	h = SimState->position(2);							//�߶�
	vx = SimState->velocity(0);							//�����ٶ�
	vy = SimState->velocity(1);							//�����ٶ�
	vz = SimState->velocity(2);							//�����ٶ�
	Q = Euler2Q(SimState->attitude);					//��̬��
	f_b			= NoiseData->f_b;						//����
	omega_ib_b	= NoiseData->omega_b;					//������

	//����������Ϣ����
	Rm = a * (1 - 2 * f + 3 * f * sin(L) * sin(L));								//����Ȧ�뾶
	Rn = a * (1 + f * sin(L) * sin(L));											//î��Ȧ�뾶
	omega_ie = { Omega_i_e * cos(L), Omega_i_e * sin(L), 0 };					//����ϵ����ת���ٶ�
	omega_en = { vz / (Rn + h), vz * tan(L) / (Rn + h), -vx / (Rm + h) };		//����ϵ�µ���ϵ��Ե���ϵת��
	omega_in = omega_ie + omega_en;												//����ϵ��Թ���ϵת��
	C_nb = Q2DCM(Q).transpose();												//����ϵ������ϵ ת��������������
	omega_in_b = C_nb * omega_in;												//����ϵ�� ����ϵ��Թ���ϵת��
	omega_nb_b = -omega_in_b + omega_ib_b;										//����ϵ�� ����ϵ��Ե���ϵת��

	//������̬
	dQ = 0.5 * Vector2Matrix(omega_nb_b) * Q;										//��̬����
	Q  = Q + INS_UPDATE_TIME * dQ;
	Q.normalize();																	//��һ��
	C_bn = Q2DCM(Q);
	f_n = C_bn * f_b;																//���� ��������ϵת������������ϵ
	SimState->attitude = Q2Euler(Q);												//д��

	//�����ٶ�
	dV = f_n - (2 * omega_ie + omega_en).cross(SimState->velocity) + g_n;			//�ٶ�����
	SimState->velocity = SimState->velocity + INS_UPDATE_TIME * dV;					//д���ٶ�

	//����λ��
	dL = vx / (Rm + h);																//γ��
	dlambda = vz / ((Rm + h) * cos(L));												//����
	dh = vy;																		//�߶�
	L = L + INS_UPDATE_TIME * dL;													//����*����ʱ��
	lambda = lambda + INS_UPDATE_TIME * dlambda;
	h = h + INS_UPDATE_TIME * dh;
	SimState->t = SimState->t + INS_UPDATE_TIME;
	SimState->position(0) = L;													//д��λ��
	SimState->position(1) = lambda;
	SimState->position(2) = h;

	WriteData();
};

void INS_Calculate::INS_Update()
{

	for (int i = 0; i < N; i++)
	{
		ty->TrajectoryGeneratorUpdate(ty->MissionState);
		ng->NoiseGeneratorUpdate(ng->sensor_noise);
		Add_Noise(ty->RawData, ng->sensor_noise, NoiseData);
		StateUpdate(NoiseData, SimState);
		//cout << SimState->position << endl;
	}
	cout << "!!!Trajectory Generated!!!" << endl;
	cout << "!!!Noise Generated!!!" << endl;
	cout << "!!!Flight Data Updated!!!" << endl;
};