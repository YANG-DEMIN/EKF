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
	g_n = { 0, -g, 0 };										//导航系下重力加速度
	Rm = 0;													//子午圈半径
	Rn = 0;													//卯酉圈半径

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

Matrix4d INS_Calculate::Vector2Matrix(Vector3d Vec3)		//向量叉乘矩阵
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

	//Euler(0) = atan2(-2 * (C * D - A * B), 1 - 2 * (B * B + D * D));		//γ角 -pi -> pi
	//Euler(1) = atan2(-2 * (B * C - A * D), 1 - 2 * (B * B + D * D));   //偏航角 0 -> 2pi
	//Euler(2) = asin(2 * (B * C + A * D));									//俯仰角 -pi/2 -> pi/2
	Euler(0) = atan2((-2 * (C * D - A * B)), (1 - 2 * (B * B + D * D)));		//γ角 -pi -> pi
	Euler(1) = atan2(-2 * (B * D - A * C), 1 - 2 * (C * C + D * D));		 //偏航角 0 -> 2pi
	Euler(2) = asin(2 * (B * C + A * D));									//俯仰角 -pi/2 -> pi/2

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
	att(0) = -atan2(-dcm(1, 1), dcm(2, 1))-PI/2;			//滚转角γ  有毒！！！开头负号和PI/2都是我为了凑结果而加上去的！！！
	att(1) = atan2(-dcm(0, 2), dcm(0, 0));					//偏航角ψ
	att(2) = asin(dcm(0, 1));								//俯仰角φ
	
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
	//局部变量赋值
	L = SimState->position(0);							//纬度
	lambda = SimState->position(1);						//经度
	h = SimState->position(2);							//高度
	vx = SimState->velocity(0);							//北向速度
	vy = SimState->velocity(1);							//天向速度
	vz = SimState->velocity(2);							//东向速度
	Q = Euler2Q(SimState->attitude);					//姿态角
	f_b			= NoiseData->f_b;						//比力
	omega_ib_b	= NoiseData->omega_b;					//陀螺仪

	//飞行物理信息更新
	Rm = a * (1 - 2 * f + 3 * f * sin(L) * sin(L));								//子午圈半径
	Rn = a * (1 + f * sin(L) * sin(L));											//卯酉圈半径
	omega_ie = { Omega_i_e * cos(L), Omega_i_e * sin(L), 0 };					//导航系下自转角速度
	omega_en = { vz / (Rn + h), vz * tan(L) / (Rn + h), -vx / (Rm + h) };		//导航系下导航系相对地球系转动
	omega_in = omega_ie + omega_en;												//导航系相对惯性系转动
	C_nb = Q2DCM(Q).transpose();												//导航系到弹体系 转换传感器数据用
	omega_in_b = C_nb * omega_in;												//弹体系下 导航系相对惯性系转动
	omega_nb_b = -omega_in_b + omega_ib_b;										//弹体系下 导航系相对弹体系转动

	//更新姿态
	dQ = 0.5 * Vector2Matrix(omega_nb_b) * Q;										//姿态增量
	Q  = Q + INS_UPDATE_TIME * dQ;
	Q.normalize();																	//归一化
	C_bn = Q2DCM(Q);
	f_n = C_bn * f_b;																//比力 箭体坐标系转换到导航坐标系
	SimState->attitude = Q2Euler(Q);												//写入

	//更新速度
	dV = f_n - (2 * omega_ie + omega_en).cross(SimState->velocity) + g_n;			//速度增量
	SimState->velocity = SimState->velocity + INS_UPDATE_TIME * dV;					//写入速度

	//更新位置
	dL = vx / (Rm + h);																//纬度
	dlambda = vz / ((Rm + h) * cos(L));												//精度
	dh = vy;																		//高度
	L = L + INS_UPDATE_TIME * dL;													//增量*更新时间
	lambda = lambda + INS_UPDATE_TIME * dlambda;
	h = h + INS_UPDATE_TIME * dh;
	SimState->t = SimState->t + INS_UPDATE_TIME;
	SimState->position(0) = L;													//写入位置
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