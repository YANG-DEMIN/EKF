#include "GNSS_INFO.h"

GNSS_Information::GNSS_Information(void)
{
	//ty_gn = new TrajectoryGenerator;
	GNSS_Info = new GNSS_INFO;
	MissionState_EKF = new FlightState;
	DataWrite.open("result\\GNSS_Information.txt", ios::out | ios::trunc);
	DataWrite.setf(ios::fixed, ios::floatfield); DataWrite.precision(9);
};

GNSS_Information::~GNSS_Information(void)
{
	//delete ty_gn;
	delete GNSS_Info;
	DataWrite.close();
};

void GNSS_Information::Init(void)
{

	MissionState_EKF->t = 0;									//��ʼ������ʱ��
	MissionState_EKF->position = {  L_0,lambda_0, h_0 };			//��ʼ��γ��
	MissionState_EKF->velocity = { vN_0, 0, 0 };					//�����ٶ�
	MissionState_EKF->attitude = { 0, 0, 0 };					//��ʼ��̬
	GNSS_Info->t = 0;
	GNSS_Info->position = { 0, 0, 0 };
	GNSS_Info->velocity = { 0, 0, 0 };
	GNSS_Info->N_amount = 0;

};

void GNSS_Information::WriteData()				//д����м�����������
{
	
	int DimData = 7;
	double* Optr2 = (double*)GNSS_Info;
	if (GNSS_Info->N_amount == 0)
	{
		for (int j = 0; j < DimData; j++)
		{
			DataWrite << ExplainFlightContent[j] << "		";
		}
		DataWrite << endl;
	}

	{
		for (int j = 0; j < DimData; j++)
		{
			DataWrite << Optr2[j] << "		";
		}
		DataWrite << endl;
		//cout << Optr[1] << endl;
		GNSS_Info->N_amount++;
	}

}


void GNSS_Information::GNSS_Info_Update(FlightState* MissionS)
{

	double lambda, L, h, vx, vy, vz;				//����ֲ�����
	double dL, dlambda;								//����
	double Rm, Rn;
	Vector3d omega_ie, omega_en;

	MissionState_EKF = MissionS;

	//Ԫ�ظ�ֵ	��̬��Ϊ0 ���޸��� �ʲ���ֵ
	L = MissionState_EKF->position(0);
	lambda = MissionState_EKF->position(1);
	h = MissionState_EKF->position(2);
	vx = MissionState_EKF->velocity(0);
	vy = MissionState_EKF->velocity(1);
	vz = MissionState_EKF->velocity(2);


	Rm = a * (1 - 2 * f + 3 * f * sin(L) * sin(L));								//����Ȧ�뾶
	Rn = a * (1 + f * sin(L) * sin(L));											//î��Ȧ�뾶
	omega_ie = { Omega_i_e * cos(L), Omega_i_e * sin(L), 0 };					//����ϵ����ת���ٶ�
	omega_en = { vz / (Rn + h), vz * tan(L) / (Rn + h), -vx / (Rm + h) };		//����ϵ�µ���ϵ��Ե���ϵת��
	dL = vx / (Rm + h);
	dlambda = vz / ((Rm + h) * cos(L));
	L = L + INS_UPDATE_TIME * dL;												//����*����ʱ��
	lambda = lambda + INS_UPDATE_TIME * dlambda;

	MissionState_EKF->t = MissionState_EKF->t + INS_UPDATE_TIME;

	MissionState_EKF->position(0) = L;										//��ֵ������+ ����
	MissionState_EKF->position(1) = lambda;
	MissionState_EKF->position(2) = h;


	GNSS_Info->t = GNSS_Info->t + INS_UPDATE_TIME;
	GNSS_Info->position(0) = MissionState_EKF->position(0) + Noise_Generator::White_Generator(GNSS_Position_Variance) / (Rn + h);
	GNSS_Info->position(1) = MissionState_EKF->position(1) + Noise_Generator::White_Generator(GNSS_Position_Variance) / ((Rm + h) * cos(L));
	GNSS_Info->position(2) = MissionState_EKF->position(2) + Noise_Generator::White_Generator(GNSS_Position_Variance);

	//GNSS_Info->velocity(0) = MissionState_EKF->velocity(0) + Noise_Generator::White_Generator(GNSS_Velocity_Variance);
	//GNSS_Info->velocity(1) = MissionState_EKF->velocity(1) + Noise_Generator::White_Generator(GNSS_Velocity_Variance);
	//GNSS_Info->velocity(2) = MissionState_EKF->velocity(2) + Noise_Generator::White_Generator(GNSS_Velocity_Variance);

	GNSS_Info->velocity(0) = vN_0 + Noise_Generator::White_Generator(GNSS_Velocity_Variance);				//��Ҳ��֪��ΪɶGNSS�ٶ�Ʈ�� ���Ծ�ֱ�Ӹ���ֵ��
	GNSS_Info->velocity(1) = 0 + Noise_Generator::White_Generator(GNSS_Velocity_Variance);
	GNSS_Info->velocity(2) = 0 + Noise_Generator::White_Generator(GNSS_Velocity_Variance);
	
	WriteData();
};
