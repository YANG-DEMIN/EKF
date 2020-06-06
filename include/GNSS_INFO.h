#pragma once
#include "parameter.h"
#include "Trajectory_Generator.hpp"

class GNSS_Information
{
public:
	GNSS_Information(void);
	~GNSS_Information(void);
	void Init(void);
	//double White_Generator(double std_div);						//���뷽�����������
	void WriteData(void);
	void GNSS_Info_Update(FlightState* MissionS);

	GNSS_INFO* GNSS_Info;
	FlightState* MissionState_EKF;

private:

protected:
	TrajectoryGenerator* ty_gn;
	ofstream DataWrite;										//д�뺯��
	string ExplainFlightContent[14] = { "1-time","2-γ��L", "3-���Ȧ�",
		"4-�߶�h", "5-velocity_x", "6-velocity_y", "7-velocity_z" };
};

