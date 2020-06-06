//============================================================================
// Name        : Nav_Sim_main.cpp
// Author      : Ydm
// Email	   : 2015300585@mail.nwpu.edu.cn
// Version     : 1.0
// Time		   : 2020.4.20
// Copyright   : Your copyright notice
// Description : The main function of Navigation Simulation
//============================================================================

#include "parameter.h"
#include "Trajectory_Generator.hpp"
#include "Noise_Generator.h"
#include "INS_Calculate.h"
#include "GNSS_INFO.h"
#include "EKF.h"

int main() {
	//
	//INS_Calculate ic;
	//ic.INS_Init();
	//ic.INS_Update();

	EKF ekf;
	ekf.Init();
	ekf.Update();
	//
	return 0;
}
