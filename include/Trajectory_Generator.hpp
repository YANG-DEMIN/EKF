/*
 * Trajectory_Generator.hpp
 * Description: 初始化、生成标准轨迹、惯性导航参数
 *  Created on: 2020年5月4日
 *      Author: ydm
 * 
 */

#ifndef MODULES_TRAJECTORY_GENERATOR_TRAJECTORY_GENERATOR_HPP_
#define MODULES_TRAJECTORY_GENERATOR_TRAJECTORY_GENERATOR_HPP_
#include "parameter.h"
#include "Noise_Generator.h"



class TrajectoryGenerator
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	TrajectoryGenerator(void);						//构造函数
	~TrajectoryGenerator(void);					//析构函数
	void Init(void);						//初始化任务及参数
	void WriteData(void);					//写入数据到文件
	double White_Generator(double std_div);						//输入方差，输出随机噪声
	void Trajectory_Generator(FlightState* MissonS);		//生成标准轨迹
	//void GNSS_Noise_Generator(void);
	void TrajectoryGeneratorUpdate(FlightState* MissonS);
	

	FlightState* MissionState;
	//FlightState* SimState;
	SensorData* RawData;
	//SensorData*	NoiseData;

private:



protected:

	double Rm, Rn;
	ofstream TraceWrite, DataWrite;
	Vector3d g_n, omega_ie, omega_en, omega_in;
	Vector3d omega_in_b, omega_nb_b;
	string ExplainFlightContent[20] = { "1-飞行时间t","2-纬度L","3-经度λ","4-高度h",
			"5-速度vx","6-速度vy","7-速度vz","8-滚转γ","9-偏航ψ","10-俯仰φ" };
	string ExplainSensorontent[14] = { "1-飞行时间t","2-fb_x","3-fb_y","4-fb_z",
			"5-wx","6-wy","7-wz" };

};
#endif /* MODULES_TRAJECTORY_GENERATOR_TRAJECTORY_GENERATOR_HPP_ */
