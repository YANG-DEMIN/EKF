This program is a homework program for integrated navigation, which is used to simulate
 the standard trajectory of missiles, noise of Inertial Measured Unit, 
and integrated navigation (EKF) calculation.

Written in VS 2019, using Eigen matrix calculation tool.

Code structure:
parameter.h 			Initial flight parameter setting
Nav_Sim_main.cpp 			main function
Trajectory_Generator.cpp 		standard trajectory output
GNSS_INFO.cpp			GNSS information output
Noise_Generator.cpp 		generates noise
INS_Calculator.cpp 			pure inertial device output
EKF.cpp 				combined navigation output

Attachment 1: 飞行器导航原理与应用 练习题目  III.doc
	Code environment
Attachment 2: 组合导航原理与应用_罗建军等_2012_西北工业大学出版社.pdf
	Integrated navigation principle for detailed content reference
Attachment 3: 报告.pdf
	This code uses mathematical calculations such as formulas and quaternions
Attachment 4: 汇报.pdf
	Running result demo

**********************************************************************************************
此程序为组合导航大作业程序，用于模拟导弹标准轨迹、惯性器件噪声、及组合导航（EKF）计算。
采用VS 2019编写，使用Eigen矩阵计算工具

代码结构：
parameter.h			初始飞行参数设定
Nav_Sim_main.cpp	 		主函数
Trajectory_Generator.cpp 		标准轨迹输出
GNSS_INFO.cpp: GNSS		信息输出
Noise_Generator.cpp		生成噪声
INS_Calculator.cpp	 		纯惯性器件输出
EKF.cpp				组合导航输出

附件1：飞行器导航原理与应用 练习题目  III.doc
	代码环境
附件2：组合导航原理与应用_罗建军等_2012_西北工业大学出版社.pdf
	组合导航原理详细内容参考
附件3：报告.pdf
	本代码使用公式及四元数等数学计算
附件4：汇报.pdf
	结果演示	