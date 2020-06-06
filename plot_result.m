clear
close all
%%
% %********************************噪声*************************************%
clear
close all
DEG = 180 / pi;
Noisedata = load('C:\Users\ydm\Desktop\Nav_Sim\result\NoiseCombinedData.txt');
Rawdata = load('C:\Users\ydm\Desktop\Nav_Sim\result\Rawdata.txt');

Noisedata(1,:) = [];
Rawdata(1,:) = [];

figure(1)
plot(Rawdata(: , 1), Rawdata(: , 2),'LineWidth',1.5);
hold on
plot(Rawdata(: , 1), Noisedata(: , 2),'LineWidth',1.5);
title('acc_x');
figure(2)
plot(Rawdata(: , 1), Rawdata(: , 3),'LineWidth',1.5);
hold on
plot(Rawdata(: , 1), Noisedata(: , 3),'LineWidth',1.5);
title('acc_y');
figure(3)
plot(Rawdata(: , 1), Rawdata(: , 4),'LineWidth',1.5);
hold on
plot(Rawdata(: , 1), Noisedata(: , 4),'LineWidth',1.5);
title('acc_z');
figure(4)
plot(Rawdata(: , 1), Rawdata(: , 5) * DEG,'LineWidth',1.5);
hold on
plot(Rawdata(: , 1), Noisedata(: , 5) * DEG,'LineWidth',1.5);
title('gyro_x');
figure(5)
plot(Rawdata(: , 1), Rawdata(: , 6) * DEG,'LineWidth',1.5);
hold on
plot(Rawdata(: , 1), Noisedata(: , 6) * DEG,'LineWidth',1.5);
title('gyro_y');
figure(6)
plot(Rawdata(: , 1), Rawdata(: , 7) * DEG,'LineWidth',1.5);
hold on
plot(Rawdata(: , 1), Noisedata(: , 7) * DEG,'LineWidth',1.5);
title('gyro_z');
%%
%************************标准轨迹与纯INS轨迹******************************%
clear
close all
FlightState = load('C:\Users\ydm\Desktop\Nav_Sim\result\Flightdata.txt');
SimuState = load('C:\Users\ydm\Desktop\Nav_Sim\result\SimuState.txt');

DEG = 180 / pi;

SimuState(1,:) = [];
FlightState(1,:) = [];

figure(1)
plot(SimuState(: , 1),FlightState(: , 2),'LineWidth',1.5);
hold on
plot(SimuState(: , 1),SimuState(: , 2),'LineWidth',1.5);
title('纬度L');

figure(2)
plot(SimuState(: , 1),FlightState(: , 3),'LineWidth',1.5);
hold on
plot(SimuState(: , 1),SimuState(: , 3),'LineWidth',1.5);
title('经度λ');

figure(3)
plot(SimuState(: , 1),FlightState(: , 4),'LineWidth',1.5);
hold on
plot(SimuState(: , 1),SimuState(: , 4),'LineWidth',1.5);
title('高度h');

figure(4)
plot(SimuState(: , 1),FlightState(: , 5),'LineWidth',1.5);
hold on
plot(SimuState(: , 1),SimuState(: , 5),'LineWidth',1.5);
title('vx');

figure(5) 
plot(SimuState(: , 1),FlightState(: , 6),'LineWidth',1.5);
hold on
plot(SimuState(: , 1),SimuState(: , 6),'LineWidth',1.5);
title('vy');

figure(6)
plot(SimuState(: , 1),FlightState(: , 7),'LineWidth',1.5);
hold on
plot(SimuState(: , 1),SimuState(: , 7),'LineWidth',1.5);
title('vz');

figure(7)
plot(SimuState(: , 1),FlightState(: , 8) * DEG,'LineWidth',1.5);
hold on
plot(SimuState(: , 1),SimuState(: , 8) * DEG,'LineWidth',1.5);
title('滚转γ');

figure(8)
plot(SimuState(: , 1),FlightState(: , 9) * DEG,'LineWidth',1.5);
hold on
plot(SimuState(: , 1),SimuState(: , 9) * DEG,'LineWidth',1.5);
title('偏航ψ');

figure(9)
plot(SimuState(: , 1),FlightState(: , 10) * DEG,'LineWidth',1.5);
hold on
plot(SimuState(: , 1),SimuState(: , 10) * DEG,'LineWidth',1.5);
title('俯仰φ');
%%
%********************标准轨迹、INS轨迹与EKF轨迹***************************%
clear
close all
DEG = 180 / pi;
FlightState = load('C:\Users\ydm\Desktop\Nav_Sim\result\Flightdata.txt');
SimuState = load('C:\Users\ydm\Desktop\Nav_Sim\result\SimuState.txt');
EKF_INS_State = load('C:\Users\ydm\Desktop\Nav_Sim\result\EKF_INS_State.txt');

FlightState(1,:) = [];
SimuState(1,:) = [];
EKF_INS_State(1,:) = [];
%不画纯INS
%SimuState = FlightState;

figure(1)
plot(FlightState(: , 1),FlightState(: , 2),'LineWidth',1.5);
hold on
plot(FlightState(: , 1),SimuState(: , 2),'LineWidth',1.5);
hold on
plot(FlightState(: , 1),EKF_INS_State(: , 2),'LineWidth',1.5);
title('纬度L');

figure(2)
plot(FlightState(: , 1),FlightState(: , 3),'LineWidth',1.5);
hold on
plot(FlightState(: , 1),SimuState(: , 3),'LineWidth',1.5);
hold on
plot(FlightState(: , 1),EKF_INS_State(: , 3),'LineWidth',1.5);
title('经度λ');

figure(3)
plot(FlightState(: , 1),FlightState(: , 4),'LineWidth',1.5);
hold on
plot(FlightState(: , 1),SimuState(: , 4),'LineWidth',1.5);
hold on
plot(FlightState(: , 1),EKF_INS_State(: , 4),'LineWidth',1.5);
title('高度h');

figure(4)
plot(FlightState(: , 1),FlightState(: , 5),'LineWidth',1.5);
hold on
plot(FlightState(: , 1),SimuState(: , 5),'LineWidth',1.5);
hold on
plot(FlightState(: , 1),EKF_INS_State(: , 5),'LineWidth',1.5);
title('vx');

figure(5) 
plot(FlightState(: , 1),FlightState(: , 6),'LineWidth',1.5);
hold on
plot(FlightState(: , 1),SimuState(: , 6),'LineWidth',1.5);
hold on
plot(FlightState(: , 1),EKF_INS_State(: , 6),'LineWidth',1.5);
title('vy');

figure(6)
plot(FlightState(: , 1),FlightState(: , 7),'LineWidth',1.5);
hold on
plot(FlightState(: , 1),SimuState(: , 7),'LineWidth',1.5);
hold on
plot(FlightState(: , 1),EKF_INS_State(: , 7),'LineWidth',1.5);
title('vz');

figure(7)
plot(FlightState(: , 1),FlightState(: , 8) * DEG,'LineWidth',1.5);
hold on
plot(FlightState(: , 1),SimuState(: , 8) * DEG,'LineWidth',1.5);
hold on
plot(FlightState(: , 1),EKF_INS_State(: , 8) * DEG,'LineWidth',1.5);
title('滚转γ');

figure(8)
plot(FlightState(: , 1),FlightState(: , 9) * DEG,'LineWidth',1.5);
hold on
plot(FlightState(: , 1),SimuState(: , 9) * DEG,'LineWidth',1.5);
hold on
plot(FlightState(: , 1),EKF_INS_State(: , 9) * DEG,'LineWidth',1.5);
title('偏航ψ');

figure(9)
plot(FlightState(: , 1),FlightState(: , 10) * DEG,'LineWidth',1.5);
hold on
plot(FlightState(: , 1),SimuState(: , 10) * DEG,'LineWidth',1.5);
hold on
plot(FlightState(: , 1),EKF_INS_State(: , 10) * DEG,'LineWidth',1.5);
title('俯仰φ');
%%
%********************************方差************************************%
clear
close all
DEG = 180 / pi;
SimuState = load('C:\Users\ydm\Desktop\Nav_Sim\result\SimuState.txt');
P_k = load('C:\Users\ydm\Desktop\Nav_Sim\result\P_k.txt');
SimuState(1,:) = [];

figure(1)
plot(SimuState(: , 1),sqrt(P_k(: , 1)),'LineWidth',1.5);
title('Vx Variance');
figure(2)
plot(SimuState(: , 1),sqrt(P_k(: , 2)),'LineWidth',1.5);
title('Vy Variance');
figure(3)
plot(SimuState(: , 1),sqrt(P_k(: , 3)),'LineWidth',1.5);
title('Vz Variance');
figure(4)
plot(SimuState(: , 1),sqrt(P_k(: , 4)) * DEG,'LineWidth',1.5);
title('Roll Variance');
figure(5)
plot(SimuState(: , 1),sqrt(P_k(: , 5)) * DEG,'LineWidth',1.5);
title('Yaw Variance');
figure(6)
plot(SimuState(: , 1),sqrt(P_k(: , 6)) * DEG,'LineWidth',1.5);
title('Pitch Variance');
figure(7)
plot(SimuState(: , 1),sqrt(P_k(: , 7)),'LineWidth',1.5);
title('Latitude Variance');
figure(8)
plot(SimuState(: , 1),sqrt(P_k(: , 8)),'LineWidth',1.5);
title('Longtitude Variance');
figure(9)
plot(SimuState(: , 1),sqrt(P_k(: , 9)),'LineWidth',1.5);
title('Altitude Variance');