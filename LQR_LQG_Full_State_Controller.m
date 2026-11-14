close all
clear
clc

A = [[-0.59 0.92 0.61 -9.82]; [-0.23 -6.31 4.42 0.19]; [ 0.78 -5.59 -5.92 0]; [0 0 1 0]];
B = [[4.26 19.43 0.59 0.01 -0.92]; [-58.88 0.25 0.23 0.02 6.31]; [-101.91 0 -0.78 0 5.59]; [0 0 0 0 0]];
C = [[1 0 0 0]; [0 0 0 1]];
D = zeros(2, 2);

B_Real_Inputs = B(:,1:2);

G = ss(A, B_Real_Inputs, C, D);

Q = diag([1, 1, 1, 1]);
% Penalize use of the thrust more than use of elevator. Conserve power. 
R = diag([1, 3.0]);

Qw = diag([1 deg2rad(1)]);
Rv = diag([1.0, 0.01]);   


% LQR and LQG calculations
K_LQR = lqr(A, B_Real_Inputs, Q, R);
L_LQG = lqe(A, B_Real_Inputs, C, Qw, Rv);

% Place poles for full state feedback controller
p = [-2 -2.3 -3 -4];
K_Full_State = place(A,B_Real_Inputs, p);

% find the Compensator state space equations 
Kc_LQG = ss(A-B_Real_Inputs*K_LQR - L_LQG*C, L_LQG, K_LQR, 0);
Kc_LQR = ss(A, B_Real_Inputs, K_LQR, 0);
Kc_Full_State = ss(A, B_Real_Inputs, K_Full_State, 0);

% Kalman poles, LQR Poles
Poles_Kalman = eig(A-L_LQG*C);
Poles_LQR = eig(A-B_Real_Inputs*K_LQR);
Poles_Full_State = eig(A-B_Real_Inputs*K_Full_State);

% Disk Margin of LQG System
Lp_LQG = Kc_LQG*G;
DM_LQG = diskmargin(Lp_LQG);
Disk_Margin_alpha_LQG = DM_LQG.DiskMargin;
Gm_range_LQG = DM_LQG.GainMargin;
PM_range_LQG = DM_LQG.PhaseMargin;

GM_range_LQG_Str = string(Gm_range_LQG(1,1)) + ' to ' + string(Gm_range_LQG(1,2));
PM_range_LQG_Str = string(PM_range_LQG(1,1)) + ' to ' + string(PM_range_LQG(1,2));

% Disk Margin of LQR System
Lp_LQR = Kc_LQR;
DM_LQR = diskmargin(Lp_LQR);
Disk_Margin_alpha_LQR = DM_LQR.DiskMargin;
Gm_range_LQR = DM_LQR.GainMargin;
PM_range_LQR = DM_LQR.PhaseMargin;

GM_range_LQR_Str = string(Gm_range_LQR(1,1)) + ' to ' + string(Gm_range_LQR(1,2));
PM_range_LQR_Str = string(PM_range_LQR(1,1)) + ' to ' + string(PM_range_LQR(1,2));

% Disk Margin of Full State System
Lp_Full_State = Kc_Full_State;
DM_Full_State = diskmargin(Lp_Full_State);
Disk_Margin_alpha_Full_State = DM_Full_State.DiskMargin;
Gm_range_Full_State = DM_Full_State.GainMargin;
PM_range_Full_State = DM_Full_State.PhaseMargin;

GM_range_Full_State_Str = string(Gm_range_Full_State(1,1)) + ' to ' + string(Gm_range_Full_State(1,2));
PM_range_Full_State_Str = string(PM_range_Full_State(1,1)) + ' to ' + string(PM_range_Full_State(1,2));

A_cl_LQG=G/(eye(2)+Lp_LQG);
A_cl_LQR = G/(eye(2)+Lp_LQR);

% Plot Impulse Response
figure;
impulse(G/(eye(2)+Lp_LQG),linewidth = 2)
hold on
impulse(G/(eye(2)+Lp_LQR),'-.',linewidth = 2)
impulse(G/(eye(2)+Lp_Full_State),'--', linewidth = 2)
grid on;
title('Impulse Response of KQR, LQG, and Full State Feedback')
legend('LQG Response', 'LQR Response', 'Full State Feedback Response')
xlim([0 10])
hold off

% LQG Overshoot and Settling Time 
LQG_System_Information = stepinfo(G/(eye(2)+Lp_LQG));
LQR_System_Information = stepinfo(G/(eye(2)+Lp_LQR));
Full_State_Information = stepinfo(G/(eye(2)+Lp_Full_State));


% Tables
Alpha = [Disk_Margin_alpha_LQG; Disk_Margin_alpha_Full_State; Disk_Margin_alpha_Full_State];
GM_Range = [GM_range_LQG_Str; GM_range_LQR_Str; GM_range_Full_State_Str];
PM_Range = [PM_range_LQG_Str; PM_range_LQR_Str; PM_range_Full_State_Str];
EndValues = [Alpha GM_Range PM_Range];

Col1 = ["Alpha", "GM Range", "PM Range"];
Row1 = ["LQG", "LQR", "Full State Feedback"];
Table1 = array2table(EndValues, 'VariableNames', Col1, 'RowNames', Row1);
disp(Table1)


fprintf('\n')

poles = [Poles_Kalman, Poles_LQR, Poles_Full_State];
Col2 = ["Kalman filter poles", "LQR Poles", 'Full State Controller Poles'];
Row2 = ["Pole 1", "Pole 2", "Pole 3", 'Pole 4'];
Table2 = array2table(poles, 'VariableNames', Col2, 'RowNames', Row2);
disp(Table2)

SettlingTime1 = [LQG_System_Information.SettlingTime]';
SettlingTime2 = [LQR_System_Information.SettlingTime]';
SettlingTime3 = [Full_State_Information.SettlingTime]';
SettlingTime = [SettlingTime1 SettlingTime2 SettlingTime3];

fprintf('\n')
Col3 = ["Kalman filter Settling Time", 'LQR Settling Time' ,'Full State Settling Time'];
Row3 = ["Output 1 from  input 1", "Output 2 from  input 1", "Output 1 from  input 2", 'Output 2 from  input 2'];
Table3 = array2table(SettlingTime, 'VariableNames', Col3, 'RowNames', Row3);
disp(Table3)

Overshoot1 = [LQG_System_Information.Overshoot]';
Overshoot2 = [LQR_System_Information.Overshoot]';
Overshoot3 = [Full_State_Information.Overshoot]';
Overshoot = [Overshoot1 Overshoot2 Overshoot3];
fprintf('\n')
Col4 = ["Kalman filter %Overshoot", 'LQR %Overshoot' ,'Full State %Overshoot'];
Row4 = ["Output 1 from  input 1", "Output 2 from  input 1", "Output 1 from  input 2", 'Output 2 from  input 2'];
Table4 = array2table(Overshoot, 'VariableNames', Col4, 'RowNames', Row4);
disp(Table4)