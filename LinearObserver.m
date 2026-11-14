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
K = lqr(A, B_Real_Inputs, Q, R);
L_LQG = lqe(A, B_Real_Inputs, C, Qw, Rv);


% Find the closed loop poles of LQR to locate optimal poles of Linear Observer
RegulatorPoles = eig(A-B_Real_Inputs*K);

p = 2*RegulatorPoles;

% Place the Observer Poles
L_LinearObserver = place(A',C',p)';

% find the Compensator state space equation 
Kc_LQG = ss(A-B_Real_Inputs*K - L_LQG*C, L_LQG, K, 0);
Kc_Liner_Observer = ss(A-B_Real_Inputs*K - L_LinearObserver*C, L_LinearObserver, K, 0);


% Compensator poles
Poles_LQG = pole(Kc_LQG);
Poles_Linear_Observer = pole(Kc_Liner_Observer);


% Disk Margin of LQG System
Lp_LQG = Kc_LQG*G;
DM_LQG = diskmargin(Lp_LQG);
Disk_Margin_alpha_LQG = DM_LQG.DiskMargin;
Gm_range_LQG = DM_LQG.GainMargin;
PM_range_LQG = DM_LQG.PhaseMargin;

GM_range_LQG_Str = string(Gm_range_LQG(1,1)) + ' to ' + string(Gm_range_LQG(1,2));
PM_range_LQG_Str = string(PM_range_LQG(1,1)) + ' to ' + string(PM_range_LQG(1,2));

% Disk Margin of Linear Observer System
Lp_Liner_Observer = Kc_Liner_Observer*G;
DM_Liner_Observer = diskmargin(Lp_Liner_Observer);
Disk_Margin_alpha_Liner_Observer = DM_Liner_Observer.DiskMargin;
Gm_range_Liner_Observer = DM_Liner_Observer.GainMargin;
PM_range_Liner_Observer = DM_Liner_Observer.PhaseMargin;

Gm_range_Liner_Observer_Str = string(Gm_range_Liner_Observer(1,1)) + ' to ' + string(Gm_range_Liner_Observer(1,2));
PM_range_Liner_Observer_Str = string(PM_range_Liner_Observer(1,1)) + ' to ' + string(PM_range_Liner_Observer(1,2));

% Plot the Impulse response
figure
impulse(G/(eye(2)+Lp_LQG),linewidth = 2)
grid on;
title('Impulse Response of Closed Loop LQG and Linear Observer')
hold on
impulse(G/(eye(2)+Lp_Liner_Observer),'--',linewidth = 2)
legend('LQG Response', 'Linear Observer Response')
xlim([0 15])
hold off

figure;
impulse(Kc_LQG,linewidth = 2)
grid on;
title('Impulse Response of Kalman and Linear Observer')
hold on
impulse(Kc_Liner_Observer,'--',linewidth = 2)
legend('LQG Response', 'Linear Observer Response')
xlim([0 0.2])
hold off

% LQG_System_Information = stepinfo(Kc_LQG);
% Linear_Observer_System_Information = stepinfo(Kc_Liner_Observer);

LQG_System_Information = stepinfo(G/(eye(2)+Lp_LQG));
Linear_Observer_System_Information = stepinfo(G/(eye(2)+Lp_Liner_Observer));

% for i = 1:length()

% Tables
Alpha = [string(Disk_Margin_alpha_LQG); string(Disk_Margin_alpha_Liner_Observer)];
GM_Range = [GM_range_LQG_Str; Gm_range_Liner_Observer_Str];
PM_Range = [PM_range_LQG_Str; PM_range_Liner_Observer_Str];

EndValues = [Alpha GM_Range PM_Range];
Col1 = ["Alpha", "GM Range", "PM Range"];
Row1 = ["LQG", "Linear Observer"];
Table1 = array2table(EndValues, 'VariableNames', Col1, 'RowNames', Row1);
disp(Table1)


fprintf('\n')
poles = [Poles_LQG, Poles_Linear_Observer, eig(A-L_LQG*C), eig(A-L_LinearObserver*C);];
Col2 = ["LQG Compensator", "Linear Observer Compensator", "Kalman filter poles", "Linear Observer Poles"];
Row2 = ["Pole 1", "Pole 2", "Pole 3", 'Pole 4'];
Table2 = array2table(poles, 'VariableNames', Col2, 'RowNames', Row2);
disp(Table2)


fprintf('\n')

SettlingTime1 = [LQG_System_Information.SettlingTime]';
SettlingTime2 = [Linear_Observer_System_Information.SettlingTime]';
SettlingTime = [SettlingTime1 SettlingTime2];

Col3 = ["Kalman filter Settling Time", 'Linear Observer Settling Time'];
Row3 = ["Output 1 from  input 1", "Output 2 from  input 1", "Output 1 from  input 2", 'Output 2 from  input 2'];
Table3 = array2table(SettlingTime, 'VariableNames', Col3, 'RowNames', Row3);
disp(Table3)


fprintf('\n')

peak1 = [LQG_System_Information.Peak]';
peak2 = [Linear_Observer_System_Information.Peak]';
peak = [peak1 peak2];

Col4 = ["Kalman filter peak", 'Linear Observer peak'];
Row4 = ["Output 1 from  input 1", "Output 2 from  input 1", "Output 1 from  input 2", 'Output 2 from  input 2'];
Table4 = array2table(peak, 'VariableNames', Col4, 'RowNames', Row4);
disp(Table4)