close all
clear
clc

A = [[-0.59 0.92 0.61 -9.82]; [-0.23 -6.31 4.42 0.19]; [ 0.78 -5.59 -5.92 0]; [0 0 1 0]];
B = [[4.26 19.43 0.59 0.01 -0.92]; [-58.88 0.25 0.23 0.02 6.31]; [-101.91 0 -0.78 0 5.59]; [0 0 0 0 0]];
C = [[1 0 0 0]; [0 0 0 1]];
D = zeros(2, 2);

B_Real_Inputs = B(:,1:2);

% Open Loop
G = ss(A, B_Real_Inputs, C, 0);
H = tf(G);

LpOpen = G;

% Open Loop Stability 
DM_Open = diskmargin(LpOpen);
Disk_Margin_alpha_Open = DM_Open.DiskMargin;
Gm_range_Open = DM_Open.GainMargin;
Pm_range_Open = DM_Open.PhaseMargin;
Gm_range_Open_str = string((Gm_range_Open(1,1))) + ' to ' + string((Gm_range_Open(1,2)));
Pm_range_Open_str = string(Pm_range_Open(1,1)) + ' to ' + string(Pm_range_Open(1,2));

OpenPoles = pole(H);

%LQR
Q = diag([1, 1, 1, 1]);
% Penalize use of the thrust more than use of elevator. Conserve power. 
R = diag([1, 3.0]);
K = lqr(A, B_Real_Inputs, Q, R);

LpLQR = ss(A, B_Real_Inputs, K, 0);


% LQR Stability
DM_LQR = diskmargin(LpLQR);
Disk_Margin_alpha_LQR = DM_LQR.DiskMargin;
Gm_range_LQR = DM_LQR.GainMargin;
Pm_range_LQR = DM_LQR.PhaseMargin;
Gm_range_LQR_str = string(Gm_range_LQR(1,1)) + ' to ' + string(Gm_range_LQR(1,2));
Pm_range_LQR_str = string(Pm_range_LQR(1,1)) + ' to ' + string(Pm_range_LQR(1,2));

LQRPoles = eig(A-B_Real_Inputs*K);

%LQG across many Q values

QwValues = [0.001, 0.01, 0.1, 0.5, 1.0, 1.5, 2.0];
Rv = diag([1.0, (0.01)]);

Disk_Margin_alpha_LQG_Values = zeros(length(QwValues),1);


for l = 1:length(QwValues)

Qw = diag([QwValues(l) deg2rad((QwValues(l)))]);
L = lqe(A, B_Real_Inputs, C, Qw, Rv);

Kc = ss(A-B_Real_Inputs*K - L*C, L, K, 0);
LpLQG = Kc*G;

% LQG Stability
DM_LQG = diskmargin(LpLQG);
Disk_Margin_alpha_LQG_Values(l,:) = DM_LQG.DiskMargin;

Gm_range_LQG_Values = DM_LQG.GainMargin;
Pm_range_LQG_Values = DM_LQG.PhaseMargin;

Gm_range_LQG_Values_str(l,:) = string(Gm_range_LQG_Values(1,1)) + ' to ' + string(Gm_range_LQG_Values(1,2));
Pm_range_LQG_Values_str(l,:) = string(Pm_range_LQG_Values(1,1)) + ' to ' + string(Pm_range_LQG_Values(1,2));

ObserverPoles = eig(A-L*C);
ObserverPolesList(1:4,l) = ObserverPoles;
end


%Tables

Gm_Range = [Gm_range_Open_str; Gm_range_LQR_str; Gm_range_LQG_Values_str];
PM_Range = [Pm_range_Open_str; Pm_range_LQR_str; Pm_range_LQG_Values_str];
Disk_Margin_alpha_Values = [Disk_Margin_alpha_Open; Disk_Margin_alpha_LQR; Disk_Margin_alpha_LQG_Values];
EndValues = [string(Disk_Margin_alpha_Values), Gm_Range, PM_Range];

Col1 = ["DM", "GM Range", 'PM Range'];
Row1 = ["Open Loop", "LQR", "LQG q = 0.001", "LQG q = 0.01", "LQG q = 0.1", "LQG q = 0.5", "LQG q = 1", "LQG q = 1.5", "LQG q = 2"];
Table1 = array2table(EndValues, 'VariableNames', Col1, 'RowNames', Row1);
disp(Table1)

fprintf('\n')
poles = [LQRPoles, ObserverPolesList];

Col2 = ["LQR Poles", "LQG Poles q = 0.001", "LQG Poles q = 0.01", "LQG Poles q = 0.1", "LQG Poles q = 0.5", "LQG Poles q = 1", "LQG Poles q = 1.5", "LQG Poles q = 2"];
Row2 = ["Pole 1", "Pole 2", "Pole 3", "Pole 4"];
Table2 = array2table(poles, 'VariableNames', Col2, 'RowNames', Row2);
disp(Table2)

fprintf('\n')
Col3 = ["Open Loop Poles"];
Row3 = ["Pole 1", "Pole 2", "Pole 3", "Pole 4", "Pole 5", "Pole 6", "Pole 7", "Pole 8"];
Table3 = array2table(OpenPoles, 'VariableNames', Col3, 'RowNames', Row3);
disp(Table3)
