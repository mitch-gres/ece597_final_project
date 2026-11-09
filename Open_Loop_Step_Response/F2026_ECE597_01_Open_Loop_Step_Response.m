clear all;
close all;
clc;

% Values are the Longitudinal Dynamics from "Modeling and Identification of a Small Fixed-Wing
% UAV Using Estimated Aerodynamic Angles" as found in 4.2.2 Eq. 41. 
A = [[-0.59 0.92 0.61 -9.82]; [-0.23 -6.31 4.42 0.19]; [ 0.78 -5.59 -5.92 0]; [0 0 1 0]];
B = [[4.26 19.43 0.59 0.01 -0.92]; [-58.88 0.25 0.23 0.02 6.31]; [-101.91 0 -0.78 0 5.59]; [0 0 0 0 0]];
C = eye(4);
D = zeros(4, 5);

G = ss(A, B, C, 0); % Open Loop
H = tf(G);

O = sim("F2026_ECE597_01_Open_Loop_Step_Response");

% Step Response (Open-Loop)
figure(1);
for i = 1:4
    subplot(2, 2, i);
    plot(O.step_response.Time, O.step_response.Data(:, i));
    title("Step Response for First Output");
    xlabel("Time (s)");
    ylabel("Amplitude");
end