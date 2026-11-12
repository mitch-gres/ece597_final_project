%% LQR Approach Toward Longitudinal UAV Dynamics
% Author: Noah S. & Mitchell G. 
% Class: ECE597-01
% Date: 11/11/2026
clear all;
close all;
clc;

%% Plant & Actuator Definition (Longitudinal UAV Dynamics)
% Values are the Longitudinal Dynamics from "Modeling and Identification of a Small Fixed-Wing
% UAV Using Estimated Aerodynamic Angles" as found in 4.2.2 Eq. 41. 
A = [[-0.59 0.92 0.61 -9.82]; [-0.23 -6.31 4.42 0.19]; [ 0.78 -5.59 -5.92 0]; [0 0 1 0]];
% The paper describes disturbances introduced the wind in the B matrix.
% When we actually analyzed our system we removed it. 
B_w_wind = [[4.26 19.43 0.59 0.01 -0.92]; [-58.88 0.25 0.23 0.02 6.31]; [-101.91 0 -0.78 0 5.59]; [0 0 0 0 0]];
C = [[1 0 0 0]; [0 0 0 1]]; % Assume we have sensors for airspeed and pitch angle. 
D = zeros(2, 2);
B = B_w_wind(:,1:2); % Some of the inputs in the paper are assumed to be 
% wind disturbances. We removed them here and then also in our C matrix
% such that we only care about the input that we can control.

% Our aircraft was characterized in this state. 
% Therefore our control system will increase or decrease from these
% setpoints. 
throttle_init = 0.44; % 44% Throttle
elevator_init = -deg2rad(2.35); % -2.35 is Elevator Pitch
airspeed_init = 17.9; % 17.9m/s is Airspeed
pitch_init = -deg2rad(1.11);

target_airspeed = 20; % m/s
delta_airspeed = target_airspeed - airspeed_init;

target_pitch = deg2rad(5.0);
delta_pitch = target_pitch - pitch_init;

elevator_max = deg2rad(20) - elevator_init; % Should be constrained between +/- 20.
elevator_min = -deg2rad(20) - elevator_init;

throttle_max = 1.0 - throttle_init; % Should be constrained between 0~1.0.
throttle_min = 0.0 - throttle_init;

% Since this is LQR what we need to do is introduce a disturbance and see
% how the system performs. What we'll do is introduce a disturbance in all
% the states as if a gust of wind has hit our drone. We'll accomplish this
% by varying gains from a impulse summed into our output and see it's
% response. 

% Should model an updraft forward of the CG that'll push the nose up
% quickly. 
forward_airspeed_disturbance_gain = 0.5; % Sudden pitch should slow our forward airspeed.  
vertical_airspeed_disturbance_gain = 3.0;
pitch_rate_disturbance_gain = 10.0;
% For this one we'll assume that the impulse will give us a one. So we need
% to with gain translate that into a change of pitch. Let's say that pitch
% will change by ten. That's because the wind sucks. 
pitch_disturbance_gain = 10 * (pi/180); % rad
%% Cross-Coupling Included. Both Pitch Angle & Airspeed as Inputs with Pitch Angle & Airspeed Observed
G_Airspeed = 1.0;
G_Pitch_Angle = 1.0;

G = ss(A, B, C, D);
H = tf(G);

Q = diag([1.0 0.1 0.1 10]);
R = diag([13.1 10]);
K = lqr(G, Q, R);

O_Coupled_Inputs = sim("F2026_ECE597_01_Closed_Loop_Step_Response_LQR");

% In this example these are cross-coupled which mean we're looking at the
% output channels when both requirements are satisfied. That is we want to
% maintain a pitch angle and airspeed at the same time. 
figure(1);
subplot(2, 2, 1);
plot(O_Coupled_Inputs.outputs.Time, O_Coupled_Inputs.outputs.Data(:, 1), "LineWidth", 2);
title("Step Response of Airspeed (u)");
legend("Output", "Reference");
grid on;
xlabel("Time (s)");
ylabel("Airspeed (u) (m/s)");

subplot(2, 2, 2);
plot(O_Coupled_Inputs.inputs.Time, O_Coupled_Inputs.inputs.Data(:, 1), "LineWidth", 2);
title("Control Effort of Elevator Angle (δe)");
grid on;
xlabel("Time (s)");
ylabel("Elevator Angle (δe) (rad)");

subplot(2, 2, 3);
plot(O_Coupled_Inputs.outputs.Time, O_Coupled_Inputs.outputs.Data(:, 2), "LineWidth", 2);
title("Step Response of Pitch Angle (θ)");
legend("Output", "Reference");
grid on;
xlabel("Time (s)");
ylabel("Pitch Angle (θ) (rad)");

subplot(2, 2, 4);
plot(O_Coupled_Inputs.inputs.Time, O_Coupled_Inputs.inputs.Data(:, 2), "LineWidth", 2);
title("Control Effort of Throttle (%)");
grid on;
xlabel("Time (s)");
ylabel("Throttle (%)");