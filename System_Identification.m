close all
clear
clc

A = [[-0.59 0.92 0.61 -9.82]; [-0.23 -6.31 4.42 0.19]; [ 0.78 -5.59 -5.92 0]; [0 0 1 0]];
B = [[4.26 19.43 0.59 0.01 -0.92]; [-58.88 0.25 0.23 0.02 6.31]; [-101.91 0 -0.78 0 5.59]; [0 0 0 0 0]];
C = eye(4);
D = zeros(4, 5);

m = 5;       % Inputs
p = 4;       % Outputs
N = 100;
dt = 1/40;   % 40 Hz sampling


%NOTE: impulse applies a unit-area impulse, meaning a pulse of height 1/Ts
%mult by dt to get a unit impulse of 1
sysCT = ss(A,B,C,D);
sysDT = c2d(sysCT,dt,'zoh'); % Could not get impulse function to work as I wanted in CT
Y = impulse(sysDT, N)*dt;


% gets the Markov param (5 inputs and 4 outputs) and puts
%them into blocks of (y1), y(2), ect.
for j = 1:m
    for k = 1:N-1
        G(:,j,k) = Y(k+1,:,j)';
    end
end

%Make the Hankel Matrix using the blocks of Markov param
r = 20; %rows (there will be 20 * p rows)
c = 20; %cols (there will be 20 * m cols)

for j = 1:r
    for k = 1:c
        H1(j*p-(p-1):j*p,k*m-(m-1):k*m) = G(:, :, j+k-1);
    end
end

%Shifted Hankel Matrix
for j = 1:r
    for k = 1:c
        H2(j*p-(p-1):j*p,k*m-(m-1):k*m) = G(:, :, j+k);
    end
end

for k = 1:m
    D_hat(:,k) = Y(1,:,p);
end

%SVD
[u,s,v] = svd(H1);
n = 4;
%truncated U, S and V matrix
ubar = (u(:,1:n)); 
sbar = s(1:n,1:n); 
vbar = v(:,1:n); 

%observability and controllability matrix 
O1 = ubar*sqrt(sbar);  
C1 = (vbar*sqrt(sbar))';  

%Estimate A from from H2, O1 and C1 
A_hat = pinv(O1)*H2*pinv(C1); 

%Estimate B and C from O1 and C1
B_hat = C1(:,1:m); 
C_hat = O1(1:p,:); 

%check poles
estSysDT = ss(A_hat, B_hat, C_hat, D_hat,dt);

estSysCT = d2c(estSysDT);

estPolesCT = pole(estSysCT);
OrigPolesCT = eig(A);

estPolesDT = eig(A_hat);
origPolesDT = pole(sysDT);

% Check bode plot DT
% figure
% bode(sysDT,'k',estSysDT,'r--');
% figure
% bode(sysCT,'k',estSysCT,'r--');

%check impulse response
figure;
impulse(sysDT,'black',estSysDT,'--');
grid on;
legend('Original System','Estimated System');
title('Impulse Response');
