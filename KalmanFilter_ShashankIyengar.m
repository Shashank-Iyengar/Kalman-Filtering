% Decision Engineering - Homework 3
% Shashank S Iyengar - M12934513
% Kalman Filter - Target Tracking Problem

close all
clear
clc

load('sen_pos_data2018.mat')
load('sen_tru_pos_data2018.mat')
load('sen_acc_data2018.mat')
% Matrix Definitions
T = 0.1;
sigma = 0.3;
F = [1 T (T^2)/2; 0 1 T; 0 0 1];
Q = (sigma^2)*[(T^4)/4 (T^3)/2 (T^2)/2; (T^3)/2 2*T^3 T^2;...
    (T^2)/2 T^2 T^2];
H = [1 0 0];
z = y;
a = y1;
X = [0; 0; 0];
P = [1 0 0; 0 1 0; 0 0 1];
R = 0.7;
I = eye(3);

% Displacement as measured value
for t = 1:200
    Xe = F*X;
    Pe = F*P*F' + Q;
    
    K = Pe*H'*inv(H*Pe*H' + R);
    Xu = Xe + K*(z(t) - H*Xe);
    Pu = (I - K*H)*Pe;
    
    XP(t) = Xu(1);
    VP(t) = Xu(2);
    AP(t) = Xu(3);
    X = Xu;
    P = Pu;
end
C=[XP; x];
figure(1)
plot((0:199),x,'-k')
hold on
plot((0:199),XP,'sr')
grid on
title('Case (a): Displacement - Estimated vs True Position')
xlabel('Time Step \Delta t (20 seconds/0.1)')
ylabel('Displacement [m]')
legend('True Data','Estimated Data')

figure(2)
plot((0:199),VP,'-r')
title('Case (a): Estimated Velocity')
grid on
xlabel('Time Step \Delta t (20 seconds/0.1)')
ylabel('Velocity [m/s]')

figure(3)
plot((0:199),AP,'-r')
title('Case (a): Estimated Acceleration')
grid on
xlabel('Time Step \Delta t (20 seconds/0.1)')
ylabel('Acceleration [m/s^2]')

% Acceleration as measured value
T = 0.1;
sigma = 0.1;
F = [1 T (T^2)/2; 0 1 T; 0 0 1];
Q = (sigma^2)*[(T^4)/4 (T^3)/2 (T^2)/2; (T^3)/2 2*T^3 T^2;...
    (T^2)/2 T^2 T^2];               % Process Noise
H = [0 0 1];
X = [0; 0; 0];
P = [1 0 0; 0 1 0; 0 0 1];          % Covariance Matrix
R = 0.9;
for t = 1:200
    Xe = F*X;                       % Status Prediction
    Pe = F*P*F' + Q;                % Covariance Prediction
    
    K = Pe*H'*inv(H*Pe*H' + R);     % Kalman Gain
    Xu = Xe + K*(y1(t) - H*Xe);     % State Update
    Pu = (I - K*H)*Pe;              % Covariance Update
    
    XP(t) = Xu(1);                  % Displacement
    VP(t) = Xu(2);                  % Velocity
    AP(t) = Xu(3);                  % Acceleration
    X = Xu;                         % State Reassignment
    P = Pu;                         % Covariance Reassignment
end
C1=[XP; x];

figure(4)
plot((0:199),x,'-k')
hold on
plot((0:199),XP,'sr')
grid on
title('Case (b): Displacement - Estimated vs True Position')
xlabel('Time Step \Delta t (20 seconds/0.1)')
ylabel('Displacement [m]')
legend('True Data','Estimated Data')

figure(5)
plot((0:199),VP,'-r')
grid on
title('Case (b): Estimated Velocity')
xlabel('Time Step \Delta t (20 seconds/0.1)')
ylabel('Velocity [m/s]')

figure(6)
plot((0:199),AP,'-r')
grid on
title('Case (b): Estimated Acceleration')
xlabel('Time Step \Delta t (20 seconds/0.1)')
ylabel('Acceleration [m/s^2]')
