V = 30;
[A,B,C,D] = lateral_model(V);
J = [-2-3.46*1i -10 -2+3.46*1i -10];
% state feedback controller
K = acker(A,B(:,1),J);

% load H-inf controller
load('RHCtrl.mat');

m = ureal('m',1573,'Percentage',1);
Iz = ureal('Iz',2873,'Percentage',1);
lf = ureal('If',1.1,'Percentage',1);
lr = ureal('Ir',1.58,'Percentage',1); %% sesnsitive 65%
Cf = ureal('Cf',80000,'Percentage',1);
Cr = ureal('Cr',80000,'Percentage',1);  %% sensitive 65%

Vx = ureal('Vx',30,'Percentage',1);
I_psi = Iz;

a22 = -(2*Cf+2*Cr)/(m*Vx);
a23 = 2*(Cf+Cr)/m;
a24 = - (2*Cf*lf - 2*Cr*lr)/(m*Vx);
a42 = -(2*lf*Cf-2*lr*Cr)/(I_psi*Vx);
a43 = 2*(lf*Cf-lr*Cr)/I_psi;
a44 = -(2*lf*lf*Cf+2*lr*lr*Cr)/(I_psi*Vx);

b21 = 2*Cf/m;
b41 = 2*lf*Cf/I_psi;

b22 = -Vx -2*(lf*Cf-lr*Cr)/(m*Vx);
b42 = -2*(lf*lf*Cf+lr*lr*Cr)/(I_psi*Vx);
                                              
A_lat = [a22 a24 0 a23;
        a42 a44 0 a43;
        1 0 0 0;
        0 1 0 0;];

B1_lat = [b22 b42 0 0]';  %psi_des_dot input
B2_lat = [b21 b41 0 0]';  %Steering angle input

C_lat = eye(4);

D0_lat = zeros(4,6);
B0_lat = zeros(4,4);

A = A_lat;
B = [B0_lat B1_lat B2_lat];
C = [0 0 0 0;C_lat;C_lat];
D = [0 0 0 0 0 1;D0_lat;1 0 0 0 0 0;0 1 0 0 0 0;0 0 1 0 0 0;0 0 0 1 0 0];
K_dk = -K;

s=tf('s');

WT=uss(A,B,C,D);
CLoop_unc_SF = lft( WT([1:9 6:9],1:6) , K_dk);
CLoop_unc_HF = lft( WT([1:9 8:9],1:6) , RHCtrl);

W= logspace(0,2.3,140);
%bodemag(WT(1,1),'k-.',CLoop_unc(1,1),'r-',W)
time = 0:0.05:200;

% no input (straight road)
psi_str = 0*time;
% step input
psi_step = 0*time;
psi_step(10:end) = 0.06;
% sin input
psi_sin = 0*time;
psi_sin(200:1000) = 0.06*(1 - sin(0.05*pi*time(200:1000)));
% input desired yaw
psi_dot_des = psi_step;

% Sample C.L. Systems
[yo,t1] = lsim(WT(1:5,1),psi_dot_des,time); % Generate O.L. response
CLoop_SF_unc20 = usample(CLoop_unc_SF,1); % Generate 1 sample of State F C.L system
CLoop_HF_unc20 = usample(CLoop_unc_HF,1); % Generate 1 sample of H inF C.L system

e1_exceed_time = zeros(2,100);
e2_exceed_time = zeros(2,100);
for i=1:100 %For each of system do a simulation until e1 exceeds 1
    % noise
    amp = 0.001; % IMU noise amplitude
    accel_bias = 0.01*i; % Accelerometer bias
    gyro_bias = 0.01*i; % Gyroscope bias
    d1 = accel_bias * time + amp * cos(8*pi*time);
    d2 = gyro_bias + amp * cos(8*pi*time); 
    d3 = cumtrapz(time,d1);
    d4 = cumtrapz(time,d2);
    % sim
    [y1,t1] = lsim(CLoop_SF_unc20(1:5,1:5,1),[d1;d2;d3;d4;psi_dot_des],time); 
    [y2,t2] = lsim(CLoop_HF_unc20(1:5,1:5,1),[d1;d2;d3;d4;psi_dot_des],time); 
    y1(end,4) = 1;
    y1(end,5) = 1;
    y2(end,4) = 1;
    y2(end,5) = 1;
    % occurance
    e1_exceed_time(1,i) = (find(abs(y1(:,4)) >= 1, 1) - 1) * 0.05;
    e1_exceed_time(2,i) = (find(abs(y2(:,4)) >= 1, 1) - 1) * 0.05;
    e2_exceed_time(1,i) = (find(abs(y1(:,5)) >= 1, 1) - 1) * 0.05;
    e2_exceed_time(2,i) = (find(abs(y2(:,5)) >= 1, 1) - 1) * 0.05;
end

bias = 0.01:0.01:1;
figure(3)
subplot(121)
plot(bias,e1_exceed_time(1,:)*V,'b-', bias,e1_exceed_time(2,:)*V,'r-','LineWidth',2)
title('Distance before lateral position error overrun'); ylabel('Distance (m)'); 
xlabel('bias (m/s^2)');
subplot(122)
plot(bias,e2_exceed_time(1,:)*V,'b-', bias,e2_exceed_time(2,:)*V,'r-','LineWidth',2);
xlabel('bias (m/s^2)');
title('Distance before yaw angle error overrun'); ylabel('Distance (m)'); 
legend('State feedback','H-inf');
figure(4)
subplot(121)
plot(bias,e1_exceed_time(1,:)*V,'b-', bias,e1_exceed_time(2,:)*V,'r-','LineWidth',2)
title('Distance before lateral position error overrun'); ylabel('Distance (m)'); 
xlabel('bias');
subplot(122)
plot(bias,e2_exceed_time(1,:)*V,'b-', bias,e2_exceed_time(2,:)*V,'r-','LineWidth',2);
xlabel('bias');
title('Distance before yaw angle error overrun'); ylabel('Distance (m)'); 
legend('State feedback','H-inf');