% compare AutoControl result with math model
% load AutoControl results
load('e1_woNoise.mat');
load('e2_woNoise.mat');
e1won = e1;
e2won = e2;
load('e1.mat');
load('e2.mat');
e1n = e1;
e2n = e2;
% load controller
load('RHCtrl.mat');

% car variables
m = ureal('m',1573,'Percentage',0.001);
Iz = ureal('Iz',2873,'Percentage',0.001);
lf = ureal('If',1.8,'Percentage',0.001);
lr = ureal('Ir',1.58,'Percentage',0.001); %% sesnsitive 65%
Cf = ureal('Cf',80000,'Percentage',0.001);
Cr = ureal('Cr',80000,'Percentage',0.001);  %% sensitive 65%

Vx = ureal('Vx',30,'Percentage',0.001);

a22 = -(2*Cf+2*Cr)/(m*Vx);
a23 = 2*(Cf+Cr)/m;
a24 = - (2*Cf*lf - 2*Cr*lr)/(m*Vx);
a42 = -(2*lf*Cf-2*lr*Cr)/(Iz*Vx);
a43 = 2*(lf*Cf-lr*Cr)/Iz;
a44 = -(2*lf*lf*Cf+2*lr*lr*Cr)/(Iz*Vx);

b21 = 2*Cf/m;
b41 = 2*lf*Cf/Iz;

b22 = -Vx -2*(lf*Cf-lr*Cr)/(m*Vx);
b42 = -2*(lf*lf*Cf+lr*lr*Cr)/(Iz*Vx);
                                              
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


s=tf('s');

WT=uss(A,B,C,D);

CLoop_unc_HF = lft( WT([1:9 8:9],1:6) , RHCtrl);

W= logspace(0,2.3,140);
%bodemag(WT(1,1),'k-.',CLoop_unc(1,1),'r-',W)
time = 0:0.05:50;


% step input
psi_step = 0*time;
psi_step(200:end) = 0.06;

% input desired yaw
psi_dot_des = psi_step;

%% with noise
amp = 0.001; % IMU noise amplitude
accel_bias = 0.01; % Accelerometer bias
gyro_bias = 0.01; % Gyroscope bias
d1 = accel_bias * time + amp * sin(8*pi*time);
d2 = gyro_bias + amp * sin(8*pi*time); 
d3 = cumtrapz(time,d1);
d4 = cumtrapz(time,d2);

% Sample C.L. Systems
CLoop_HF_unc20 = usample(CLoop_unc_HF,1); % Generate 1 sample of H inF C.L system

[y2,t2] = lsim(CLoop_HF_unc20(1:5,1:5,1),[d1;d2;d3;d4;psi_dot_des],time); 
figure(2)
subplot(121)
plot(e1n,'b-', t2,y2(:,4),'r--','linewidth',2)
ylabel('e_{1} (m)'); title('Lateral position error'); 
xlabel('Time (sec)');
subplot(122)
plot(e2n*180/pi,'b-', t2,y2(:,5)*180/pi,'r--','linewidth',2) %Plot 3nd CL and OL output
title('Yaw angle error');legend('AutoControl','MathModel');
xlabel('Time (sec)');ylabel('e_2 (deg)');


%% wo noise
% noise
amp = 0; % IMU noise amplitude
accel_bias = 0; % Accelerometer bias
gyro_bias = 0; % Gyroscope bias
d1 = accel_bias * time + amp * sin(8*pi*time);
d2 = gyro_bias + amp * sin(8*pi*time); 
d3 = cumtrapz(time,d1);
d4 = cumtrapz(time,d2);

% Sample C.L. Systems
CLoop_HF_unc20 = usample(CLoop_unc_HF,1); % Generate 1 sample of H inF C.L system

[y2,t2] = lsim(CLoop_HF_unc20(1:5,1:5,1),[d1;d2;d3;d4;psi_dot_des],time); 
figure(3)
subplot(121)
plot(e1won,'b-', t2,y2(:,4),'r--','linewidth',2)
title('Lateral position error'); ylabel('e_{1} (m)');
xlabel('Time (sec)');
subplot(122)
plot(e2won*180/pi,'b-', t2,y2(:,5)*180/pi,'r--','linewidth',2) %Plot 3nd CL and OL output
title('Yaw angle error');legend('AutoControl','MathModel');
xlabel('Time (sec)');ylabel('e_2 (deg)');

