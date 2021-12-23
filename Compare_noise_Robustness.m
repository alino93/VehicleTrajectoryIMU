V = 30;
[A,B,C,D] = lateral_model(V);
J = [-2-3.46*1i -10 -2+3.46*1i -10];
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
time = 0:0.05:100;

% no input (straight road)
psi_str = 0*time;
% step input
psi_step = 0*time;
psi_step(200:end) = 0.06;
% sin input
psi_sin = 0*time;
psi_sin(200:1000) = 0.06*(1 - sin(0.05*pi*time(200:1000)));
% input desired yaw
psi_dot_des = psi_sin;

% noise
amp = 0.001; % IMU noise amplitude
accel_bias = 0.01; % Accelerometer bias
gyro_bias = 0.01; % Gyroscope bias
d1 = accel_bias * time + amp * cos(8*pi*time);
d2 = gyro_bias + amp * cos(8*pi*time); 
d3 = cumtrapz(time,d1);
d4 = cumtrapz(time,d2);

% Sample C.L. Systems
[yo,t1] = lsim(WT(1:5,1),psi_dot_des,time); % Generate O.L. response
CLoop_SF_unc20 = usample(CLoop_unc_SF,1); % Generate 1 sample of State F C.L system
CLoop_HF_unc20 = usample(CLoop_unc_HF,1); % Generate 1 sample of H inF C.L system

nsamp = 1;
for i=1:nsamp %For each of samples do a simulation and plot response
[y1,t1] = lsim(CLoop_SF_unc20(1:5,1:5,i),[d1;d2;d3;d4;psi_dot_des],time); 
[y2,t2] = lsim(CLoop_HF_unc20(1:5,1:5,i),[d1;d2;d3;d4;psi_dot_des],time); 
figure(2)
subplot(321)
plot(t1,y1(:,4),'b-', t2,y2(:,4),'r-')
title('Lateral position error'); ylabel('e_{1} (m)'); hold on
subplot(322)
plot(t1,y1(:,2),'b-', t2,y2(:,2),'r-') %plot(t,y(:,2),'b-',t,yo(:,2),'k--') %Plot 2 nd CL and OL output
title('Lateral velocity error');
legend('State feedback','H-inf');
ylabel('e^._{1} (m/s)'); hold on
subplot(323)
plot(t1,y1(:,5)*180/pi,'b-', t2,y2(:,5)*180/pi,'r-') %Plot 3nd CL and OL output
title('Yaw angle error')
xlabel('Time (sec)');ylabel('e_2 (deg)');hold on
subplot(324)
plot(t1,y1(:,3)*180/pi,'b-', t2,y2(:,3)*180/pi,'r-') %Plot 4 nd CL and OL output
title('Yaw rate error');xlabel('Time (sec)')
ylabel('e^._{2} (deg/s)'); hold on
subplot(325)
plot(t1,psi_dot_des*180/pi,'b-') %Plot 4 nd CL and OL output
title('Desired yaw rate');xlabel('Time (sec)')
ylabel('\psi_{des}^. (deg/s)'); hold on
subplot(326)
plot(t1,y1(:,1)*180/pi,'b-', t2,y2(:,1)*180/pi,'r-') %Plot 5 nd CL and OL output
title('Steering angle');xlabel('Time (sec)')
ylabel('\delta (deg)'); hold on
end