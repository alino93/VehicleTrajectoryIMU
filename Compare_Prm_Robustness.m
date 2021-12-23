V = 30;
[A,B,C,D] = lateral_model(V);
J = [-2-3.46*1i -10 -2+3.46*1i -10];
K = acker(A,B(:,1),J);

% load H-inf controller
load('RHCtrl.mat');

m = ureal('m',1573,'Percentage',5);
Iz = ureal('Iz',2873,'Percentage',5);
lf = ureal('If',1.1,'Percentage',5);
lr = ureal('Ir',1.58,'Percentage',5); %% sesnsitive 65%
Cf = ureal('Cf',80000,'Percentage',5);
Cr = ureal('Cr',80000,'Percentage',5);  %% sensitive 65%

Vx = ureal('Vx',30,'Percentage',5);
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

D1_lat = zeros(4,1);
D2_lat = zeros(4,1);

A = A_lat;
B = [B1_lat B2_lat];
C = [0 0 0 0;C_lat];
D = [0 1;D1_lat D2_lat];
K_dk = -K;

s=tf('s');
noise = 1 + 0.1*8*pi/(s^2 + (8*pi)^2);
WT=uss(A,B,C,D);
CLoop_unc_SF = lft( WT([1:5 2:5],1:2) , K_dk);
CLoop_unc_HF = lft( WT([1:5 4:5],1:2) , RHCtrl);

W= logspace(0,2.3,140);
%bodemag(WT(1,1),'k-.',CLoop_unc(1,1),'r-',W)
time = 0:0.05:100;
% step input
psi_step = 0*time;
psi_step(200:end) = 0.06;%*(1-cos(0.05*pi*time(1:801))); %
% sin input
psi_sin = 0*time;
psi_sin(200:1000) = 0.06*(1 - sin(0.05*pi*time(200:1000)));
% input desired yaw
psi_dot_des = psi_step;
% Sample C.L. Systems
[yo,t1] = lsim(WT(1:5,1),psi_dot_des,time); % Generate O.L. response
CLoop_SF_unc20 = usample(CLoop_unc_SF,20); % Generate 20 sample of State F C.L system
CLoop_HF_unc20 = usample(CLoop_unc_HF,20); % Generate 20 sample of H inF C.L system

nsamp = 20;
for i=1:nsamp %For each of 20 random samples do a simulation and plot response
[y1,t1] = lsim(CLoop_SF_unc20(1:5,1,i),psi_dot_des,time); % simul. C.L. response
[y2,t2] = lsim(CLoop_HF_unc20(1:5,1,i),psi_dot_des,time); 
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
plot(t1,y1(:,5),'b-', t2,y2(:,5),'r-') %Plot 3nd CL and OL output
title('Yaw angle error')
xlabel('Time (sec)');ylabel('e_2 (rad)');hold on
subplot(324)
plot(t1,y1(:,3),'b-', t2,y2(:,3),'r-') %Plot 4 nd CL and OL output
title('Yaw rate error');xlabel('Time (sec)')
ylabel('e^._{2} (rad/s)'); hold on
subplot(325)
plot(t1,psi_dot_des,'b-') %Plot 4 nd CL and OL output
title('Yaw rate desired');xlabel('Time (sec)')
ylabel('\psi_{des}^. (rad/s)'); hold on
subplot(326)
plot(t1,y1(:,1),'b-', t2,y2(:,1),'r-') %Plot 5 nd CL and OL output
title('Steering angle');xlabel('Time (sec)')
ylabel('\delta (rad)'); hold on
end