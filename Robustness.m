V = 30;
[A,B,C,D] = lateral_model(V);
J = [-2+3.46*1i -2-3.46*1i -10 -10];
K = acker(A,B(:,1),J);

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
                                              
A_lat = [0 1 0 0;
     0 a22 a23 a24;
     0 0 0 1;
     0 a42 a43 a44;];

B1_lat = [0 b22 0 b42]';  %psi_des_dot input
B2_lat = [0 b21 0 b41]';  %Steering angle input

C_lat = eye(4);
ds = 2.0; 
C_look_ahead = [1 0 ds 0];
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
CLoop_unc = lft( WT([1:5 2:5],1:2) , K_dk * blkdiag(noise,noise,noise,noise));
%CLoop_unc = lft( WT([1:5 2:5],1:2) , K_dk);
W= logspace(0,2.3,140);
%bodemag(WT(1,1),'k-.',CLoop_unc(1,1),'r-',W)
time = 0:0.05:100;
psi_des = 0*time;
psi_des(200:end) = 0.06;%*(1-cos(0.05*pi*time(1:801))); %

[yo,t] = lsim(WT(1:5,1),psi_des,time); % Generate O.L. response
CLoop_unc20 = usample(CLoop_unc,20); % Generate 20 sample of
% uncertain C.L. system
nsamp = 20;
for i=1:nsamp %For each of 20 random samples do a simulation and plot response
[y,t] = lsim(CLoop_unc20(1:5,1,i),psi_des,time); % simul. C.L. response
subplot(321)
plot(t,y(:,2),'b-')
title('lateral position error'); ylabel('e_{1} (m)'); hold on
subplot(322)
plot(t,y(:,3),'b-') %plot(t,y(:,2),'b-',t,yo(:,2),'k--') %Plot 2 nd CL and OL output
title('lateral velocity error')
ylabel('e^._{1} (m/s)'); hold on
subplot(323)
plot(t,y(:,4),'b-') %Plot 3nd CL and OL output
title('Yaw angle error')
xlabel('Time (sec)');ylabel('e_2 (rad)');hold on
subplot(324)
plot(t,y(:,5),'b-') %Plot 4 nd CL and OL output
title('Yaw angular speed error');xlabel('Time (sec)')
ylabel('e^._{2} (rad/s)'); hold on
subplot(325)
plot(t,psi_des,'b-') %Plot 4 nd CL and OL output
title('psi dot desired');xlabel('Time (sec)')
ylabel('\psi_{des}^. (rad/s)'); hold on
subplot(326)
plot(t,y(:,1),'b-') %Plot 5 nd CL and OL output
title('steering angle');xlabel('Time (sec)')
ylabel('\delta (rad)'); hold on
end