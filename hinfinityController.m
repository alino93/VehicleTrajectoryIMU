V = 30;
% [A,B,C,D] = lateral_model(V);
% J = [-2+3.46*1i -2-3.46*1i -10 -10];
% K = acker(A,B(:,1),J);

m = ureal('m',1573,'Percentage',1);
Iz = 2873;
lf = 1.1;
lr = 1.58;
Cf = 80000;
Cr = 80000;
Vx = 30;

%m = ureal('m',1573,'Percentage',1);
%Iz = ureal('Iz',2873,'Percentage',0.1);

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
                                              
A_lat = [0 1 0 0;
     0 a22 a23 a24;
     0 0 0 1;
     0 a42 a43 a44;];

B1_lat = [0 b22 0 b42]';  %psi_des_dot input
B2_lat = [0 b21 0 b41]';  %Steering angle input

C_lat = eye(4);
D1_lat = zeros(4,1);
D2_lat = zeros(4,1);

A = A_lat;
B = [B1_lat B2_lat];
C = C_lat;
D = [D1_lat D2_lat];

s = tf('s');
Car = uss(A,B,C,D);
Wref = 0.2;
Wn1 = 0.01;
Wn2 = 0.01;
Wr = 0.1;
Wact = (s+64)/(0.01*s+100);
Wp1 = (s+1)/(s+0.01);
Wp2 = (2*s+1)/(s+0.01);

systemnames='Car Wn1 Wn2 Wref Wact Wp1 Wr Wp2';
inputvar='[psid; d1; d2; delta]';
outputvar='[Wact; Wp1; Wr; Wp2; Car(1)+Wn1; Car(3)+Wn2]';
input_to_Car='[Wref; delta]';
input_to_Wn1='[d1]';
input_to_Wn2='[d2]';
input_to_Wp1='[Car(1)]';
input_to_Wr='[Car(2)]';
input_to_Wp2='[Car(3)]';
input_to_Wref='[psid]';
input_to_Wact='[delta]';

Caricunc=sysic; nmeas=2; ncont=1;
[K_dk, CLoop, gama]=dksyn(Caricunc,nmeas,ncont);
gama
HCtrl=zpk(K_dk)
RHCtrl = balred(HCtrl,6)
figure(1)
bodemag(HCtrl,'--',RHCtrl)

%% stability performance
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

D1_lat = zeros(4,1);
D2_lat = zeros(4,1);

A = A_lat;
B = [B1_lat B2_lat];
C = [0 0 0 0;C_lat];
D = [0 1;D1_lat D2_lat];
WT=uss(A,B,C,D);
CLoop_unc_HF = lft( WT([1:5 4:5],1:2) , RHCtrl);

[stabilityMargin] = robuststab(CLoop_unc_HF)
% CLoop_unc = lft( Caricunc([1:4 1:4],1:2) , K_dk);
% W= logspace(0,2.3,140);
% %bodemag(WT(1,1),'k-.',CLoop_unc(1,1),'r-',W)
% time = 0:0.05:100;
% psi_des = 0*time;
% psi_des(200:end) = 0.06;%*(1-cos(0.05*pi*time(1:801))); %
% 
% [yo,t] = lsim(Car(1:4,1),psi_des,time); % Generate O.L. response
% CLoop_unc20 = usample(CLoop_unc,20); % Generate 20 sample of
% % uncertain C.L. system
% nsamp = 20;
% for i=1:nsamp %For each of 20 random samples do a simulation and plot response
% [y,t] = lsim(CLoop_unc20(1:4,1,i),psi_des,time); % simul. C.L. response
% subplot(321)
% plot(t,y(:,1),'b-')
% title('lateral position error'); ylabel('e_{1} (m)'); hold on
% subplot(322)
% plot(t,y(:,2),'b-',t,yo(:,2),'k--') %Plot 2 nd CL and OL output
% title('lateral velocity error')
% ylabel('e^._{1} (m/s)'); hold on
% subplot(323)
% plot(t,y(:,3),'b-', t,yo(:,3),'k--') %Plot 3nd CL and OL output
% title('Yaw angle error')
% xlabel('Time (sec)');ylabel('e_2 (rad)');hold on
% subplot(324)
% plot(t,y(:,4),'b-',t,yo(:,4),'k--') %Plot 4 nd CL and OL output
% title('Yaw angular speed error');xlabel('Time (sec)')
% ylabel('e^._{2} (rad/s)'); hold on
% subplot(325)
% plot(t,psi_des,'b-') %Plot 4 nd CL and OL output
% title('psi dot desired');xlabel('Time (sec)')
% ylabel('\psi_{des}^. (rad/s)'); hold on
% end