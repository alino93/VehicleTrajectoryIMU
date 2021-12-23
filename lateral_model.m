function [A,B,C,D] = lateral_model(V)

% Rajesh Rajamani
%
% This function calculates the matrices for the state space model of
% the lateral vehicle system, for a given longitudinal vehicle speed V
%
% function [A,B,C,D] = lateral_model(V)
% V - longitudinal velocity
% B - first column steering input, second column desired psi_dot
% C - All states as output
% D - zeros (4 x 2)


m = 1573;
Iz = 2873;
lf = 1.1;
lr = 1.58;
Cf = 80000;
Cr = 80000;


Vx = V;
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
B = [B2_lat B1_lat];
C = C_lat;
D = [D1_lat D2_lat];

