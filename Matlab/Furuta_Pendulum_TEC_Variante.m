clear;close all;clc;
% % parameters
% g=9.8; r1=0.035; l1=0.14; lc=0.133; l2=0.29; 
% m1=56.1/1000; mc=16.7/1000; me=22.1/1000; m2=49.9/1000;
% R=12.486; K=0.2665;
% 
% % A matrix
% A32 = 3*g*l1*m2/(4*lc^2*mc+3*l1^2*(m2+4*me)+6*m1*r1^2);
% A33 = -12*K^2/(R*(4*lc^2*mc+3*l1^2*(m2+4*me)+6*m1*r1^2));
% A42 = 3*(g+9*g*l1^2*m2/(4*lc^2*mc+3*l1^2*(m2+4*me)+6*m1*r1^2))/(2*l2);
% A43 = -18*K^2*l1/(R*l2*(4*lc^2*mc+3*l1^2*(m2+4*me)+6*m1*r1^2));
% A = [0 0 1 0; 0 0 0 1; 0 A32 A33 0; 0 A42 A43 0];
% 
% % B matrix
% B3 = 12*K/(R*(4*lc^2*mc+3*l1^2*(m2+4*me)+6*m1*r1^2));
% B4 = 18*K*l1/(R*l2*(4*lc^2*mc+3*l1^2*(m2+4*me)+6*m1*r1^2));
% B = [0; 0; B3; B4];

% C matrix
C = eye(4);

% D matrix
D = 0;


warning off
I0=0.000678;        %inercia del brazo (kg*m^2)
l0=0.22;            %Largo del brazo (m)
m1=0.01481;         %Masa Pendulo (kg)
l1=0.148;           %Distancia al centro de masa (m)
J1=0.0003;          %Inercia del pendulo en el centro de gravedad (kg*m^2)
theta0=0.001;       %Angulo rotacional del brazo
theta1=0.001;       %Angulo rotacional del pendulo
tau=0.071;          %Torque del motor (kg*cm)
g = 9.74;           %Graveda (m/s^2)

% syms I0 l0 m1 l1 J1 tau g;
X1=-((m1^2)*(l1^2)*l0*g)/(I0*(J1+m1*(l1^2))+J1*m1*(l0^2));
Y1=((I0+m1*(l0^2))*m1*l1*g)/(I0*(J1+m1*(l1^2))+J1*m1*(l0^2));
X2=(J1+m1*(l1^2))/(I0*(J1+m1*(l1^2))+J1*m1*(l0^2));
Y2=(-m1*l1*l0)/(I0*(J1+m1*(l1^2))+J1*m1*(l0^2));

A=[0 1 0 0; 0 0 X1 0; 0 0 0 1; 0 0 Y1 0];

B=[0; X2; 0; Y2];

% C=[1 0 0 0;
%    0 0 1 0];
% 
% D=[0;
%   0];

% LQR
Q = diag([1.5 6 0 0]);
R = 0.0028;
[K, S, EIG] = lqr(A, B, Q, R);
display(K);
display(EIG);

% system simulation
sys = ss(A,B,C,D);
sys_feedback = feedback(sys,K);
step(sys_feedback);