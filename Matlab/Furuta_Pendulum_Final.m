warning off
clear
clc

g = 9.74;          %Gravedad (m/s^2)
r=0.22;            %Largo del brazo (m)
J=0.000678;        %inercia del brazo (kg*m^2)
l=0.148;           %Distancia al centro de masa (m)
m=0.01481;         %Masa Pendulo (kg)
R=8.8;             %Resistencia Motor Ohms
Ke=1.037747;       %NmA^-1
        
X1=-m*r*g/J;
X2=-(Ke^2)/(J*R);
Y1=g*(J+m*r^2)/(J*l);
Y2=(r*(Ke^2))/(J*R*l);
X3=Ke/(J*R);
Y3=-(Ke*r)/(J*R*l);

A=[0 0 1 0;
   0 0 0 1;
   0 X1 X2 0;
   0 Y1 Y2 0];
B=[0;
   0;
   X3;
   Y3];
C=[1 0 0 0;
   0 1 0 0];
D=[0;
   0];
I=[1 0 0 0;
   0 1 0 0;
   0 0 1 0;
   0 0 0 1];

Q=diag([0.04988 438.401 0.0862 0]);
%Q=diag([0.05 449.35 0.0832 0]); %matriz más cercana
R = 0.0161;
%R = 0.0161;
K = lqr(A,B,Q,R);

Ac = [(A-B*K)];
Bc = [B];
Cc = [C];
Dc = [D];

states = {'theta0' 'theta0_dot' 'theta1' 'theta1_dot'};
inputs = {'torque'};
outputs = {'theta0'; 'theta1'};

sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
display(sys_ss);
poles = eig(A)
co = ctrb(sys_ss);
controllability = rank(co)
display(Q);
display(K);
sys_cl = ss(Ac,Bc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);
[num,den]=ss2tf(A,B,[1 0 1 0],[0]);
pend=tf(num,den);
t = 0:0.01:5;
r =0.2*ones(size(t));
[y,t,x]=lsim(sys_cl,r,t);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','cart position (m)')
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')
title('Step Response with LQR Control')
