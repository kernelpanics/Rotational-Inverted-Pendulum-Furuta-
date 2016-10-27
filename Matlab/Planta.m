
warning off
clear
clc
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
A=[0 1 0 0;
   0 0 X1 0;
   0 0 0 1;
   0 0 Y1 0];
B=[0;
   X2;
   0;
   Y2];
C=[1 0 0 0;
   0 0 1 0];
D=[0;
   0];
I=[1 0 0 0;
   0 1 0 0;
   0 0 1 0;
   0 0 0 1];

% [num,den]=ss2tf(A,B,C,0);
% P=tf(num,den);

% P=C*((s*I-A)^-1)*B;
% k1=(J1-l0*l1*m1+(l1^2)*m1)/(I0*J1+I0*(l1^2)*m1+J1*(l0^2)*m1);
% w1=(g*l1*m1)/(l0*l1*m1-(l1^2)*m1-J1);
% w2=(g*l1*m1*(I0+(l0^2)*m1))/(I0*J1+I0+(l1^2)*m1+J1*(l0^2)*m1);
% s=tf('s');
% display(P);
% display(k1);
% display(w1);
% display(w2);

%Q = C'*C;

Q=diag([1 0 1 0]);
R = 1;
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
