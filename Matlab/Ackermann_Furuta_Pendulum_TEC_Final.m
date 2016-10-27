
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

%k1=-1.76; %k2=-170.31; %k3=-5.5; %k4=-2.45;
poles = eig(A);
P=[-5;-110;-3.5;-0.5];
%P=[-4.5+5i;-4.5-5i;-6.4;-3];
%P=[-2+7.5i;-2-7.5i;-6.4;-6.4];
K=acker(A,B,P)
%[K,PREC]=place(A,B,P)
% 
% Ac = [(A-B*K)];
% Bc = [B];
% Cc = [C];
% Dc = [D];
% 
% states = {'x' 'x_dot' 'phi' 'phi_dot'};
% inputs = {'r'};
% outputs = {'x'; 'phi'};
% 
% sys_cl = ss(Ac,Bc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);
% 
% t = 0:0.01:5;
% r =0.2*ones(size(t));
% [y,t,x]=lsim(sys_cl,r,t);
% [AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
% set(get(AX(1),'Ylabel'),'String','Posicion masa(brazo)')
% set(get(AX(2),'Ylabel'),'String','Ang Pendulo (radianes)')
% title('Respuesta del Sistema con Ackermann')
