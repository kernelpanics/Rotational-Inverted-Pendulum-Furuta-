%% List of parameters
warning off
m_p= 0.31; % mass of pendulum (kg)
l_p=0.58; % pendulum length (m)
m_c=0.53; % cart mass (kg)
l_c=0.36; % base link length (m)
l_cw=0.075; % counter weight link length
m_cw= 0.31; % counter weight mass
b_c=0.025; % rotation damping associated with base link (N.m.sec)

%newtimevec = find(tout>=0.2,1):find(tout<=0.5,1,'last+1');


J=m_c*l_c^2 + m_cw*l_cw^2;
g=9.81; % gravity


%% Matrices

A=[0 1 0 0;
   0 -b_c/J -(m_p*g*l_c/J) 0;
   0 0 0 1;
   0 (l_c*b_c)/(l_p*J) (g/l_p)+(m_p*g*l_c^2)/(J*l_p) 0];

B=[ 0;
    1/J;
    0;
    -l_c/(l_p*J)];

%C=[1;0;0;0];
C=[1 0 0 0;
   0 0 1 0];
D=[0;
   0];

P=[-1.7+7.5i;-1.7-7.5i;-4.8;-3];

K=acker(A,B,P);

% t = 0:0.01:5;
% subplot(4,1,1)
% 
% plot(t,yout(:,1)*180/pi);grid on;
% xlabel('Time');ylabel('\alpha');
% 
% subplot(4,1,2)
% 
% plot(t,yout(:,2)*180/pi);grid on;
% xlabel('Time');ylabel('$\dot{\alpha}$','interpreter','latex');
% 
% subplot(4,1,3)
% 
% plot(t,yout(:,3)*180/pi);grid on;
% xlabel('Time');ylabel('\theta');
% 
% subplot(4,1,4)
% 
% plot(t,yout(:,4)*180/pi);grid on;
% xlabel('Time');ylabel('$\dot{\theta}$','interpreter','latex');

Ac = [(A-B*K)];
Bc = [B];
Cc = [C];
Dc = [D];

states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'r'};
outputs = {'x'; 'phi'};

sys_cl = ss(Ac,Bc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);

t = 0:0.01:5;
r =0.2*ones(size(t));
[y,t,x]=lsim(sys_cl,r,t);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','cart position (m)')
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')
title('Respuesta del Sistema con Ackermann')
