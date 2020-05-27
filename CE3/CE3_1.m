%% Pole placement
clc; close all; clear all;

load Gyro300
Z=iddata(y,u,Ts);
Zd=detrend(Z);
G1 = oe(Zd,[6 6 1]);
G1f=spa(Zd,100);

load Gyro400
Z=iddata(y,u,Ts);
Zd=detrend(Z);
G2 = oe(Zd,[6 6 1]);
G2f=spa(Zd,100);

load Gyro500
Z=iddata(y,u,Ts);
Zd=detrend(Z);
G3 = oe(Zd,[6 6 1]);
G3f=spa(Zd,100);

clear Z
clear Zd
%% Extracting the coefficients of the polynomials A & B

B=G2.b;
A=G2.f;

%% Dominant poles
overshoot=0.05;
Tset=0.6;
dampfact=sqrt(log(overshoot)^2/(log(overshoot)^2+pi^2));
natfreq=-log(0.02)/(dampfact*Tset);
p1=-2*exp(-dampfact*natfreq*Ts)*cos(natfreq*Ts*sqrt(1-dampfact^2));
p2=exp(-2*dampfact*natfreq*Ts);
P=[1;p1;p2];

%% Find R & S
H_S=[1,-1]; %integrator
H_R=[1,1]; %open loop at Nyqu frequ

[R,S]=poleplace(B,A,H_R,H_S,P);

%% Verify closed loop poles
P_achieved=conv(A,S)+conv(B,R)
P_tf = tf(1,P_achieved',1)
pzmap(P_tf) %poles around zero should be zero, nummerical error

%% Calculate T
%same tracking and regulation dynamics -> sum up R
T=sum(R)

%% Calculate tracking step response of cl-system

%result: we exceed the given maximal control signal (30dB), so the solution would
%be to slow down the poles. But there we have not a lot of margin as the
%settling time is 0.58<0.6s just slightly below the constraint -> hence we
%need Q parametrisation
CL=tf(conv(T,B),P',Ts,'variable','z^-1');

stepinfo(CL)

Sensitivity=1-CL;

U_sens=tf(conv(A,R)',P',Ts,'variable','z^-1');
U=tf(conv(A,T),P',Ts,'variable','z^-1');

figure(1)
subplot(2,2,1)
step(CL)
axis([0 1.5 0 1.2])
title('Step response output')

subplot(2,2,2)
step(U)
title('Step response control signal')

subplot(2,2,3)
bodemag(U_sens,db2mag(30)*tf([1],[1]))
title('Input sensitivity U')

s = tf('s')
W1 = c2d((s+20)*0.5/ (s+0.00001),Ts);
subplot(2,2,4)
bodemag(Sensitivity,1/W1)
title('Sensitivity function S')
set(gcf,'Renderer', 'painters', 'Position', [10 10 1100 800]);
print(gcf,'RSTPolePlacement.png','-dpng','-r300');

save('CE3_1','R','S','T','P')


