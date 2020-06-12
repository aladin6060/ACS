%% Pole placement
clc; close all; clear all;

load Gyro300
Z=iddata(y,u,Ts);
Zd=detrend(Z);
G1 = oe(Zd,[6 6 1]);
G1f=spa(Zd,100);
B1=G1.b;
A1=G1.f;

load Gyro400
Z=iddata(y,u,Ts);
Zd=detrend(Z);
G2 = oe(Zd,[6 6 1]);
G2f=spa(Zd,100);
B2=G2.b;
A2=G2.f;

load Gyro500
Z=iddata(y,u,Ts);
Zd=detrend(Z);
G3 = oe(Zd,[6 6 1]);
G3f=spa(Zd,100);
B3=G3.b;
A3=G3.f;


Ts=G1.Ts
[R1,S1,T1] = rtscontroller(B1,A1,Ts)
[R2,S2,T2] = rtscontroller(B2,A2,Ts)
[R3,S3,T3] = rtscontroller(B3,A3,Ts)


save('CE3_3')


function  [R,S,T] = rtscontroller(B,A,Ts)
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
%pzmap(P_tf) %poles around zero should be zero, nummerical error

%% Calculate T
%same tracking and regulation dynamics -> sum up R
T=sum(R)
end