clc; clear variables; close all;
addpath('C:\Users\Mirko\Documents\Studium\5eme année\Advanced Control Systems\ACS\CE2')
%% Test if mosek is properly implemented
import mosek.fusion.*;
M = Model()

%% Load models
load('CE2_1')

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

G_tilde=stack(1,G1,G2,G3,G1f,G2f,G3f);

clear Z
clear Zd

%% Define constraints and objectives
Bwidth=15; %rad/s
Mmargin=0.5;
Ts=G1.Ts;

s = tf('s');
cinfW1 = (s+Bwidth)*Mmargin/(s+0.00001);
cinfW3 = 1/5;
o2W1 = 1/s;

%% Define Initial controller

z=tf('z',Ts);
Kinit= 1/(z-1);
Gx=stack(1,G1,G2,G3);
T_init=feedback(Kinit*Gx,1);

figure;
pzmap(T_init)

%% Run solver

P = datadrivenACS;
P.Model.Plant = G_tilde;

P.setKinit(Kinit);
P.Feedback.controller.order = 4;

P.Feedback.constraints.cinfW1 = cinfW1;
P.Feedback.constraints.cinfW3 = cinfW3;
P.Feedback.objective.o2W1 = 1/s;
P.Feedback.parameters.maxIter = 200;
P.Feedback.parameters.tol=0.0001;
P.Feedback.parameters.c2=0.1;
[Kmat,obj] = solveFB(P);
KFB = computeKFB(P);

figure;
bode(KFB)

%% Create plots
S=feedback(1,Gx*KFB);
T=feedback(Gx*KFB,1);
U=feedback(KFB,Gx);
t=0:Ts:1.5;

figure;
subplot(2,2,1);
step(T,t);
title('Step response output')

subplot(2,2,2);
step(U,t);
title('Step response control signal')

subplot(2,2,3);
bodemag(U)
hold on
bodemag(1/cinfW3*tf([1] ,[1]))
title('Input Sensivity U')

subplot(2,2,4);
bodemag(S)
hold on
bodemag(1/cinfW1)
title('Sensitivity function S')
%%
load('CE2_2')

figure;
subplot(2,2,1);
step(T,t,'-r');
hold on
step(Tred,t,'-b')
title('Step response output')
lgd=legend('Datadriven','H_\infty')
lgd.Location='southeast'
lgd.Box='off'

subplot(2,2,2);
step(U,t,'-r');
hold on
step(Ured,t,'-b')
title('Step response control signal')
lgd=legend('Datadriven','H_\infty')
lgd.Location='northeast'
lgd.Box='off'

subplot(2,2,3);
bodemag(U,'-r')
hold on
bodemag(Ured,'-b')
hold on
bodemag(1/cinfW3*tf([1] ,[1]))
title('Input Sensivity U')
lgd=legend('Datadriven','H_\infty')
lgd.Location='southwest'
lgd.Box='off'
xlim([0.1 200])

subplot(2,2,4);
bodemag(S,'-r')
hold on
bodemag(Sred,'-b')
hold on
bodemag(1/cinfW1)
title('Sensitivity function S')
lgd=legend('Datadriven','H_\infty')
lgd.Location='southeast'
lgd.Box='off'
xlim([0.1 200])


