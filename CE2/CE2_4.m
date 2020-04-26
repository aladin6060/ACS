clc; clear variables; close all;


%javaaddpath('C:\Program Files\Mosek/9.2/tools/platform/win64x86/bin/mosek.jar')
%Mirkos Path, @Silvio: comment mine wes muesch �ndere vorem pushe
%addpath('C:\Program Files\MATLAB\R2018a\Mosek\9.2\toolbox\R2015a')
%@Silvio: Muesch dr ganz Mosek-Folder wo i Program Files hesch ou i di MATLAB
%folder inekopiere
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

%% Define constraints

Bwidth=15; %rad/s
Mmargin=0.5;

s = tf('s');
oinfW1 = (s+Bwidth)*Mmargin/(s+0.00001);
cinfW3 = 1/5;
o2W1 = 1/s;



%% Define Initial controller

z=tf('z',Ts);
Kinit= 0.01/(z-1);
K0=0.01;
Fy=z-1;
Gx=stack(1,G1,G2,G3);
T_init=feedback(Kinit*Gx,1);

figure;
pzmap(T_init)

%% Run solver

P = datadrivenACS;
P.Model.Plant = G_tilde;
P.Model.Frequency = logspace(-2,log10(pi/Ts),400);
P.Feedback.controller.K0=K0;
P.Feedback.controller.Ts=G1.Ts;
P.Feedback.controller.Fy=Fy;
%P.setKinit(Kinit);
P.Feedback.controller.order = 6;

P.Feedback.objective.oinfW1 = oinfW1;
P.Feedback.objective.o2W1 = o2W1;
P.Feedback.constraints.cinfW3 = cinfW3;
P.Feedback.parameters.maxIter = 100;
[Kmat,obj] = solveFB(P);
KFB = computeKFB(P);
