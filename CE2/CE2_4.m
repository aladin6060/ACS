clc; clear variables; close all;


addpath('C:\Users\Mirko\Documents\Studium\5eme année\Advanced Control Systems\ACS\CE2')

%@Silvio: 
%Mach Command prompt: open startup, när gheisch dert dr javaaddpath ine, so
%wirds jedes mau usgfüert wedes startisch
%tbxmanager restorepath
%javaaddpath('C:\Program Files\Mosek/9.2/tools/platform/win64x86/bin/mosek.jar')
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
Ts=G1.Ts;

s = tf('s');
oinfW1 = c2d((s+Bwidth)*Mmargin/(s+0.00001),Ts);
cinfW3 = 1/5;
o2W1 = 1/s;



%% Define Initial controller

z=tf('z',Ts);
Kinit= 1/(z-1);
%%K0=0.01;
%Fy=z-1;
%K=K0*Fy^-1;
Gx=stack(1,G1,G2,G3);
T_init=feedback(Kinit*Gx,1);

figure;
pzmap(T_init)

%% Run solver

P = datadrivenACS;
P.Model.Plant = G_tilde;
%P.Model.Frequency = logspace(-2,log10(pi/Ts),400);

%P.Feedback.controller.K0=K0;
%P.Feedback.controller.Ts=Ts;
%P.Feedback.controller.Fy=Fy;
P.setKinit(Kinit);
P.Feedback.controller.order = 7;

P.Feedback.constraints.cinfW1 = oinfW1;
P.Feedback.constraints.cinfW3 = cinfW3;
P.Feedback.objective.o2W1 = 1/s;
P.Feedback.parameters.maxIter = 200;
P.Feedback.parameters.tol=0.0001;
P.Feedback.parameters.c2=10^-3;
[Kmat,obj] = solveFB(P);
KFB = computeKFB(P);
%%
bode(KFB)
shg
%%
S=feedback(1,G_tilde*KFB);
T=feedback(G_tilde*KFB,1);
U=feedback(KFB,G_tilde);
