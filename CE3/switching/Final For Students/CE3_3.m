%% Reset
clc; close all; clear all;
%% Initialisation

load('id_data')
[B1,A1,Ts]= tfdata(G1,'v');
[B2,A2]= tfdata(G2,'v');
[B3,A3]= tfdata(G3,'v');
clear u;
clear y;


alpha = 2; %factor instantaneous weight
lambda = 0.05; % forgetting factor
DT = 4; %dwell time

%% Controller Calculation


[N1,D1] = controller(G1,G1f,Ts);
[N2,D2] = controller(G2,G2f,Ts);
[N3,D3] = controller(G3,G3f,Ts);



function [Knum,Kdenum]=controller(G,Gf,Ts)

s = tf('s')
W1 = (s+20)*0.5/ (s+0.00001)
W1d = c2d(W1,Ts); 
W3 = 1/5;
G_tilde = stack(1,G,Gf);
[sys,info]=ucover(G_tilde,G,7);
 
[K,CL,Gamma] = mixsyn(G,W1d,W3,info.W1);

Kred = reduce(K,6);

[Knum,Kdenum] = ss2tf(Kred.A,Kred.B,Kred.C,Kred.D);

end