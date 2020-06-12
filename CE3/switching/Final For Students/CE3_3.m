%% Reset
clc; close all; clear all;
%% Initialisation

load('id_data')
B1=G1.b(2:end);
A1=G2.f(2:end);
B2=G2.b(2:end);
A2=G2.f(2:end);
B3=G3.b(2:end);
A3=G3.f(2:end);

alpha = 2; %factor instantaneous weight
lambda = 0.05; % forgetting factor
DT = 5; %dwell time

%% Controller Calculation


[N1,D1] = controller(G1,G1f,Ts)
[N2,D2] = controller(G2,G2f,Ts)
[N3,D3] = controller(G3,G3f,Ts)


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