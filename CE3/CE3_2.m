%% Robust RST Controller with Q parametrisation

load('CE2_1')
load('CE3_1')
S=S';
R=R';
P=P';
s = tf('s')
Ts=W2.Ts;
W1 = c2d((s+20)*0.5/ (s+0.00001),Ts);
W3 = 1/5*tf([1] ,[1]);

Hr= [1 1]
Hs= [1 -1]

%% Extracting the coefficients of the polynomials A & B

B=G2.b;
A=G2.f;

Q_order=1; 

%Robust performance ||W_1S|+|W_2T||_inf<1
%S=(AS_0-ABQ)/P and Q=H_RH_SQ'
%T=1-S
S0=[S, zeros(1,Q_order)];
R0=[R, zeros(1,Q_order)];
Q0= zeros(1,Q_order);

[R,S,T,fval] = QParametrization(Q0,A,B,R0,S0,Hr,Hs,Ts,P,W1,W2,W3)

%% Plots

CL=tf(conv(T,B),P,Ts,'variable','z^-1');

stepinfo(CL)

Sensitivity=1-CL;
U=tf(conv(A,R),P,Ts,'variable','z^-1')

figure(1)
subplot(2,2,1)
step(CL)
axis([0 1.5 0 1.2])
title('Step response output')

subplot(2,2,2)
step(U)
title('Step response control signal')

subplot(2,2,3)
bodemag(U,db2mag(30)*tf([1],[1]))
title('Input sensitivity U')

s = tf('s')
W1 = c2d((s+20)*0.5/ (s+0.00001),Ts);
subplot(2,2,4)
bodemag(Sensitivity,1/W1)
title('Sensitivity function S')
set(gcf,'Renderer', 'painters', 'Position', [10 10 1100 800]);