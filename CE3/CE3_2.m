%% Robust RST Controller with Q parametrisation

load('CE2_1')
load('CE3_1')
S=S';
R=R';
P=P';
s = tf('s')
Ts=W2.Ts;
W1 = c2d((s+15)*0.5/ (s+0.00001),Ts);
%W1 = 0.5;
W3 = 1/5*tf([1] ,[1]);

Hr= [1 1];
Hs= [1 -1];

%% Extracting the coefficients of the polynomials A & B

B=G2.b;
A=G2.f;

Q_order=8; 

%Robust performance ||W_1S|+|W_2T||_inf<1
%S=(AS_0-ABQ)/P and Q=H_RH_SQ'
%T=1-S
S0=[S, zeros(1,Q_order)];
R0=[R, zeros(1,Q_order)];
Q0= zeros(1,Q_order);

[R,S,T,fval] = QParametrization(Q0,A,B,R0,S0,Hr,Hs,Ts,P,W1,W2,W3)



%% Plots


B1=G1.b;
A1=G1.f;
P1=conv(A1,S)+conv(B1,R);
B3=G3.b;
A3=G3.f;
P3=conv(A3,S)+conv(B3,R);
CL=tf(conv(R,B),P,Ts,'variable','z^-1');
CL1=tf(conv(R,B1),P1,Ts,'variable','z^-1');
CL3=tf(conv(R,B3),P3,Ts,'variable','z^-1');

CL_bigLoop=tf(conv(T,B),P,Ts,'variable','z^-1');
CL1_bigLoop=tf(conv(T,B1),P1,Ts,'variable','z^-1');
CL3_bigLoop=tf(conv(T,B3),P3,Ts,'variable','z^-1');
stepinfo(CL_bigLoop)
stepinfo(CL1_bigLoop)
stepinfo(CL3_bigLoop)

Sensitivity1=1-CL1;
Sensitivity=1-CL;
Sensitivity3=1-CL3;
U1=tf(conv(A1,T),P1,Ts,'variable','z^-1');
U=tf(conv(A,T),P,Ts,'variable','z^-1');
U3=tf(conv(A3,T),P3,Ts,'variable','z^-1');

U_sens1=tf(conv(A1,R),P1,Ts,'variable','z^-1');
U_sens=tf(conv(A,R),P,Ts,'variable','z^-1');
U_sens3=tf(conv(A3,R),P3,Ts,'variable','z^-1');
%%
figure;
subplot(2,2,1)
step(CL_bigLoop,CL1_bigLoop,CL3_bigLoop)
axis([0 1 0 1.5])
title('Step response output')

subplot(2,2,2)
step(U,U1,U3)
title('Step response control signal')

subplot(2,2,3)
bodemag(U_sens,U_sens1,U_sens3,db2mag(30)*tf([1],[1]))
title('Input sensitivity U')

s = tf('s');
subplot(2,2,4)
bodemag(Sensitivity,Sensitivity1,Sensitivity3,1/W1)

title('Sensitivity function S')
set(gcf,'Renderer', 'painters', 'Position', [10 10 1100 800]);
saveas(gcf,'QParam.png')

%%
figure; %should be below 0db to satisfy constriant
bodemag((W1*Sensitivity)*conj(W1*Sensitivity) + W2*(1-Sensitivity)*conj(W2*(1-Sensitivity)))