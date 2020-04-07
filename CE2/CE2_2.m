clc; clear all; close all;

s = tf('s')

% 1. Design the weighting filter W1(z) for
% — A desired closed-loop bandwidth of around 15 rad/s. -> pole at 15 rad/s
% — Zero steady state tracking error for a step reference.-> Two zeros at 0
% — A modulus margin of at least 0.5.-> max magnitude 1/r = 2


W1 = (s+15)*0.5/ (s+0.00001)
bodemag(W1)

% 2. Use the best nominal model and its multiplicative uncertainty filter W2(z) to design an
% H? controller for robust performance using the mixed sensitivity approach (use mixsyn).

%I just take a random W2 till we know the best one
%%%%%%%%%%%%%%%%%%%%%%%%%%
load Gyro300
Z=iddata(y,u,Ts);
Zd = detrend(Z);
G1 = oe(Zd,[6 6 1]);
G1f = spa(Zd,100);

load Gyro400
Z = iddata(y,u,Ts);
Zd = detrend(Z);
G2 = oe(Zd,[6 6 1]);
G2f = spa(Zd,100);

load Gyro500
Z = iddata(y,u,Ts);
Zd = detrend(Z);
G3 = oe(Zd,[6 6 1]);
G3f = spa(Zd,100);
%%%%%%%%%%%%%%%%%%%%%%%%%%

Gn = G1;
G = stack(1,G1,G2,G3,G1f,G2f,G3f);
[Gu, Info] = ucover(G,Gn,4);

W2Ex1 = load('W2.mat'); % load W2 from exercise 1
W2Ex1=W2Ex1.bestW2;
W2 = W2Ex1;

W3 = 1/5; % ensuring that control signal doesnt saturate

W1d = c2d(W1,W2.Ts); %transform W1 to a descrete time model

K = mixsyn(Gn,W1d,W3,W2); 

% 3. Plot the step response of the closed-loop system (output and control signal), the magnitude
% of the input sensitivity function U(z) and the sensitivity function S(z).

G = stack(1,G1,G2,G3);

T = feedback(K*G,1);
U = feedback(K,G);
S = feedback(1,K*G);

figure(1)
subplot(2,2,1)
step(T)
title('Step response output')

subplot(2,2,2)
step(U)
title('Step response control signal')

subplot(2,2,3)
bodemag(U,5*tf([1],[1]))
title('Input sensitivity U')

subplot(2,2,4)
bodemag(S,1/W1d)
title('Sensitivity function S')


% 5. The order of the final controller may be too large (especially if the order of W2(z) is large).
% Check if there is zero/pole cancellation in the controller using pzmap. The order of the
% controller can be reduced using the reduce command. Check the stability and performance
% of the closed-loop system with the reduced order controller.

%figure(2)
%subplot(1,2,1)
%pzmap(K)
%subplot(1,2,2)
%hsvd(K) %calculating the Hankel singular values 

Kred = reduce(K,1); %reducing the controller to first order

Tred = feedback(Kred*G,1);
Ured = feedback(Kred,G);
Sred = feedback(1,Kred*G);

figure(3)
subplot(2,2,1)
step(Tred)
title('Step response output')

subplot(2,2,2)
step(Ured)
title('Step response control signal')

subplot(2,2,3)
bodemag(Ured,5*tf([1],[1]))
title('Input sensitivity U')

subplot(2,2,4)
bodemag(Sred,1/W1d)
title('Sensitivity function S')





