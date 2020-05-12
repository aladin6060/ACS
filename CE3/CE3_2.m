%% Robust RST Controller with Q parametrisation

load('CE2_1')
W1 = c2d((s+20)*0.5/ (s+0.00001),W2.Ts);

H_R= [1 1]

%% Extracting the coefficients of the polynomials A & B

B=G2.b;
A=G2.f;

Q_order=1; 

%Robust performance ||W_1S|+|W_2T||_inf<1
%S=(AS_0-ABQ)/P and Q=H_RH_SQ'
%T=





%%