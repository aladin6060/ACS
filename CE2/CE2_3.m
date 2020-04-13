clc; close all; clear variables;
%% 1. Importing the models and transformation

load Gyro300
Z=iddata(y,u,Ts);
Zd=detrend(Z);
G1 = oe(Zd,[6 6 1]);
G1f=spa(Zd,100);
G1c = ss(d2c(G1)) %transforming the id model to a continous state space model
 
A = G1c.A
B = G1c.B
C = G1c.C
D = G1c.D


%% 2. Setting up the LMI

n = 6
Y = sdpvar(1,n);
L = sdpvar(n,n,'symmetric');
alpha = sdpvar(1,1);
beta = sdpvar(1,1);
lmi = A*L+L*A'-B*Y-Y'*B'+B*B'<=0;
lmi = [lmi , C*L*C' <= alpha ];
lmi = [lmi , [ beta Y;Y' L ] >= 0 ];


%% 3. Solving the SDP problem
options = sdpsettings('solver','lmilab')
optimize(lmi,alpha+beta)

L=value (L) ;
Y=value (Y) ;
K=Y*L^( -1) ;
sys = ss(A-B*K,B,[C;-K],0);

%% 4.Calculating a controller with LQR

Klqr = lqr(G1c,C'*C,1);
sys_lqr = ss(A-B*Klqr,B,[C;-Klqr],0);
figure
step(sys)
set(gcf,'Renderer', 'painters', 'Position', [10 10 1100 800]);
print(gcf,'LQRvsSDP.png','-dpng','-r300');



