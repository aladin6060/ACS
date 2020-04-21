clc; close all; clear variables;


load Gyro300
Z=iddata(y,u,Ts);
Zd=detrend(Z);
G1 = oe(Zd,[6 6 1]);
G1f=spa(Zd,100);

G1c = d2c(G1)
G1ss=ss(G1c) % weiss nid was dr ungerschid zwüsche G1c.A und däm isch, aber es git wenigstens ä 6x6 Matrix use

A = G1ss.A
B = G1ss.B
C = G1ss.C
D = G1ss.D



Y = sdpvar(6,1);
L = sdpvar(6,6,'symmetric');
lmi = A*L+L*A'-B*Y-Y'*B'+B*B'<=0;
lmi = [lmi, L>0];
options = sdpsettings('solver','lmilab')

optimize(lmi, C*L*C' + Y'*Y) %falsch

