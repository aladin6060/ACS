clc; clear all; close all;

a = ureal('a',10,'Plusminus',[-2,2]);
b = ureal('b',2);
c = ureal('c',5);

g = tf(a, [1 b c]);
g_norm = tf(10,[1 2 5]);

B = usample(g,8);
[sys1,info1] = ucover(B,g_norm,1);
[sys2,info2] = ucover(B,g_norm,3);

bodemag(info1.W1,info2.W1,{0.1,100})
legend("first order W2","third order W2",'Interpreter','latex')
title("")

figure
bodemag(g_norm*(1+info1.W1),"r",g_norm*(1+info2.W1),"g",B,"--b",{0.1,100})
legend("$\widetilde{G}$, first order $W2$","$\widetilde{G}$, third order $W2$","samples",'Interpreter','latex')
title(" ")





