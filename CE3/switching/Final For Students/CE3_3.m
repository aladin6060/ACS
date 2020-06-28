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
lambda = 0.005; % forgetting factor
DT = 25; %dwell time

%% Controller Calculation


[N1,D1] = controller(G1,G1f,Ts);
[N2,D2] = controller(G2,G2f,Ts);
[N3,D3] = controller(G3,G3f,Ts);

%% Simulation different thetas

for i=1:5
    theta_switch = -1.5+0.5*i
sim('CE3_3_sim.slx',25)
    sigma_results(i) = sigma
end
%% Visualisation different thetas
%theta set to variable theta switch, simulation time 25
close all;
subplot(121)
hold on
plot(sigma_results(1))
plot(sigma_results(3))
plot(sigma_results(5))
hold off
xlabel('time[s]')
ylabel('sigma')
legend('theta=-1','theta=0','theta=1')

subplot(122)
hold on
plot(sigma_results(2))
plot(sigma_results(4))

xlabel('time[s]')
legend('theta=-0.5','theta=0.5')
set(gcf,'Renderer', 'painters', 'Position', [10 10 1100 500]);
print(gcf,'diffrent_thetas.png','-dpng','-r300');

%% Visualisation normal
%theta = fixed value 0 simulation time 25
 close all;
subplot(121)
hold on
plot(y)
plot(y_r)
xlabel('time[s]')
ylabel('Amplitude')
legend('Plant output y','Reference signal y_r')

subplot(122)
hold on
plot(u)
xlabel('time[s]')
title('')
ylabel('Amplitude')
legend('Control signal u')
set(gcf,'Renderer', 'painters', 'Position', [10 10 1100 500]);
print(gcf,'normal_plot.png','-dpng','-r300');

%% Visualtisation variable theta
%Manual switch to theta, simulation time 500
 close all;
subplot(121)
hold on
plot(y)
plot(y_r)
ylabel('Amplitude')
xlabel('time[s]')
legend('Plant output y','Reference signal y_r')

subplot(122)
hold on
plot(sigma)
plot(theta)
title('')
ylabel('')
xlabel('time[s]')
legend('sigma','theta')
set(gcf,'Renderer', 'painters', 'Position', [10 10 1100 500]);
print(gcf,'Variable_theta_noise.png','-dpng','-r300');


%% Functions

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