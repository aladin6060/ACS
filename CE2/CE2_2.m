clc; clear variables; close all;
s = tf('s')

%% 1. Definition of W1 and importing W2 from first exercice

W1 = (s+15)*0.5/ (s+0.00001)


load('CE2_1')
G = stack(1,G1,G2,G3);

%% 2. Calculating the controller and plotting the sensitivity function and step response


W1d = c2d(W1,W2.Ts); %transform W1 to a discrete time model
K = mixsyn(G_nom,W1d,[],W2);

T = feedback(K*G,1);%closed-loop transfer function
U = feedback(K,G); %input sensitivity function
S = feedback(1,K*G); %sensitivity function

figure(1)
subplot(2,2,1)
step(T)
axis([0 1.5 0 1.2])
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
set(gcf,'Renderer', 'painters', 'Position', [10 10 1100 800]);
print(gcf,'Stepresponse.png','-dpng','-r300');

W3 = 1/5; % ensuring that control signal doesn't saturate
K = mixsyn(G_nom,W1d,W3,W2); %recalculate K with W3 filter
T = feedback(K*G,1);
U = feedback(K,G);
S = feedback(1,K*G);

%% 3. Reduction of the controller order

figure(2)
subplot(1,2,1)
pzmap(K)
subplot(1,2,2)
hsvd(K) %calculating the Hankel singular values 
set(gcf,'Renderer', 'painters', 'Position', [10 10 800 400]);
print(gcf,'PoleAnalysis.png','-dpng','-r300');

Kred = reduce(K,1); %reducing the controller to 6th order

Tred = feedback(Kred*G,1);
Ured = feedback(Kred,G);
Sred = feedback(1,Kred*G);


%% 4. Final comparison of the 14th order and 6th order controller

figure(3)
subplot(2,2,1)
step(Tred,T)
title('Step response output')
axis([0 1.5 0 1.2])

subplot(2,2,2)
step(Ured,U)
title('Step response control signal')
legend('6th order controller','14th order controller')

subplot(2,2,3)
bodemag(Ured,U,5*tf([1],[1]))
title('Input sensitivity U')

subplot(2,2,4)
bodemag(Sred,S,1/W1d)
title('Sensitivity function S')
set(gcf,'Renderer', 'painters', 'Position', [10 10 1100 800]);
print(gcf,'StepresponseSimplified.png','-dpng','-r300');



if Gamma<1/sqrt(2)
    fprintf('The controller is working at robust performance')
else 
    fprintf('The controller is not working at robust performance')
end





