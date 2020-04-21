%% 2.4 Multiplicative uncertainity

clear variables;
close all;

load Gyro300
Z=iddata(y,u,Ts);
Zd=detrend(Z);
G1 = oe(Zd,[6 6 1]);
G1f=spa(Zd,100);

load Gyro400
Z=iddata(y,u,Ts);
Zd=detrend(Z);
G2 = oe(Zd,[6 6 1]);
G2f=spa(Zd,100);

load Gyro500
Z=iddata(y,u,Ts);
Zd=detrend(Z);
G3 = oe(Zd,[6 6 1]);
G3f=spa(Zd,100);

clear Z
clear Zd

figure(1)
nyquist(G3,'sd',2)
title('Nyquist diagram of the parametric ID (OE)')
axis([-1 1.5 -0.5 0.5])
axis equal
% Elyptic uncertainity because of parametric uncertainity

figure(2)
nyquist(G3f,'sd',2)
title('Nyquist diagram of the non-parametric ID (SPA)')
axis([-1 1.5 -0.5 0.5])


figure(3) % Plot the frequency response of the non-parametric ( spa ) ID
bodemag(G1f,G2f,G3f)
legend('G1f','G2f','G3f')
title('Bode diagram of the non-parametric ID (SPA)')
axis([10^-2 10^3 -60 10])

figure(4) % Plot the frequency response of the parametric (OE) ID
bodemag(G1,G2,G3)
legend('G1','G2','G3')
axis([10^-2 10^3 -60 10])
title('Bode diagram of the parametric ID (OE)')

%%
%Create a model array based on all identified models

G_tilde=stack(1,G1,G2,G3,G1f,G2f,G3f);
MaxModelOrder=15; %fix the model order



W2_2norms=[];
W2_infnorms=[];
calctime=[];

for o=1:MaxModelOrder
    
    tic

    [sys,info]=ucover(G_tilde,G1,o);
    W2_2norms(1,o)=norm(info.W1);
    W2_infnorms(1,o)=norm(info.W1,Inf);
    
    [sys,info]=ucover(G_tilde,G2,o);
    W2_2norms(2,o)=norm(info.W1);
    W2_infnorms(2,o)=norm(info.W1,Inf);
    
    [sys,info]=ucover(G_tilde,G3,o);
    W2_2norms(3,o)=norm(info.W1);
    W2_infnorms(3,o)=norm(info.W1,Inf);
    
    [sys,info]=ucover(G_tilde,G1f,o);
    W2_2norms(4,o)=norm(info.W1);
    W2_infnorms(4,o)=norm(info.W1,Inf);
    
    [sys,info]=ucover(G_tilde,G2f,o);
    W2_2norms(5,o)=norm(info.W1);
    W2_infnorms(5,o)=norm(info.W1,Inf);
    
    [sys,info]=ucover(G_tilde,G3f,o);
    W2_2norms(6,o)=norm(info.W1);
    W2_infnorms(6,o)=norm(info.W1,Inf);
    
    %the time is calculated in order to compare the order vs computational
    %cost
    calctime=[calctime,toc];
end

modellabels= {'G1_OE','G2_OE','G3_OE','G1_SPA','G2_SPA','G3_SPA'};
modelorders=1:MaxModelOrder;
%heatmap(W2_2norms)
heatmap(modelorders,modellabels,W2_2norms)
%hold on
title('2-Norm of W2-Filters')
xlabel='Order';
ylabel='Model';
%%
W2_infnormsplot=W2_infnorms;
W2_infnormsplot(W2_infnormsplot>1)=[1];

%%
figure;
heatinf=heatmap(modelorders,modellabels,W2_infnormsplot)

%hold on
heatinf.Colormap= hot;
heatinf.ColorScaling= 'log';
title('Inf-Norm of W2-Filters')
heatinf.Xlabel='Order';
heatinf.Ylabel='Model';


%% Choice of best model
%The lower the 2-norm the better will the controller perform over the whole
%frequency range (less conservative)

%The lower the inf-norm, the better will the controller perform on its peak
%value

min2N=min(min(W2_2norms));
[min2NModel,min2NOrder]=find(W2_2norms==min2N)
minInfN=min(min(W2_infnorms));
[min2NModel,minInfNOrder]=find(W2_infnorms==minInfN)

%Introduction of a punisment term for high order in order to reduce
%overfitting. Also the two performances (2Norm & InfNorm) are combined

OrderMatrix=ones(6,1)*calctime
Pfactor=0.005 %punishment factor
W2_combined=W2_2norms+W2_infnorms+OrderMatrix*Pfactor;
minComb=min(min(W2_combined));
[minNcombModel,minNcombOrder]=find(W2_combined==minComb);

minNcombModel
minNcombOrder

figure;
W2_combplot=W2_combined;
W2_combplot(W2_combplot>2)=[2];

heatcomb=heatmap(modelorders,modellabels,W2_combplot)

heatcomb.Colormap= hot;
heatcomb.ColorScaling= 'log';
heatcomb.heatmap(W2_combplot);
heatcomb.Title('Combined Score')
heatcomb.Xlabel('Order')
heatcomb.Ylabel('Model')


%% Plot W2 with 1-Gtilda/Gnom to verify correctness
W2=["G1","G2","G3","G1f","G2f","G3f"];
Gs = ["G1","G2","G3","G1f","G2f","G3f"];

G_nom=eval(Gs(minNcombModel)); %ugly but works, because if stored in matrices the idpoly data gets converted to a channel of idfrd data
[sys,Info] = ucover(G_tilde,eval(W2(minNcombModel)),minNcombOrder);
W2 = Info.W1;


figure;
bodemag(W2,'r')
hold on
bodemag(1-G_tilde/G_nom,'--b')
set(gca,'YLim',[-35 0])
title('Plot of the best W2-filter and 1-G_{tilda}/G_{nom}')

save('CE2_1','G_nom','W2','G1','G2','G3')



