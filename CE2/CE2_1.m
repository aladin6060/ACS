clc;close all;
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
 
% nyquist(G3,'sd',2)
% legend('$\widetilde{G}3$','Interpreter','latex');
% title('');
% set(gcf,'Renderer', 'painters', 'Position', [10 10 900 600]);
% print(gcf,'nyquist_g3.png','-dpng','-r300')


Gf = stack(1,G1f,G2f,G3f);
Gn = (G1f+G2f+G3f)/3; % gits sicher no bessere methode

[Gu, Info] = ucover(Gf,Gn,4,'InputMult');
  
% bodemag(Info.W1,(1-Gf/Gn),{1,200})
% title('');
% legend("4th order W{2}","$1-\frac{\widetilde{G}}{G{nom}}$",'Interpreter','latex','Location','northwest')
% set(gcf,'Renderer', 'painters', 'Position', [10 10 900 600]);
% print(gcf,'bode_w2.png','-dpng','-r300')

%What is the best coice for the nomianl model???? Tscheggi nid

%test2

