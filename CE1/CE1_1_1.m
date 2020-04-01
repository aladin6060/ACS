clc; clear all;
%-------------------------------------------------------------
%System Definition

s = tf('s');
G = (s-3)/(s^2+4*s+13);


%-------------------------------------------------------------
%2. Integration of the magnitude plot

 [mag,phase,wout] = bode(G);
 G_int = trapz(wout,mag.^2);
 G_2 = sqrt((2*G_int/(2*pi)))
 


%-------------------------------------------------------------
%3. Integration of the impulse response of the systemfun(

[y,t] = impulse(G);
G_3 = sqrt(trapz(t,y.^2))

%-------------------------------------------------------------
%4. Solving Algebric Riccati Equation

b = [1 -3];
a = [1 4 13];
[A,B,C,D] = tf2ss(b,a);
L = sylvester(A,A.',-B*B.');
G_4 = sqrt(trace(C*L*C.'))


%-------------------------------------------------------------
%5.Direct calculation with "norm"

G_5 = norm(G)

%-------------------------------------------------------------
%1. Infinity norm with frequency response

G_inf_1=max(mag)

%-------------------------------------------------------------
%2. Infinity norm with the bounded real lemma

y_upper = 1000;
y_lower = 0;
d = 1;

while d > 10^-6
 y_old = y;
 y = (y_upper+y_lower)/2;
 H = [A y^(-2).*B*B';-C'*C -A'];
 realpart = abs(real(eig(H))); 
 test = realpart < 10^-6;      
 
 if any(test) == 0             
     y_upper = y;
 else
     y_lower = y;
 end
     
 d = abs(y-y_old);
 
end
G_inf_2 = y



%-------------------------------------------------------------
%3. Infinity norm with matlab function

G_inf_3 = norm(G,Inf)

