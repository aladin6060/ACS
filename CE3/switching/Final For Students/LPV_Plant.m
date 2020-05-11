function [sys,x0,str,ts]=LPV_Plant(t,x,u,flag,A1,B1,A2,B2,A3,B3,Ts)
% This S-Function computes the output signal of the Gyroscope system at 
% different operating point defined by a scheduling parameter theta. 
% The input vector u includes the input signal of the plant model and the
% scheduling parameter theta. 
% The output of the block is the output signal. The vector x includes the 
% state of the system (passed inputs and outputs).


nA=length(A1)-1;
nB=length(B1)-1;

n=nA+nB;

switch flag,
    % Initialization
    case 0,
        sizes = simsizes;
        sizes.NumContStates  = 0;
        sizes.NumDiscStates  = n+1;
        sizes.NumOutputs     = 1;
        sizes.NumInputs      = 2;
        sizes.DirFeedthrough = 0;
        sizes.NumSampleTimes = 1;

        sys = simsizes(sizes);

        x0  = zeros(sizes.NumDiscStates,1);
        str = [];
        ts  = [Ts 0]; 
        
     % state update 
    case 2
        u_k=u(1);
        theta=u(2);
        
        if u(2) < -1, theta=-1;end
        if u(2) > 1, theta=1;end
        
        if theta <= 0,
            B=B2+theta*(B2-B1);
            A=
        else
            B=
            A=
        end
        
        y_k=

        sys=[-y_k;x(1:nA-1);u(1);x(nA+1:n-1);theta];
               
     % output update
    case 3  
        theta=x(end);      
        if theta <= 0
            B=B2+theta*(B2-B1);
            A=
        else
            B=
            A=
        end
        
        sys=
         
    case 9
        sys=[];
 end
    