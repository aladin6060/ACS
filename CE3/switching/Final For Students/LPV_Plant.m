function [sys,x0,str,ts]=LPV_Plant(t,x,u,flag,A1,B1,A2,B2,A3,B3,Ts)
% This S-Function computes the output signal of the Gyroscope system at 
% different operating point defined by a scheduling parameter theta. 
% The input vector u includes the input signal of the plant model and the
% scheduling parameter theta.
% u-> u(1) = input signal u
%     u(2) = theta
% The output of the block is the output signal. The vector x includes the 
% state of the system (passed inputs and outputs).


n = length(B1); 
 
switch flag
    % Initialization
    case 0
        sizes = simsizes;
        sizes.NumContStates  = 0;
        sizes.NumDiscStates  = 2*n;
        sizes.NumOutputs     = 1;
        sizes.NumInputs      = 2;
        sizes.DirFeedthrough = 0;
        sizes.NumSampleTimes = 1;

        sys = simsizes(sizes);

        x0  = zeros(sizes.NumDiscStates,1);
        str = [];
        ts  = [Ts 0];
        
%x =    [1,n]past n system outputs
%       [n+1,end]past n control signals
       
        
     % state update 
    case 2
        u_k=u(1)       
        theta=u(2) 
        if u(2) < -1, theta=-1;end
        if u(2) > 1, theta=1;end
        
        if theta <= 0
            B=B2+theta*(B2-B1);
            A=A2+theta*(A2-A1);
        else
            B=B2-theta*(B2-B3);
            A=A2-theta*(A2-A3);
        end

        y_k = B*[u_k;x(1:n-1)] - A(2:end)*x(n+1:end-1); %claculating output of plant 
        
        x = circshift(x,1); % push all states 1 sample back
        x([1,n+1]) = [y_k,u(1)];                        %updtate the state of output, input
        sys = x;                                        %write update

        
               
    
    case 3          
        sys=x(1)            %write output
        
    case 9
        sys=[];
 end
    