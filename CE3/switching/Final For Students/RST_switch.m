function [sys,x0,str,ts]=RST_switch(t,x,u,flag,R1,S1,T1,R2,S2,T2,R3,S3,T3,Ts)
% This S-Function computes the control signal of multi RST controllers.
% There are three fixed RST controllers corresponding to the switching
% signal {1,2,3}.
% The input vector u has three input signals for the switching signal, 
% reference signal and the plant output. 
% The output of the block is the control signal. Ts is the sampling period.

% It is assumed that all controller have the same order, otherwise the code
% should be modified.

nr=length(R1)-1;
ns=length(S1)-1;
nt=length(T1)-1;

n=nr+ns+nt;

%u(1): Switching signal
%u(2): reference signal
%u(3): output signal



switch flag,
    % Initialization
    case 0,
        sizes = simsizes;
        sizes.NumContStates  = 0;
        sizes.NumDiscStates  = n;
        sizes.NumOutputs     = 1;
        sizes.NumInputs      = 3;

        sizes.DirFeedthrough = 1;
        sizes.NumSampleTimes = 1;

        sys = simsizes(sizes);

        x0  = zeros(sizes.NumDiscStates,1);
        str = [];
        ts  = [Ts 0]; 
        
     % state update 
    case 2     
        switch u(1)
            case 1
                R=R1;S=S1;T=T1;
            case 2
                R=R2  ;S=S2  ;T=T2  ;
            case 3
                R=R3  ;S=S3  ;T=T3  ;
        end

      
        u_k= 
%hint: Update the state vector (including past inputs, past outputs and past reference signals)
%why do we have to save that?             

        if nt>0  % For the case that T is a vector 
            sys= [u;u_k(1:nt)]
        else
            sys= [u;u_k;]
        end
     % output update
    case 3
     % Compute again u_k and send it out  
        sys=u_k;
  
    case 9
        sys=[];
 end
    
        
        