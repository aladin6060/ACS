function [sys,x0,str,ts]=RST_switch(t,x,u,flag,N1,D1,N2,D2,N3,D3,Ts)
% This S-Function computes the control signal of multi RST controllers.
% There are three fixed RST controllers corresponding to the switching
% signal {1,2,3}.
% The input vector u has three input signals for the switching signal, 
% reference signal and the plant output. 
% The output of the block is the control signal. Ts is the sampling period.

% It is assumed that all controller have the same order, otherwise the code
% should be modified.

 n = length(N1);
 persistent u_2 
 persistent e

%u(1): Switching signal
%u(2): reference signal
%u(3): output signal

%x =    [1,n]past n error signals
%       [n+1,end]past n control signals



switch flag
    % Initialization
    case 0
        sizes = simsizes;
        sizes.NumContStates  = 0;
        sizes.NumDiscStates  = 2*n;
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
       
      
      x = circshift(x,1); % push all states 1 sample back
      x([1,n+1]) = [e,u_2]; % replace with correct U[k], E[k]
      sys = x; % new internal states for NEXT iteration

    case 3
      switch u(1)
            case 1
                NUM=N1 ; DEN=D1;
            case 2
                NUM=N2 ; DEN=D2;
            case 3
                NUM=N3 ; DEN=D3;
      end
      
        e = u(2)-u(3); %error
       u_2 =  NUM*[e;x(1:n-1)] - DEN(2:end)*x(n+1:end-1); %claculate controller output
       sys = u_2 %write controller output
  
    case 9
        sys=[];
 end
    
        
        