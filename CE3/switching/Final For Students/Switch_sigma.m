function [sys,x0,str,ts]=Switch_sigma(t,x,u,flag,alpha,lambda,DT,Ts)
% This S-Function computes the switching signal for switching adaptive control. 
% The input vector u has m inputs that are the estimation errors from a 
% multi-estimator. The parameters are the weighting factor, forgetting factor
% and Dwell-Time. (beta is assumed to be equal to one)


m=3;
x_p=zeros(m,1);

switch flag,
    % Initialization
    case 0,
        sizes = simsizes;
        sizes.NumContStates  = 0;
        sizes.NumDiscStates  = m+2;
        sizes.NumOutputs     = 1;
        sizes.NumInputs      = m;
        sizes.DirFeedthrough = 1;
        sizes.NumSampleTimes = 1;

        sys = simsizes(sizes);

        x0  = [zeros(m,1);1;DT];
        str = [];
        ts  = [Ts 0]; 
        
     % state update 
    case 2
    
%hint: Based on u(i) compute the monintoring signal Ji in a recursive
%      way for all inputs (prediction errors)
        
        for i=1:m
            J(i)= alpha*u(i)^2+  
            
            
        end
        
        [V,x(m+1)] = min(J) 
        
       
%hint: Write an algorithm for the Dwell-Time
%      DT shows the number of sampling period of waiting between two
%      switchings and is saved in x(m+2).
%      x(m+1) is the choice of the best predictor.
        
        
        sys=[Ji;x(m+1);x(m+2)];
        
     % output update
    case 3
    sys=x(m+1); 
    case 9
        sys=[];
 end
    
        
        