function [sys,x0,str,ts]=Switch_sigma(t,x,u,flag,alpha,lambda,DT,Ts)
% This S-Function computes the switching signal for switching adaptive control. 
% The input vector u has m inputs that are the estimation errors from a 
% multi-estimator. The parameters are the weighting factor, forgetting factor
% and Dwell-Time. (beta is assumed to be equal to one)

 
m = 3;

switch flag
    % Initialization
    case 0
        sizes = simsizes;
        sizes.NumContStates  = 0;
        sizes.NumDiscStates  = 2*m+2;
        sizes.NumOutputs     = 1;
        sizes.NumInputs      = m;
        sizes.DirFeedthrough = 1;
        sizes.NumSampleTimes = 1;

        sys = simsizes(sizes);

        x0  = [zeros(m,1);1;DT;zeros(m,1)];
        str = [];
        ts  = [Ts 0]; 
% u = [1-3] estimation errors 
        
%x =    [1-3]Past monitoring signals
%       [4] best modle (sigma)
%       [5] dwell time counter
%       [6-9] past estimation errors
        
        
        
     % state update 
    case 2
    
%hint: Based on u(i) compute the monintoring signal Ji in a recursive
%      way for all inputs (prediction errors)
        
        for i=1:m
            J(i)=exp(-lambda)*(x(i) +(1-alpha)*x(5+i)^2) + (1+alpha)*u(i)^2;   
                    %removing old instantaneous mesure, shifting forgetting
                    %factor, adding new therm forgetting factor and new
                    %inestantaneous measure
                   
        end
      
        if x(5)==0
           [V,x(4)] = min(J); %getting the index of the lowest J
           x(5) = DT    ;     % Resetting counter 
        else
            x(5)=x(5)-1  ;  %Reducing counter by one
        end
        
  
        sys=[J';x([4,5]);u];
        
     % output update
    case 3
    sys=x(m+1); 
    case 9
        sys=[];
 end
    
        
        