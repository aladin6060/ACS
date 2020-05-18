function [R,S,T,fval] = QParametrization(Q0,A,B,R0,S0,Hr,Hs,Ts,P,W1,W2,W3)
%QParametrization Calculates RST controller with Q parametrization and
%fixed terms Hr and Hs, R0 and S0 have to be padded with zeros according to
%the order of Q
    function [c,ceq] = constraints(Q)
       
        Rqparam=R0+ conv(conv(conv(A,Hr),Hs),Q);%script equation (3.92) 
        U=tf(conv(A,Rqparam),P,Ts,'variable','z^-1');
        
        c=[norm(W3*U,inf)-1];
        ceq=[];
    end

    function obj = objective(Q)
        Rqparam=R0+ conv(conv(conv(A,Hr),Hs),Q);%script equation (3.92)  
        
        Sqparam=S0-conv(conv(conv(B,Hr),Hs),Q);%script equation (3.93)
        
        Tqparam=sum(Rqparam);
        
        Sensitivity=tf(conv(Sqparam,A),P,Ts,'variable','z^-1');
        Tsens=1-Sensitivity;
        
        vect=[W1*Sensitivity;W2*Tsens];
        obj=norm(vect,inf);
    end

    [Q,fval,Exitflag]=fmincon(@objective,Q0,[],[],[],[],[],[],@constraints);
    
    R=R0+ conv(conv(conv(A,Hr),Hs),Q);%script equation (3.92)
    S=S0-conv(conv(conv(B,Hr),Hs),Q);%script equation (3.93)
    T=sum(R);
end

