function [R,S] = poleplace(B,A,H_R,H_S,P)
%POLEPLACE computing the coefficients of polynomial R and S that places the
%closed-loop poles at P
%   Fixed parts Hr and Hs are taken into account (script p. 119)
%   Solves equation A*S'*H_S+q^-d*B*R'*H_R = P assuming delay d=0


    Aprime=conv(A,H_S);
    n_Aprime=length(Aprime);

    Bprime=conv(B,H_R);
    n_Bprime=length(Bprime);

    n_HS=length(H_S);
    n_HR=length(H_R);

    n_Rprime=n_Aprime-1;
    n_Sprime=n_Bprime-1;


    Msize=n_Aprime+n_Bprime;
    M=zeros(Msize);

    for i = 1:n_Bprime
        M(i:i+n_Aprime-1,i)=Aprime;
    end
    for i = (n_Bprime+1):Msize
        M(i-n_Bprime+1:i,i)=Bprime;
    end
    p=zeros(Msize,1);
    p(1:length(P))=P;
    x = inv(M)*p
    
    Sprime=x(1:n_Sprime+1);
    Rprime=x(n_Sprime+2:end);
    
    R=conv(Rprime,H_R)
    S=conv(Sprime,H_S)

end

