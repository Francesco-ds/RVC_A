function [time,q,dq,ddq,dddq] = multipoint_continuos(qk,tk,st,dqi,dqf)
    
    n = size(qk,2); 
    T = zeros(1,n-1);
    time = tk(1):st:tk(n);
    
    for k = 1:n-1
       T(k) = tk(k+1)-tk(k); 
    end

    A = zeros(n-2);
    C = zeros(1,n-2);
    dqk = zeros(1,n-2);

    for k= 1: n-2
            Dqk= qk(k+1) - qk(k);
            Dqk_next = qk(k+2)-qk(k+1);
            C(k) = 3*T(k+1) * Dqk/T(k) + 3*T(k)*Dqk_next/T(k+1);
    end


    % ho 5 punti, prova a farlo se ne hai meno
    C(1) = C(1)-T(2)*dqi;
    C(n-2) = C(n-2) - T(n-2)*dqf;


    %matrix A
    A(1,1) = 2*(T(1) + T(2));
    A(1,2) = T(1);

    for k = 2:n-3
        A(k,k-1) = T(k+1);
        A(k,k) = 2*(T(k) + T(k+1));
        A(k,k+1) = T(k);
    end

    A(end,end) = 2*(T(n-2) + T(n-1));
    A(end,end-1) = T(n-1);


    %Thomas alghorithm
    
    for k = 2:size(A,1)
        m = A(k,k-1)/A(k-1,k-1);
        A(k,k)= A(k,k)-m*A(k-1,k);
        C(k) = C(k)-m*C(k-1);
    end

    dim = size(A,1);
    dqk(dim)= C(dim)/A(dim,dim);

    for k = dim-1:-1:1
        dqk(k) = (C(k)-A(k,k+1)*dqk(k+1))/A(k,k);
    end
    dqk = dqk';
    
    dqk = [dqi;dqk;dqf];
    a=[];

    for k=1:size(qk,2)-1
    
        DTk = tk(k+1)-tk(k);

        a(k,1)=qk(k);

        a(k,2)=dqk(k);

        a(k,3)=(3*(qk(k+1)-qk(k))/DTk -2*dqk(k)-dqk(k+1))/DTk;

        a(k,4)=(2*(qk(k)-qk(k+1))/DTk +dqk(k)+dqk(k+1))/DTk^2; 
    end

     
q = [];
dq = [];
ddq = [];
dddq = [];

for k=1:n-1

       if (k==size(qk,2)-1)
        times=tk(k):st:tk(k+1);
    else
        times=tk(k):st:tk(k+1)-st;
       end

       diff = (times-tk(k));
       producer = a(k,4) * diff.^3+a(k,3)*diff.^2+a(k,2) * diff + a(k,1);

       dproducer=3*a(k,4)*diff.^2+2*a(k,3)*diff+a(k,2);
       ddproducer=6*a(k,4)*diff+2*a(k,3);
       dddproducer=6*a(k,4)+ones(1,size(times,2));

        q=[q producer];
        dq=[dq dproducer];
        ddq=[ddq ddproducer];
        dddq=[dddq dddproducer];
   


end

% a ok, A ok, C ok, qk