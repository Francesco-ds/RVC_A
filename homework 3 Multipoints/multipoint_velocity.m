function [time,q,dq,ddq,dddq] = multipoint_velocity(qk,tk,st,dqi,dqf)

n = size(qk,2);
T = zeros(1,n-1);
time = tk(1):st:tk(n);

for k= 1:n-1
    T(k) = tk(k+1) - tk(k);
end

v = [];
dqk = [];

dqk(1)= dqi;
dqk(n) = dqf;

for k = 2:n
    v(k) = (qk(k)-qk(k-1))/(tk(k)-tk(k-1));
end

for k = 2: n-1
    if (sign(v(k))==sign(v(k+1)))
        dqk(k) = ( v(k) + v(k+1)) /2;
    else
        dqk(k) = 0;
    end

end

for k = 1:n-1

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

for k=1:size(qk,2)-1

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




end