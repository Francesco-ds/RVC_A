function [time,q,dq,ddq,dddq]=smoothing_traj(qk,tk,Ts,w_vector,mu)

n=size(tk,2);

if (n~=size(qk,2))
    error("Positions and times vectors must have the same lenght");
end
if (n~=size(w_vector,2))
    error("Weight vector must have the same lenght of the positions and times vectors");
end

t=tk(1):Ts:tk(n);


W_inv=diag(w_vector);

%function

T=zeros(1,n-1);

for k=1:n-1
    if tk(k)>tk(k+1)
        error("Time at position k+1 must be always greater than time at position k")
    end
    T(k)=tk(k+1)-tk(k);
end

lamba=(1-mu)/(6*mu);

sA=zeros(n);
sC=zeros(n);

sA(1,1)=2*T(1);
sA(1,2)=T(1);
sA(n,n-1)=T(n-1);
sA(n,n)=2*T(n-1);

sC(1,1)=-6/T(1);
sC(1,2)=6/T(1);
sC(n,n-1)=6/T(n-1);
sC(n,n)=-6/T(n-1);

for k=2:n-1
    sA(k,k-1)=T(k-1);
    sA(k,k)=2*T(k-1)+2*T(k);
    sA(k,k+1)=T(k);
    
    sC(k,k-1)=6/T(k-1);
    sC(k,k)=-(6/T(k-1)+6/T(k));
    sC(k,k+1)=6/T(k);
end

%computation of the best sk

ddsk=(sA+lamba.*sC*W_inv*sC')\sC*qk';

sk=qk'-lamba.*W_inv*sC'*ddsk;
sk=sk';

[time,q,dq,ddq,dddq]=multipoints_cubical_spline(sk,tk,Ts,0,0);

end