function [t,q,dq,ddq,dddq]=multipoint_acc_test(qk,tk,Ts,dqi,dqf)

n=size(qk,2);
if(n==2)
    error("Positions vector must have at least one path point");
end

if (n~=size(tk,2))
    error("Positions and times vectors must have the same lenght");
end

T=zeros(1,n-1);
t=tk(1):Ts:tk(n);

for k=1:n-1
    if tk(k)>tk(k+1)
        error("Time at position k+1 must be always greater than time at position k")
    end
    T(k)=tk(k+1)-tk(k);
end

A=zeros(n-2);
C=zeros(1,n-2);
dqk=zeros(1,n-2);

for k=1:n-2
    C(k)=3*T(k+1)*(qk(k+1)-qk(k))/T(k) + 3*T(k)*(qk(k+2)-qk(k+1))/T(k+1);
end

%the matrix A of the slides is usable only if we have at least 5 points
%including the initial and final positions, if not the following code
%represent the computation to retrieve the velocities when we have only 1
%or 2 path points

if(n==3)
    dqk(1)=(C(1)-T(2)*dqi-T(1)*dqf)/(2*(T(1)+T(2)));
elseif (n==4)
        A(1,1)=2*(T(1)+T(2));
        A(1,2)=T(1);
        A(end,end)=2*(T(n-2)+T(n-1));
        A(end,end-1)=T(n-1);
        
        C(1)=C(1)-T(1)*dqi;
        C(n-2)=C(n-2)-T(1)*dqf;
        %Thomas not appliable, not tridiagonal matrix
        dqk=A\C';
else
    C(1)=C(1)-T(2)*dqi;
    C(n-2)=C(n-2)-T(n-2)*dqf;


    %system definition

    A(1,1)=2*(T(1)+T(2));
    A(1,2)=T(1);


    A(end,end)=2*(T(n-2)+T(n-1));
    A(end,end-1)=T(n-1);

    for i=2:n-3
        A(i,i-1)=T(i+1);
        A(i,i)=2*(T(i)+T(i+1));
        A(i,i+1)=T(i);
    end

    % Thomas to compute the velocities

    for k=2:size(A,1)
        m=A(k,k-1)/A(k-1,k-1);
        A(k,k)=A(k,k)-m*A(k-1,k);
        C(k)=C(k)-m*C(k-1);
    end

    dqk(size(A,1))=C(size(A,1))/A(size(A,1),size(A,1));
    for k=size(A,1)-1:-1:1
        dqk(k)=(C(k)-A(k,k+1)*dqk(k+1))/A(k,k);
    end
    dqk=dqk';
end


dqk=[dqi;dqk;dqf];

%spline computation

a=[];

for k=1:size(qk,2)-1
    
    Tk=tk(k+1)-tk(k);
   
    %ak0
    a(k,1)=qk(k);
    %ak1
    a(k,2)=dqk(k);
    %ak2
    a(k,3)=(3*(qk(k+1)-qk(k))/Tk -2*dqk(k)-dqk(k+1))/Tk;
    %ak3
    a(k,4)=(2*(qk(k)-qk(k+1))/Tk +dqk(k)+dqk(k+1))/Tk^2; 
end

total_time=[];
q=[];
dq=[];
ddq=[];
dddq=[];


for k=1:size(qk,2)-1
    if (k==size(qk,2)-1)
        time=tk(k):Ts:tk(k+1);
    else
        time=tk(k):Ts:tk(k+1)-Ts;
    end
    PI=a(k,4)*(time-tk(k)).^3+a(k,3)*(time-tk(k)).^2+a(k,2)*(time-tk(k))+a(k,1);
    dPI=3*a(k,4)*(time-tk(k)).^2+2*a(k,3)*(time-tk(k))+a(k,2);
    ddPI=6*a(k,4)*(time-tk(k))+2*a(k,3);
    dddPI=6*a(k,4)+ones(1,size(time,2));


%     total_time=[total_time time];
    q=[q PI];
    dq=[dq dPI];
    ddq=[ddq ddPI];
    dddq=[dddq dddPI];
end

dqk=dqk';

end