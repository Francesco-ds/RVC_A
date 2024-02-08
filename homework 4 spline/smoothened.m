function [time,q,dq,ddq,dddq] = smoothened(qk,tk,st,wk,mu)

    num_waypoints = size(qk,2);

    time = tk(1):st:tk(num_waypoints);

    W = diag(wk);

    DT = [];

    for i = 1:num_waypoints-1
    
        DT(i)= tk(i+1)-tk(i);

    end

    lambda = (1-mu)/(6*mu);

    A= zeros(num_waypoints);
    A(1,1) = 2*DT(1);
    A(1,2)=DT(1);
    C= zeros(num_waypoints);
    C(1,1) = -6/DT(1);
    C(1,2) = 6/DT(1);

    for i = 2:num_waypoints-1
    
        A(i,i-1) = DT(i-1);
        A(i,i) = 2*DT(i-1)+2*DT(i);
        A(i,i+1) = DT(i);

        C(i,i-1) = 6/DT(i-1);
        C(i,i) = -(6/DT(i-1)+6/DT(i));
        C(i,i+1) = 6/DT(i);
    end

    A(num_waypoints,num_waypoints-1) = DT(num_waypoints-1);
    A(num_waypoints,num_waypoints) = 2*DT(num_waypoints-1);
    C(num_waypoints,num_waypoints-1) = 6/DT(num_waypoints-1);
    C(num_waypoints,num_waypoints)=-6/DT(num_waypoints-1);

    dds=(A+lambda.*C*inv(W)*C')\C*qk';
    %dds = C*qk * inv(A+lambda.*C*inv(W)*C')
    s = qk'-lambda.*inv(W)*C'*dds;
    s=s';

    [time,q,dq,ddq,dddq] = multipoints_cubical_spline(s,tk,st,0,0);
end