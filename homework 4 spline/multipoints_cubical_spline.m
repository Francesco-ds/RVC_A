function [time,q,dq,ddq,dddq] = multipoints_cubical_spline(qk,tk,st,dqi,dqf)
    
    num_waypoints = size(qk,2);
    %time = tk(1):st:tk(num_waypoints);
    DT = zeros(1,num_waypoints);

    for i = 1: num_waypoints-1
        DT(i) = tk(i+1) - tk(i);      
    end

    A = zeros(num_waypoints);
    C = zeros(1,num_waypoints);
    ddqk = zeros(1,num_waypoints);

    C(1) = 6*((qk(2)-qk(1))/DT(1) -dqi);
    C(num_waypoints) = 6*(dqf-(qk(num_waypoints)-qk(num_waypoints-1))/DT(num_waypoints-1));

    A(1,1) = 2*DT(1);
    A(1,2) = DT(1);

    for i= 2:num_waypoints-1
        A(i,i-1) = DT(i-1);
        A(i,i) = 2*DT(i-1)+2*DT(i);
        A(i,i+1) = DT(i);
        C(i)= 6*((qk(i+1)-qk(i))/DT(i) - (qk(i)-qk(i-1))/DT(i-1));
    end
    A(num_waypoints,num_waypoints-1) = DT(num_waypoints-1);
    A(num_waypoints,num_waypoints) = 2*DT(num_waypoints-1);

    dim = size(A,1);

    %Thomas algorithm
    for i = 2:dim
    
        m = A(i,i-1)/A(i-1,i-1);
        A(i,i) = A(i,i) - m* A(i-1,i);
        C(i) = C(i) - m * C(i-1);
    end

    ddqk(dim) = C(dim)/A(dim,dim);

    for i = dim-1:-1:1
        ddqk(i) = (C(i)-A(i,i+1) * ddqk(i+1))/A(i,i);
    end

    a = [];

    for i = 1:num_waypoints-1
    
        DQ = qk(i+1)-qk(i);
        a(i,1) = qk(i);
        a(i,2)= DQ/DT(i) - DT(i) * (ddqk(i+1)+2*ddqk(i))/6;
        a(i,3)= ddqk(i)/2;
        a(i,4) = (ddqk(i+1) - ddqk(i))/(6*DT(i));
    
    end

    time = [];
    q = [];
    dq=[];
    ddq=[];
    dddq=[];

    for i = 1:num_waypoints-1
        if i == num_waypoints-1
            times = tk(i):st:tk(i+1);
        else
            times = tk(i):st:tk(i+1)-st;
        end
        producer = a(i,4)*(times-tk(i)).^3+a(i,3)*(times-tk(i)).^2+a(i,2)*(times-tk(i))+a(i,1);
        dproducer = 3*a(i,4)*(times-tk(i)).^2+2*a(i,3)*(times-tk(i))+a(i,2);
        ddproducer = 6*a(i,4)*(times-tk(i))+2*a(i,3);
        dddproducer = 6*a(i,4)+ones(1,size(times,2));

        time = [time times];
        q = [q producer];
        dq = [dq dproducer];
        ddq = [ddq ddproducer];
        dddq = [dddq dddproducer];
    end


end