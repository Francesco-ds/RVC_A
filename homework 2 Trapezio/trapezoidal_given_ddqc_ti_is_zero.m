function [time,q,dq,ddq,dddq,ddddq] = trapezoidal_given_ddqc_ti_is_zero(ti,tf,st,qi,qf,ddqc)
    tf = tf-ti;
    shift = ti;
    ti = 0;
    segno = sign(qf-qi);
    qf = segno*qf;
    qi = segno*qi;
    dQ = qf-qi;
    delta = ((tf^2)*ddqc - 4*dQ);
    if delta < 0
        error('Trajectory is not feasible');
    end
    tc = tf/2 - sqrt(tf^2-4*dQ/ddqc)/2;
    if (tc > tf/2) || (tc < 0)
    error('The cut speed should be less than half the final time');
    end
    dqc = ddqc * tc;
    time = 0:st:tf;
    end_acc = (find(time>tc,1)-1);
    time_acc = time(1 : end_acc);
    end_vel= find(time>(tf-tc),1)-1;
    time_vel =  time(end_acc+1: end_vel);
    time_dec = time(end_vel+1 : end);
    
    q = [   qi + (dqc /(2*tc)) * time_acc.^2 ...
            qi + dqc * (time_vel - (tc/2)) ...
            qf - (dqc/(2*tc)) * ((tf-time_dec).^2)
    ];
    
    dq = [  (dqc*time_acc)/tc ...
            dqc*ones(size(time_vel)) ...
            -(dqc*(2*time_dec - 2*tf))/(2*tc)
    ];
    
    ddq = [ dqc/tc * ones(size(time_acc))...
            0*ones(size(time_vel)) ...
            -dqc/tc* ones(size(time_dec))
    ];
    dddq = [];
    ddddq =[];
    q = segno*q;
    dq = segno*dq;
    ddq=segno*ddq;
    time = time+shift;
end