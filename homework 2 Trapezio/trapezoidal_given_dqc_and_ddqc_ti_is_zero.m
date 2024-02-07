function[time,q,dq,ddq,dddq,ddddq] = trapezoidal_given_dqc_and_ddqc_ti_is_zero(ti,st,qi,qf,dqc,ddqc)
    
    shift = ti;
    ti = 0;
    segno = sign(qf-qi);
    qf = segno*qf;
    qi = segno*qi;
    dQ = qf-qi;
    tc = dqc / ddqc;
    tf = (dqc^2 + ddqc*(dQ))/(dqc*ddqc);
    if (tc > tf/2) || (tc < 0)
    error('The cut speed should be less than half the final time');
    end

    
%     syms qi qf ddqc tc tf t
%     acc_period_q = qi + (1/2)*ddqc*t^2
%     acc_period_dq = ddqc*t
%     acc_period_ddq = ddqc
%     stationary_period_q = qi + (ddqc*tc*(t-(tc/2)))
%     stationary_period_dq = ddqc*tc
%     stationary_period_ddq = 0
%     dec_period_q = qf - (1/2)*(ddqc)*(tf-t)^2
%     dec_period_dq = -(ddqc*(2*t - 2*tf))/2
%     dec_period_ddq = -ddqc
%     


    time = 0:st:tf;
    end_acc = (find(time>tc,1)-1);
    time_acc = time(1 : end_acc);
    end_vel= find(time>(tf-tc),1)-1;
    time_vel =  time(end_acc+1: end_vel);
    time_dec = time(end_vel+1 : end);
    
    q = [   qi + (1/2)*ddqc*time_acc.^2 ...
             qi + (ddqc*tc*(time_vel-(tc/2)))...
            qf - (1/2)*(ddqc)*(tf-time_dec).^2
    ];
    
    dq = [   ddqc*time_acc ...
            ddqc*tc*ones(size(time_vel))...
           -(ddqc*(2*time_dec - 2*tf))/2
    ];
    
    ddq = [ ddqc*ones(size(time_acc))...
             0*ones(size(time_vel))...
           -ddqc*ones(size(time_dec))
    ];
    dddq = [];
    ddddq =[];
    q = segno*q;
    dq = segno*dq;
    ddq=segno*ddq;
    time = time+shift;
end