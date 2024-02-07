function [time,q,dq,ddq,dddq,ddddq] = trapezoidal_given_tc_ti_is_zero(ti,tc,tf,st,qi,qf)
% assumptions
% ta = td = tc
% ti != 0
delta = ti;
tf = tf-ti;
ti = 0;

%check on tc value
if (tc > tf/2) || (tc < 0)
    error('The cut speed should be less than half the final time');
end

% 
% syms qi qf dqc tc tf t
% acc_period_q = qi + (dqc /(2*tc)) * t^2
% acc_period_dq = (dqc*t)/tc
% acc_period_ddq = (dqc)/tc
% stationary_period_q = qi + dqc * (t - (tc/2))
% stationary_period_dq = dqc
% stationary_period_ddq = 0
% dec_period_q = qf - (dqc/(2*tc)) * ((tf-t)^2)
% dec_period_dq = -(dqc*(2*t - 2*tf))/(2*tc)
% dec_period_ddq = -dqc/tc

dQ = qf-qi;
alfa = tc/tf;
time = 0:st:tf;
dqc = dQ/ ((1-alfa)*tf);
ddqc = dQ/(alfa * (1-alfa) * (tf.^2));
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
time = time + delta;
end