function [time_fin,q_fin,dq_fin,ddq_fin] = trapezoidal_multipoint(qk,st,tk,ddqcmax,dqcmax)
%final values
time_fin = [];
q_fin = [];
dq_fin = [];
ddq_fin = [];

%previous velocity and time initialization
prev_dqf = 0;
prev_tf = tk(1); % set the previous final as the initial time
for i = 1:size(qk,2)-1
    
    %set initial and final point
    qi = qk(i);
    qf = qk(i+1);
    %continuous velocity
    dqi = prev_dqf;
    dqf = 0;
    if i < size(qk,2) - 1
        sign_qk = sign(qf-qi);
        sign_qk1 = sign(qk(i+2)-qf);

        if sign_qk == sign_qk1
           dqf = sign_qk * dqcmax; % put 0 to have less errors
        end
    end

    prev_dqf = dqf;


    ti = prev_tf;
    [time,q,dq,ddq,dddq,ddddq,tf] = trapezoidal_ddqcmax_dqcmax_with_tf(qi,qf,st,ti,dqi,dqf,dqcmax,ddqcmax);
    prev_tf = tf;
    
    time_fin = [time_fin,time];
    q_fin = [q_fin,q];
    dq_fin = [dq_fin,dq];
    ddq_fin = [ddq_fin,ddq];

end