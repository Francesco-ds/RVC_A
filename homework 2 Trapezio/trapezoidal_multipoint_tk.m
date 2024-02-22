function [time_fin,q_fin,dq_fin,ddq_fin] = trapezoidal_multipoint_tk(qk,st,tk,ddqcmax,dqcmax)
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
    DT = tk(i+1) - ti;
    [time,q,dq,ddq,~,~] = trapezoidal_ddqcmax_DT(qi,qf,st,ti,DT,dqi,dqf,ddqcmax);
    prev_tf = ti + DT;
    
    time_fin = [time_fin,time];
    q_fin = [q_fin,q];
    dq_fin = [dq_fin,dq];
    ddq_fin = [ddq_fin,ddq];

end