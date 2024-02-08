function [time,q,dq,ddq,dddq,ddddq] = trapezoidal_lot_of_params(qi,qf,ta,td,ti,tf,dqi,dqf,st,dqc)

%     syms qi qf ta td ti tf ddqf ddqi dqi dqf st dqc t
%     acc_period_q = qi + dqi*(t-ti) + ((dqc-dqi)/(2*ta)) * ((t-ti)^2)
%     acc_period_dq = dqi + ((2*t - 2*ti)*(dqc - dqi))/(2*ta)
%     acc_period_ddq = (dqc - dqi)/ta
%     stationary_period_q = qi + dqi*ta / 2 + dqc*(t-ti-ta/2)
%     stationary_period_dq = dqc
%     stationary_period_ddq = 0
%     dec_period_q = qf-dqf*(tf-t) - (dqc-dqf)*((tf-t)^2) / (2*td)
%     dec_period_dq = dqf - ((2*t - 2*tf)*(dqc - dqf))/(2*td)
%     dec_period_ddq = -(dqc - dqf)/td
%     acc period goes from ti to ti+ta
%     stationary period goes from ti+ta to tf-td

if ta <= 0 
    error('acceleration time should be a positive number')
end
if td <= 0 
    error('deceleration time should be a positive number')
end

if ta+td > tf-ti
    error('Total time should be greater than the acceleration and deceleration phases')
end

time = ti:st:tf;
end_acc = find(time>ti+ta,1)-1;
start_dec = find(time>(tf-td),1)-1;
time_acc = time(1:end_acc);
% if time_acc(2) ~= time(2)
%     time_acc = [time(2), time_acc]
% if time_acc(1) ~= ti
%     time_acc = [ti, time_acc]
% end

time_vel = time(end_acc+1:start_dec);
time_dec = time(start_dec+1:end);
qDDc = (dqc - dqi)/ta;

q = [ qi + dqi*(time_acc-ti) + ((dqc-dqi)/(2*ta)) * ((time_acc-ti).^2)...
      qi + dqi*(ta/2) + dqc*(time_vel-ti-(ta/2))...
     qf - (dqf) * (tf-time_dec) - ((dqc-dqf)/(2*td)) * ((tf-time_dec).^2)];

dq = [dqi + ((2*time_acc - 2*ti)*(dqc - dqi))/(2*ta)...
      dqc*ones(size(time_vel))...
       dqf - ((2*time_dec - 2*tf)*(dqc - dqf))/(2*td)];
ddq = [(dqc - dqi)/ta * ones(size(time_acc))...
       0 * ones(size(time_vel))...
       -(dqc - dqf)/td * ones(size(time_dec))];
dddq = [];
ddddq = [];

end


