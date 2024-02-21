function [] = plot_trajectories_comparison(derivative_order,title_graph,time,q,dq,ddq,dddq,qk,tk,q_n,dq_n,ddq_n,dddq_n,time_n)

rows = derivative_order;
figure(3)

if exist('q', 'var')    
    subplot(rows,1,1), plot(time, q), title(title_graph),ylabel({'Position';'[m]'});
    hold on
    plot(time_n,q_n,'r--')
    plot(tk,qk,'r*')
end
if exist('dq', 'var')
    subplot(rows,1,2), plot(time, dq), ylabel({'Speed';'[m/s]'});
    hold on
    plot(time_n,dq_n,'r--')
end
if exist('ddq', 'var')
   subplot(rows,1,3), plot(time, ddq), ylabel({'Acceleration';'[m/s^2]'});
   hold on
    plot(time_n,ddq_n,'r--')
end
if exist('dddq', 'var')
    subplot(rows,1,4), plot(time, dddq), ylabel({'Jerk';'[m/s^3]'});
    hold on
    plot(time_n,dddq_n,'r--')

if exist('ddddq', 'var')
    subplot(rows,1,5), plot(time, ddddq), ylabel({'Snap';'[m/s^4]'});
end

xlabel('Time [s]');


end