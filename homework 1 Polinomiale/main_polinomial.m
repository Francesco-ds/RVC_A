clear all
clc

%positions
qi=2;
qf=14;

%velocities
dqi=2;
dqf=5;

%accelerations
ddqi=0;
ddqf=0;

%jerk
dddqi=0;
dddqf=2;

%times
ti=5;
tf=10;
st = 0.01;
%% Cubic delta
[time,q,dq,ddq,dddq,~] = polynomial_cubic_delta(st,ti,tf,qi,qf,dqi,dqf);
title_graph = sprintf('Cubic delta with \n ti=%.0f,tf=%.0f,qi=%.0f,dqi=%.0f,qf=%.0f,dqf=%.0f',ti,tf,qi,dqi,qf,dqf);
plot_trajectories(3,title_graph,time,q,dq,ddq);

%% Cubic ti tf
[time,q,dq,ddq,dddq,~] = polynomial_cubic_ti_tf(st,ti,tf,qi,qf,dqi,dqf);
title_graph = sprintf('Cubic ti tf with \n ti=%.0f,tf=%.0f,qi=%.0f,dqi=%.0f,qf=%.0f,dqf=%.0f',ti,tf,qi,dqi,qf,dqf)
plot_trajectories(3,title_graph,time,q,dq,ddq);

%% 5th delta
[time,q,dq,ddq,dddq,ddddq]=trajectory_5th_deltaT(st,ti,tf,qi,qf,dqi,dqf,ddqi,ddqf);
title_graph = sprintf('5th delta with \n ti=%.0f,tf=%.0f,qi=%.0f,dqi=%.0f,ddqi=%.0f,qf=%.0f,dqf=%.0f,ddqf=%.0f',ti,tf,qi,dqi,ddqi,qf,dqf,ddqf)
plot_trajectories(5,title_graph,time,q,dq,ddq,dddq,ddddq);

%% 5th ti tf
[time,q,dq,ddq,dddq,ddddq]=trajectory_5th_ti_tf(st,ti,tf,qi,qf,dqi,dqf,ddqi,ddqf);
title_graph = sprintf('5th ti tf with \n ti=%.0f,tf=%.0f,qi=%.0f,dqi=%.0f,ddqi=%.0f,qf=%.0f,dqf=%.0f,ddqf=%.0f',ti,tf,qi,dqi,ddqi,qf,dqf,ddqf)
plot_trajectories(5,title_graph,time,q,dq,ddq,dddq,ddddq);

%% 7th delta
[time,q,dq,ddq,dddq,ddddq]=trajectory_7th_deltaT(st,ti,tf,qi,qf,dqi,dqf,ddqi,ddqf,dddqi,dddqf);
title_graph = sprintf('7th delta with \n ti=%.0f,tf=%.0f,qi=%.0f,dqi=%.0f,ddqi=%.0f,dddqi=%.0f,qf=%.0f,dqf=%.0f,ddqf=%.0f,dddqf=%.0f',ti,tf,qi,dqi,ddqi,dddqi,qf,dqf,ddqf,dddqf)
plot_trajectories(5,title_graph,time,q,dq,ddq,dddq,ddddq);

%% 7th ti tf
[time,q,dq,ddq,dddq,ddddq]=trajectory_7th_ti_tf(st,ti,tf,qi,qf,dqi,dqf,ddqi,ddqf,dddqi,dddqf);
title_graph = sprintf('7th ti tf with \n ti=%.0f,tf=%.0f,qi=%.0f,dqi=%.0f,ddqi=%.0f,dddqi=%.0f,qf=%.0f,dqf=%.0f,ddqf=%.0f,dddqf=%.0f',ti,tf,qi,dqi,ddqi,dddqi,qf,dqf,ddqf,dddqf)
plot_trajectories(5,title_graph,time,q,dq,ddq,dddq,ddddq);

