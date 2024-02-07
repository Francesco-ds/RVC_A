clc;
clear;

tk = [0 4 8 12];
qk = [0 20 0 20];

st = 0.001;

dqi = -3;
dqf = 3;

[time,q,dq,ddq,dddq] = multipoint_velocity(qk,tk,st,dqi,dqf);
title_graph = sprintf('Multipoints continuous velocity with qk = %.f, %.f, %.f, %.f and tk = %.f, %.f, %.f, %.f dqi=%.0f,dqf=%.0f' , qk(1), qk(2), qk(3), qk(4), tk(1), tk(2), tk(3), tk(4),dqi,dqf)
plot_trajectories(3,title_graph,time,q,dq,ddq);


%% acc
clc
clear
qk=[0 20 0 10 20];
tk=[0 4 8 10 12];
st = 0.01;

dqi = 3;
dqf = 5;
%[time,q,dq,ddq,dddq] = multipoints_acc_test(qk,tk,st,dqi,dqf);
[time,q,dq,ddq,dddq] = multipoint_continuos(qk,tk,st,dqi,dqf);
title_graph = sprintf('Multipoints continuous acceleration with qk = %.f, %.f, %.f, %.f and tk = %.f, %.f, %.f, %.f dqi=%.0f,dqf=%.0f' , qk(1), qk(2), qk(3), qk(4), tk(1), tk(2), tk(3), tk(4),dqi,dqf);
plot_trajectories(3,title_graph,time,q,dq,ddq);