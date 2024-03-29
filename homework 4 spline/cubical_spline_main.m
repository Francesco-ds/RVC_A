%%
clc
clear 

qk=[5 20 -25 6 -5 10];
tk=[0 1 4 6 7.5 10];
st = 0.01;
dqi = 0;
dqf = 0;
title_graph = sprintf('Cubical spline with qk = %.f, %.f, %.f, %.f, %.f, %.f and tk = %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, st = %.2f, dqi = %.2f, dqf = %.2f', qk(1), qk(2), qk(3), qk(4), qk(5), qk(6), tk(1), tk(2), tk(3), tk(4), tk(5), tk(6), st, dqi, dqf);

[time,q,dq,ddq,dddq] = multipoints_cubical_spline(qk,tk,st,dqi,dqf);

plot_trajectories(4,title_graph,time,q,dq,ddq,dddq,qk,tk);

%%
clc
clear 

qk=[5 20 -15 8 -16 17];
tk=[0 1 4 6 7.5 10];
st=0.01;
mu = 0.7;
w =[1 1 1 1 1 1]; % inv W
%w = [0 0 0 0 0 0];

[time,q,dq,ddq,dddq] = smoothened(qk,tk,st,w,mu);
[time_n,q_n,dq_n,ddq_n,dddq_n] = multipoints_cubical_spline(qk,tk,st,0,0);
title_graph = sprintf('Smooth with qk = %.f, %.f, %.f, %.f, %.f, %.f, tk = %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, st = %.2f, mu = %.2f', qk(1), qk(2), qk(3), qk(4), qk(5), qk(6), tk(1), tk(2), tk(3), tk(4), tk(5), tk(6), st, mu);
plot_trajectories_comparison(4,title_graph,time,q,dq,ddq,dddq,qk,tk,q_n,dq_n,ddq_n,dddq_n,time_n);