%% reset
clc 
clear all

% code works for both qf> or < qi; simply calculate the trajectory on -qi
% -qf in case of qi < qf and then mirror over x the graphs

%code works for ti != 0, simply shifting the graph on ti = 0 to use formula
%and calculate trajectory and then adding the shifting back to time when
%returning the value

%% set manual or automatic values
clc 
clear all
manual = false;
if manual == true

    prompt ="qi ="
    qi = str2double(input(prompt, "s"))
    prompt="qf ="
    qf = str2double(input(prompt, "s"))
    prompt="dqi ="
    dqi = str2double(input(prompt, "s"))
    prompt="dqqi ="
    dqqi = input(prompt,"s")
    prompt="dqqqi ="
    dqqqi = str2double(input(prompt, "s"))
    prompt="dqf ="
    dqf = str2double(str2double(input(prompt, "s")))
    prompt="ddqf ="
    ddqf = str2double(input(prompt, "s"))
    prompt="dddqf ="
    dddqf = str2double(input(prompt, "s"))
    prompt="ti ="
    ti = str2double(input(prompt, "s"))
    prompt="tf ="
    tf = str2double(input(prompt, "s"))
    prompt="st ="
    st = str2double(input(prompt, "s"))
else
    qi = 3;
    dqi = 0;
    ddqi = 0;
    dddqi = 1000;
    qf = 70;
    dqf = 0;
    ddqf = 0;
    dddqf = 0;
    ti = 3;
    tf = 10;
    st = 0.01;   
end

%% case 1: ti = 0 given tc done
clear all
qi = 1;
qf = 8;
tf = 7;
ti = 1;
tc = 2;
st = 0.01;
[time,q,dq,ddq,~,~] = trapezoidal_given_tc_ti_is_zero(ti,tc,tf,st,qi,qf);

title_graph = sprintf('Trapezoidal with \n ti=%.0f,tc=%.0f,tf=%.0f,qi=%.0f,qf=%.0f',ti,tc,tf,qi,qf);

plot_trajectories(3,title_graph,time,q,dq,ddq);

%% case 2: ti 0 given ddqc DONE
clear all
qi = 25;
qf = 100;
ti = 1;
tf = 6;
ddqc = 12;
st = 0.01;
[time,q,dq,ddq,~,~] = trapezoidal_given_ddqc_ti_is_zero(ti,tf,st,qi,qf,ddqc);
title_graph = sprintf('Trapezoidal with \n ddqc=%.0f,ti=%.0f,tf=%.0f,qi=%.0f,qf=%.0f',ddqc,ti,tf,qi,qf);
plot_trajectories(3,title_graph,time,q,dq,ddq);

%% case 3: ti 0 given dqc DONE
clear all
qi = 5;
qf = 50;
ti = 0;
tf = 15;
dqc = 5;
st = 0.01;
[time,q,dq,ddq,~,~] = trapezoidal_given_dqc_ti_is_zero(ti,tf,st,qi,qf,dqc);
title_graph = sprintf('Trapezoidal with \n dqc=%.0f,tf=%.0f,ti=%.0f,qi=%.0f,qf=%.0f',dqc,ti,tf,qi,qf);
plot_trajectories(3,title_graph,time,q,dq,ddq);

%% case: ti 0 given ddqc and dqc
clear all
qi = 100;
qf = 25;
st = 0.01;
ti = 0;
dqc = 20;
ddqc = 35;
[time,q,dq,ddq,~,~] = trapezoidal_given_dqc_and_ddqc_ti_is_zero(ti,st,qi,qf,dqc,ddqc);
title_graph = sprintf('Trapezoidal with \n ti=%0.f,dqc=%.0f,ddqc=%.0f,qi=%.0f,qf=%.0f',ti,dqc,ddqc,qi,qf);
plot_trajectories(3,title_graph,time,q,dq,ddq);

%% case ti not zero dqi != 0 qf != 0 ddqi ddqf = 0 (qf > qi) is generic and used in the next 2 examples
clear all
qi = 25;
qf = 100;
ta = 1.5;
td = 2;
ti = 0;
tf = 10;
dqi = 3;
dqf = 7;
dqc = 15;
st = 0.01;
[time,q,dq,ddq,~,~] = trapezoidal_lot_of_params(qi,qf,ta,td,ti,tf,dqi,dqf,st,dqc);
title_graph = sprintf('Trapezoidal with \n ti=%0.f,tf=%.0f,ta=%.1f,td=%.0f,dqc=%.0f,qi=%.0f,qf=%.0f,dqi=%.0f,dqf=%.0f',ti,tf,ta,td,dqc,qi,qf,dqi,dqf);
%title_graph = sprintf('Trapezoidal with \n ti=0,dqc=%.0f,ddqc=%.0f,qi=%.0f,qf=%.0f',dqc,ddqc,qi,qf)
plot_trajectories(3,title_graph,time,q,dq,ddq);

%% case A: DT and ddqc max
clear all
clc
qi = 35;
qf = 5;
ti =3;
dqi = 5;
dqf = 2;
st = 0.01;
ddqcmax = 2.1662;
DT = 5;
[time,q,dq,ddq,~,~] = trapezoidal_ddqcmax_DT(qi,qf,st,ti,DT,dqi,dqf,ddqcmax);
title_graph = sprintf('Trapezoidal with \n ti=%0.f,qi =%.0f,qf=%.0f,dqi=%.0f,dqf=%.0f,DT=%.0f,ddqcmax =%.5f%',ti,qi,qf,dqi,dqf,DT,ddqcmax);
plot_trajectories(3,title_graph,time,q,dq,ddq);

%% case ddqc max dqc max (DT is an output)
clear all
clc
qi = 0;
qf = 35;
ti = 0;
dqi = 5;
dqf = 2;
st = 0.01;
ddqcmax = 10;
dqcmax = 15; % put dqcmax = 20 to see the case in which the maximum velocity is not reached
[time,q,dq,ddq,dddq,ddddq] = trapezoidal_ddqcmax_dqcmax(qi,qf,st,ti,dqi,dqf,dqcmax,ddqcmax);
title_graph = sprintf('Trapezoidal with \n ti=%0.f,qi =%.0f,qf=%.0f,dqi=%.0f,dqf=%.0f,dqcmax=%0.f,ddqcmax =%0.f%',ti,qi,qf,dqi,dqf,dqcmax,ddqcmax);
plot_trajectories(3,title_graph,time,q,dq,ddq);

%% EXTRA case multipoint trajectory still working on it
% clear all
% qi = 5;
% qf = 500;
% ti = 0;
% tf = 60;
% dqc = 10;
% st = 0.01;
% points_and_time = [ 200 30 ];
% 
% %points_and_time = [];
% points_and_time = [ qi ti ; points_and_time; qf tf ]
% [time,q,dq,ddq ~,~] = trapezoidal_multipoint(points_and_time,st,dqc)
% title_graph = sprintf('ciao')
% plot_trajectories(3,title_graph,time,q,dq,ddq);

