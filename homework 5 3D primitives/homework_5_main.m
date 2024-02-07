% function [p,dp,ddp,dddp] = circular(p1,p2,centre,st,orientation_for_centre_case)
% function [p,dp,ddp,dddp] = rectilinear(pi,pf,st)

% first rectilinear from 0 0 0 to 1 0 0
% second circular centre 1 1 0 || pi 1 0 0 || pf 2 1 0
% third circular  centre 2 1 1 || pi 2 1 0 || pf 2 1 2
% fourth rectilinear from 2 1 2 to 2 0 2 
%% variabile declaration

p_tot = [];
dp_tot = [];
ddp_tot = [];
dddp_tot = [];
st = 0.01;

%% first 
% first rectilinear from 0 0 0 to 1 0 0

pi = [0;0;0];
pf = [1;0;0];

[p,dp,ddp,dddp] = rectilinear(pi,pf,st);
p_tot = [p_tot p];
dp_tot = [dp_tot dp];
ddp_tot = [ddp_tot ddp];
dddp_tot = [dddp_tot dddp];

%% second
% second circular centre 1 1 0 || pi 1 0 0 || pf 2 1 0
pi = [1;0;0];
pf = [2;1;0];
centre = [1;1;0];

[p,dp,ddp,dddp] = circular(pi,pf,centre,st,1);
p_tot = [p_tot p];
dp_tot = [dp_tot dp];
ddp_tot = [ddp_tot ddp];
dddp_tot = [dddp_tot dddp];

%% third
% third circular  centre 2 1 1 || pi 2 1 0 || pf 2 1 2
pi = [2;1;0];
pf = [2;1;2];
centre = [2;1;1];

[p,dp,ddp,dddp] = circular(pi,pf,centre,st,-1);
p_tot = [p_tot p];
dp_tot = [dp_tot dp];
ddp_tot = [ddp_tot ddp];
dddp_tot = [dddp_tot dddp];
%% fourth
% fourth rectilinear from 2 1 2 to 2 0 2 
pi = [2;1;2];
pf = [2;0;2];

[p,dp,ddp,dddp] = rectilinear(pi,pf,st);
p_tot = [p_tot p];
dp_tot = [dp_tot dp];
ddp_tot = [ddp_tot ddp];
dddp_tot = [dddp_tot dddp];

%% plot
figure
title_s ='Forward';
plotty(p_tot, dp_tot, ddp_tot, dddp_tot,title_s);

%% spline method
%%
clc
clear 

qtx=[0 1 2 2 2];
qty=[0 0 1 1 0];
qtz=[0 0 0 2 2];
tk=[0 1 2 3 4];
st = 0.01;
dqi = 0;
dqf = 0;
% first rectilinear from 0 0 0 to 1 0 0
% second circular centre 1 1 0 || pi 1 0 0 || pf 2 1 0
% third circular  centre 2 1 1 || pi 2 1 0 || pf 2 1 2
% fourth rectilinear from 2 1 2 to 2 0 2 
% 0 0 0
% 1 0 0
% 2 1 0
% 2 1 2
% 2 0 2

[timex,qx,dqx,ddqx,dddqx] = multipoints_cubical_spline(qtx,tk,st,dqi,dqf);
[timey,qy,dqy,ddqy,dddqy] = multipoints_cubical_spline(qty,tk,st,dqi,dqf);
[timez,qz,dqz,ddqz,dddqz] = multipoints_cubical_spline(qtz,tk,st,dqi,dqf);
figure
plot3(qx(:),qy(:),qz(:))
grid on
axis equal
hold on
plot3(qtx(:),qty(:),qtz(:),'x')
title_graph = sprintf('Cubical spline');



