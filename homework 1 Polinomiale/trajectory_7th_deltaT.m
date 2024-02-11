function [time,q,dq,ddq,dddq,ddddq]=trajectory_7th_deltaT(st,ti,tf,qi,qf,dqi,dqf,ddqi,ddqf,dddqi,dddqf)
Q=[qi;dqi;ddqi;dddqi;qf;dqf;ddqf;dddqf];

time=ti:st:tf;

DT=tf-ti;

X=[ 1,      0,      0,      0,      0,      0,      0,      0;
    0,      1,      0,      0,      0,      0,      0,      0;
    0,      0,      2,      0,      0,      0,      0,      0;
    0,      0,      0,      6,      0,      0,      0,      0;
    1,      DT,     DT^2,   DT^3,   DT^4,   DT^5,   DT^6,   DT^7;
    0,      1,      2*DT,   3*DT^2, 4*DT^3, 5*DT^4, 6*DT^5, 7*DT^6;
    0,      0,      2,      6*DT,   12*DT^2,20*DT^3,30*DT^4,42*DT^5;
    0,      0,      0,      6,      24*DT,60*DT^2,120*DT^3,210*DT^4];

%XA=Q, A=inv(X)Q

A=X\Q;

a0=A(1);
a1=A(2);
a2=A(3);
a3=A(4);
a4=A(5);
a5=A(6);
a6=A(7);
a7=A(8);

q=      a7*(time-ti).^7+        a6*(time-ti).^6+        a5*(time-ti).^5+    a4*(time-ti).^4+    a3*(time-ti).^3+    a2*(time-ti).^2+    a1*(time-ti)+   a0;
dq=     7*a7*(time-ti).^6+      6*a6*(time-ti).^5+      5*a5*(time-ti).^4+  4*a4*(time-ti).^3+  3*a3*(time-ti).^2+  2*a2*(time-ti)+     a1;
ddq=    42*a7*(time-ti).^5+     30*a6*(time-ti).^4+     20*a5*(time-ti).^3+ 12*a4*(time-ti).^2+ 6*a3*(time-ti)+     2*a2;
dddq=   210*a7*(time-ti).^4+    120*a6*(time-ti).^3+    60*a5*(time-ti).^2+ 24*a4*(time-ti)+    6*a3;
ddddq=  840*a7*(time-ti).^3+    360*a6*(time-ti).^2+    120*a5*(time-ti)+   24*a4;


end