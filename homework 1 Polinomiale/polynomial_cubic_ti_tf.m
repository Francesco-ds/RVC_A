function [time,q,dq,ddq,dddq,ddddq] = polynomial_cubic_ti_tf(st,ti,tf,qi,qf,dqi,dqf)




Q = [qi;qf;dqi;dqf];
time = ti:st:tf;


X=[1,ti,ti^2,ti^3;
   1,tf,tf^2,tf^3;
   0,1,2*ti,3*ti^2;
   0,1,2*tf,3*tf^2];

A = X\Q;

a0 = A(1);
a1 = A(2);
a2 = A(3);
a3 = A(4);

q=a3*(time).^3+a2*(time).^2+a1*(time)+a0;
dq=3*a3*(time).^2+2*a2*(time)+a1;
ddq=6*a3*(time)+2*a2;
dddq = 6 * a3;
ddddq = [];

end