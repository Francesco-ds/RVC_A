function [time,q,dq,ddq,dddq,ddddq] = polynomial_cubic_delta(st,ti,tf,qi,qf,dqi,dqf)

% solve parameters for the polynomial in matrix form
% qi=a0;
% qf=a0+a1*dt+a2*dt^2+a3*dt^3;
% dqi=a1;
% dqf=a1+2*a2*dt+3*a3*dt^2;

Q = [qi;qf;dqi;dqf];
DT = tf-ti;
time = ti:st:tf;


X=[1,0,0,0;
   1,DT,DT^2,DT^3;
   0,1,0,0;
   0,1,2*DT,3*DT^2];

A = X\Q;

a0 = A(1);
a1 = A(2);
a2 = A(3);
a3 = A(4);

q=a3*(time-ti).^3+a2*(time-ti).^2+a1*(time-ti)+a0;
dq=3*a3*(time-ti).^2+2*a2*(time-ti)+a1;
ddq=6*a3*(time-ti)+2*a2;
dddq = 6 * a3;
ddddq = [];

end