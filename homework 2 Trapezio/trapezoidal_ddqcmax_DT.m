function [time,q,dq,ddq,dddq,ddddq] = trapezoidal_ddqcmax_DT(qi,qf,st,ti,DT,dqi,dqf,ddqcmax)
    segno = sign(qf-qi);
    qf = segno*qf;
    qi = segno*qi;
%check feasibility
DQ = qf - qi;
%togli abs a sx
DDQ = dqf - dqi;
if ddqcmax*DQ <= abs(dqi^2-dqf^2)/2
    error('Trajectory is not feasible');
end

ddqclim = (2*DQ-(dqi+dqf)*DT+sqrt(4*DQ^2-4*DQ*(dqi+dqf)*DT+2*(dqi^2+dqf^2)*DT^2))/DT^2;

if ddqcmax < ddqclim
    error('Maximum acceleration cant be smaller than the limit one')
end


condition = ddqcmax^2*DT^2-4*ddqcmax*DQ+2*ddqcmax*(dqi+dqf)*DT-DDQ^2;
if condition <= 0
    error('Maximum acceleration has something wrong')
end

dqc = (dqi + dqf + ddqcmax*DT - sqrt(condition))/2;
ta = (dqc - dqi)/ddqcmax;
td = (dqc - dqf)/ddqcmax;
tf = ti + DT;
st = 0.01;

[time,q,dq,ddq,dddq,ddddq] = trapezoidal_lot_of_params(qi,qf,ta,td,ti,tf,dqi,dqf,st,dqc);
    q = segno*q;
    dq = segno*dq;
    ddq=segno*ddq;

end