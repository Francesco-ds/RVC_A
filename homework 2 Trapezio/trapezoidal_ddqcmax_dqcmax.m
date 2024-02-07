function [time,q,dq,ddq,dddq,ddddq] = trapezoidal_ddqcmax_dqcmax(qi,qf,st,ti,dqi,dqf,dqcmax,ddqcmax)

segno=sign(qf-qi);

qi=segno*qi;
qf=segno*qf;
dqi=segno*dqi;
dqf=segno*dqf;

DQ  = qf-qi
%check feasibility 
if ddqcmax * (DQ) <= (abs(dqi^2 - dqf^2))/ 2
    error('Trajectory is not feasible')
end

is_reached = 0;

if ddqcmax * DQ >= (dqcmax)^2 - (dqi^2 + dqf^2)/2

    is_reached = 1;
end

% case dqcmax is reached (i have velocity phase)
if is_reached == 1
    DQ-1;
    dqc = dqcmax;
    ta = (dqcmax - dqi) / ddqcmax;
    td = (dqcmax-dqf) / ddqcmax;
    DT = DQ/dqc + dqc/(2*ddqcmax)*((1-dqi/dqcmax)^2+(1-dqf/dqcmax)^2);    
    tf = DT + ti;
   [time,q,dq,ddq,dddq,ddddq] = trapezoidal_lot_of_params(qi,qf,ta,td,ti,tf,dqi,dqf,st,dqc);
end  
% case dqcmax isnt reached (triangle)
if is_reached == 0
    dqc = sqrt((ddqcmax*DQ) + (dqi^2+dqf^2)/2);
    if dqc > dqcmax 
        error('The velocity shouldnt be greater than the maximum one')
    end
    dqclim = dqc;
    ta = (dqclim-dqi)/ddqcmax;
    td = (dqclim-dqf)/ddqcmax;
    DT = ta+td;
    tf = DT-ti;
    %sprintf(' I am here')
    [time,q,dq,ddq,dddq,ddddq] = trapezoidal_lot_of_params(qi,qf,ta,td,ti,tf,dqi,dqf,st,dqc);
end
    
q=segno*q;
dq=segno*dq;
ddq=segno*ddq;

end