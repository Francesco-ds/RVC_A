function [time,q,dq,ddq,dddq,ddddq]=trajectory_5th_deltaT(st,ti,tf,qi,qf,dqi,dqf,ddqi,ddqf)

    Q=[qi;dqi;ddqi;qf;dqf;ddqf];
    time = ti:st:tf;
    DT = tf - ti;

    X=[ 1,      0,      0,      0,      0,      0;
        0,      1,      0,      0,      0,      0;
        0,      0,      2,      0,      0,      0;
        1,      DT,     DT^2,   DT^3,   DT^4,   DT^5;
        0,      1,      2*DT,   3*DT^2, 4*DT^3, 5*DT^4;
        0,      0,      2,      6*DT,   12*DT^2,20*DT^3];

    A = X\Q;


    a0=A(1);
    a1=A(2);
    a2=A(3);
    a3=A(4);
    a4=A(5);
    a5=A(6);

    q=a5*(time-ti).^5+a4*(time-ti).^4+a3*(time-ti).^3+a2*(time-ti).^2+a1*(time-ti)+a0;
    dq=5*a5*(time-ti).^4+4*a4*(time-ti).^3+3*a3*(time-ti).^2+2*a2*(time-ti)+a1;
    ddq=20*a5*(time-ti).^3+12*a4*(time-ti).^2+6*a3*(time-ti)+2*a2;
    dddq=60*a5*(time-ti).^2+24*a4*(time-ti)+6*a3;
    ddddq=120*a5*(time-ti)+24*a4;

end