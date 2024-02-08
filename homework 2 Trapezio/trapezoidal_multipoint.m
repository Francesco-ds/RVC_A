function [time,q,dq,ddq,dddq,ddddq] = trapezoidal_multipoint(points_and_time,st,dqc)
% qD_max = -qD_min, qDD_max = -qDD_min, qDDD_max = -qDDD_mi
    dqmax = dqc
    ntrajectories = size(points_and_time,1);
    time = [];
    q = [];
    dq= [];
    ddq =[];
    velocities = [0]
    for i = 2 : (ntrajectories -1)

        qk  = points_and_time(i) 
        qk_prec= points_and_time(i-1) 
        qk_next= points_and_time(i+1)
        DQK = qk-qk_prec
        DQK1 = qk_next - qk
        if DQK * DQK1 > 0
            velocities=[velocities sign(DQK)*dqmax];
        
        else
            velocities = [velocities 0];
        end
   
    end
    velocities = [velocities 0];
    
    for j = 1:(ntrajectories-1)
        qi = points_and_time(j,1)
        qf = points_and_time(j+1,1)
        dqi= velocities(j)
        dqf= velocities(j+1)
        ti= points_and_time(j,2)
        tf =points_and_time(j+1,2)
        tf = tf;
        DT = tf-ti
        [time_add,q_add,dq_add,ddq_add,~,~]= trapezoidal_given_dqc_ti_is_zero(ti,tf,st,qi,qf,dqc)
        %title_graph = sprintf('ciao')
        %plot_trajectories(3,title_graph,time_add,q_add,dq_add,ddq_add);
        if isempty(time)
            time = time_add;
        q = q_add;
        dq = dq_add;
        ddq = ddq_add;
        else
        time = [time time_add(2:end)];
        q = [q q_add(2:end)];
        dq = [dq dq_add(2:end)];
        ddq = [ddq ddq_add(2:end)];
        end
        
        
      
    end

    dddq =[]
    ddddq = []
      


   

end

