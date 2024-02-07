function [p,dp,ddp,dddp] = rectilinear(pi,pf,st)


    %arc parametrization
    length = norm(pf-pi);
    s = 0:st:length;
    tangent_vector = (pf-pi)/length;

    p = pi + s.* tangent_vector;
    dp = ones(3,size(s,2)).* tangent_vector;
    ddp = zeros(3,size(s,2));
    dddp = zeros(3,size(s,2));


end