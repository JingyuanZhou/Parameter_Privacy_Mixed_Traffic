function [dx,y] = CF_OVM(t,x,u,alpha,beta,s_st,s_go,varargin)
    v_max = 30;
    % calculate V(s)
    if x(1) <= s_st
        V = 0;
    elseif x(1) >= s_go
        V = v_max;
    else
        V = (v_max/2)*(1-cos(pi*(x(1)-s_st)/(s_go-s_st)));
    end
    % calculate derivative
    acel_max = 2;
    dcel_max = -5;
    dv = alpha*(V-x(2))+beta*(u-x(2));
    ds = u-x(2);

    if dv>acel_max
        dv = acel_max;
    elseif dv<dcel_max
        dv = dcel_max;
    end
    dx = [ds ; dv];
    y = [x(1);x(2)];
end