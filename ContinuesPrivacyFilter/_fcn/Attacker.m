function [nmse,est] = Attacker(S_tilde,dt,test_index)

    Ts =0;
    
    y1_bar = S_tilde(1:end-1,test_index,1);
    y2_bar = S_tilde(1:end-1,test_index,2);
    u_bar = S_tilde(1:end-1,test_index-1,2);
    
    y_bar = [y1_bar,y2_bar];
    
    z_bar = iddata(y_bar,u_bar,dt,'Name','OVM');
    
    z_bar.InputName = 'Head vehicle velocity';
    z_bar.InputUnit = 'm/s';
    z_bar.OutputName = {'Spacing','Velocity'};
    z_bar.OutputUnit = {'m','m/s'};
    z_bar.Tstart = 0;
    z_bar.TimeUnit = 's';
    
    FileName = 'CF_OVM';
    Order = [2,1,2];
%    Parameters = {0.3;0.4;5;35};
    Parameters = {0.3;0.4;5;35};
    InitialStates = [20;15];
    
    nlgr_bar = idnlgrey(FileName,Order,Parameters,InitialStates,Ts, ...
        'Name','OVM');
    nlgr_bar.Parameters(2).Minimum = 0;
    nlgr_bar.Parameters(1).Minimum = 0;
    nlgr_bar.Parameters(1).Maximum = 5;
    nlgr_bar.Parameters(2).Maximum = 5;

    nlgr_bar.Parameters(3).Fixed = true;
    nlgr_bar.Parameters(4).Fixed = true;
%     nlgr_bar.Parameters(3).Minimum = 3.5;
%     nlgr_bar.Parameters(4).Minimum = 33;
%     nlgr_bar.Parameters(3).Maximum = 6.5;
%     nlgr_bar.Parameters(4).Maximum = 37;
    
    set(nlgr_bar, 'InputName', 'Head vehicle velocity', 'InputUnit', 'm/s',               ...
              'OutputName', {'Spacing', 'Velocity'}, ...
              'OutputUnit', {'m', 'm/s'},                         ...
              'TimeUnit', 's');
    
    nlgr_bar = nlgreyest(z_bar,nlgr_bar);
    gt = [0.45,0.65,5,35]; %
    for i=1:4
        est(i) = nlgr_bar.Parameters(i).Value;
    end
    nmse = sum(((est-gt)./gt).^2);

end




