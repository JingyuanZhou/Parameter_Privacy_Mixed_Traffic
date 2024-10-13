function [tilde_vel_cur,tilde_spacing_cur,PF] = pseudo_state_generator(vel_cur,pos_cur,vel_last,pos_last,PF,ID)
    
    tilde_vel_cur = zeros(1,length(ID)-1);
    tilde_spacing_cur = zeros(1,length(ID)-1);
    spacing_cur = pos_cur(1:end-1) - pos_cur(2:end);
    spacing_last = pos_last(1:end-1) - pos_last(2:end);

    for i = 1:length(ID)
        if ID(i) == 0
            x_temp = [spacing_cur(i),vel_cur(i)];
            x_temp_last = [spacing_last(i),vel_last(i)];
            [PF{i},x_tilde] = PF{i}.NonlinearTransformation(x_temp,x_temp_last,x_temp,[0.1,0.1],vel_last(i));
            tilde_vel_cur(i) = x_tilde(2);
            tilde_spacing_cur(i) = x_tilde(1);
        else
            tilde_vel_cur(i) = vel_cur(i);
            tilde_spacing_cur(i) = spacing_cur(i);
        end
    end

end

