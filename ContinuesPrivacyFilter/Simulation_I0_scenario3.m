clear; clc;
TotalTime = 20; %20

vehicle_index = 2;
test_index = vehicle_index+1;
I0_preceding_max = 0.8;
I0_following_max = 0.8;
sample_size = 1;
timestep = 0.01;
I0_preceding_range = 0.1:0.1:I0_preceding_max;
I0_following_range = 0.1:0.1:I0_following_max;

cnt_preceding = 1;

PerturbedType = 1;

h=waitbar(0,'please wait');
for I0_preceding = I0_preceding_range
    cnt_following = 1;
    for I0_following = I0_following_range
        [S_LCC,S_tilde] = PF_LCC_scenario3(I0_preceding,I0_following,TotalTime,vehicle_index,PerturbedType); 
    
        [nmse_kappa(cnt_preceding,cnt_following),est(cnt_preceding,cnt_following,:)] = Attacker(S_tilde,timestep,test_index);
        y1_bar = S_tilde(1:end-1,test_index,1);
        y2_bar = S_tilde(1:end-1,test_index,2);
        y1 = S_LCC(1:end-1,test_index-1,1) - S_LCC(1:end-1,test_index,1);
        y2 = S_LCC(1:end-1,test_index,2);
        nmse_dist(cnt_preceding,cnt_following) = sum((y1_bar - y1).^2 + (y2_bar - y2).^2)/(TotalTime/timestep);
        spacing_error(cnt_preceding,cnt_following,:) = y1_bar - y1;
        cnt_following = cnt_following+1;
    end
    cnt_preceding = cnt_preceding + 1;
    waitbar(cnt_preceding/length(I0_preceding_range),h)
end

delete(h);

label_size  = 18;
total_size  = 14;
line_width  = 2;

figure(1)
[X,Y] = meshgrid(I0_preceding_range,I0_following_range);
surf(X, Y, nmse_kappa, 'EdgeColor', 'none');

xlabel('$I_0$ following', 'Fontname', 'Times New Roman','FontSize',18,'Interpreter','latex')
ylabel('$I_0$ preceding', 'Fontname', 'Times New Roman','FontSize',18,'Interpreter','latex')
zlabel('normalized MSE $\hat{\kappa}$', 'Fontname', 'Times New Roman','FontSize',18,'Interpreter','latex')
shading interp

