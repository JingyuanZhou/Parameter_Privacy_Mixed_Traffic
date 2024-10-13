clear; clc;
TotalTime = 20; %20

vehicle_index = 3;
test_index = vehicle_index;
I0_max = 0.8;
timestep = 0.01;
I0_range = 0.1:0.1:I0_max;

Avg_times = 1;
nmse_kappa_avg = 0;
nmse_dist_avg = 0;

PerturbedType = 2;

for Avg_time = 1:Avg_times
    disp(Avg_time)
    cnt = 1;
    for I0 = I0_range
        [S_LCC,S_tilde] = PF_LCC(I0,TotalTime,vehicle_index,PerturbedType); %UpdateTimeStep
        
        [nmse_kappa(cnt),est(cnt,:)] = Attacker(S_tilde,timestep,test_index);
        y1_bar = S_tilde(1:end-1,test_index,1);
        y2_bar = S_tilde(1:end-1,test_index,2);
        y1 = S_LCC(1:end-1,test_index-1,1) - S_LCC(1:end-1,test_index,1);
        y2 = S_LCC(1:end-1,test_index,2);
        nmse_dist(cnt) = sum((y1_bar - y1).^2 + (y2_bar - y2).^2)/(TotalTime/timestep);
        spacing_error(cnt,:) = y1_bar - y1;
        cnt = cnt+1;

    end
    nmse_kappa_avg = nmse_kappa_avg + nmse_kappa/Avg_times;
    nmse_dist_avg = nmse_dist_avg + nmse_dist/Avg_times;
end

label_size  = 18;
total_size  = 14;
line_width  = 2;

figure
plot(I0_range,nmse_kappa_avg,'linewidth',line_width)
grid on;
set(gca,'FontSize',15)
xlabel('$I_0$','fontsize',label_size,'Interpreter','latex','Color','k')
xlim([0.1 I0_max])
ylabel('normalized MSE $\hat{\kappa}$','fontsize',label_size,'Interpreter','latex','Color','k')

figure
plot(I0_range,nmse_dist_avg,'linewidth',line_width)
grid on;
set(gca,'FontSize',15)
xlabel('$I_0$','fontsize',label_size,'Interpreter','latex','Color','k')
xlim([0.1 I0_max])
ylabel('MSE distoration','fontsize',label_size,'Interpreter','latex','Color','k')

