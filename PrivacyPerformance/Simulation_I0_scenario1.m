clear; clc;
TotalTime = 20; %20
cnt = 1;
vehicle_index = 2;
test_index = vehicle_index;
I0_max = 0.8;
sample_size = 1;
timestep = 0.01;
I0_range = 0:0.1:I0_max;
PerturbedType = 2;
for I0 = I0_range
    disp(cnt)
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

label_size  = 18;
total_size  = 14;
line_width  = 2;

figure
plot(I0_range,nmse_kappa,'linewidth',line_width)
grid on;
set(gca,'FontSize',15)
xlabel('$I_0$','fontsize',label_size,'Interpreter','latex','Color','k')
ylabel('normalized MSE $\hat{\kappa}$','fontsize',label_size,'Interpreter','latex','Color','k')

figure
plot(I0_range,nmse_dist,'linewidth',line_width)
grid on;
set(gca,'FontSize',15)
xlabel('$I_0$','fontsize',label_size,'Interpreter','latex','Color','k')
ylabel('MSE distoration','fontsize',label_size,'Interpreter','latex','Color','k')

