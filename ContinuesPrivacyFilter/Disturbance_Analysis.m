clear; clc;
TotalTime = 1000; %20

vehicle_index = 3;
test_index = vehicle_index;
I0 = 0.1;
PerturbedType = 3;

label_size  = 20;

[S_LCC,S_tilde] = PF_LCC(I0,TotalTime,vehicle_index, PerturbedType); %UpdateTimeStep

target_veh_true_state = S_LCC(:,vehicle_index,:);
target_veh_pseudo_state = S_tilde(:,vehicle_index,:);

spacing_LCC = S_LCC(:,1:(end-1),1) - S_LCC(:,2:end,1);

acceleration_diff = target_veh_true_state(:,1,3) - target_veh_pseudo_state(:,1,3);
velocity_diff = target_veh_true_state(1:end-1,1,2) - target_veh_pseudo_state(1:end-1,1,2);
spacing_diff = spacing_LCC(1:end-1,vehicle_index - 1,1) - S_tilde(1:end-1,vehicle_index,1);

figure(1)
histfit(acceleration_diff)

figure(2)
set(gcf, 'Renderer', 'Painters');
set(gcf, 'Units', 'pixels', 'Position', [100, 100, 560, 420]);
set(gca,'FontSize',15)
set(gca, 'XColor', 'k', 'YColor', 'k');
histfit(velocity_diff)
pd = fitdist(velocity_diff, 'normal')
%title('Velocity Disturbance, $$I_0=0.1$$','interpreter','latex')
xlabel('$$m/s$$','interpreter','latex','fontsize',label_size)

%saveas(gcf, '../figs/disturbance_dist/velocity_dist.eps', '-depsc')
print('../figs/disturbance_dist/velocity_dist.eps', '-depsc')

figure(3)
set(gcf, 'Renderer', 'Painters');
set(gcf, 'Units', 'pixels', 'Position', [100, 100, 560, 420]);
set(gca,'FontSize',15)
set(gca, 'XColor', 'k', 'YColor', 'k');
histfit(spacing_diff)
pd = fitdist(spacing_diff, 'normal')
%title('Spacing Disturbance, $$I_0=0.1$$','interpreter','latex')
xlabel('$$m$$','interpreter','latex','fontsize',label_size)

%saveas(gcf, '../figs/disturbance_dist/spacing_dist.eps', '-depsc')
print('../figs/disturbance_dist/spacing_dist.eps', '-depsc')
