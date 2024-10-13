clear; clc;
TotalTime = 40; %600
cnt = 1;
I0_max = 0.8;
sample_size = 1;
timestep = 0.01;
I0_range = 0:0.1:I0_max;
alpha = 0.4;
beta = 0.4;
lower_bound_ori = 5;
upper_bound_ori = 35;
for I0 = I0_range
    disp(cnt)
    [S_LCC,S_tilde] = PF_LCC(I0,TotalTime); 
    y1_bar = S_tilde(1:end-1,4,1);
    y2_bar = S_tilde(1:end-1,4,2);
    y1 = S_LCC(1:end-1,3,1) - S_LCC(1:end-1,4,1);
    y2 = S_LCC(1:end-1,4,2);
    error = y1_bar - y1;
    spacing_error(:,cnt) = error;
    mean_se(cnt) = mean(error);
    var_se(cnt) = var(error);
    lower_bound(cnt) = lower_bound_ori + mean_se(cnt) + var_se(cnt)*sqrt((1-alpha)/alpha);
    upper_bound(cnt) = upper_bound_ori - mean_se(cnt) - var_se(cnt)*sqrt(beta/(1-beta));
    cnt = cnt+1;
end

label_size  = 14;
total_size  = 14;
line_width  = 2;

for i = 1:(cnt-1)
    figure(i)
    histogram(spacing_error(:,i))
    grid on;
end


figure(cnt)
tiledlayout(2,1)
ax1 = nexttile;
plot(I0_range,lower_bound,'linewidth',line_width)
xlabel('$I_0$','fontsize',label_size,'Interpreter','latex','Color','k')
line([0,I0_max],[5,5],'color','k','linestyle','--');
ylabel('Lower bound','fontsize',label_size,'Interpreter','latex','Color','k')
ylim([4.8 inf])
grid on;

ax2 = nexttile;
plot(I0_range,upper_bound,'linewidth',line_width)
line([0,I0_max],[35,35],'color','k','linestyle','--');
ylim([-inf 35.2])
grid on;
xlabel('$I_0$','fontsize',label_size,'Interpreter','latex','Color','k')
ylabel('Upper bound','fontsize',label_size,'Interpreter','latex','Color','k')




