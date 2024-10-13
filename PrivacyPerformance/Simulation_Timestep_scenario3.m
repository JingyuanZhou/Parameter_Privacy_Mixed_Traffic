clc; clear;
cnt = 1;
I0_preceding = 0.2;
I0_following = 0.2;
noise = 0.2;
TotalTime = 20;

vehicle_index = 2;
test_index = vehicle_index+1;

PerturbedType = 1;

[~,S_tilde_AN] = AN_LCC(noise,TotalTime,vehicle_index,PerturbedType);
[S_LCC,S_tilde_PF] = PF_LCC_scenario3(I0_preceding,I0_following,TotalTime,vehicle_index,PerturbedType);
S_LCC(:,test_index,1) = S_LCC(:,test_index-1,1) - S_LCC(:,test_index,1);
ST_range = 100:50:2000;
dt = 0.01;
rng(1024);

for ST = ST_range
    disp(ST)
    S_used_PF = S_tilde_PF(1:ST,:,:);
    S_used_AN = S_tilde_AN(1:ST,:,:);
    S_ori_used = S_LCC(1:ST,:,:);
    [nmse_st_kappa_PF(cnt),est_PF(cnt,:)] = Attacker(S_used_PF,dt,test_index);
    [nmse_st_kappa_AN(cnt),est_AN(cnt,:)] = Attacker(S_used_AN,dt,test_index);
    [nmse_st_kappa_ori(cnt),est_ori(cnt,:)] = Attacker(S_ori_used,dt,test_index);
    y1_bar_PF = S_used_PF(1:end-1,test_index,1);
    y2_bar_PF = S_used_PF(1:end-1,test_index,2); 
    y1_bar_AN = S_used_AN(1:end-1,test_index,1);
    y2_bar_AN = S_used_AN(1:end-1,test_index,2); 
    y1 = S_LCC(1:ST-1,test_index,1);
    y2 = S_LCC(1:ST-1,test_index,2);
    nmse_st_dist_PF(cnt) = sum(((y1_bar_PF - y1)).^2 + ((y2_bar_PF - y2)).^2)/ST;
    nmse_st_dist_AN(cnt) = sum(((y1_bar_AN - y1)).^2 + ((y2_bar_AN - y2)).^2)/ST;
    cnt = cnt+1;
end

label_size  = 18;
total_size  = 14;
line_width  = 2;

figure
plot(ST_range,nmse_st_kappa_PF,'linewidth',line_width)
hold on;
plot(ST_range,nmse_st_kappa_AN,'linewidth',line_width)
plot(ST_range,nmse_st_kappa_ori,'linewidth',line_width)
grid on;
set(gca,'FontSize',15)
xlim([100 2000])
ylim([0 6])
xlabel('Time Step','fontsize',label_size,'Interpreter','latex','Color','k')
ylabel('normalized MSE $\hat{\kappa}$','fontsize',label_size,'Interpreter','latex','Color','k')
legend('Privacy filter','Additive noise','w/o privacy guarantee')

figure
plot(ST_range,nmse_st_dist_PF,'linewidth',line_width)
hold on;
plot(ST_range,nmse_st_dist_AN,'linewidth',line_width)
grid on;
set(gca,'FontSize',15)
xlim([300 2000])
ylim([0 6])
xlabel('Time Step','fontsize',label_size,'Interpreter','latex','Color','k')
ylabel('normalized MSE distoration','fontsize',label_size,'Interpreter','latex','Color','k')
legend('privacy filter','Additive noise')