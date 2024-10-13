cnt = 1;
TotalTime = 10;
[S_LCC,S_tilde] = AN_LCC(TotalTime);
S_LCC(:,4,1) = S_LCC(:,3,1) - S_LCC(:,4,1);
ST_range = 600:10:1000;
for ST = ST_range
    S_used = S_tilde(1:ST,:,:);
    S_ori_used = S_LCC(1:ST,:,:);
    [nmse_st_kappa(cnt),~] = Attacker(S_used);
    [nmse_st_kappa_ori(cnt),~] = Attacker(S_ori_used);
    y1_bar = S_used(1:end-1,4,1);
    y2_bar = S_used(1:end-1,4,2);
    y1 = S_LCC(1:ST-1,4,1);
    y2 = S_LCC(1:ST-1,4,2);
    nmse_st_dist(cnt) = sum((y1_bar - y1).^2 + (y2_bar - y2).^2)/ST;
    cnt = cnt+1;
end

figure(1)
plot(ST_range,nmse_st_kappa)
hold on;
plot(ST_range,nmse_st_kappa_ori)
xlabel('Time Step')
ylabel('normalized mse kappa')
legend('Additive noise','w/o privacy guarantee')

figure(2)
plot(ST_range,nmse_st_dist)
xlabel('Time Step')
ylabel('normalized mse distoration')