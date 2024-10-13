% choose pseudo parameter
clc;
close all;
clear;

n = 2;
m = 2;

pf_index = 5;

DriverDynamics = 1;
ControlGains = 1;

load(['data\SSRegion_n_',num2str(n),'_m_',num2str(m),...
       '_DriverDynamics_',num2str(DriverDynamics),'_ControlGains_',num2str(ControlGains),'_PrivacyFilter_',num2str(pf_index),'.mat'])

figure;

Wsize = 18;

[pseudo_dynamics_alpha_3D, pseudo_dynamics_beta_3D] = meshgrid(pseudo_dynamics_beta, pseudo_dynamics_alpha);


surf(pseudo_dynamics_alpha_3D, pseudo_dynamics_beta_3D, SS_bool,'EdgeColor','none');

mymap = [235,235,235;
    255, 181, 190;
    113, 178, 246]/255;

colormap(mymap);

hold on;

scatter3(alpha,beta,1,100,'x','MarkerFaceColor', 'black', ...
        'MarkerEdgeColor', 'black');

view([0 90]);


set(gca,'TickLabelInterpreter','latex','fontsize',Wsize-2);
set(gca,'xlim',[min(pseudo_dynamics_alpha) max(pseudo_dynamics_alpha)]);
set(gca,'ylim',[min(pseudo_dynamics_beta) max(pseudo_dynamics_beta)]);

set(gcf,'Position',[250 150 350 280]);
fig = gcf;
fig.PaperPositionMode = 'auto';

xlabel('pseudo \alpha','FontSize',24,'FontName','Times New Roman')
ylabel('pseudo \beta','FontSize',24,'FontName','Times New Roman')

exportgraphics(fig, ['figure\stable_region_pf_idx_',num2str(pf_index),'.pdf'])