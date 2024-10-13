% choose true control parameter(no need)
clc;
close all;
clear;

n = 2;
m = 2;

DriverDynamics = 1;
PseudoDynamics = 1;

load(['data\SSRegion_n_2_m_2_DriverDynamics_',num2str(DriverDynamics),'_PseudoDynamics_',num2str(PseudoDynamics),'.mat'])

figure;

Wsize = 18;

[pseudo_dynamics_alpha_3D, pseudo_dynamics_beta_3D] = meshgrid(pseudo_dynamics_beta, pseudo_dynamics_alpha);

p = surf(pseudo_dynamics_alpha_3D, pseudo_dynamics_beta_3D, SS_bool);

mymap = [235,235,235;
    255, 181, 190;
    113, 178, 246]/255;

colormap(mymap);

view([0 90]);

p.EdgeColor = 'none';

hold on;

set(gca,'TickLabelInterpreter','latex','fontsize',Wsize-2);
set(gca,'xlim',[min(pseudo_dynamics_alpha) max(pseudo_dynamics_alpha)]);
set(gca,'ylim',[min(pseudo_dynamics_beta) max(pseudo_dynamics_beta)]);

set(gcf,'Position',[250 150 350 280]);
fig = gcf;
fig.PaperPositionMode = 'auto';

xlabel('k')
ylabel('\mu')