% =========================================================================
%               Analysis for simulation results under same data sample               
% =========================================================================

clc; clear; close all;

% Simulation Time
begin_time       = 0.01;
end_time         = 30;              
PF_enable        = 0;
mix = 0;
controller_type = 1;
ID = [0,0,1,0,0,0,0];   
Tstep = 0.01;

if mix
    switch controller_type
        case 1
            if PF_enable
                load("../data/S_mix_1_PF_1_Controller_1.mat")
            else
                load("../data/S_mix_1_PF_0_Controller_1.mat")
            end
        case 2
            if PF_enable
                load("../data/S_mix_1_PF_1_Controller_2.mat")
            else
                load("../data/S_mix_1_PF_0_Controller_2.mat")
            end
     end
else
     load("../data/S_mix_0_PF_0_Controller_1.mat")   
end


n_vehicle   = length(ID);           % number of vehicles



% -------------------------------------------------------------------------
%   Plot Results
%--------------------------------------------------------------------------
color_gray  = [190 190 190]/255;
color_red   = [244, 53, 124]/255;
color_blue  = [67, 121, 227]/255;
color_black = [0 0 0];
color_orange = [255,132,31]/255;
label_size  = 18;
total_size  = 14;
line_width  = 2;

% Velocity
figure;
id_cav = 1;
plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,1,2),'Color',color_black,'linewidth',line_width-0.5); hold on;
for i = 1:n_vehicle
    if ID(i) == 0
        plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i+1,2),'Color',color_gray,'linewidth',line_width-0.5); hold on; % line for velocity of HDVs
    end
end
for i = 1:n_vehicle
    if ID(i) == 1
        if id_cav == 1
            plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i+1,2),'Color',color_red,'linewidth',line_width); hold on; % line for velocity of CAVs
            id_cav  = id_cav+1;
        elseif id_cav == 2
            plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i+1,2),'Color',color_blue,'linewidth',line_width); hold on; % line for velocity of CAVs
        end
    end 
end
grid on;
set(gca,'TickLabelInterpreter','latex','fontsize',total_size);
set(gca,'YLim',[5 25]);
set(gca,'XLim',[0 30]);

xl = xlabel('$t$ [$\mathrm{s}$]','fontsize',label_size,'Interpreter','latex','Color','k');
yl = ylabel('Velocity [$\mathrm{m/s}$]','fontsize',label_size,'Interpreter','latex','Color','k');

set(gcf,'Position',[250 150 400 300]);
fig = gcf;
fig.PaperPositionMode = 'auto';

print(gcf, ['controller_',num2str(controller_type),'_mix_',num2str(mix), '_PF_' ,num2str(PF_enable) ,'_vecloty.eps'],'-vector','-depsc2','-r300');

% if mix
%     print(gcf,['.\figs\BrakePerturbation_VelocityProfile_Controller_',num2str(controller_type)],'-painters','-depsc2','-r300');
% else
%     print(gcf,'.\figs\BrakePerturbation_VelocityProfile_AllHDVs','-painters','-depsc2','-r300');
% end

% Spacing
figure;
id_cav = 1;
for i = 1:n_vehicle
   if ID(i) == 1
        if id_cav ==1
        plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i,1)-S(begin_time/Tstep:end_time/Tstep,i+1,1),'Color',color_red,'linewidth',line_width); hold on; % line for velocity of CAVs
        id_cav = id_cav + 1;
        elseif id_cav == 2
        plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i,1)-S(begin_time/Tstep:end_time/Tstep,i+1,1),'Color',color_blue,'linewidth',line_width); hold on; % line for velocity of CAVs
        end
   else
       plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i,1)-S(begin_time/Tstep:end_time/Tstep,i+1,1),'Color',color_gray,'linewidth',line_width); hold on;
   end 
end
% if mix
%     plot(begin_time:Tstep:end_time,5*ones(round((end_time-begin_time)/Tstep)+1),'--k','linewidth',line_width);
%     text(11,6.2,'$s_\mathrm{min}=5\,\mathrm{m}$','Interpreter','latex','FontSize',label_size);
% end
grid on;
set(gca,'TickLabelInterpreter','latex','fontsize',total_size);
set(gca,'YLim',[0 40]);
set(gca,'XLim',[0 30]);
set(gca,'YTick',5:10:35);

xl = xlabel('$t$ [$\mathrm{s}$]','fontsize',label_size,'Interpreter','latex','Color','k');
yl = ylabel('Spacing [$\mathrm{m}$]','fontsize',label_size,'Interpreter','latex','Color','k');

set(gcf,'Position',[550 150 400 300]);
fig = gcf;
fig.PaperPositionMode = 'auto';

print(gcf, ['controller_',num2str(controller_type),'_mix_',num2str(mix), '_PF_' ,num2str(PF_enable),'_spacing.eps'],'-vector','-depsc2','-r300');

% if mix
%     print(gcf,['.\figs\BrakePerturbation_SpacingProfile_Controller_',num2str(controller_type)],'-painters','-depsc2','-r300');
% else
%     print(gcf,'.\figs\BrakePerturbation_SpacingProfile_AllHDVs','-painters','-depsc2','-r300');
% end

% Acceleration
figure;
id_cav = 1;
%plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,1,3),'Color',color_black,'linewidth',line_width-0.5); hold on;
for i = 1:n_vehicle
    if ID(i) == 1
        plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i+1,3),'Color',color_red,'linewidth',line_width); hold on; % line for velocity of CAVs

    end 
end
grid on;
set(gca,'TickLabelInterpreter','latex','fontsize',total_size);
set(gca,'YLim',[-6 4]);
set(gca,'XLim',[0 30]);

xl = xlabel('$t$ [$\mathrm{s}$]','fontsize',label_size,'Interpreter','latex','Color','k');
yl = ylabel('Acceleration [$\mathrm{m/s^2}$]','fontsize',label_size,'Interpreter','latex','Color','k');

set(gcf,'Position',[250 450 400 300]);
fig = gcf;
fig.PaperPositionMode = 'auto';
print(gcf, ['controller_',num2str(controller_type),'_mix_',num2str(mix), '_PF_' ,num2str(PF_enable),'_acceleration.eps'],'-vector','-depsc2','-r300');
