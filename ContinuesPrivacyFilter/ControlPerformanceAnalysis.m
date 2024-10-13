I0 = 0.3;
TotalTime = 30;
If_cal = 1;

Controller_type = 2;
PF_enable = 1;
mix                 = 1;                    % 0. all HDVs; 1. there exist CAVs
perturb_type = 2;

% Calculate Control Traj
if If_cal
    if Controller_type == 1
        ControlPerformance_Linear(I0, TotalTime, PF_enable, mix, perturb_type)
    else
        ControlPerformance_DeePC(I0, TotalTime, PF_enable, mix, perturb_type)
    end
end

% -------------------------------------------------------------------------
%   Calculate Performance Indexes
%--------------------------------------------------------------------------
if perturb_type == 1
    load(['data\S_mix_',num2str(mix),'_PF_',num2str(PF_enable),'_Controller_',num2str(Controller_type),'.mat'])
elseif perturb_type == 2
    load(['data\S_mix_',num2str(mix),'_PF_',num2str(PF_enable),'_Controller_',num2str(Controller_type), '_PerturbType_', num2str(perturb_type),'.mat'])
end

smooth_window = 10;
n_vehicle = 7;
begin_time = 0.01;
end_time = TotalTime;
Tstep = 0.01;

for i = 2:n_vehicle+1
   S(:,i,3)   = smooth(S(:,i,3),smooth_window); 
end

FuelConsumption = 0;
VelocityError   = 0;
for i=begin_time/Tstep:end_time/Tstep
    R  = 0.333 + 0.00108*S(i,3:end,2).^2 + 1.2*S(i,3:end,3);
    Fuel  = 0.444 + 0.09*R.*S(i,3:end,2) + 0.054 * max(0,S(i,3:end,3)).^2.*S(i,3:end,2);
    Fuel(R <= 0) = 0.444;
    FuelConsumption = FuelConsumption + sum(Fuel)*Tstep;
    
    VelocityError = VelocityError + sum(abs(S(i,3:end,2)-S(i,1,2))/S(i,1,2));
    
end

VelocityError = VelocityError/n_vehicle/((end_time-begin_time)/Tstep);

fprintf('Fuel comsumption:   %4.2f \n',FuelConsumption);
fprintf('Velocity error:   %4.2f \n',VelocityError);