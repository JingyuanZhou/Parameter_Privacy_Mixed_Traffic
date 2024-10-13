% =========================================================================
%               Data collection for random times
% =========================================================================

clc; close all; clear;
addpath('_fcn');

data_total_number = 100;

h_wait = waitbar(0,'please wait');

% Initialization: Privacy filter
ID          = [0,1,0,0,1,0,0];  
pos_cav     = find(ID==1);
n_vehicle   = length(ID); 
v_star      = 15;       
I0 = 0.1;
PF_enable = 1;
if PF_enable
    PF = cell(1,n_vehicle);
    for i=1:n_vehicle
        if ID(i) == 0 && i>1
            PF{i} = PrivacyFilter(1,v_star);
            PF{i} = PF{i}.MonteCarlo(1000);
            PF{i} = PF{i}.Randomizer(I0);
        else
            PF{i} = 0;
        end
    end
end

for i_data = 1:data_total_number

% -------------------------------------------------------------------------
%   Parameter setup
% -------------------------------------------------------------------------

% Type for HDV car-following model
hdv_type        = 1;    % 1. OVM   2. IDM
% Uncertainty for HDV behavior
acel_noise      = 0.1;  % A white noise signal on HDV's original acceleration %0.1
% Data set
data_str        = '3';  % 1. random ovm  2. manual ovm  3. homogeneous ovm

% Parameters in Simulation
total_time       = 40;              % Total Simulation Time
Tstep            = 0.01;            % Time Step
total_time_step  = total_time/Tstep;

% DeePC Formulation
T       = 2000;      % length of data samples
Tini    = 20;        % length of past data
N       = 50;        % length of predicted horizon

weight_v     = 1;        % weight coefficient for velocity error
weight_s     = 0.5;      % weight coefficient for spacing error   
weight_u     = 0.1;      % weight coefficient for control input

lambda_g     = 1;        % penalty on ||g||_2^2 in objective
lambda_y     = 1e3;      % penalty on ||sigma_y||_2^2 in objective

% System Dynamics
% vel_noise = 0.1;         % noise signal in velocity signal

% ------------------------------------------
% Parameters in Mixed Traffic
% ------------------------------------------
ID          = [0,1,0,0,1,0,0];    % ID of vehicle types
                                    % 1: CAV  0: HDV
pos_cav     = find(ID==1);          % position of CAVs
n_vehicle   = length(ID);           % number of vehicles
n_cav       = length(pos_cav);      % number of CAVs
n_hdv       = n_vehicle-n_cav;      % number of HDVs

mix         = 1;                    % whether mixed traffic flow

v_star      = 15;                   % Equilibrium velocity
s_star      = 20;                   % Equilibrium spacing for CAV

switch hdv_type
    case 1
        % Driver Model: OVM
        load(['../_data/hdv_ovm_',num2str(data_str),'.mat']);
    case 2
        % Driver Model: IDM
        load('../_data/hdv_idm.mat');  
end

acel_max = 2;
dcel_max = -5;
    
measure_type = 1;
% ------------------
%  size in DeePC
% ------------------

n_ctr = 2*n_vehicle;    % number of state variables
m_ctr = n_cav;          % number of input variables
switch measure_type     % number of output variables
    case 1
        p_ctr = n_vehicle;
    case 2
        p_ctr = 2*n_vehicle;
    case 3
        p_ctr = n_vehicle + n_cav;
end

% -------------------------------------------------------------------------
%   Scenario initialization
%-------------------------------------------------------------------------- 

% There is one head vehicle at the very beginning
S           = zeros(total_time_step,n_vehicle+1,3);
S(1,1,1)    = 0;
for i = 2 : n_vehicle+1
    S(1,i,1) = S(1,i-1,1) - hdv_parameter.s_star(i-1);
end
S(1,:,2)    = v_star * ones(n_vehicle+1,1);

% -------------------------------------------------------------------------
%   Data collection
%-------------------------------------------------------------------------- 

% ------------------
%  persistently exciting input data
% ------------------
ud          = -1+2*rand(m_ctr,T);
ed          = -1+2*rand(1,T);
yd          = zeros(p_ctr,T);
yd_GT       = zeros(p_ctr,T);

% ------------------
%  generate output data
% ------------------
for k = 1:T-1
    % Update acceleration
    acel               = HDV_dynamics(S(k,:,:),hdv_parameter) ...
                         -acel_noise + 2*acel_noise*rand(n_vehicle,1);
    
    S(k,1,3)           = 0;         % the head vehicle
    S(k,2:end,3)       = acel;      % all the vehicles using HDV model
    S(k,pos_cav+1,3)   = ud(:,k);   % the CAVs
    
    S(k+1,:,2) = S(k,:,2) + Tstep*S(k,:,3);
    S(k+1,1,2) = ed(k) + v_star;   % the velocity of the head vehicle
    S(k+1,:,1) = S(k,:,1) + Tstep*S(k,:,2);    
    
    if k == 1
        pos = S(k,:,1);
        spacing = pos(1:end-1) - pos(2:end);
        yd(:,k) = spacing';
        yd_GT(:,k) = spacing';
    else
        pos = S(k,:,1);
        spacing = pos(1:end-1) - pos(2:end);
        [tilde_vel_cur,tilde_spacing_cur] = pseudo_state_generator(S(k,2:end,2),S(k,:,1),S(k-1,2:end,2),S(k-1,:,1),PF,ID);
        yd(:,k) = tilde_spacing_cur;
        yd_GT(:,k) = spacing';
    end
end
k = k+1;
pos = S(k,:,1);
spacing = pos(1:end-1) - pos(2:end);
yd_GT(:,k) = spacing';

[tilde_vel_cur,tilde_spacing_cur] = pseudo_state_generator(S(k,2:end,2),S(k,:,1),S(k-1,2:end,2),S(k-1,:,1),PF,ID);
yd(:,k) = tilde_spacing_cur;


end
error = yd-yd_GT;
close(h_wait);

figure(1)
histogram(error(3,:))
figure(2)
histogram(error(4,:))
figure(3)
histogram(error(6,:))
figure(4)
histogram(error(7,:))
