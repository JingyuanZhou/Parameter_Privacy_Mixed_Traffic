function ControlPerformance_Linear(I0, TotalTime, PF_enable, mix, perturb_type)

    addpath('_fcn');
    warning off;
    
    ID                  = [0,0,1,0,0,0,0];    % ID of vehicle types
                                                % 1: CAV  0: HDV
    pos_cav             = find(ID==1);          % position of CAVs
    n_vehicle           = length(ID);           % number of vehicles
    n_cav               = length(pos_cav);      % number of CAVs
    n_hdv               = n_vehicle-n_cav;      % number of HDVs
    
    K                   = zeros(n_cav,2*n_vehicle);
    for i = 1:n_cav
        if i == 1
            K(i,1:n_vehicle) = [0.05,0.3,-0.6,0.15,0.1,0.08,0.05];
            K(i,n_vehicle+1:2*n_vehicle) = [-0.01,-0.05,0.2,-0.1,-0.05,-0.01,-0.01];
        end
        %K(i,(pos_cav(i)*2-1):(pos_cav(i)*2+4))              = [-0.5,0.05,0.05,0.1,-0.2,-0.1];
    end
    %perturbation on the head vehicle
    per_type            = perturb_type;    % 1. sinuoid perturbation 2. brake perturbation 
    sine_amp            = 5;    % amplitidue of sinuoid perturbation
    brake_amp           = 5;   % brake amplitude of brake perturbation
    
    % time
    total_time          = TotalTime;              % Total Simulation Time
    Tstep               = 0.01;            % Time Step
    total_time_step     = total_time/Tstep;
    
    % ----------------
    % HDV setup
    % ----------------
    % Type for HDV car-following model
    hdv_type            = 1;    % 1. OVM   2. IDM
    % Parameter setup for HDV 
    data_str            = '3';  % 1. random ovm  2. manual heterogeneous ovm  3. homogeneous ovm
    switch hdv_type
        case 1
            load(['_data/hdv_ovm_',data_str,'.mat']);
        case 2
            load('_data/hdv_idm.mat');      
    end
    % Uncertainty for HDV acceleration
    acel_noise          = 0.1;  % A white noise signal on HDV's acceleration
    
    % -------------------------------------------------------------------------
    %   Formulation for DeeP-LCC
    % ------------------------------------------------------------------------- 
    % ----------------
    % Parameter setup
    % ----------------
    % Initialize Equilibrium Setup (they might be updated in the control process)
    v_star              = 15;   % Equilibrium velocity
    s_star              = 20;   % Equilibrium spacing for CAV
    % Setup in DeeP-LCC
    T                   = 2000; % length of data samples
    % Constraints
    acel_max            = 5;    % maximum acceleration
    dcel_max            = -5;   % minimum acceleration (maximum deceleration)
    spacing_max         = 40;   % maximum spacing
    spacing_min         = 5;    % minimum spacing
    u_limit             = [dcel_max,acel_max];
    s_limit             = [spacing_min,spacing_max]-s_star;
    % what signals are measurable (for output definition)
    measure_type        = 2;    % 1. Only the velocity errors of all the vehicles are measurable;
                                % 2. All the states, including velocity error and spacing error are measurable;
                                % 3. Velocity error and spacing error of the CAVs are measurable, 
                                %    and the velocity error of the HDVs are measurable.
    
    
    % ----------------
    % Process parameters
    % ----------------
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
    
    
    u           = zeros(m_ctr,total_time_step);     % control input
    x           = zeros(n_ctr,total_time_step);     % state variables
    y           = zeros(p_ctr,total_time_step);     % output variables
    e           = zeros(1,total_time_step);         % external input
    
    % Initialization: Privacy filter
    if PF_enable
        N_tilde_kappa = 10;
        PF = cell(1,n_vehicle);
        for i=1:n_vehicle
            if ID(i) == 0 && PF_enable
                PF{i} = PrivacyFilter(1,N_tilde_kappa);
                PF{i} = PF{i}.Randomizer(I0);
            else
                PF{i} = 0;
            end
        end
    end
    
    
    
    % -------------------------------------------------------------------------
    %   Simulation
    %--------------------------------------------------------------------------
    % Mixed traffic states
    % S(time,vehicle id,state variable), in state variable: 1. position; 2. velocity; 3. acceleration
    S               = zeros(total_time_step,n_vehicle+1,3); 
    S(1,1,1)        = 0;
    for i = 2 : n_vehicle+1
        S(1,i,1)    = S(1,i-1,1) - hdv_parameter.s_star(i-1);
    end
    S(1,:,2)        = v_star * ones(n_vehicle+1,1);
    
    
    % -------------------------------------------------------------------------
    %   Experiment starts here
    %--------------------------------------------------------------------------
    
    
    % ------------------
    %  Continue the simulation
    % ------------------
    for k = 1:total_time_step-1
        % calculate acceleration for the HDVs
        acel         =  HDV_dynamics(S(k,:,:),hdv_parameter) ...
                         -acel_noise + 2*acel_noise*rand(n_vehicle,1);
        %acel         =  HDV_dynamics(S(k,:,:),hdv_parameter);
        S(k,2:end,3) = acel;     % all the vehicles use HDV model
    
        if mix
            if PF_enable && k>1
                [tilde_vel_cur,tilde_spacing_cur,PF] = pseudo_state_generator(S(k,2:end,2),S(k,:,1),S(k-1,2:end,2),S(k-1,:,1),PF,ID);
                y(:,k) = [(tilde_vel_cur-v_star)';(tilde_spacing_cur-s_star)'];
            else
                y(:,k) = measure_mixed_traffic(S(k,2:end,2),S(k,:,1),ID,v_star,s_star,measure_type);
            end
            % one-step implementation in receding horizon manner

            for i = 1:n_cav
                u(i,k) = K(i,:) * y(:,k);
            end
            % update accleration for the CAV
            S(k,pos_cav+1,3)   = u(:,k);
            % judge whether AEB (automatic emergency braking, which is implemented in the function of 'HDV_dynamics') commands to brake
            brake_vehicle_ID = find(acel==dcel_max);                % the vehicles that need to brake
        end
      
        % update traffic states
        S(k+1,:,2) = S(k,:,2) + Tstep*S(k,:,3);
        % perturbation for the head vehicle
        switch per_type
            case 1
                S(k+1,1,2) = v_star + sine_amp*sin(2*pi/(10/Tstep)*k);
                S(k+1,:,1) = S(k,:,1) + Tstep*S(k,:,2);
            case 2
                if k*Tstep < brake_amp/2
                    S(k+1,1,3) = -brake_amp;
                elseif k*Tstep < (brake_amp/2)*2
                    S(k+1,1,3) = brake_amp;
                else
                    S(k+1,1,3) = 0;
                end
                S(k+1,:,2) = S(k,:,2) + Tstep*S(k,:,3);
                S(k+1,:,1) = S(k,:,1) + Tstep*S(k,:,2);
        end
      
    end
    save(['data\S_mix_',num2str(mix),'_PF_',num2str(PF_enable),'_Controller_1_PerturbType_', num2str(perturb_type),'.mat'],"S")
end
 



