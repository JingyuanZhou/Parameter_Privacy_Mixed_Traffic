function ControlPerformance_DeePC(I0, TotalTime, PF_enable, mix, perturb_type)

    addpath('_fcn');
    warning off;
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
    % ----------------
    % Pre-collected data
    % ----------------
    % load pre-collected data for DeeP-LCC
    i_data              = 1;    % id of the pre-collected data
    if PF_enable
        load(['_data\trajectory_data_collection\data',data_str,'_',num2str(i_data),'_noiseLevel_',num2str(acel_noise),'_PfEnable_','.mat']);
    else
        load(['_data\trajectory_data_collection\data',data_str,'_',num2str(i_data),'_noiseLevel_',num2str(acel_noise),'_PfDisable_','.mat']);
    end


    ID                  = [0,0,1,0,0,0,0];    % ID of vehicle types
                                                % 1: CAV  0: HDV
    pos_cav             = find(ID==1);          % position of CAVs
    n_vehicle           = length(ID);           % number of vehicles
    n_cav               = length(pos_cav);      % number of CAVs
    n_hdv               = n_vehicle-n_cav;      % number of HDVs
   
    %perturbation on the head vehicle
    per_type            = perturb_type;    % 1. sinuoid perturbation 2. brake perturbation 
    sine_amp            = 5;    % amplitidue of sinuoid perturbation
    brake_amp           = 5;   % brake amplitude of brake perturbation
    
    % time
    total_time          = TotalTime;              % Total Simulation Time
    Tstep               = 0.01;            % Time Step
    total_time_step     = total_time/Tstep;
    
    
    % -------------------------------------------------------------------------
    %   Formulation for DeeP-LCC
    % ------------------------------------------------------------------------- 
    % ----------------
    % Parameter setup
    % ----------------
    % Type of the controller
    controller_type     = 1;    % 1. DeeP-LCC  2. MPC 
    % Initialize Equilibrium Setup (they might be updated in the control process)
    v_star              = 15;   % Equilibrium velocity
    s_star              = 20;   % Equilibrium spacing for CAV
    % Horizon setup 
    Tini                = 20;   % length of past data in control process
    N                   = 50;   % length of future data in control process
   % Performance cost
    weight_v            = 1;    % weight coefficient for velocity error
    weight_s            = 0.5;  % weight coefficient for spacing error   
    weight_u            = 0.1;  % weight coefficient for control input
    % Setup in DeeP-LCC
    T                   = 2000; % length of data samples
    lambda_g            = 1;  % penalty on ||g||_2^2 in objective
    lambda_y            = 1e3;  % penalty on ||sigma_y||_2^2 in objective
    % Constraints
    constraint_bool     = 1;    % whether there exist constraints
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
    % equilibrium setup
    fixed_spacing_bool      = 0;    % wheter fix equilibrium spacing
                                    % 0. the equilibrium spacing will be updated via an OVM-type spacing policy
                                    % 1. fix the equilibrium spacing
                            
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
    
    Q_v         = weight_v*eye(n_vehicle);          % penalty for velocity error
    Q_s         = weight_s*eye(p_ctr-n_vehicle);    % penalty for spacing error
    Q           = blkdiag(Q_v,Q_s);                 % penalty for trajectory error
    R           = weight_u*eye(m_ctr);              % penalty for control input
    
    u           = zeros(m_ctr,total_time_step);     % control input
    x           = zeros(n_ctr,total_time_step);     % state variables
    y           = zeros(p_ctr,total_time_step);     % output variables
    pr_status   = zeros(total_time_step,1);         % problem status
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
    
    %  reference trajectory is all zeros: stabilize the system to equilibrium
    r               = zeros(p_ctr,total_time_step+N); 
    
    % ------------------
    %  Initialization: the CAVs and the head vehicle have zero control input
    % ------------------
    % initial past data in control process
    uini = zeros(m_ctr,Tini);
    eini = zeros(1,Tini);
    yini = zeros(p_ctr,Tini);
    
    for k = 1:Tini-1
    
        % calculate acceleration for the HDVs
        acel                =  HDV_dynamics(S(k,:,:),hdv_parameter) ...
                                    -acel_noise + 2*acel_noise*rand(n_vehicle,1);
        
        S(k,1,3)           = 0;               % the head vehicle
        S(k,2:end,3)       = acel;            % all the vehicles use HDV model
        S(k,pos_cav+1,3)   = uini(:,k);       % the CAV
        
        % update traffic states
        S(k+1,:,2)          = S(k,:,2) + Tstep*S(k,:,3);
        S(k+1,1,2)          = eini(k) + v_star;          % the velocity of the head vehicle
        S(k+1,:,1)          = S(k,:,1) + Tstep*S(k,:,2);
    
        % update past output data
        if PF_enable
            if k == 1
                yini(:,k)           = measure_mixed_traffic(S(k,2:end,2),S(k,:,1),ID,v_star,s_star,measure_type);
            else
                [tilde_vel_cur,tilde_spacing_cur] = pseudo_state_generator(S(k,2:end,2),S(k,:,1),S(k-1,2:end,2),S(k-1,:,1),PF,ID);
                %yini(:,k) = [(tilde_vel_cur-v_star)';(tilde_spacing_cur(pos_cav)-s_star)'];
                yini(:,k) = [(tilde_vel_cur-v_star)';(tilde_spacing_cur-s_star)'];
            end
        else
            yini(:,k)           = measure_mixed_traffic(S(k,2:end,2),S(k,:,1),ID,v_star,s_star,measure_type);
        end
        
    end
    
    k_end = k+1;
    if PF_enable
        [tilde_vel_cur,tilde_spacing_cur] = pseudo_state_generator(S(k_end,2:end,2),S(k_end,:,1),S(k_end-1,2:end,2),S(k_end-1,:,1),PF,ID);
        %yini(:,k_end) = [(tilde_vel_cur-v_star)';(tilde_spacing_cur(pos_cav)-s_star)'];
        yini(:,k_end) = [(tilde_vel_cur-v_star)';(tilde_spacing_cur-s_star)'];
    else
        yini(:,k_end)           = measure_mixed_traffic(S(k_end,2:end,2),S(k_end,:,1),ID,v_star,s_star,measure_type);
    end
    
    % update data in u,e,y
    u(:,1:Tini) = uini;
    e(:,1:Tini) = eini;
    y(:,1:Tini) = yini;
    
    % For MPC, which might have infeasible cases
    previous_u_opt = 0; 

    % ------------------
    %  Continue the simulation
    % ------------------
    for k = Tini:total_time_step-1
        % calculate acceleration for the HDVs
        acel         =  HDV_dynamics(S(k,:,:),hdv_parameter) ...
                        -acel_noise + 2*acel_noise*rand(n_vehicle,1);
        S(k,2:end,3) = acel;     % all the vehicles use HDV model
    
        
        if mix 
            if constraint_bool
                [u_opt,y_opt,pr] = qp_DeeP_LCC(Up,Yp,Uf,Yf,Ep,Ef,uini,yini,eini,Q,R,r(:,k:k+N-1),...
                    lambda_g,lambda_y,u_limit,s_limit);
            else
                [u_opt,y_opt,pr] = qp_DeeP_LCC(Up,Yp,Uf,Yf,Ep,Ef,uini,yini,eini,Q,R,r(:,k:k+N-1),...
                    lambda_g,lambda_y);
            end
            % one-step implementation in receding horizon manner
            u(:,k) = u_opt(1:m_ctr,1);
            % update accleration for the CAV
            S(k,pos_cav+1,3)   = u(:,k);
            % judge whether AEB (automatic emergency braking, which is implemented in the function of 'HDV_dynamics') commands to brake
            brake_vehicle_ID = find(acel==dcel_max);                % the vehicles that need to brake
            brake_cav_ID     = intersect(brake_vehicle_ID,pos_cav); % the CAVs that need to brake
            if ~isempty(brake_cav_ID)
                S(k,brake_cav_ID+1,3) = dcel_max;
            end
            % record problem status
            pr_status(k) = pr;
        end
      
        % update traffic states
        S(k+1,:,2) = S(k,:,2) + Tstep*S(k,:,3);
        % perturbation for the head vehicle
        switch per_type
            case 1
                S(k+1,1,2) = v_star + sine_amp*sin(2*pi/(10/Tstep)*(k-Tini));
                S(k+1,:,1) = S(k,:,1) + Tstep*S(k,:,2);
            case 2
                if (k-Tini)*Tstep < brake_amp/2
                    S(k+1,1,3) = -brake_amp;
                elseif (k-Tini)*Tstep < (brake_amp/2)*2
                    S(k+1,1,3) = brake_amp;
                else
                    S(k+1,1,3) = 0;
                end
                S(k+1,:,2) = S(k,:,2) + Tstep*S(k,:,3);
                S(k+1,:,1) = S(k,:,1) + Tstep*S(k,:,2);
        end
        
        % update equilibrium setup for the CAVs
        % v_star = mean(S(k-Tini+1:k,1,2));               % update v_star
        % if ~fixed_spacing_bool        
        %     s_star = acos(1-v_star/30*2)/pi*(35-5) + 5; % update s_star
        % 
        % end
        
        % update past data in control process
        uini = u(:,k-Tini+1:k);
        % the output needs to be re-calculated since the equilibrium might have been updated
        for k_past = k-Tini+1:k
        % Record output
            if PF_enable
                if k_past>1
                    [tilde_vel_cur,tilde_spacing_cur] = pseudo_state_generator(S(k_past,2:end,2),S(k_past,:,1),S(k_past-1,2:end,2),S(k_past-1,:,1),PF,ID);
                    %y(:,k_past) = [(tilde_vel_cur-v_star)';(tilde_spacing_cur(pos_cav)-s_star)'];
                    y(:,k_past) = [(tilde_vel_cur-v_star)';(tilde_spacing_cur-s_star)'];
                else
                    y(:,k_past) = measure_mixed_traffic(S(k_past,2:end,2),S(k_past,:,1),ID,v_star,s_star,measure_type);
                end
            else
                y(:,k_past) = measure_mixed_traffic(S(k_past,2:end,2),S(k_past,:,1),ID,v_star,s_star,measure_type);
            end
            e(k_past)   = S(k_past,1,2) - v_star;
        end
        yini = y(:,k-Tini+1:k);
        eini = S(k-Tini+1:k,1,2) - v_star;
      
        fprintf('Simulation number: %d  |  process... %2.2f%% \n',i_data,(k-Tini)/total_time_step*100);
        %fprintf('Fixed Spacing: %d',fixed_spacing_bool);
        %fprintf('Current spacing of the first CAV: %4.2f \n',S(k,3,1)-S(k,4,1));
      
    end
    k_end = k+1;
    y(:,k_end) = measure_mixed_traffic(S(k_end,2:end,2),S(k_end,:,1),ID,v_star,s_star,measure_type);
    
    save(['data\S_mix_',num2str(mix),'_PF_',num2str(PF_enable),'_Controller_2_PerturbType_', num2str(perturb_type),'.mat'],"S")
end
 



