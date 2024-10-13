function [S_LCC,S_tilde] = AN_LCC(noise, TotalTime, vehicle_index, PerturbedType)

    % -------------------------------------------------------------------------
    %   Parameter setup
    %--------------------------------------------------------------------------
    % mode of te LCC system
    FD_bool = 0;       % 0. CF-LCC; 1. FD-LCC
    
    m       = 0;       % number of preceding vehicles
    n       = 2;      % number of following vehicles
    PerturbedID = 0;   % perturbation on vehicle
                       % 0. Head vehicle
                       % 1 - m. Preceding vehicles
                       % m+2 - n+m+1. Following vehicles
    % PerturbedType = 1; % perturbation type
                       % 1:Sine-wave Perturbation;  2: Braking;  3: Random
    brake_amp           = 5;   % brake amplitude of brake perturbation                       
    % ------------------------------------------
    % Parameters in the car-following model
    % ------------------------------------------
    alpha = 0.45; % Driver Model: OVM
    beta  = 0.65;
    s_st  = 5;
    s_go  = 35;
    
    % Traffic equilibrium
    v_star   = 15;   % Equilibrium velocity
    acel_max = 2;
    dcel_max = -5;
    v_max    = 30;
    s_star   = acos(1-v_star/v_max*2)/pi*(s_go-s_st)+s_st; % Equilibrium spacing
    
    % linearized model
    alpha1 = alpha*v_max/2*pi/(s_go-s_st)*sin(pi*(s_star-s_st)/(s_go-s_st));
    alpha2 = alpha+beta;
    alpha3 = beta;
    
    % Simulation length
    Tstep     = 0.01;
    NumStep   = TotalTime/Tstep;

    % Car-following noise
    acel_noise          = 0.01;
    % ------------------------------------------------------------------------
    % Experiment starts here
    % ------------------------------------------------------------------------
    % Pravacy Filter works or not
    %   PF_enable  = 0: Pravacy Filter not work
    %   PF_enable  = 1: Pravacy Filter work
    
    tic
    for AN_enable = 0:1 
        switch AN_enable  
            case 1
                ActuationTime = 0; % Privacy sizeFilter works or not
            case 0
                ActuationTime = 99999;
        end
        
        % -----------------------------------------------
        % Define state variables
        % -----------------------------------------------
        % Initial State for each vehicle
        S     = zeros(NumStep,m+n+2,3);
        if AN_enable
            S_tilde = zeros(NumStep,m+n+2,3);
        end
    
        dev_s = 0;
        dev_v = 0;
        co_v  = 1.0;
        v_ini = co_v*v_star; %Initial velocity
        % from - dev to dev
        S(1,:,1) = linspace(0,-(m+n+1)*s_star,m+n+2)' + (rand(m+n+2,1)*2*dev_s - dev_s);
        % The vehicles are uniformly distributed on the straight road with a random deviation
        S(1,:,2) = v_ini * ones(m+n+2,1) + (rand(m+n+2,1)*2*dev_v-dev_v);
        
        % meaning of parameters
        % 1:        head vehicle
        % 2-(m+1):  Preceding vehicles
        % m+2:      CAV
        % (m+3)-(m+n+2): Following vehicles
        
        X = zeros(2*(m+n+1),NumStep);
        u = zeros(NumStep,1);               % 0. HDV  1. CAV
        V_diff = zeros(NumStep,m+n+1);      % Velocity Difference
        D_diff = zeros(NumStep,m+n+1);      % Following Distance
        
        % ---------------------------------------------------------
        % LCC controller: the following choice is used in our paper
        % ---------------------------------------------------------
        K = zeros(1,2*(n+1));
        if FD_bool
            K(1:6) = [0,-0.5,-0.2,0.05,-0.1,0.05];
        else
            K(1:6) = [0.1,-0.5,-0.2,0.05,-0.1,0.05];
        end
    
        % ---------------------------------------------------------
        % Simulation starts here
        % ---------------------------------------------------------
        for k = 1:NumStep - 1
            % Update acceleration
            V_diff(k,:) = S(k,1:(end-1),2) - S(k,2:end,2); 
            D_diff(k,:) = S(k,1:(end-1),1) - S(k,2:end,1);
            cal_D = D_diff(k,:); % For the boundary of Optimal Veloicity Calculation
            for i = 1:m+n+1
                if cal_D(i) > s_go
                    cal_D(i) = s_go;
                elseif cal_D(i) < s_st
                    cal_D(i) = s_st;
                end
            end
            
            % nonlinear OVM Model
            % V_d = v_max/2*(1-cos(pi*(h-h_st)/(h_go-h_st)));
            % a2 = alpha*(V_d-v2)+beta*(v1-v2);
            acel = alpha*(v_max/2*(1-cos(pi*(cal_D-s_st)/(s_go-s_st))) - S(k,2:end,2))+beta*V_diff(k,:) -acel_noise + 2*acel_noise*rand(1,n+1);
            acel(acel>acel_max) = acel_max;
            acel(acel<dcel_max) = dcel_max;
            
            S(k,2:end,3) = acel;
            S(k,1,3)     = 0; % the preceding vehicle
            
            % Perturbation type
            switch PerturbedType
                case 1     % sine wave
                    P_A = 3;
                    P_T = 5;
                    if k*Tstep>0 && k*Tstep<P_T
                        S(k,PerturbedID+1,3) = P_A*cos(2*pi/P_T*(k*Tstep-20));
                    end
                case 2     % braking
                    if k*Tstep < brake_amp/2
                        S(k+1,1,3) = -brake_amp;
                    elseif k*Tstep < (brake_amp/2)*2
                        S(k+1,1,3) = brake_amp;
                    else
                        S(k+1,1,3) = 0;
                    end
                case 3
                    S(k,PerturbedID+1,3) = 6*(rand(1)-0.5);
            end
            
            X(1:2:end,k) = reshape(D_diff(k,:),m+n+1,1) - s_star;
            X(2:2:end,k) = reshape(S(k,2:end,2),m+n+1,1) - v_star;
    
            % States transformation
            if AN_enable
                index = vehicle_index;
                %for index = 2:(n+2)
                x_tilde = [D_diff(k,index-1),S(k,index,2)] + noise*(randn([1,2])-0.5);
                X(index*2-3,k) = x_tilde(1) - s_star;
                X(index*2-2,k) = x_tilde(2) - v_star;

                S_tilde(k,index,1) = x_tilde(1);
                S_tilde(k,index,2) = x_tilde(2);
                S_tilde(k,index-1,2) = S(k,index-1,2);
                
                S_tilde(k,index+1,1) = D_diff(k,index);
                S_tilde(k,index+1,2) = S(k,index+1,2);
                %end
            end
                 
            %
            % SD as ADAS to prevent crash
            % 
            acel_sd     = (S(k,2:end,2).^2-S(k,1:(end-1),2).^2)./2./D_diff(k,:);
            acel_temp   = S(k,:,3);
            acel_temp(acel_sd>abs(dcel_max)) = dcel_max;
            S(k,:,3)    = acel_temp;
            
            S(k+1,:,2) = S(k,:,2) + Tstep*S(k,:,3);
            S(k+1,:,1) = S(k,:,1) + Tstep*S(k,:,2);
        end
        
        
       %% Data Recording
        S_LCC = S;
    end
    tsim = toc;
    
end
 



