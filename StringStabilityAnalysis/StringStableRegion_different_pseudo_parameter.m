clear; clc;
global alpha1;
global alpha2;
global alpha3;
global pseudo_alpha1;
global pseudo_alpha2;
global pseudo_alpha3;
global n;
global m;
global n_sensing;
global m_sensing;
global cav_index;
global pf_index;
global u_ls;
global k_ls;

% number of the following HDVs
n = 2;
n_sensing = 2;
% number of the preceding HDVs
m = 2;
m_sensing = 2;

cav_index = n_sensing + 1;
pf_index = 5;

DriverDynamics = 1;
ControlGains = 1;

pseudo_dynamics_alpha = 0:0.01:1.2;
pseudo_dynamics_beta = 0:0.01:1.2;

s_star = 20;

switch DriverDynamics
    case 1
        alpha = 0.6;
        beta = 0.9;
    case 2
        alpha = 0.4;
        beta = 0.6;
end

switch ControlGains
    case 1
        u_ls = [-0.1,-0.15,0.2,-0.15,-0.1];
        k_ls = [0.25,0.35,-0.5,0.35,0.25];
    case 2
        u_ls = [-0.1,-0.15,0.2,-0.15,-0.1];
        k_ls = [0.2,0.3,-0.4,0.3,0.2];
end

% Other OVM parameters
v_max  = 30;
s_st   = 5;
s_go   = 35;
alpha1 = alpha*v_max/2*pi/(s_go-s_st)*sin(pi*(s_star-s_st)/(s_go-s_st));
alpha2 = alpha+beta;
alpha3 = beta;

SS_bool = zeros(length(pseudo_dynamics_alpha),length(pseudo_dynamics_beta));

h=waitbar(0,'please wait');
iTest = 1;
TestNumber = length(pseudo_dynamics_alpha)*length(pseudo_dynamics_beta);

for ik0 = 1:length(pseudo_dynamics_alpha)
    for ik1 = 1:length(pseudo_dynamics_beta)
        pseudo_alpha = pseudo_dynamics_alpha(ik0);
        pseudo_beta = pseudo_dynamics_beta(ik1);
        pseudo_alpha1 = pseudo_alpha*v_max/2*pi/(s_go-s_st)*sin(pi*(s_star-s_st)/(s_go-s_st));
        pseudo_alpha2 = pseudo_alpha+pseudo_beta;
        pseudo_alpha3 = pseudo_beta;
        
        % calculate the abs of the transfer function
        [x,fval] = fminbnd(@minus_HeadTail_Tf,1e-8,100);
        fval = -fval;
        if fval<=1
            
            SS_bool(ik0,ik1) = 1;
            
        end
        str=['DriverDynamics=',num2str(DriverDynamics),...
            '; Processing...',num2str(iTest/TestNumber*100),'%'];
        waitbar(iTest/TestNumber,h,str);
        iTest = iTest+1;
    end
end
close(h);

save(['data\','SSRegion_n_',num2str(n),'_m_',num2str(m),...
       '_DriverDynamics_',num2str(DriverDynamics),'_ControlGains_',num2str(ControlGains),'_PrivacyFilter_',num2str(pf_index),'.mat']);

function [Amplitude] = minus_HeadTail_Tf(w)
    global alpha1;
    global alpha2;
    global alpha3;
    global pseudo_alpha1;
    global pseudo_alpha2;
    global pseudo_alpha3;
    global n;
    global m;
    global n_sensing;
    global m_sensing;
    global cav_index;
    global pf_index;
    global u_ls;
    global k_ls;
    
    Other_HDVs = -abs(((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^(n+m));
    K_prime = ((pseudo_alpha3*1i*w+ pseudo_alpha1)/(-w^2+pseudo_alpha2*1i*w+pseudo_alpha1));
    K = ((alpha3*1i*w+ alpha1)/(-w^2+alpha2*1i*w+alpha1));
    % W1 and W3 are for the preceding vehicles, W2 and W4 are for following
    % vehicles
    W1 = 0;
    W2 = 0;
    W3 = 0;
    W4 = 0;

    R = (w^2 - k_ls(cav_index)*w-u_ls(cav_index))/w;
    for i = 1:m_sensing
        if i == pf_index
            W3 = W3 + u_ls(i)*(1-K_prime)/(w*(K^(cav_index-i))) + k_ls(i)*K_prime/(K^(cav_index-i));
        else
            W1 = W1 + u_ls(i)*(1-K)/(w*(K^(cav_index-i))) + k_ls(i)/(K^(cav_index-i-1));
        end
    end
    for i = m_sensing+1:m_sensing+n_sensing+1
        if i == pf_index
            W4 = W4 + u_ls(i)*(1-K_prime)*(K^(i-cav_index-1))/w + k_ls(i)*K_prime*K^(i-cav_index-1);
        else
            W2 = W2 + u_ls(i)*(1-K)*(K^(i-cav_index-1))/w + k_ls(i)*K^(i-cav_index);
        end
    end

    G = abs((W2+W4)/(R+W1+W3));
    Amplitude = Other_HDVs*G;
end








