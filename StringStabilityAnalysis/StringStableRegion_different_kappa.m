clear; clc;
global alpha1;
global alpha2;
global alpha3;
global pseudo_alpha1;
global pseudo_alpha2;
global pseudo_alpha3;
global n;
global m;
global u_ls;
global k_ls;

% number of the following HDVs
n = 2;
% number of the preceding HDVs
m = 2;

DriverDynamics = 1;
PseudoDynamics = 1;

k = -10:0.1:10;
u = -10:0.1:10;

s_star = 20;

switch DriverDynamics
    case 1
        alpha = 0.6;
        beta = 0.9;
    case 2
        alpha = 0.4;
        beta = 0.6;
end

switch PseudoDynamics
    case 1
        pseudo_alpha = 0.2;
        pseudo_beta = 0.3;
    case 2
        pseudo_alpha = 0.8;
        pseudo_beta = 1.2;
    case 4
        pseudo_alpha = 0.6;
        pseudo_beta = 0.9;
end

% Other OVM parameters
v_max  = 30;
s_st   = 5;
s_go   = 35;
alpha1 = alpha*v_max/2*pi/(s_go-s_st)*sin(pi*(s_star-s_st)/(s_go-s_st));
alpha2 = alpha+beta;
alpha3 = beta;

pseudo_alpha1 = pseudo_alpha*v_max/2*pi/(s_go-s_st)*sin(pi*(s_star-s_st)/(s_go-s_st));
pseudo_alpha2 = pseudo_alpha+pseudo_beta;
pseudo_alpha3 = pseudo_beta;

SS_bool = zeros(length(k),length(u));

h=waitbar(0,'please wait');
TestNumber = length(k)*length(u);
iTest = 1;

for ik0 = 1:length(k)
    for ik1 = 1:length(u)
        control_gain_u = u(ik0);
        control_gain_k = k(ik1);
        
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
       '_DriverDynamics_',num2str(DriverDynamics),'_PseudoDynamics_',num2str(PseudoDynamics),'.mat']);

function [Amplitude] = minus_HeadTail_Tf(w)
    global alpha1;
    global alpha2;
    global alpha3;
    global pseudo_alpha1;
    global pseudo_alpha2;
    global pseudo_alpha3;
    global n;
    global m;
    global u_ls;
    global k_ls;
    
    Other_HDVs = -abs(((alpha3*1i*w+alpha1)/(-w^2+alpha2*1i*w+alpha1))^(n+m));
    preceding_part = 0;
    following_part = 0;
    K_prime = ((pseudo_alpha3*1i*w+ pseudo_alpha1)/(-w^2+pseudo_alpha2*1i*w+pseudo_alpha1));
    K = ((alpha3*1i*w+ alpha1)/(-w^2+alpha2*1i*w+alpha1));
    for i = 1:m
        preceding_part = preceding_part + (u_ls(i)*(1-K_prime) + k_ls(i)*1i*w*K_prime)*K;
    end
    for i = m+1:m+n
        following_part = following_part + (u_ls(i)*(1-K_prime) - k_ls(i)*1i*w*K_prime)*K;
    end
    G = abs(((alpha3*1i*w+alpha1) + preceding_part)/((-w^2+alpha2*1i*w+alpha1)-following_part));
    Amplitude = Other_HDVs*G;
end








