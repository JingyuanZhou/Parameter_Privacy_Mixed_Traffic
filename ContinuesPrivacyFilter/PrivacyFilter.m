classdef PrivacyFilter
    properties
        % Both tilde_kappa and kappa are distributions with sets
        % and corresponding probs (alpha,beta,s_st,s_go)
        kappa
        tilde_kappa 
        tilde_x 
        v_star
        s_star
        E
        kappa_index
        p_kappa
        p_kappa_tilde
        transfer_kernel
        kappa_choice
        x_last
    end

    methods
        function item = PrivacyFilter(kappa_index, N_tilde_kappa)
            item.kappa = [0.45,0.65,5,35;0.45,0.65,5,35;0.65,0.85,5,35;0.6,0.8,5,35;0.65,0.90,5,35]; %[0.6,0.9,5,35];
            
            ta = linspace(0.1, 1, N_tilde_kappa); % discreted tilde_a
            tb = linspace(0.1, 1, N_tilde_kappa); % discreted tilde_b
            s_st = 5;
            s_go = 35;
            item.tilde_kappa = [];
            for alpha = ta
                for beta = tb
                    item.tilde_kappa = [item.tilde_kappa; alpha, beta, s_st, s_go];
                end
            end

            item.tilde_x = NaN;
            item.v_star = 15;
            item.s_star = 20; 
            item.kappa_index = kappa_index;
            item.x_last = -9999;
        end

        function obj = Randomizer(obj, I0)
            select_kappa = obj.kappa(obj.kappa_index, 1:2);
            if_privacy_bound = 0;
            if_lips = false;
            if if_lips
                path = ['model/model_',num2str(I0),'_low_privacy.pth'];
            else
                path = ['model/model_',num2str(I0),'_low_privacy.pth'];
            end
            obj.transfer_kernel = NN_test(path, select_kappa,I0,if_privacy_bound, if_lips);
        end

        function [obj,x_inv_tilde] = NonlinearTransformation(obj,x_cur,x_last_temp,mu,sigma,v_head)
            idx = 1:length(obj.transfer_kernel);
            chosen_idx = randsample(idx, 1, true, obj.transfer_kernel);
            obj.kappa_choice = obj.tilde_kappa(chosen_idx,:);   

            if obj.x_last == -9999
                obj.x_last = x_last_temp;
            end

            v_max = 30;
            dt = 0.01;
            dv = obj.OVM_Iter(obj.kappa_choice,obj.x_last,v_head,v_max);
            x_cur_tilde(2) = obj.x_last(2) + dv*dt;
            x_cur_tilde(1) = obj.x_last(1) + (v_head - x_cur_tilde(2))*dt;
            for len_states = 1:length(x_cur)
                p = normcdf(x_cur(len_states),mu(len_states),sigma(len_states));
                mu_tilde = x_cur_tilde(len_states);
                sigma_tilde = 0.1; %test
                x_inv_tilde(len_states) = norminv(p,mu_tilde,sigma_tilde);
            end

            obj.x_last = x_inv_tilde;
        end

        function F = OVM_Iter(obj,iter_kappa,x,v_head,v_max)
            % calculate V(s)
            if x(1) <= iter_kappa(3)
                V = 0;
            elseif x(1) >= iter_kappa(4)
                V = v_max;
            else
                V = (v_max/2)*(1-cos(pi*(x(1)-iter_kappa(3))/(iter_kappa(4)-iter_kappa(3))));
            end
            % calculate derivative
            F = iter_kappa(1)*(V-x(2))+iter_kappa(2)*(v_head-x(2));
        end
    end
end







