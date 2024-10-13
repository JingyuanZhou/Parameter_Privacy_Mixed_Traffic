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
        function item = PrivacyFilter(kappa_index,v_star)
            item.kappa = [0.6,0.9,5,35;0.5,0.95,5,35;0.65,0.85,5,35;0.6,0.8,5,35;0.65,0.90,5,35]; %[0.6,0.9,5,35];
            item.tilde_kappa = [0.5,0.8,5,35;0.2,0.4,5,35]; 
            item.p_kappa = [0.2,0.2,0.2,0.2,0.2]; 
            item.p_kappa_tilde = [0.5,0.5];
            item.tilde_x = NaN;
            item.v_star = 15;
            item.s_star = 20; 
            item.kappa_index = kappa_index;
            item.x_last = -9999;
        end

        function obj = Randomizer(obj,I0)
            fun = @objfun;

            lenth_p_kappa = length(obj.p_kappa);
            lenth_p_kappa_tilde = length(obj.p_kappa_tilde);

            function f = objfun(x)
                f = 0;
                cnt = 1;
                for i = 1:lenth_p_kappa
                    for j = 1:lenth_p_kappa_tilde
                        f = f + obj.E(i,j)*x(cnt)*obj.p_kappa(i);
                        cnt=cnt+1;
                    end
                end
            end

            function [c,ceq] = MutualInformation(x)
                %I0 = 0.5;
                c = -I0;
                cnt = 1;
                for i = 1:lenth_p_kappa
                    for j = 1:lenth_p_kappa_tilde
                        %c = c + x(cnt)*obj.p_kappa(i)*log2(x(cnt)/obj.p_kappa_tilde(i));
                        p_kappa_tilde_true = 0;
                        for k = 1:lenth_p_kappa
                            if cnt <= 5
                                p_kappa_tilde_true = p_kappa_tilde_true + 0.2*x(k);
                            else
                                p_kappa_tilde_true = p_kappa_tilde_true + 0.2*x(k+5);
                            end
                        end
                        c = c + x(cnt)*obj.p_kappa(i)*log2(x(cnt)/(p_kappa_tilde_true));
                        cnt=cnt+1;
                    end
                end
                ceq = [];
            end
            
            x0 = [0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1]';
            A = -eye(lenth_p_kappa*lenth_p_kappa_tilde);
            b = zeros(lenth_p_kappa*lenth_p_kappa_tilde,1);
            Aeq = [1 1 0 0 0 0 0 0 0 0;0 0 1 1 0 0 0 0 0 0;0 0 0 0 1 1 0 0 0 0;0 0 0 0 0 0 1 1 0 0;0 0 0 0 0 0 0 0 1 1];
            beq = ones(lenth_p_kappa,1);
            lb = [];
            ub = [];
            [obj.transfer_kernel,~] = fmincon(fun,x0,A,b,Aeq,beq,lb,ub,@MutualInformation);
            
            % average
            %obj.kappa_choice = obj.transfer_kernel((obj.kappa_index-1)*lenth_p_kappa_tilde+1 : (obj.kappa_index-1)*lenth_p_kappa_tilde+lenth_p_kappa_tilde)'*obj.tilde_kappa;

        end

        function [obj,x_inv_tilde] = NonlinearTransformation(obj,x_cur,x_last,mu,sigma,v_head)
            random_choose = rand(1);
            if random_choose < obj.transfer_kernel(1)
                obj.kappa_choice = obj.tilde_kappa(1,:);
            else
                obj.kappa_choice = obj.tilde_kappa(2,:);
            end
            v_max = 30;
            dt = 0.01;
            dv = obj.OVM_Iter(obj.kappa_choice,x_last,v_head,v_max);
            x_cur_tilde(2) = x_last(2) + dv*dt;
            x_cur_tilde(1) = x_last(1) + (v_head - x_cur_tilde(2))*dt;
            for len_states = 1:length(x_cur)
                p = normcdf(x_cur(len_states),mu(len_states),sigma(len_states));
                mu_tilde = x_cur_tilde(len_states);
                sigma_tilde = 0.1; %test
                x_inv_tilde(len_states) = norminv(p,mu_tilde,sigma_tilde);
            end
            % for testing
            %disp(x_inv_tilde)
            %disp(obj.kappa_choice)
        end

        function obj = MonteCarlo(obj,TotalStep,scenario)
            if scenario == 1
                if exist('data\E_sine.mat','file')
                    data = load('data\E_sine.mat','E');
                    obj.E = data.E;
                    return
                end
            elseif scenario == 2
                if exist('data\E_bracking.mat','file')
                    %data = load('data\E_bracking.mat','E');
                    data = load('data\E_sine.mat','E');
                    obj.E = data.E;
                    return
                end
            end
            MC_iter = 5000; %iteration number
            num_kappa = size(obj.kappa,1);
            num_tilde_kappa = size(obj.tilde_kappa,1);
            obj.E = zeros(num_kappa,num_tilde_kappa);

            % iterate for kappa, tilde_kappa, montecarlo
            for i = 1:num_kappa
                for j = 1:num_tilde_kappa

                    E_ij = 0;
                    for mc_iter = 1:MC_iter
                        mc_init_state = [20 - 2 + rand(1)*4, 15 - 2 + rand(1)*4];%[rand(1)*40,rand(1)*30];
                        mc_x = mc_init_state;
                        mc_tilde_x = mc_init_state;
                        v_head = 15;
                        v_max = 30;
                        dt = 0.01;
                        E_iter = 0;
                        brake_amp = 5;
                        for k = 1:TotalStep
                            if scenario == 1
                                P_T = TotalStep;
                                P_A = 6 +  4*randn(1)- 2;
                                if k*dt>0 && k*dt<P_T
                                    a = P_A*cos(2*pi/P_T*(k*dt-20));
                                end
                            elseif scenario == 2
                                if k*dt>0 && k*dt<brake_amp/2
                                    a = -brake_amp;
                                elseif k*dt>=brake_amp/2 && k*dt<brake_amp
                                    a = brake_amp;
                                end
                            end
                            v_head = v_head + dt*a;
        
                            acel = obj.OVM_Iter(obj.kappa(i,:),mc_x,v_head,v_max);
                            acel_tilde = obj.OVM_Iter(obj.tilde_kappa(j,:),mc_tilde_x,v_head,v_max);
            
                            mc_x(2) = mc_x(2) + acel*dt;
                            mc_x(1) = mc_x(1) + (v_head - mc_x(2))*dt;
                            mc_tilde_x(2) = mc_tilde_x(2) + acel_tilde*dt;
                            mc_tilde_x(1) = mc_tilde_x(1) + (v_head - mc_tilde_x(2))*dt;
                            
                            diff_x = mc_x - mc_tilde_x;
                            E_iter = E_iter + sum(diff_x.^2);
                        end
                        E_ij = E_ij + E_iter/TotalStep;
                    end
                    obj.E(i,j) = E_ij/MC_iter;
                end
            end
            E = obj.E;
            if scenario == 1
                save('data\E_sine.mat','E')
            elseif scenario == 2
                save('data\E_bracking.mat','E')
            end
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

        function result = MSE_distoration(obj)
            result = sum(((obj.kappa_choice-obj.kappa(1,:))./obj.kappa(1,:)).^2);
        end
    end
end







