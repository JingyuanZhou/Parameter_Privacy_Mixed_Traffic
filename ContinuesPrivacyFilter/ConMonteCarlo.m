classdef ConMonteCarlo
    properties
        % Both tilde_kappa and kappa are distributions with sets
        % and corresponding probs (alpha,beta,s_st,s_go)
        kappa
        tilde_kappa
        tilde_x
        v_star
        s_star
        E
        p_kappa
        transfer_kernel
    end

    methods
        function item = ConMonteCarlo(kappa,p_kappa,tilde_kappa)
            item.kappa = kappa; 
            item.tilde_kappa = tilde_kappa; 
            item.p_kappa = p_kappa; %1
            item.tilde_x = NaN;
            item.v_star = 15;
            item.s_star = 20; 
        end

        function obj = MonteCarlo(obj,TotalStep)
            MC_iter = 5000; %iteration number
            num_kappa = size(obj.kappa,1);
            num_tilde_kappa = size(obj.tilde_kappa,1);
            obj.E = zeros(num_kappa,num_tilde_kappa);

            wb = waitbar(0, 'Monte Carlo Process');
            % iterate for kappa, tilde_kappa, montecarlo
            cnt = 0;
            for i = 1:num_kappa
                for j = 1:num_tilde_kappa
                    cnt = cnt + 1;
                    str = ['Monte Carlo Process: ',num2str(cnt)];
                    waitbar(cnt/(num_kappa*num_tilde_kappa), wb, str);

                    E_ij = 0;
                    for mc_iter = 1:MC_iter
                        mc_init_state = [rand(1)*40,rand(1)*30];
                        mc_x = mc_init_state;
                        mc_tilde_x = mc_init_state;
                        v_head = 15;
                        v_max = 30;
                        dt = 0.01;
                        E_iter = 0;
                        for k = 1:TotalStep
                            P_T = TotalStep;
                            P_A = 6 +  4*randn(1)- 2;
                            if k*dt>0 && k*dt<P_T
                                a = P_A*cos(2*pi/P_T*(k*dt-20));
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







