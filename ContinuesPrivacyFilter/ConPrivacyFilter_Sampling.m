function [pi_opt, obj_val] = ConPrivacyFilter_Sampling(N, N_tilde_kappa, I0, load_E)
    % Parameters setting
    a = linspace(0.1, 1, N); % discreted a
    b = linspace(0.1, 1, N); % discreted b
    ta = linspace(0.1, 1, N_tilde_kappa); % discreted tilde_a
    tb = linspace(0.1, 1, N_tilde_kappa); % discreted tilde_b

    % if using the testing error function
    test_error = 2; 

    % 定义 e(a, b, ta, tb) 的函数
    function error = e(a,b,ta,tb)
        error = (a-ta).^2+(b-tb).^2;
    end

    if test_error == 1
        % 计算关于 (a, b) 和 (ta, tb) 的网格
        [a_grid, b_grid, ta_grid, tb_grid] = ndgrid(a, b, ta, tb);
        error_hash = reshape(e(a_grid,b_grid,ta_grid,tb_grid),[N^2, N^2]);
    elseif test_error == 2 && load_E == 0
        s_st = 5;
        s_go = 35;
        para = [];
        p_para = (1/N^2)*ones(1, N^2);
        for alpha = a
            for beta = b
                para = [para; alpha, beta, s_st, s_go];
            end
        end
        
        tilde_para = [];
        for alpha = ta
            for beta = tb
                tilde_para = [tilde_para; alpha, beta, s_st, s_go];
            end
        end
        MC = ConMonteCarlo(para, p_para, tilde_para);
        MC = MC.MonteCarlo(1500);
        error_hash = MC.E;
        save("data/MC.mat", "MC")
    elseif load_E == 1
        data = load("data/MC.mat");
        error_hash = data.MC.E;
    end
    
    % 初始化 p(tilde_kappa|kappa) 和 p(kappa) 矩阵
    p_tilde_kappa_given_kappa = ones(N^2, N_tilde_kappa^2) / (N_tilde_kappa^2);
    p_kappa = ones(N^2, 1) / (N^2);

    % 定义目标函数
    function obj_val = obj_fun(pi_vec)
        pi = reshape(pi_vec, [N^2, N_tilde_kappa^2]);
        obj_val = 0;
        for i = 1:N^2
            for j = 1:N_tilde_kappa^2
                obj_val = obj_val + pi(i,j)*p_kappa(j)*error_hash(i,j); %
            end
        end
    end

    % 定义约束条件
    Aeq = kron(speye(N^2), ones(1, N_tilde_kappa^2));
    beq = ones(N^2, 1);
    lb = zeros(N*N*N_tilde_kappa*N_tilde_kappa, 1);
    ub = ones(N*N*N_tilde_kappa*N_tilde_kappa, 1);

    % 计算互信息
    function MI = mutual_information(pi)
        MI = 0;
        p_kappa_tilde = p_kappa'*pi;
        for i = 1:N^2
            for j = 1:N_tilde_kappa^2
                MI = MI + pi(i,j)*p_kappa(j)*log2((pi(i,j)+eps)/(p_kappa_tilde(j)+eps));
            end
        end
    end

    % 定义非线性约束
    function [c, ceq] = nonlcon(pi_vec)
        pi = reshape(pi_vec, [N^2, N_tilde_kappa^2]);
        MI = mutual_information(pi);
        c = MI - I0;
        ceq = [];
    end

    % 优化问题求解
    options = optimoptions('fmincon','Display','none','MaxIterations',1000,'MaxFunctionEvaluations',100000,'Algorithm','interior-point');
    [pi_opt, obj_val] = fmincon(@(pi_vec)obj_fun(pi_vec), p_tilde_kappa_given_kappa(:), [], [], Aeq, beq, lb, ub, @(pi_vec)nonlcon(pi_vec),options);

    % 重新调整结果矩阵的形状
    pi_opt = reshape(pi_opt, [N^2, N_tilde_kappa^2]);
end
