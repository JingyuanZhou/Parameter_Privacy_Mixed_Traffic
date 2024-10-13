mode = 2; % 1 for data generation, 2 for testing neural randomizer

if mode == 1
    N = 10;
    N_tilde_kappa = 3;

    % true para and tilde para
    a = linspace(0.1, 1, N); % discreted a
    b = linspace(0.1, 1, N); % discreted b
    ta = linspace(0.1, 1, N_tilde_kappa); % discreted tilde_a
    tb = linspace(0.1, 1, N_tilde_kappa); % discreted tilde_b
    
    para = cell(N^2,1);
    cnt = 1;
    for alpha = a
        for beta = b
            para{cnt,1} = ['(' , num2str(alpha) , ',' , num2str(beta) , ')'];
            cnt = cnt + 1;
        end
    end
    
    tilde_para = cell(N_tilde_kappa^2,1);
    cnt = 1;
    for alpha = ta
        for beta = tb
            tilde_para{cnt,1} = ['(' , num2str(alpha) , ',' , num2str(beta) , ')'];
            cnt = cnt + 1;
        end
    end

    for I0 = [0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8]
    
        load_E = 1;
    
        P = ConPrivacyFilter_Sampling(N, N_tilde_kappa, I0, load_E);
        save("data/sampling_data_" + num2str(I0) + ".mat", "P")
        writematrix(P, "data/sampling_data_" + num2str(I0) + ".csv")
        figure;
        h = heatmap(P);
        h.XDisplayLabels = tilde_para;
        h.YDisplayLabels = para;
    end
    
elseif mode == 2

    data = [0.55,0.95];
    for I0 = [0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8]
        results = NN_test(['model/model_',num2str(I0),'.pth'], data);
        disp(results)
    end
    
end




