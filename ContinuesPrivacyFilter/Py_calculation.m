function pred_mean = Py_calculation(params_path, I0, T_enabled, if_lips)
    N = 10;
    a = linspace(0.1, 1, N); % discreted a
    b = linspace(0.1, 1, N); % discreted b
    Px = 1/(N^2);
    pred_mean = 0;
    for alpha = a
        for beta = b
            para = [alpha, beta];
            NR = py.importlib.import_module('NeuralRandomizer');
            py.importlib.reload(NR);
            model = py.importlib.import_module('NR_prediction');
            py.importlib.reload(model);
            pred = model.prediction(pyargs('params_path', params_path, 'data', para, 'I0', I0, 'T_enabled', T_enabled, 'if_LipsNet', if_lips));
            pred_mean = pred_mean + single(double(pred))*Px;
        end
    end
    
end