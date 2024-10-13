function pred = NN_test(params_path, data, I0, T_enabled, if_lips)
    NR = py.importlib.import_module('NeuralRandomizer');
    py.importlib.reload(NR);
    model = py.importlib.import_module('NR_prediction');
    py.importlib.reload(model);
    pred = model.prediction(pyargs('params_path', params_path, 'data', data, 'I0', I0, 'T_enabled', T_enabled, 'if_LipsNet', if_lips));
    pred = double(pred);
end

