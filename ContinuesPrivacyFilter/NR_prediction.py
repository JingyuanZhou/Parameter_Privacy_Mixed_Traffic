import torch
import numpy as np
from NeuralRandomizer import Net
from lipsnet import LipsNet

def prediction(params_path, data, I0, T_enabled=False, if_LipsNet=True):
    input_size = 2
    output_size = 9
    if if_LipsNet:
        model = LipsNet(f_sizes=[input_size, 64, 64, output_size], f_hid_nonliear=torch.nn.ReLU, f_out_nonliear=torch.nn.Identity,
                         global_lips=False, k_init=1, k_sizes=[input_size, 32, 1], k_hid_act=torch.nn.Tanh, k_out_act=torch.nn.Softplus,
                         loss_lambda=0.1, eps=1e-4, squash_action=True, T_enabled=T_enabled, I0=I0)
    else:
        model = Net(input_size, output_size, I0, T_enabled)
    model.load_state_dict(torch.load(params_path, map_location=torch.device('cpu')))
    model.eval()
    data = np.asarray(data)
    data = np.ascontiguousarray(data.T)
    data = torch.from_numpy(data)
    data = data.type(torch.FloatTensor)
    data = data.reshape(1, -1)

    with torch.no_grad():
        pred = model(data)
    pred = pred.detach().numpy().squeeze()
    pred = np.ascontiguousarray(pred.T)

    return pred