import torch
import torch.nn as nn
import torch.optim as optim
import torch.onnx
import pandas as pd
import numpy as np
from scipy.optimize import root
import torch.nn.functional as F
from lipsnet import LipsNet

# Define the neural network
class Net(nn.Module):
    def __init__(self, input_size, output_size, I0, T_enabled=True):
        super(Net, self).__init__()
        self.fc1 = nn.Linear(input_size, 200)
        self.fc2 = nn.Linear(200, 300)
        self.fc3 = nn.Linear(300, output_size)
        self.T_enabled = T_enabled
        self.output_size = output_size
        self.I0 = I0

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        s = self.fc3(x)
        x_temp = torch.softmax(s, dim=1)
        entrophy = -torch.sum(x_temp.mul(torch.log2(x_temp)), dim=1)
        H0 = 3.15
        if self.T_enabled and entrophy < H0 - self.I0:
            
            # n = self.output_size
            # T = (-torch.tensor(H0 - self.I0) + torch.log2(torch.tensor(n)))/(torch.max(s) - torch.mean(s)) 

            def f1(T):
                s_new = torch.softmax(s/T, dim=1)
                s_sum = torch.sum(s_new, dim=1)
                return -torch.sum((s_new/s_sum)*torch.log2(s_new/s_sum), dim = 1) - torch.tensor(H0 - self.I0)
            T = root(f1, 1).x
            s = s/T
        
        x = torch.softmax(s, dim=1)
        return x

def data_loader(path):
    N = 10
    label = pd.read_csv(path, header=None, index_col=False).to_numpy()
    training_data = np.array([[i,j] for i in np.linspace(0.05,0.95,N) for j in np.linspace(0.05,0.95,N)])
    # np.linspace(0.1,1,N)
    return label, training_data

if __name__ == '__main__':
    # Training with different I0
    if_LipsNet = True
    for I0 in [0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8]:
        # Load the data
        label, training_data = data_loader('data/sampling_data_'+str(I0)+'.csv')

        # Set the input and output sizes
        input_size = 2
        output_size = 9
        T_enabled = False

        if if_LipsNet:
            net = LipsNet(f_sizes=[input_size,64,64,output_size], f_hid_nonliear=nn.ReLU, f_out_nonliear=nn.Identity,
                          global_lips=False, k_init=1, k_sizes=[input_size,32,1], k_hid_act=nn.Tanh, k_out_act=nn.Softplus)
        else:
            net = Net(input_size, output_size, I0, T_enabled)

        # Define the loss function and optimizer
        criterion = nn.KLDivLoss(reduction="batchmean") #nn.MSELoss()
        optimizer = optim.Adam(net.parameters())

        # Assume we have a set of input data X and corresponding target data Y
        # Here we randomly generate these data as an example
        
        X = torch.tensor(training_data, dtype=torch.float32)
        Y = torch.tensor(label, dtype=torch.float32)

        # Train the neural network
        for epoch in range(100):
            # Forward propagation
            outputs = net(X)
            # Compute the loss
            loss = criterion(outputs.log(), Y) #F.log_softmax(outputs, dim=1)
            # Backward propagation and optimization
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            if epoch % 10 == 0:
                print(f"Epoch {epoch}, Loss: {loss.item()}")

        # Save the trained model
        if T_enabled == False:
            if if_LipsNet:
                torch.save(net.state_dict(), 'model/model_'+str(I0)+'_low_privacy_lips.pth')
            else:
                torch.save(net.state_dict(), 'model/model_'+str(I0)+'_low_privacy.pth')
        else:
            torch.save(net.state_dict(), 'model/model_'+str(I0)+'.pth')

        # Use the trained model to predict test data
        newX = X[60:61]
        net.T_enabled = True
        predictions = net(newX)

        # Compute the test error
        test_error = criterion(predictions, Y[60:61]).item()
        print(f"Test error: {test_error}")
        print(f"Prediction: {predictions}")
        # print(f"Target: {Y[60:61]}")

        # Save the model in ONNX format
        # dummy_input = torch.randn(1, 2)
        # torch.onnx.export(net, dummy_input, 'model/model'+str(I0)+'.onnx')
