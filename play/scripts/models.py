
from matplotlib import pyplot as plt
import numpy as np
import  torch
import torchvision
from torch import nn
from torch.autograd import Variable
from torch.utils.data import DataLoader
from torchvision import transforms
import torchvision.datasets as dset
from torchvision.utils import save_image
import torchvision.utils as vutils
from torchsummary import summary
from IPython import display
import torchvision.models as models
from torchvision import datasets, models, transforms
from matplotlib import pyplot as plt
import numpy as np

from matplotlib import pyplot as plt
import numpy as np

import torch
import torchvision
from torch import nn
from torch.autograd import Variable
from torch.utils.data import DataLoader
from torchvision import transforms
import torchvision.datasets as dset
from torchvision.utils import save_image
import torchvision.utils as vutils
from torchsummary import summary
from torch.utils.data.sampler import SubsetRandomSampler


nb_channls=1
image_size1 = 96
image_size2 = 128 
def to_img(x):
    x = 0.5 * (x + 1)
    x = x.clamp(0, 1)
    x = x.view(x.size(0), nb_channls, image_size1, image_size2)
    return x


class autoencoderBLACK(nn.Module):
    def __init__(self):
        super(autoencoderBLACK, self).__init__()
        self.encoder = nn.Sequential(
            nn.Conv2d(1, 8, kernel_size=5,stride=2),
            nn.ReLU(),
            nn.BatchNorm2d(8),
            nn.Conv2d(8,16,kernel_size=5,stride=2),
            nn.ReLU(),
            nn.BatchNorm2d(16),
            nn.Conv2d(16,32,kernel_size=5,stride=2),
            nn.ReLU(),
            nn.BatchNorm2d(32),
            nn.Conv2d(32,16,kernel_size=5,stride=2),
            nn.ReLU(),
            ) 
        
        self.decoder = nn.Sequential(
            nn.BatchNorm2d(16),
            nn.ConvTranspose2d(16,32,kernel_size=5,stride=2),
            nn.ReLU(),
            nn.BatchNorm2d(32),
            nn.ConvTranspose2d(32,16,kernel_size=5,stride=2),
            nn.ReLU(),
            nn.BatchNorm2d(16),
            nn.ConvTranspose2d(16,8,kernel_size=5,stride=2,output_padding=1),
            nn.ReLU(),
            nn.BatchNorm2d(8),
            nn.ConvTranspose2d(8,1,kernel_size=5,stride=2,output_padding=1),
            nn.Sigmoid(),
            nn.BatchNorm2d(1)
            )    
    
    def forward(self,x):
        x = self.encoder(x)
        x = self.decoder(x)
        return x


nb_channls=3
image_size1 = 96
image_size2 = 128 
def to_img(x):
    x = 0.5 * (x + 1)
    x = x.clamp(0, 1)
    x = x.view(x.size(0), nb_channls, image_size1, image_size2)
    return x


class autoencoder(nn.Module):
    def __init__(self):
        super(autoencoder, self).__init__()
        self.encoder = nn.Sequential(
            nn.Conv2d(3, 8, kernel_size=5,stride=2),
            nn.ReLU(),
            nn.BatchNorm2d(8),
            nn.Conv2d(8,16,kernel_size=5,stride=2),
            nn.ReLU(),
            nn.BatchNorm2d(16),
            nn.Conv2d(16,32,kernel_size=5,stride=2),
            nn.ReLU(),
            nn.BatchNorm2d(32),
            nn.Conv2d(32,16,kernel_size=5,stride=2),
            nn.ReLU(),
            ) 
        
        self.decoder = nn.Sequential(
            nn.BatchNorm2d(16),
            nn.ConvTranspose2d(16,32,kernel_size=5,stride=2),
            nn.ReLU(),
            nn.BatchNorm2d(32),
            nn.ConvTranspose2d(32,16,kernel_size=5,stride=2),
            nn.ReLU(),
            nn.BatchNorm2d(16),
            nn.ConvTranspose2d(16,8,kernel_size=5,stride=2,output_padding=1),
            nn.ReLU(),
            nn.BatchNorm2d(8),
            nn.ConvTranspose2d(8,3,kernel_size=5,stride=2,output_padding=1),
            nn.Sigmoid(),
            nn.BatchNorm2d(3)
            )    
    
    def forward(self,x):
        x = self.encoder(x)
        x = self.decoder(x)
        return x

autoencoderBLACK = autoencoderBLACK()
autoencoder_COLOR = autoencoder()

class LSTMPredictor(nn.Module):
    def __init__(self,n_hidden=4096):
        super(LSTMPredictor,self).__init__()
        self.n_hidden = n_hidden
        self.lstm1 = nn.LSTMCell(492,self.n_hidden)
        self.lstm2 = nn.LSTMCell(self.n_hidden,self.n_hidden)
        self.lstm3 = nn.LSTMCell(self.n_hidden,self.n_hidden)
        self.lstm4 = nn.LSTMCell(self.n_hidden,self.n_hidden)
        self.linear = nn.Linear(self.n_hidden,12)
        self.encoder = autoencoder_COLOR.encoder
        self.encoderB = autoencoderBLACK.encoder

    def forward(self,x,image_current,depth_current,future = 0):
        outputs = []
        n_samples = 1
        latent_RGB = self.encoder(image_current)
        latent_D  = self.encoderB(depth_current)
        flattenx = torch.flatten(x,start_dim=1)
        latentImage = torch.flatten(latent_RGB,start_dim=1)
        latentDepth = torch.flatten(latent_D,start_dim=1)
        lstm_input = torch.cat((latentImage,latentDepth,flattenx),1)
        lstm_input = lstm_input[:,None,:]
        h_t = torch.zeros(n_samples,self.n_hidden,dtype=torch.float32)
        c_t = torch.zeros(n_samples,self.n_hidden,dtype=torch.float32)
        h_t2 = torch.zeros(n_samples,self.n_hidden,dtype=torch.float32)
        c_t2 = torch.zeros(n_samples,self.n_hidden,dtype=torch.float32)
        h_t3 = torch.zeros(n_samples,self.n_hidden,dtype=torch.float32)
        c_t3 = torch.zeros(n_samples,self.n_hidden,dtype=torch.float32)
        h_t4 = torch.zeros(n_samples,self.n_hidden,dtype=torch.float32)
        c_t4 = torch.zeros(n_samples,self.n_hidden,dtype=torch.float32)
        for input_t in lstm_input.split(1,dim=0):
            h_t , c_t = self.lstm1(input_t[0],(h_t,c_t))
            h_t2 , c_t2 = self.lstm2(h_t,(h_t,c_t))
            h_t3 , c_t3 = self.lstm2(h_t,(h_t2,c_t2))
            h_t4 , c_t4 = self.lstm2(h_t,(h_t3,c_t3))
            output = self.linear(h_t4)
            outputs.append(output)
        for i in range(future):
            h_t , c_t = self.lstm1(output,(h_t,c_t))
            h_t2 , c_t2 = self.lstm2(h_t,(h_t2,c_t2))
            output = self.linear(h_t2)
            outputs.append(output)

        outputs = torch.cat(outputs,dim=0)
        return outputs
