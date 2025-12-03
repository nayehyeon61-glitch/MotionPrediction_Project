import os
# from turtle import forward 
current_dir = os.path.dirname(os.path.realpath(__file__))
parent_dir = os.path.dirname(current_dir)
# print(parent_dir)
# print(current_dir)
import torch 
import torch.nn as nn
import torch.nn.functional as F 
import pdb
import math

class NormalNN(nn.Module):
    def __init__(self,input_size,hidden_size,output_size):
        super().__init__()
        self.fc1 = nn.Linear(input_size,hidden_size)
        self.fc2 = nn.Linear(hidden_size,hidden_size)
        self.fc3 = nn.Linear(hidden_size,hidden_size)
        self.fc4 = nn.Linear(hidden_size,output_size)
#        self.fc5 = nn.Linear(hidden_size,hidden_size)
#        self.fc6 = nn.Linear(hidden_size,output_size)

    def forward(self,input_data):
        input_data = input_data.flatten(-2)
        out = self.fc1(F.elu(input_data))
        out = self.fc2(F.elu(out))
        out = self.fc3(F.elu(out))
#        out4 = self.fc4(F.elu(out3))
#        out5 = self.fc4(F.elu(out4))
        return self.fc4(out)

    def set_normalization(self,mu,std):
        self.mu = mu
        self.std = std

    def normalize(self,t):
        return (t-self.mu)/self.std

    def denormalize(self,t):
        return t*self.std + self.mu

class Encoder(nn.Module):
    def __init__(self,input_size,latent_size, input_frame=1):
        super().__init__()
        self.input_size = input_size
        self.latent_size = latent_size
        self.hidden_size = 512
        self.input_frame = input_frame
        real_input = self.input_size*input_frame + input_size
        self.fc1 = nn.Linear(real_input,self.hidden_size)
        self.fc2 = nn.Linear(input_size*input_frame+self.hidden_size,self.hidden_size)
        self.mu  = nn.Linear(input_size*input_frame+self.hidden_size,latent_size)
        self.var = nn.Linear(input_size*input_frame+self.hidden_size,latent_size)
    
    def reparameterize(self,mu,var):
        std = torch.exp(0.5*var)
        eps = torch.randn_like(std)
        return mu + eps*std
    
    def encode(self,input,condition_input):
        out1 = F.elu(self.fc1(torch.cat((input,condition_input),dim=1)))
        out2 = F.elu(self.fc2(torch.cat((input,out1),dim=1)))
        #out3 = F.elu(self.fc3(torch.cat((input,out2),dim=1)))
        #out4 = F.elu(self.fc4(torch.cat((input,out3),dim=1)))
        out5 = torch.cat((input,out2),dim=1)
        return self.mu(out5) , self.var(out5)
    
    # conditional frame 관련해서 해결
    def encode2(self, input, condition_input):
        
        pass

    def forward(self,input, condition_input):
        mu , var = self.encode(input,condition_input)
        z = self.reparameterize(mu,var)
        return z,mu,var
    
    


class Decoder(nn.Module):
    def __init__(self,input_size,latent_size,num_experts,output_size):
        super().__init__()
        input_size = latent_size + input_size
        output_size = output_size
        hidden_size = 256
        self.fc1 = nn.Linear(input_size,hidden_size)
        self.fc2 = nn.Linear(hidden_size+latent_size,hidden_size)
        self.fc3 = nn.Linear(hidden_size+latent_size,output_size)
        #self.fc4 = nn.Linear(hidden_size+latent_size,hidden_size)
        #self.fc5 = nn.Linear(latent_size+hidden_size,output_size)

    def forward(self,z,condition_input):
        # print("latent_shape : ",z.shape)
        out6 = F.elu(self.fc1(torch.cat((z,condition_input),dim=1)))
        out7 = F.elu(self.fc2(torch.cat((z,out6),dim=1)))
        #out8 = F.elu(self.fc2(torch.cat((z,out7),dim=1)))
        #out9 = F.elu(self.fc2(torch.cat((z,out8),dim=1)))
        
        return self.fc3(torch.cat((z,out7),dim=1))


class MixedDecoder(nn.Module):
    def __init__(
        self,
        frame_size,
        latent_size,
        hidden_size,
        num_condition_frames,
        num_future_predictions,
        num_experts,
    ):
        super().__init__()

        input_size = latent_size + frame_size * num_condition_frames
        inter_size = latent_size + hidden_size
        output_size = num_future_predictions * frame_size
        self.decoder_layers = [
            (
                nn.Parameter(torch.empty(num_experts, input_size, hidden_size)),
                nn.Parameter(torch.empty(num_experts, hidden_size)),
                F.elu,
            ),
            (
                nn.Parameter(torch.empty(num_experts, inter_size, hidden_size)),
                nn.Parameter(torch.empty(num_experts, hidden_size)),
                F.elu,
            ),
            (
                nn.Parameter(torch.empty(num_experts, inter_size, output_size)),
                nn.Parameter(torch.empty(num_experts, output_size)),
                None,
            ),
        ]

        for index, (weight, bias, _) in enumerate(self.decoder_layers):
            index = str(index)
            torch.nn.init.kaiming_uniform_(weight)
            bias.data.fill_(0.01)
            self.register_parameter("w" + index, weight)
            self.register_parameter("b" + index, bias)

        # Gating network
        gate_hsize = 64
        self.gate = nn.Sequential(
            nn.Linear(input_size, gate_hsize),
            nn.ELU(),
            nn.Linear(gate_hsize, gate_hsize),
            nn.ELU(),
            nn.Linear(gate_hsize, num_experts),
        )

    def forward(self, z ):
        coefficients = F.softmax(self.gate(z), dim=1)
        for (weight, bias, activation) in self.decoder_layers:
            flat_weight = weight.flatten(start_dim=1, end_dim=2)
            mixed_weight = torch.matmul(coefficients, flat_weight).view(
                coefficients.shape[0], *weight.shape[1:3]
            )

            input = z.unsqueeze(1)
            mixed_bias = torch.matmul(coefficients, bias).unsqueeze(1)
            pdb.set_trace
            out = torch.baddbmm(mixed_bias, input, mixed_weight).squeeze(1)
            layer_out = activation(out) if activation is not None else out

        return layer_out



class VAE(nn.Module):
    def __init__(self,input_size,latent_size,num_experts,output_size):
        super().__init__()
        self.encoder = Encoder(input_size,latent_size)
        #self.decoder = MixedDecoder(input_size,latent_size,256,1,1,num_experts)
        self.decoder = Decoder(input_size,latent_size,num_experts,output_size)
        
        
        ############################change initialization orer##########################3
        self.data_std = 0
        self.data_avg = 0
        ############################change initialization orer##########################3
        self.latent_list = []

    def encode(self,x,c):
        z,mu,logvar = self.encoder(x,c)
        return z,mu,logvar
    def forward(self,x,c):
        z,mu,logvar = self.encoder(x,c)
        return self.decoder(z,c),mu,logvar
    def sample(self,z,c):
        return self.decoder(z,c)
    def set_normalization(self,std,avg):
        self.data_std=std
        self.data_avg=avg
    def set_latent_list(self,latent_vectors):
        self.latent_list = latent_vectors

    #######################
    def normalize(self, t):
        return (t - self.data_avg) / self.data_std
    def denormalize(self, t):
        return t * self.data_std + self.data_avg
    #######################


class BetaDerivatives():
    def __init__(self,time_steps,beta_start,beta_end):
        self.beta_start = beta_start
        self.beta_end = beta_end
        self.time_steps = time_steps
        self.betas = self.prepare_noise_schedule().to(device="cpu")
        self.alpha = 1-self.betas
        self.alpha_hat = torch.cumprod(self.alpha,dim=0)
        
    def prepare_noise_schedule(self):
        return torch.linspace(self.beta_start,self.beta_end,self.time_steps)

    def sample_timesteps(self,n):
        return torch.randint(low=1,high=self.time_steps-1,size=(n,))

    def gather(self,a,t):
        return torch.gather(a,1,t)


class GaussianDiffusion():
    def __init__(self,input_size,noise_step,output_size):
        self.device = "cpu"
        self.input_size = input_size
        self.output_size = output_size
        self.noise_step = noise_step
        self.beta_start = 1e-4
        self.beta_end = 0.02
        self.betaderivative = BetaDerivatives(noise_step,self.beta_start,self.beta_end)
        
        self.beta = self.betaderivative.prepare_noise_schedule().to(self.device)
        self.alpha = self.betaderivative.alpha
        self.alpha_hat = self.betaderivative.alpha_hat

    def q_sample(self,x_0,t,noise=None):
        if noise is None:
            noise = torch.randn((t.shape[0],x_0.shape[0]))
        sqrt_alpha_hat = torch.sqrt(self.alpha_hat[t])
        sqrt_one_minus_alpha_hat = torch.sqrt(1-self.alpha_hat[t])
        return sqrt_alpha_hat*x_0 + sqrt_one_minus_alpha_hat*noise,noise.to(self.device)

class TimeEmbedding(nn.Module):
    def __init__(self,n):
        super().__init__()
        self.n = n
        self.fc1 = nn.Linear(n,n)
        self.fc2 = nn.Linear(n,n)

    def activation(self,x):
        return x*F.sigmoid(x)

    def forward(self,t):
        half_dim = self.n//2
        emb = torch.log(torch.tensor(1000.0)/(half_dim-1))
        emb = torch.exp(torch.arange(half_dim)*-emb)
        emb = t*emb
        emb = torch.cat((emb.sin(),emb.cos()),dim=1)
        emb = self.activation(self.fc1(emb))
        emb = self.fc2(emb)
        return emb


class DenoiseDiffusion(nn.Module):
    def __init__(self,input_size,output_size,noise_steps):
        super().__init__()
        self.input_size = input_size
        self.output_size = output_size
        self.noise_steps = noise_steps
        self.hidden_size = 32
        self.time_dim = self.hidden_size
        self.gaussiandiffusion = GaussianDiffusion(self.input_size,self.noise_steps,self.output_size)
        self.timeembedding = TimeEmbedding(self.time_dim)
        self.betas = self.gaussiandiffusion.beta
        self.alpha = 1-self.betas
        self.alpha_bar = torch.cumprod(self.alpha,dim=0)

        self.fc1 = nn.Linear(input_size,self.hidden_size)
        self.fc2 = nn.Linear(self.hidden_size+self.time_dim,self.hidden_size)
        self.fc3 = nn.Linear(self.hidden_size+self.time_dim,self.hidden_size)
        self.fc4 = nn.Linear(self.hidden_size+self.time_dim,self.output_size)


    def q_xt_x0(self,x0,t):
        mean = self.gaussiandiffusion.alpha_hat
        mean = self.gaussiandiffusion.alpha_hat[t]**0.5*x0
        var = 1-self.gaussiandiffusion.alpha_hat[t]
        return mean , var 

    def q_sample(self,x0,t,eps):
        if eps is None:
            eps = torch.randn_like(x0)
        mean,var = self.q_xt_x0(x0,t)
        return mean+(var**0.5)*eps

    def p_sample(self,xt,t):
        eps_theta = self.forward(xt,t)
        alpha_hat = self.gaussiandiffusion.alpha_hat[t]
        alpha = self.gaussiandiffusion.alpha[t]
        eps_coef = (1-alpha)/(1-alpha_hat)**0.5
        mean = 1/(alpha**0.5)*(xt-eps_coef*eps_theta)
        var = self.gaussiandiffusion.beta[t]
        eps = torch.randn_like(xt)
        return mean + (var**0.5)*eps

    def forward(self,xt,t):
        t = self.timeembedding(t)
        emb = self.fc1(xt)
        emb = torch.cat((emb,t),dim=1)
        emb = F.elu(self.fc2(emb))
        emb = torch.cat((emb,t),dim=1)
        emb = F.elu(self.fc3(emb))
        emb = torch.cat((emb,t),dim=1)
        emb = F.elu(self.fc4(emb))
        return emb

class DanceEncoder10(nn.Module):
    def __init__(self,pose_size,hidden_size,latent_size):
        super().__init__()
        self.input_size = pose_size*10
        self.pose_size = pose_size
        self.latent_size = latent_size
        self.hidden_size = hidden_size
        self.fc1 = nn.Linear(self.input_size,self.hidden_size)
        self.fc2 = nn.Linear(self.hidden_size,self.hidden_size)
        self.fc3 = nn.Linear(self.hidden_size,self.hidden_size)
        self.fc4 = nn.Linear(self.hidden_size,self.hidden_size)
        self.mu = nn.Linear(self.hidden_size,self.latent_size)
        self.std = nn.Linear(self.hidden_size,self.latent_size)


    def encode(self,t1,t2,t3,t4,t5,t6,t7,t8,t9,t10):
        data = torch.cat((t1,t2,t3,t4,t5,t6,t7,t8,t9,t10),dim=1)
        out1 = self.fc1(F.elu(data))
        out2 = self.fc2(F.elu(out1))
        out3 = self.fc3(F.elu(out2))
        out4 = self.fc4(F.elu(out3))
        return self.mu(out4),self.std(out4) 

    def reparameterize(self,mu,var):
        std = torch.exp(0.5*var)
        eps = torch.randn_like(std)
        return mu+std*eps

    def forward(self,t1,t2,t3,t4,t5,t6,t7,t8,t9,t10):
        mu , var = self.encode(t1,t2,t3,t4,t5,t6,t7,t8,t9,t10)
        z = self.reparameterize(mu,var)
        return z,mu,var

class DanceDecoder10(nn.Module):
    def __init__(self,latent_size,pose_size,hidden_size,output_size):
        super().__init__()
        self.latent_size = latent_size
        self.pose_size = pose_size
        self.output_size = output_size
        self.hidden_size = hidden_size
        self.fc1 = nn.Linear(self.pose_size*5+self.latent_size,self.hidden_size)
        self.fc2 = nn.Linear(self.hidden_size+self.pose_size*5,self.hidden_size)
        self.fc3 = nn.Linear(self.hidden_size+self.pose_size*5,self.hidden_size)
        self.fc4 = nn.Linear(self.hidden_size+self.pose_size*5,self.hidden_size)
        self.fc5 = nn.Linear(self.hidden_size+self.pose_size*5,self.output_size)

    def forward(self,z,t1,t2,t3,t4,t5):
        out1 = self.fc1(F.elu(torch.cat((z,t1,t2,t3,t4,t5),dim=1)))
        out2 = self.fc2(F.elu(torch.cat((out1,t1,t2,t3,t4,t5),dim=1)))
        out3 = self.fc3(F.elu(torch.cat((out2,t1,t2,t3,t4,t5),dim=1)))
        out4 = self.fc4(F.elu(torch.cat((out3,t1,t2,t3,t4,t5),dim=1)))
        return self.fc5(torch.cat((out4,t1,t2,t3,t4,t5),dim=1))

class DanceVAE10(nn.Module):
    def __init__(self,pose_size,encode_hidden_size,latent_size,decode_hidden_size,output_size):
        super().__init__()
        self.encoder = DanceEncoder10(pose_size,encode_hidden_size,latent_size)
        self.decoder = DanceDecoder10(latent_size,pose_size,decode_hidden_size,output_size)
        self.pose_data_mu = 0
        self.pose_data_std = 0
    
    def encode(self,t1,t2,t3,t4,t5,t6,t7,t8,t9,t10):
        z,mu,logvar = self.encoder(t1,t2,t3,t4,t5,t6,t7,t8,t9,t10)
        return z,mu,logvar

    def forward(self,t1,t2,t3,t4,t5,t6,t7,t8,t9,t10):
        z,mu,logvar= self.encoder(t1,t2,t3,t4,t5,t6,t7,t8,t9,t10)
        return self.decoder(z,t1,t3,t5,t7,t9),mu,logvar

    def sample(self,z,t1,t3,t5,t7,t9):
        return self.decoder(z,t1,t3,t5,t7,t9)

    def set_normalize(self,pose_mu,pose_std):
        self.pose_data_mu = pose_mu
        self.pose_data_std = pose_std

    def normalize_pose(self,x):
        return (x-self.pose_data_mu)/self.pose_data_std
    
    def denormalize_pose(self,x):
        return x*self.pose_data_std+self.pose_data_mu

class TrackerEncoder(nn.Module):
    def __init__(self,tracker_size,hidden_size,latent_size):
        super().__init__()
        self.input_size = tracker_size*10
        self.latent_size = latent_size
        self.hidden_size = hidden_size
        self.fc1 = nn.Linear(self.input_size,self.hidden_size)
        self.fc2 = nn.Linear(self.hidden_size+self.input_size,self.hidden_size)
        self.fc3 = nn.Linear(self.hidden_size+self.input_size,self.hidden_size)
        self.fc4 = nn.Linear(self.hidden_size+self.input_size,self.hidden_size)
        self.mu = nn.Linear(self.hidden_size+self.input_size,self.latent_size)
        self.std = nn.Linear(self.hidden_size+self.input_size,self.latent_size)

    def encode(self,t1,t2,t3,t4,t5,t6,t7,t8,t9,t10):
        data = torch.cat((t1,t2,t3,t4,t5,t6,t7,t8,t9,t10),dim=1)
        out1 = self.fc1(F.elu(data))
        out2 = self.fc2(F.elu(torch.cat((out1,t1,t2,t3,t4,t5,t6,t7,t8,t9,t10),dim=1)))
        out3 = self.fc3(F.elu(torch.cat((out2,t1,t2,t3,t4,t5,t6,t7,t8,t9,t10),dim=1)))
        out4 = self.fc4(F.elu(torch.cat((out3,t1,t2,t3,t4,t5,t6,t7,t8,t9,t10),dim=1)))
        return self.mu(torch.cat((out4,t1,t2,t3,t4,t5,t6,t7,t8,t9,t10),dim=1)),self.std(torch.cat((out4,t1,t2,t3,t4,t5,t6,t7,t8,t9,t10),dim=1)) 

    def reparameterize(self,mu,var):
        std = torch.exp(0.5*var)
        eps = torch.randn_like(std)
        return mu+std*eps

    def forward(self,t1,t2,t3,t4,t5,t6,t7,t8,t9,t10):
        mu , var = self.encode(t1,t2,t3,t4,t5,t6,t7,t8,t9,t10)
        z = self.reparameterize(mu,var)
        return z,mu,var

class TrackerDecoder(nn.Module):
    def __init__(self,latent_size,tracker_size,hidden_size,output_size):
        super().__init__()
        self.tracker_size = tracker_size
        self.input_size = latent_size+self.tracker_size*5
        self.output_size = output_size
        self.hidden_size = hidden_size
        self.fc1 = nn.Linear(self.input_size,self.hidden_size)
        self.fc2 = nn.Linear(self.hidden_size+latent_size,self.hidden_size)
        self.fc3 = nn.Linear(self.hidden_size+latent_size,self.hidden_size)
        self.fc4 = nn.Linear(self.hidden_size+latent_size,self.hidden_size)
        self.fc5 = nn.Linear(self.hidden_size+latent_size,self.output_size)

    def forward(self,z,t1,t3,t5,t7,t9):
        out1 = self.fc1(F.elu(torch.cat((z,t1,t3,t5,t7,t9),dim=1)))
        out2 = self.fc2(F.elu(torch.cat((out1,z),dim=1)))
        out3 = self.fc3(F.elu(torch.cat((out2,z),dim=1)))
        out4 = self.fc4(F.elu(torch.cat((out3,z),dim=1)))
        return self.fc5(torch.cat((out4,z),dim=1))

class TrackerVAE(nn.Module):
    def __init__(self,tracker_size,encode_hidden_size,latent_size,decode_hidden_size,output_size):
        super().__init__()
        self.encoder = TrackerEncoder(tracker_size,encode_hidden_size,latent_size)
        self.decoder = TrackerDecoder(latent_size,tracker_size,decode_hidden_size,output_size)
    
    def encode(self,t1,t2,t3,t4,t5,t6,t7,t8,t9,t10):
        z,mu,logvar = self.encoder(t1,t2,t3,t4,t5,t6,t7,t8,t9,t10)
        return z,mu,logvar

    def forward(self,t1,t2,t3,t4,t5,t6,t7,t8,t9,t10):
        z,mu,logvar= self.encoder(t1,t2,t3,t4,t5,t6,t7,t8,t9,t10)
        return self.decoder(z,t1,t3,t5,t7,t9),mu,logvar

    def sample(self,z,t1,t3,t5,t7,t9):
        return self.decoder(z,t1,t3,t5,t7,t9)


class TrackerAutoEncoder(nn.Module):
    def __init__(self,tracker_size,num_condition_frames,hidden_size,latent_size):
        super().__init__()
        self.input_size = tracker_size*num_condition_frames
        self.hidden_size = hidden_size
        self.latent_size = latent_size
        self.fc1 = nn.Linear(self.input_size,self.hidden_size)
        self.fc2 = nn.Linear(self.hidden_size+self.input_size,self.hidden_size)
        self.fc3 = nn.Linear(self.hidden_size+self.input_size,self.hidden_size)
        self.fc4 = nn.Linear(self.hidden_size+self.input_size,self.latent_size)

    def encode(self,tracker_data):
        data = tracker_data.flatten(-2)
        out = F.elu(self.fc1(data))
        out = F.elu(self.fc2(torch.cat((out,data),dim=1)))
        out = F.elu(self.fc3(torch.cat((out,data),dim=1)))
        out = self.fc4((torch.cat((out,data),dim=1)))
        return out

    def forward(self,tracker_data):
        latent = self.encode(tracker_data)
        return latent 

class TrackerAutoDecoder(nn.Module):
    def __init__(self,latent_size,tracker_size,num_condition_frames,hidden_size,output_size):
        super().__init__()
        self.input_size = latent_size
        self.tracker_size = tracker_size
        self.hidden_size = hidden_size
        self.output_size = output_size
        self.fc1 = nn.Linear(self.input_size,self.hidden_size)
        self.fc2 = nn.Linear(self.hidden_size+self.input_size,self.hidden_size)
        self.fc3 = nn.Linear(self.hidden_size+self.input_size,self.hidden_size)
        self.fc4 = nn.Linear(self.hidden_size+self.input_size,self.output_size)

    def forward(self,latent,tracker):
        out = F.elu(self.fc1(latent))
        out = F.elu(self.fc2(torch.cat((out,latent),dim=1)))
        out = F.elu(self.fc3(torch.cat((out,latent),dim=1)))
        out = self.fc4(torch.cat((out,latent),dim=1))
        return out

class TrackerAuto(nn.Module):
    def __init__(self,tracker_size,num_condition_frames,encoder_hidden_size,latent_size,decoder_hidden_size,output_size):
        super().__init__()
        self.encoder = TrackerAutoEncoder(tracker_size,num_condition_frames,encoder_hidden_size,latent_size)
        self.decoder = TrackerAutoDecoder(latent_size,tracker_size,num_condition_frames,decoder_hidden_size,output_size)
        self.num_condition_frames = num_condition_frames
        #self.decoder = MixedDecoder(35,latent_size,decoder_hidden_size,0,1,2)

    def forward(self,tracker_data):
        z = self.encoder(tracker_data)
        return self.decoder(z,tracker_data[:,int(self.num_condition_frames/2-1),:])
    
class CNN(nn.Module):
    def __init__(self,tracker_size,condition_size,output_size):
        super().__init__()
        self.tracker_size = tracker_size*condition_size
        self.output_size = output_size
        self.layer1 = torch.nn.Sequential(
            torch.nn.Conv2d(1,512,(1,3),stride=(1,3)),
            torch.nn.ELU(),
        )
        self.layer2 = torch.nn.Sequential(
            torch.nn.Conv2d(512,32,1),
            torch.nn.ELU(),
        )
        self.fc1 = torch.nn.Linear(19200,1024)
        self.fc2 = torch.nn.Linear(1024,output_size)


    def forward(self,history):
        history = history.unsqueeze(1)
        #print(history.shape)
        out = self.layer1(history)
        #print(out.shape)
        out = self.layer2(out)
        #print(out.shape)
        out = out.view(out.shape[0],-1)
        #print(out.shape)
        out = F.elu(self.fc1(out))
        out = self.fc2(out)
        #print(out.shape)
        return out

# below this is for VQVAE(0822,2024)


import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np

class EncoderOnly(nn.Module):
    def __init__(self,input_size,condition_size,latent_size, hidden_size=512):
        super().__init__()
        self.input_size = input_size
        self.latent_size = latent_size
        self.hidden_size = hidden_size
        
        real_input = self.input_size + condition_size
        self.fc1 = nn.Linear(real_input,self.hidden_size)
        self.ln1 = nn.LayerNorm(self.hidden_size)
        self.fc2 = nn.Linear(self.hidden_size,self.hidden_size)
        self.ln2 = nn.LayerNorm(self.hidden_size)
        self.fc3 = nn.Linear(self.hidden_size,self.hidden_size)
        self.ln3 = nn.LayerNorm(self.hidden_size)
        self.fc4 = nn.Linear(self.hidden_size,self.hidden_size)
        self.ln4 = nn.LayerNorm(self.hidden_size)
        self.fc5 = nn.Linear(self.hidden_size,latent_size)
        
        self.actf = nn.Mish()
    
    def encode(self,input,condition_input):
        out1 = self.ln1(self.actf(self.fc1(torch.cat((input,condition_input),dim=1))))
        out2 = self.ln2(self.actf(self.fc2(out1)) + out1)
        out3 = self.ln3(self.actf(self.fc3(out2)) + out2)
        out4 = self.ln4(self.actf(self.fc4(out3)) + out3)
        return self.actf(self.fc5(out4))
    
    def forward(self,input, condition_input):
        output= self.encode(input,condition_input)
        return output
    

class EncoderTuna(nn.Module):
    def __init__(self,input_size,latent_size, hidden_size=512):
        super(EncoderTuna, self).__init__()
        self.input_size = input_size
        self.latent_size = latent_size
        self.hidden_size = hidden_size
        
        real_input = self.input_size
        self.fc1 = nn.Linear(real_input,self.hidden_size)
        self.ln1 = nn.LayerNorm(self.hidden_size, elementwise_affine=False)
        self.fc2 = nn.Linear(self.hidden_size,self.hidden_size)
        self.ln2 = nn.LayerNorm(self.hidden_size, elementwise_affine=False)
        self.fc3 = nn.Linear(self.hidden_size,self.hidden_size)
        self.ln3 = nn.LayerNorm(self.hidden_size, elementwise_affine=False)
        self.fc4 = nn.Linear(self.hidden_size,self.hidden_size)
        self.ln4 = nn.LayerNorm(self.hidden_size, elementwise_affine=False)
        self.fc5 = nn.Linear(self.hidden_size,latent_size)
        
        self.actf = nn.Mish()
    
    def encode(self,input):
        out1 = self.ln1(self.actf(self.fc1(input)))
        out2 = self.ln2(self.actf(self.fc2(out1)) + out1)
        out3 = self.ln3(self.actf(self.fc3(out2)) + out2)
        out4 = self.ln4(self.actf(self.fc4(out3)) + out3)
        return self.actf(self.fc5(out4))
    
    def forward(self,input):
        output= self.encode(input)
        return output
    
class EncoderTunaWithCNN(nn.Module):
    def __init__(self,input_size,latent_size, hidden_size=512):
        super().__init__()
        self.input_size = input_size
        self.latent_size = latent_size
        self.hidden_size = hidden_size
        
        real_input = self.input_size
        self.fc1 = nn.Linear(real_input,self.hidden_size)
        self.ln1 = nn.LayerNorm(self.hidden_size, elementwise_affine=False)
        self.fc2 = nn.Linear(self.hidden_size,self.hidden_size)
        self.ln2 = nn.LayerNorm(self.hidden_size, elementwise_affine=False)
        self.fc3 = nn.Linear(self.hidden_size,self.hidden_size)
        self.ln3 = nn.LayerNorm(self.hidden_size, elementwise_affine=False)
        self.fc4 = nn.Linear(self.hidden_size,self.hidden_size)
        self.ln4 = nn.LayerNorm(self.hidden_size, elementwise_affine=False)
        self.fc5 = nn.Linear(self.hidden_size,latent_size)
        
        self.actf = nn.Mish()
    
    def encode(self,input):
        out1 = self.ln1(self.actf(self.fc1(input)))
        out2 = self.ln2(self.actf(self.fc2(out1)) + out1)
        out3 = self.ln3(self.actf(self.fc3(out2)) + out2)
        out4 = self.ln4(self.actf(self.fc4(out3)) + out3)
        return self.actf(self.fc5(out4))
    
    def forward(self,input):
        output= self.encode(input)
        return output    
    


class VectorQuantizer2(nn.Module):
    def __init__(self, num_embeddings, embedding_dim, commitment_cost):
        super(VectorQuantizer2, self).__init__()
        
        self.embedding_dim = embedding_dim
        self.num_embeddings = num_embeddings
        self.commitment_cost = commitment_cost
        
        self.embeddings =nn.Embedding(self.num_embeddings, self.embedding_dim)
        self.embeddings.weight.data.uniform_(-1/self.num_embeddings, 1/self.num_embeddings)
        
    def forward(self, z):
        z = z.view(-1, self.embedding_dim)
        distances = (torch.sum(z**2, dim=1, keepdim=True)
                     + torch.sum(self.embeddings.weight**2, dim=1)
                     - 2 * torch.matmul(z, self.embeddings.weight.t()))
        
        encoding_indices = torch.argmin(distances, dim=1).unsqueeze(1)
        z_q = torch.index_select(self.embeddings.weight, dim=0, index=encoding_indices.squeeze()).view_as(z)
        
        loss = F.mse_loss(z_q, z.detach()) + self.commitment_cost * F.mse_loss(z_q.detach(), z)
        
        z_q = z + (z_q - z).detach()
        
        return z_q, loss


class VectorQuantizer(nn.Module):
    """
    Discretization bottleneck part of the VQ-VAE.

    Inputs:
    - n_e : number of embeddings
    - e_dim : dimension of embedding
    - beta : commitment cost used in loss term, beta * ||z_e(x)-sg[e]||^2
    """

    def __init__(self, n_e, e_dim, beta):
        super(VectorQuantizer, self).__init__()
        self.n_e = n_e
        self.e_dim = e_dim
        self.beta = beta

        self.embedding = nn.Embedding(self.n_e, self.e_dim)
        self.embedding.weight.data.uniform_(-1.0 / self.n_e, 1.0 / self.n_e)

    def forward(self, z):
        """
        Inputs the output of the encoder network z and maps it to a discrete 
        one-hot vector that is the index of the closest embedding vector e_j

        z (continuous) -> z_q (discrete)

        z.shape = (batch, channel, height, width)

        quantization pipeline:

            1. get encoder input (B,C,H,W)
            2. flatten input to (B*H*W,C)

        """
        # reshape z -> (batch, height, width, channel) and flatten
        z = z.permute(0, 2, 3, 1).contiguous()
        z_flattened = z.view(-1, self.e_dim)
        # distances from z to embeddings e_j (z - e)^2 = z^2 + e^2 - 2 e * z

        d = torch.sum(z_flattened ** 2, dim=1, keepdim=True) + \
            torch.sum(self.embedding.weight**2, dim=1) - 2 * \
            torch.matmul(z_flattened, self.embedding.weight.t())

        # find closest encodings
        min_encoding_indices = torch.argmin(d, dim=1).unsqueeze(1)
        min_encodings = torch.zeros(
            min_encoding_indices.shape[0], self.n_e).to(device)
        min_encodings.scatter_(1, min_encoding_indices, 1)

        # get quantized latent vectors
        z_q = torch.matmul(min_encodings, self.embedding.weight).view(z.shape)

        # compute loss for embedding
        loss = torch.mean((z_q.detach()-z)**2) + self.beta * \
            torch.mean((z_q - z.detach()) ** 2)

        # preserve gradients
        z_q = z + (z_q - z).detach()

        # perplexity
        e_mean = torch.mean(min_encodings, dim=0)
        perplexity = torch.exp(-torch.sum(e_mean * torch.log(e_mean + 1e-10)))

        # reshape back to match original input shape
        z_q = z_q.permute(0, 3, 1, 2).contiguous()

        return loss, z_q, perplexity, min_encodings, min_encoding_indices


class MoEDecoder(nn.Module):
    def __init__(self, input_dim, output_dim, num_experts, hidden_dim):
        super(MoEDecoder, self).__init__()
        self.num_experts = num_experts

        # Experts: 각 전문가에 대해 별도의 네트워크를 생성
        self.experts = nn.ModuleList([nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.Mish(),
            nn.Dropout(0.1),
            nn.LayerNorm(hidden_dim, elementwise_affine=False),
            nn.Linear(hidden_dim, hidden_dim),
            nn.Mish(),
            nn.Dropout(0.1),
            nn.LayerNorm(hidden_dim, elementwise_affine=False),
            nn.Linear(hidden_dim, hidden_dim),
            nn.Mish(),
            nn.Dropout(0.1),
            nn.LayerNorm(hidden_dim, elementwise_affine=False),
            nn.Linear(hidden_dim, hidden_dim),
            nn.Mish(),
            nn.Dropout(0.1),
            nn.LayerNorm(hidden_dim, elementwise_affine=False),
            nn.Linear(hidden_dim, output_dim)
        ) for _ in range(num_experts)])

        # Gating network: 각 전문가의 가중치를 결정
        self.gating_network = nn.Linear(input_dim, num_experts)
        
        

    def forward(self, x):
        # Gating network를 통해 각 전문가의 가중치 계산
        gate_values = F.softmax(self.gating_network(x), dim=-1)

        # 각 전문가의 출력을 계산하고, 가중치를 곱한 후 합산
        output = sum(gate_values[:, i].unsqueeze(1) * self.experts[i](x) for i in range(self.num_experts))

        return output

def get_dct_matrix(N):
    dct_m = np.eye(N)
    for k in np.arange(N):
        for i in np.arange(N):
            w = np.sqrt(2 / N)
            if k == 0:
                w = np.sqrt(1 / N)
            dct_m[k, i] = w * np.cos(np.pi * (i + 1 / 2) * k / N)
    idct_m = np.linalg.inv(dct_m)
    return dct_m, idct_m

class MotionAttetionLayer(nn.Module):
    """Motion Attention Layer for using Motion Transformer

    Args:
        nn (_type_): _description_
    """
    def __init__(self, embed_dim, num_heads, input_frame, dropout, device):
        """initiator

        Args:
            embed_dim (int): _description_
            num_heads (int): _description_
            dropout (float): _description_
            device (string): _description_
        """
        super(MotionAttetionLayer, self).__init__()
        self.device = device
        self.attention1 = nn.MultiheadAttention(embed_dim, 5, dropout, batch_first=True).to(device)
        
        
        self.attention2 = nn.MultiheadAttention(input_frame, 5, dropout, batch_first=True ).to(device)
        self.LN_spa = nn.LayerNorm(input_frame, elementwise_affine=False).to(device)
 
        self.LN1 = nn.LayerNorm(embed_dim, elementwise_affine=False).to(device)
        self.FF1 = nn.Linear(embed_dim, embed_dim).to(device)
        self.dropout = nn.Dropout(dropout).to(device)
        self.LN2 = nn.LayerNorm(embed_dim, elementwise_affine=False).to(device)
        self.act = nn.Mish()
        self.FF2 = nn.Linear(embed_dim, embed_dim).to(device)
        
    def set_normalization(self,std,avg):
        self.data_std=std
        self.data_avg=avg
        
    def normalize(self, t):
        return (t - self.data_avg) / self.data_std
    
    def denormalize(self, t):
        return t*self.data_std + self.data_avg
        
    def forward(self, x, mask):
        
        # spatio        
        x2 = x.transpose(1,2)
        out_spa, _ = self.attention2(x2,x2,x2)
        out_spa = out_spa + x2
        out_spa = self.LN_spa(out_spa)
        out_spa = out_spa.transpose(1,2)
        
        # temporal
        out1, _ = self.attention1(x, x, x, attn_mask=mask)
        out2 = out1 + x  
        out2 = self.LN1(out2)
        
        assemb = out2 + out_spa
        out3 = self.act(self.FF1(assemb))
        out4 = out3 + x
        out5 = self.LN2(out4)
        out6 = self.act(self.FF2(out5))
        #print(torch.max(out6) - torch.min(out6))
        out6 = self.LN2(out6)
        return out6




class VQVAE(nn.Module):
    def __init__(self, input_size,hidden_dim, latent_size, output_size, 
                 n_embeddings, embedding_dim, beta, save_img_embedding_map=False, input_frame=1, expert_num = 8, input_mult_frame = False, output_mult_frame = False, device='cpu'):
        
        super(VQVAE, self).__init__()
        self.input_frame = input_frame
        self.input_size = input_size
        
       
        arr = [[-np.inf for _ in range(input_frame)] for _ in range(input_frame)]
        arr = torch.tensor(arr).to(device)
        self.mask = torch.triu(arr, diagonal=1).to(device)
        
        
        dct_m, idct_m = get_dct_matrix(input_size)
        self.dct_m = torch.tensor(dct_m).float().unsqueeze(0).to(device)
        self.dct_m = self.dct_m.requires_grad_(False)
        self.idct_m = torch.tensor(idct_m).float().unsqueeze(0).to(device)
        self.idct_m = self.idct_m.requires_grad_(False)
        
        
        
        self.Conv1 = nn.Conv1d(1, 1, 9, 1, 4 )
        self.Conv2 = nn.Conv1d(1, 1, 9, 1, 4 )
        self.ConvLN = nn.LayerNorm(input_size*input_frame, elementwise_affine=False)
        
        
        self.LN = nn.LayerNorm(latent_size, elementwise_affine=False)
        self.encoder = EncoderTuna(input_size*input_frame, latent_size, hidden_dim)
        self.encoder2 = MoEDecoder(input_size*input_frame, latent_size, expert_num, hidden_dim )
        self.LN2 = nn.LayerNorm(embedding_dim, elementwise_affine=False)
        
        self.encoder_totheTop = EncoderTuna(latent_size, 16,32)
        self.LN_TOP = nn.LayerNorm(16)
        self.VQTop = VectorQuantizer2(
            512, 16, beta
        )            
        self.FFNforTopToBottom = nn.Linear(16, latent_size)
        
         
            
        # pass continuous latent vector through discretization bottleneck
        self.VQBottom = VectorQuantizer2(
            n_embeddings, embedding_dim, beta)
        # decode the discrete latent representation

        
        self.decoder = MoEDecoder( latent_size+16+input_size, latent_size, expert_num, hidden_dim )
        
        if  output_mult_frame == False:
            self.decoder2 = MoEDecoder(latent_size+input_size, output_size*2, expert_num , hidden_dim)
        else :
            self.decoder2 = MoEDecoder(latent_size + input_size, output_size*input_frame, expert_num , hidden_dim)
            
            
        self.data_std = 0
        self.data_avg = 0
        self.config = {'model_name': 'VQVAE',
                       'input' : input_size, 
                       'latent' : latent_size,
                       'output_size' : output_size,
                       'embeddings' : n_embeddings,
                       'embedding_dim' : embedding_dim,
                       'beta': beta,
                       'input_frame': input_frame,
                       'isRecursive': False
                       }

        if save_img_embedding_map:
            self.img_to_embedding_map = {i: [] for i in range(n_embeddings)}
        else:
            self.img_to_embedding_map = None
            
    def get_param(self):
        return self.config
        
    def set_normalization(self,std,avg):
        self.data_std=std
        self.data_avg=avg
        
    def normalize(self, t):
        return (t - self.data_avg) / self.data_std
    
    def denormalize(self, t):

        return t*self.data_std + self.data_avg

    def forward(self, x, condition2, verbose=False):

        # for i in 1 , 10:
        #     tmpinput = x[:,35*(i-1):35*i]
        #     tmpinput = torch.matmul(tmpinput, self.dct_m).squeeze(0)
        #     x[:,35*(i-1):35*i] = tmpinput
        
        
        
        # condition2 = torch.matmul(condition2, self.dct_m)
        # condition2 = condition2.squeeze(0)
        
        tx = x.unsqueeze(1)
        nx = self.ConvLN(F.elu(self.Conv1(tx)))
        nx = F.elu(self.Conv2(nx))
        nx = nx.squeeze(1)
        z_e_2 = self.encoder2(nx)
        
        z_e = self.LN(z_e_2)
        
        z_e_top = self.encoder_totheTop(z_e)
        z_q_top, loss1 = self.VQTop(z_e_top)
        
        z_q_t2 = self.FFNforTopToBottom(z_q_top)
        z_e_bottom = z_e + z_q_t2
        
        z_e_bottom = self.LN2(z_e_bottom)
        
        
        z_q, loss2 = self.VQBottom(
            z_e_bottom)
        #x_hat = self.decoder(z_q, condition2) 
        
        #z_q = z_q + z_q_t2
        
        x_hat = self.decoder(torch.cat((z_q,z_q_top, condition2), dim=1)) 
        x_hat = self.decoder2(torch.cat((x_hat, condition2), dim=1))
        #x_hat = x_hat + x[:,140:]
        
        newout = x_hat[:,35:] + x[:,315:]
        # x_hat = torch.matmul(x_hat, self.idct_m )
        #newout = newout.squeeze(0)
        
        
        if verbose:
            print('original data shape:', x.shape)
            print('encoded data shape:', z_e.shape)
            print('recon data shape:', x_hat.shape)
            assert False

        return loss1+loss2, newout



'''
no used anymore
'''
class VQVAE2(nn.Module):
    def __init__(self, input_size,hidden_dim, latent_size, output_size, 
                 n_embeddings, embedding_dim, beta, save_img_embedding_map=False, input_frame=1, expert_num = 8, input_mult_frame = False, output_mult_frame = False, device='cpu'):
        
        super(VQVAE2, self).__init__()
        self.input_frame = input_frame
        self.input_size = input_size
        
       
        arr = [[-np.inf for _ in range(input_frame)] for _ in range(input_frame)]
        arr = torch.tensor(arr).to(device)
        self.mask = torch.triu(arr, diagonal=1).to(device)
        
        
        dct_m, idct_m = get_dct_matrix(input_size)
        self.dct_m = torch.tensor(dct_m).float().unsqueeze(0).to(device)
        self.dct_m = self.dct_m.requires_grad_(False)
        self.idct_m = torch.tensor(idct_m).float().unsqueeze(0).to(device)
        self.idct_m = self.idct_m.requires_grad_(False)
        
        
        self.encoder = EncoderTuna(input_size, latent_size, hidden_dim)
        
        self.encoder_ToTheTop = nn.Linear(latent_size, math.floor(latent_size/4))
        self.LN_TOP = nn.LayerNorm(math.floor(latent_size/4))
        self.VQTop = VectorQuantizer2(
            128, math.floor(latent_size/4), beta
        )            
        self.FFNforTopToBottom = nn.Linear(math.floor(latent_size/4), latent_size)
        
         
            
        # pass continuous latent vector through discretization bottleneck
        self.VQBottom = VectorQuantizer2(
            n_embeddings, latent_size, beta)
        # decode the discrete latent representation

        self.decoder_1 = EncoderTuna(latent_size*2 + input_size, latent_size)
        self.skipde = nn.Linear(latent_size*2 + input_size, latent_size)
        self.LNskipdecoder = nn.LayerNorm(latent_size, elementwise_affine=False)
        self.decoder = MoEDecoder( latent_size*3 , latent_size, expert_num, hidden_dim )
        
        if  output_mult_frame == False:
            self.decoder2 = MoEDecoder(latent_size, output_size, expert_num , hidden_dim)
        else :
            self.decoder2 = MoEDecoder(latent_size, output_size*input_frame, expert_num , hidden_dim)
            
            
        self.data_std = 0
        self.data_avg = 0
        self.config = {'model_name': 'VQVAE',
                       'input' : input_size, 
                       'latent' : latent_size,
                       'output_size' : output_size,
                       'embeddings' : n_embeddings,
                       'embedding_dim' : embedding_dim,
                       'beta': beta,
                       'input_frame': input_frame,
                       'isRecursive': False
                       }

        if save_img_embedding_map:
            self.img_to_embedding_map = {i: [] for i in range(n_embeddings)}
        else:
            self.img_to_embedding_map = None
            
    def get_param(self):
        return self.config
        
    def set_normalization(self,std,avg):
        self.data_std=std
        self.data_avg=avg
        
    def normalize(self, t):
        return (t - self.data_avg) / self.data_std
    
    def denormalize(self, t):

        return t*self.data_std + self.data_avg

    def forward(self, x, condition2, verbose=False):

        # for i in 1 , 10:
        #     tmpinput = x[:,35*(i-1):35*i]
        #     tmpinput = torch.matmul(tmpinput, self.dct_m).squeeze(0)
        #     x[:,35*(i-1):35*i] = tmpinput
        
        

                
        z_e= self.encoder(x)
        
        z_e_top = self.encoder_ToTheTop(z_e)
        z_q_top, loss1 = self.VQTop(z_e_top)
        
        z_q_t2 = self.FFNforTopToBottom(z_q_top)
        z_e_bottom = z_q_t2
        
        
        z_q, loss2 = self.VQBottom(
            z_e_bottom)
        
        x_hat = self.decoder_1(torch.cat((z_q, z_q_t2, condition2), dim=1))
        x_hat_Skip = self.skipde(torch.cat((z_q, z_q_t2, condition2), dim=1))
        
        x_hat = x_hat_Skip + x_hat
        x_hat = self.LNskipdecoder(x_hat)
        
        x_hat = self.decoder(torch.cat((x_hat,z_q,z_q_t2), dim=1)) 
        x_hat = self.decoder2(x_hat)
        
        x_hat = x_hat + condition2
        x_hat = x_hat.squeeze(0)
        
        
        if verbose:
            print('original data shape:', x.shape)
            print('encoded data shape:', z_e.shape)
            print('recon data shape:', x_hat.shape)
            assert False
        return loss1+loss2, x_hat


'''
this is CNN based VQVAE Model
'''
class VQVAE_old(nn.Module):
    def __init__(self, input_size,hidden_dim, latent_size, output_size, 
                 n_embeddings, embedding_dim, beta, save_img_embedding_map=False, input_frame=1, expert_num = 8, input_mult_frame = False, output_mult_frame = False, device='cpu'):
        
        super(VQVAE_old, self).__init__()
        self.input_frame = input_frame
        self.input_size = input_size
        
       
        arr = [[-np.inf for _ in range(input_frame)] for _ in range(input_frame)]
        arr = torch.tensor(arr).to(device)
        self.mask = torch.triu(arr, diagonal=1).to(device)
        
        
        dct_m, idct_m = get_dct_matrix(input_size)
        self.dct_m = torch.tensor(dct_m).float().unsqueeze(0).to(device)
        self.dct_m = self.dct_m.requires_grad_(False)
        self.idct_m = torch.tensor(idct_m).float().unsqueeze(0).to(device)
        self.idct_m = self.idct_m.requires_grad_(False)
        
        
        
        self.Conv1 = nn.Conv1d(1, 1, 9, 1, 4 )
        self.Conv2 = nn.Conv1d(1, 1, 9, 1, 4 )
        self.ConvLN = nn.LayerNorm(input_size*input_frame, elementwise_affine=False)
        
        
        self.LN = nn.LayerNorm(latent_size, elementwise_affine=False)
        self.encoder = EncoderTuna(input_size*input_frame, latent_size, hidden_dim)
        self.encoder2 = MoEDecoder(input_size*input_frame, latent_size, expert_num, hidden_dim )
        self.LN2 = nn.LayerNorm(embedding_dim, elementwise_affine=False)
        
        self.encoder_totheTop = EncoderTuna(latent_size, 16,32)
        self.LN_TOP = nn.LayerNorm(16)
        self.VQTop = VectorQuantizer2(
            512, 16, beta
        )            
        self.FFNforTopToBottom = nn.Linear(16, latent_size)
        
         
            
        # pass continuous latent vector through discretization bottleneck
        self.VQBottom = VectorQuantizer2(
            n_embeddings, embedding_dim, beta)
        # decode the discrete latent representation

        
        self.decoder = MoEDecoder( latent_size+16+input_size, latent_size, expert_num, hidden_dim )
        
        if  output_mult_frame == False:
            self.decoder2 = MoEDecoder(latent_size+input_size, output_size*2, expert_num , hidden_dim)
        else :
            self.decoder2 = MoEDecoder(latent_size + input_size, output_size*input_frame, expert_num , hidden_dim)
            
            
        self.data_std = 0
        self.data_avg = 0
        self.config = {'model_name': 'VQVAE',
                       'input' : input_size, 
                       'latent' : latent_size,
                       'output_size' : output_size,
                       'embeddings' : n_embeddings,
                       'embedding_dim' : embedding_dim,
                       'beta': beta,
                       'input_frame': input_frame,
                       'isRecursive': False
                       }

        if save_img_embedding_map:
            self.img_to_embedding_map = {i: [] for i in range(n_embeddings)}
        else:
            self.img_to_embedding_map = None
            
    def get_param(self):
        return self.config
        
    def set_normalization(self,std,avg):
        self.data_std=std
        self.data_avg=avg
        
    def normalize(self, t):
        return (t - self.data_avg) / self.data_std
    
    def denormalize(self, t):

        return t*self.data_std + self.data_avg

    def forward(self, x, condition2, verbose=False):

        # for i in 1 , 10:
        #     tmpinput = x[:,35*(i-1):35*i]
        #     tmpinput = torch.matmul(tmpinput, self.dct_m).squeeze(0)
        #     x[:,35*(i-1):35*i] = tmpinput
        
        
        
        # condition2 = torch.matmul(condition2, self.dct_m)
        # condition2 = condition2.squeeze(0)
        
        tx = x.unsqueeze(1)
        nx = self.ConvLN(F.elu(self.Conv1(tx)))
        nx = F.elu(self.Conv2(nx))
        nx = nx.squeeze(1)
        z_e_2 = self.encoder2(nx)
        
        z_e = self.LN(z_e_2)
        
        z_e_top = self.encoder_totheTop(z_e)
        z_q_top, loss1 = self.VQTop(z_e_top)
        
        z_q_t2 = self.FFNforTopToBottom(z_q_top)
        z_e_bottom = z_e + z_q_t2
        
        z_e_bottom = self.LN2(z_e_bottom)
        
        
        z_q, loss2 = self.VQBottom(
            z_e_bottom)
        #x_hat = self.decoder(z_q, condition2) 
        
        #z_q = z_q + z_q_t2
        
        x_hat = self.decoder(torch.cat((z_q,z_q_top, condition2), dim=1)) 
        x_hat = self.decoder2(torch.cat((x_hat, condition2), dim=1))
        #x_hat = x_hat + x[:,140:]
        newout = x_hat[:,35:] + x[:,315:]
        # x_hat = torch.matmul(x_hat, self.idct_m )
        #newout = newout.squeeze(0)
        
        
        if verbose:
            print('original data shape:', x.shape)
            print('encoded data shape:', z_e.shape)
            print('recon data shape:', x_hat.shape)
            assert False

        return loss1+loss2, newout