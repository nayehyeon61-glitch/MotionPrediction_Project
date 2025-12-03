import copy
from json import load
import json
from types import SimpleNamespace
import time 
from multiprocessing import Condition
import os
from random import randint, random
import sys
import platform
import pdb
from unittest import loader
from xml.dom import minicompat # use pdb.set_trace() for debugging
sys.path.append(os.getcwd())
import libmainlib as m   
import luamodule as lua  # see luamodule.py
import numpy as np 
import torch 
from gym_mp.models import Encoder,Decoder,VAE, VQVAE, VQVAE2, VQVAE_old
import torch.optim as optim
from torch.utils.data.sampler import BatchSampler, SubsetRandomSampler , RandomSampler
from tensorboardX import SummaryWriter 
import torch.nn.functional as F 
import settings


# simply forward UI events to lua
def onCallback(mid, userdata):
    lua.onCallback(mid, userdata)

def onFrameChanged(currFrame):
    lua.onFrameChanged(currFrame)

def frameMove(fElapsedTime):
    lua.frameMove(fElapsedTime)

def handleRendererEvent(ev, button, x,y):
    return lua.handleRendererEvent(ev, button, x,y)

def lua_getdim():
    l=m.getPythonWin()
    l.getglobal("get_dim")
    l.call(0,1)
    info= l.popvectorn()
    state_dim=int(info.get(0))
    action_dim=int(info.get(1))
    # print(state_dim)
    # print(action_dim)
    return state_dim,action_dim

def lua_getIframe(iframe):
    l=m.getPythonWin()
    l.getglobal("seok_getIframeDOF")
    input = m.vectorn()
    input.setSize(0)
    input = iframe
    # print(input)
    l.push(input)
    l.call(1,1)
    return l.popvectorn()

def lua_getFrame():
    l = m.getPythonWin()
    l.getglobal("get_frame")
    l.call(0,1)
    return l.popnumber()

def Tonumpy(data):
    return np.array(data.ref())

def Discontinuity():
    l = m.getPythonWin()
    l.getglobal("getDiscontinuity")
    l.call(0,1)
    return l.popvectorn()

def Alldata_load():
    l=m.getPythonWin()
    l.getglobal("All_data_frames")
    l.call(0,1)
    return l.popmatrixn()

def main():
    
    with open("config/vqvaecnn.json", "r") as file:
        config = json.load(file)
    
    train = True
    
    load_save_model = False
    latent_size = config["train_parameter"]["latent_size"]
    #walk : 64~128 stitch:512~1024
    mini_batch = config["train_parameter"]["mini_batch"]
    teacher_epochs = config["train_parameter"]["teacher_epochs"]
    ramping_epochs = config["train_parameter"]["ramping_epochs"]
    student_epochs = config["train_parameter"]["student_epochs"]
    num_epochs = teacher_epochs+ramping_epochs+student_epochs
    num_experts = config["train_parameter"]["num_experts"]
    input_frames = config["model"]["input_frames"]
    initial_lr = config["train_parameter"]["initial_lr"]
    # final_lr = 1e-7
    prediction_frames = config["train_parameter"]["recursive_frames"]
    condition_frame = config["train_parameter"]["condition_frames"]
    hidden_dim = config["train_parameter"]["hidden_dim"]
    codebook_size = config["train_parameter"]["code_dim"]
    beta = config["train_parameter"]["beta"]
    pt_path = config["path"]["pt_path"] + config["model"]["pt_name"] + ".pt"
    
    option=''
    if len(sys.argv)==1:
        scriptFile = config["path"]["lua_path"]
    elif len(sys.argv)==2:
        scriptFile=config["path"]["lua_path"]
        if sys.argv[1] == 'False':
            train = False
            settings.train = False
        else :
            train = True
            settings.train = True
    elif len(sys.argv)==3:
        option=sys.argv[1]
        scriptFile=sys.argv[2] 
    uiscale=1.5
    
    if train :
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    else:
        device = torch.device("cpu")
        
    if platform.system()=='Darwin':
        m.createMainWin(int((600+220)*uiscale),int((400+100)*uiscale), int(600*uiscale), int(400*uiscale),uiscale, "../Resource/ogreconfig_mac.txt", "plugins_mac.cfg", "ogre_mac.cfg")
    else:
        if option=='--sep':
           m.createMainWin(int((10+220)*uiscale),int((400+100)*uiscale), int(10*uiscale), int(400*uiscale),uiscale, "../Resource/ogreconfig_linux_sepwin.txt", "plugins_linux.cfg", "ogre_linux.cfg")
        else:
            m.createMainWin(int((1024+180)*uiscale),int((600+100)*uiscale), int(1024*uiscale), int(600*uiscale),uiscale)
    m.showMainWin()
    if scriptFile[0:1]!='.':
        scriptFile=os.path.relpath(scriptFile, os.getcwd())
    if scriptFile[-3:]!='lua':
        scriptFile=scriptFile+'.lua'
    l=m.getPythonWin()

    print('loading', scriptFile)
    #loadScrit == ctor 까지 수행됨
    l.loadScript(scriptFile)
    # mode_setting function
    # python에서 argument, return 받아오는 법
    # getglobal ("함수 이름")
    l.getglobal('mode_setting')
    # push (넣으려고 하는 parameter "스택"형식으로 쌓기)
    l.push(settings.train)
    # call로 불러 오면서 return (앞이 인자개수, 리턴 개수)
    l.call(1,0)
    # 써진 반대로 (스택이니까) pop! 

    mocap_data = torch.from_numpy(Tonumpy(Alldata_load())).float().to(device)
    if torch.isnan(mocap_data).any():
        print("mocap data has nan value")
        nan_mask = torch.isnan(mocap_data)
        nan_count = nan_mask.sum().item()
        mocap_data = torch.where(torch.isnan(mocap_data), torch.tensor(0.0).to(device), mocap_data)

    discont = Tonumpy(Discontinuity())
    discont = list(map(int,discont))
    mocap_size = mocap_data.shape[0] - 200 #numframes = 26909
    frame_size = mocap_data.shape[1] #frame_size = 35
    input_size = frame_size
    output_size = frame_size

    ######data normalize##########
    avg = mocap_data.mean(dim=0)
    std = mocap_data.std(dim=0)
    for i in range(len(std)):
        if std[i] == 0:
            std[i] = 1
    mocap_data = (mocap_data-avg)/std
    ##############################
    vqvae = VQVAE(input_size,hidden_dim, latent_size, output_size, codebook_size, latent_size, beta, False, input_frames, num_experts, True, False, device).to(device)
    vqvae.set_normalization(std.to(device="cpu"),avg.to(device="cpu"))

    if load_save_model :
        print("loading model")
        vqvae = torch.load("vqvae_model_10_large_to_one.pt",map_location=device)

    vqvae.train()
    vqvae_optimizer = optim.Adam(vqvae.parameters(),lr=initial_lr)
    scheduler_lr = optim.lr_scheduler.StepLR(vqvae_optimizer,step_size=10,gamma=0.9)
    writer = SummaryWriter(comment="MVQVAE"+"\nenv_name" + scriptFile + "\nbatch size"+str(num_epochs) + "\nmini batch size" + str(mini_batch) + "\nlearning rate"+str(vqvae_optimizer.param_groups[0]['lr']))
    all_indices = np.linspace(0,mocap_size-1,mocap_size)
    bad_indices = []
    for i in range(len(discont)-1):
        for j in range(12):
            bad_indices.append(discont[i+1] - j)
    bad_indices.sort()
    good_mask = np.isin(all_indices,bad_indices,assume_unique=True,invert=True)
    selectable_indices = all_indices[good_mask]
    sample_schedule = torch.cat(
    (
        # First part is pure teacher forcing
        torch.zeros(teacher_epochs),
        # Second part with schedule sampling
        torch.linspace(0.0,1.0,ramping_epochs),
        # last part is pure student
        torch.ones(student_epochs),
    ))
    
    print(selectable_indices)
    if train:
        shape = (mini_batch,condition_frame,output_size)
        input_shape = (mini_batch,input_frames,output_size)
        history = torch.empty(shape).to(device)
        encode_buffer = torch.empty(input_shape).to(device)
        
        start = time.time()
        
        for ep in range(1,num_epochs+1):
            sampler = BatchSampler(SubsetRandomSampler(selectable_indices),mini_batch,drop_last=True)
            ep_recon_loss = 0
            ep_q_loss = 0
            
            num_of_minibatch = 1  
            
            for num_of_minibatch,indices in enumerate(sampler):
                t_indices = torch.LongTensor(indices)

                t_indices += 1 
                condition_range = (
                t_indices.repeat((1, 1)).t()
                + torch.arange(0, -1, -1).long()
                )
                history[:,:1].copy_(mocap_data[condition_range])
                in_range =condition_range
                
                
                for i in range(1, input_frames):
                    in_range = torch.cat((in_range[:,0].unsqueeze(1)-1,in_range),dim=1)
                    
                encode_buffer = mocap_data[in_range]
                
                #바꿔  
                for offset in range(1,prediction_frames):
                    use_student = torch.rand(1) < sample_schedule[ep - 1]
                    prediction_range = (
                            t_indices.repeat((1, 1)).t()
                            + torch.arange(offset, offset + 1).long()
                        )
                    ground_truth = mocap_data[prediction_range]
                    conditionfr = encode_buffer[:,input_frames-1]
                    gt = conditionfr
                    
                    
                    ground_truth = ground_truth.flatten(start_dim=1,end_dim=2)
                    curr_pose = encode_buffer.flatten(start_dim=1, end_dim=2)
                    loss, x_hat = vqvae(curr_pose, conditionfr)
                    history = history.roll(1,dims=1)

                    next_frame = x_hat if use_student else ground_truth
                    encode_buffer = encode_buffer.roll(-1, dims=1)
                    encode_buffer[:,input_frames-1].copy_(next_frame.detach())
                    
                    history[:,0].copy_(next_frame.detach())
                    recon_loss = (x_hat - ground_truth).pow(2).mean(dim=(0,-1))
                    recon_loss = recon_loss.sum()
                    
                    
                    vqvae_optimizer.zero_grad()
                    (recon_loss+loss).backward()
                    vqvae_optimizer.step()
                    
                    ep_q_loss = loss
                    ep_recon_loss += float(recon_loss) / prediction_frames
                   
            avg_recon_loss = ep_recon_loss / mini_batch
            avg_kl_loss = ep_q_loss / mini_batch
            scheduler_lr.step()
            end = time.time()

            print("epoch : {ep}, ep_recon_loss : {ep_recon_loss:0.08f},quat_loss : {q:0.08f}  learning_rate : {lr:0.07f} , FPS : {FPS}".format(ep=ep,ep_recon_loss=avg_recon_loss,q=ep_q_loss,lr=vqvae_optimizer.param_groups[0]['lr'],FPS=int((ep/(end-start))*100)))
            writer.add_scalar('ep_recon_loss',avg_recon_loss,ep)
            writer.add_scalar('ep_kl_loss',avg_kl_loss,ep)
            writer.add_scalar('learning_rate',vqvae_optimizer.param_groups[0]['lr'],ep)
            torch.save(copy.deepcopy(vqvae).cpu(), pt_path)
    else:
        settings.VQVAELTO = torch.load(pt_path, map_location='cpu').to(device)
        settings.VQVAELTO.dct_m = settings.VQVAELTO.dct_m.to(device)
        settings.VQVAELTO.idct_m = settings.VQVAELTO.idct_m.to(device)
        settings.VQVAELTO.mask = settings.VQVAELTO.mask.to(device)
        
        
        codebook = settings.VQVAELTO.VQBottom.embeddings.weight
        
        codebook = codebook.detach().numpy()
        
        
        m.startMainLoop()
        
  
    
def getTrain(flag):
    if settings.train == True:
        tmp = np.array([1])
        flag.ref()[:] = tmp
    else:
        tmp = np.array([0])
        flag.ref()[:] = tmp
        
        
    return flag


    
    
def test_vqvaelto(input, conditionfr, vqvae_output):
    with torch.no_grad():
        settings.VQVAELTO.eval()
        
        conditionfr = torch.tensor(Tonumpy(conditionfr)).float()
        conditionfr=settings.VQVAELTO.normalize(conditionfr)
        condition=conditionfr.view(1,-1)
        inp = torch.tensor(Tonumpy(input)).float()
        inp = inp.view(1,-1)

        real_inp = inp
        for i in 1 , 10:
            tmpinput = inp[:,35*(i-1):35*i]
            tmpinput = settings.VQVAELTO.normalize(tmpinput)
            real_inp[:,35*(i-1):35*i] = tmpinput
        

        _, output = settings.VQVAELTO(real_inp, condition)
        
        output = settings.VQVAELTO.denormalize(output)
        
        # # newoutput = newoutput.squeeze()
        # output = output[:,35*9:35*10]
        # output = settings.VQVAE.denormalize(output)
                
        newoutput = output.detach().numpy()
        
        vqvae_output.ref()[:] = newoutput
        
        return vqvae_output
    
    
    

if __name__=="__main__":
    main()
