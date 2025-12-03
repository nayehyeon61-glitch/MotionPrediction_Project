import pickle
import copy
from json import load
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
from gym_mp.models import *
import torch.optim as optim
from torch.utils.data.sampler import BatchSampler, SubsetRandomSampler , RandomSampler
from tensorboardX import SummaryWriter 


def Tonumpy(data):
    if data.ref().ndim == 1 :
        return np.array(data.ref())
    else :
        numpy_data = np.empty((data.rows(),data.cols()))
        numpy_data[:data.rows(),:data.cols()]=data.ref()
        return numpy_data

def All_hand_data_load():
    l=m.getPythonWin()
    l.getglobal("All_hand_data_frames")
    l.call(0,1)
    return l.popmatrixn()

def All_pose_data_load():
    l=m.getPythonWin()
    l.getglobal("All_data_frames")
    l.call(0,1)
    return l.popmatrixn()

def All_tracker_data_load():
    l=m.getPythonWin()
    l.getglobal("All_tracker_data")
    l.call(0,1)
    return l.popmatrixn()

def main():
    if len(sys.argv)==2:
        scriptFile=sys.argv[1]
    elif len(sys.argv)==3:
        option=sys.argv[1]
        scriptFile=sys.argv[2]
    uiscale=0
    option = ""
    if option=='--sep':
        if platform.system()=='Darwin':
            m.createMainWin(int((10+220)*uiscale),int((400+100)*uiscale), int(10*uiscale), int(400*uiscale),uiscale, "../Resource/ogreconfig_linux_sepwin.txt", "plugins_mac.cfg", "ogre_mac.cfg")
        else:
            m.createMainWin(int((10+220)*uiscale),int((400+100)*uiscale), int(10*uiscale), int(400*uiscale),uiscale, "../Resource/ogreconfig_linux_sepwin.txt", "plugins_linux.cfg", "ogre_linux.cfg")
    else:
        m.createMainWin(int((1024+180)*uiscale),int((600+100)*uiscale), int(1024*uiscale), int(600*uiscale),uiscale)
    m.showMainWin()
    if scriptFile[0:1]!='.':
        scriptFile=os.path.relpath(scriptFile, os.getcwd())
    if scriptFile[-3:]!='lua':
        scriptFile=scriptFile+'.lua'
    print('loading', scriptFile)
    m.getPythonWin().loadScript(scriptFile)
    train = True
    device = 'cpu'
    pose_data = torch.from_numpy(Tonumpy(All_pose_data_load())).float().to(device)
    print(pose_data.shape)
    hand_data = torch.from_numpy(Tonumpy(All_hand_data_load())).float().to(device)
    print(hand_data.shape)
    frame_size = pose_data.shape[0]
    pose_size = pose_data.shape[1]
    hand_size = hand_data.shape[1]
    mini_batch = 1024
    iteration = 5
    condition_size =3 
    epochs = 500
    load_model = False
    model = NormalNN(pose_size*3,64,hand_size)
    optimizer = optim.Adam(model.parameters(),lr=1e-4)
    scheduler_lr = optim.lr_scheduler.StepLR(optimizer=optimizer,step_size=10,gamma=0.95)
    
    if load_model :
        model = torch.load("hand_model.pt")
        print("pretrained model loaded")

    all_indices = np.linspace(0,frame_size-1,frame_size)
    bad_indices = []
    for i in range(condition_size):
        bad_indices.append(frame_size-i)

    good_mask = np.isin(all_indices,bad_indices,assume_unique=True,invert=True)
    selectable_indices = all_indices[good_mask]

    if train:
        shape = (mini_batch,condition_size,pose_size)
        history = torch.empty(shape).to(device)
        model.train()
        for ep in range(1,epochs+1):
            sampler = BatchSampler(SubsetRandomSampler(selectable_indices),mini_batch,drop_last=True)
            ep_recon_loss = 0
            for step,indices in enumerate(sampler):
                t_indices = torch.LongTensor(indices)
                condition_frame = (t_indices.repeat((3,1)).t())
                for i in range(condition_size):
                    condition_frame[:,i] += i
                    history[:,i,:].copy_(pose_data[condition_frame[:,i]])

                t_indices += 1
                for offset in range(iteration):
                    prediction_frame = t_indices
                    ground_truth = hand_data[prediction_frame,:]
                    output = model(history)
                    recon_loss = F.mse_loss(output,ground_truth)
                    optimizer.zero_grad()
                    (recon_loss).backward()
                    optimizer.step()
                    ep_recon_loss += float(recon_loss)/iteration

            avg_vae_loss = ep_recon_loss/mini_batch
            scheduler_lr.step()
            print("epochs : {ep} , ae_loss : {avg_vae_loss:0.08f} ,  lr : {lr:0.08f}".format(ep = ep,avg_vae_loss = avg_vae_loss,lr=optimizer.param_groups[0]['lr']))
            torch.save(copy.deepcopy(model).cpu(),"hand_model.pt")

if __name__ == "__main__":
    main()

