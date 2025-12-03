import pickle
import copy
import time 
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

def Discontinuity():
    l = m.getPythonWin()
    l.getglobal("getDiscontinuity")
    l.call(0,1)
    return l.popvectorn()

def All_tracker_data_load():
    l=m.getPythonWin()
    l.getglobal("All_tracker_data")
    l.call(0,1)
    return l.popmatrixn()

def All_mocap_data_load():
    l=m.getPythonWin()
    l.getglobal("All_data_frames")
    l.call(0,1)
    return l.popmatrixn()

def tracker2local(centerCoord,tracker_data):
    centerCoord = np.array(centerCoord)
    tracker_data = np.array(tracker_data)
    l=m.getPythonWin()
    l.getglobal("changeTrackerLocal")
    center = m.vectorn()
    center.setSize(len(centerCoord))
    center.ref()[:] = centerCoord
    tracker_data_l = m.vectorn()
    tracker_data_l.setSize(len(tracker_data))
    tracker_data_l.ref()[:] = tracker_data
    l.push(center)
    l.push(tracker_data_l)
    l.call(2,1)
    return l.popvectorn()

def pose2local(centerCoord,pose):
    centerCoord = np.array(centerCoord)
    pose = np.array(pose)
    l=m.getPythonWin()
    l.getglobal("changePoseLocal")
    center = m.vectorn()
    center.setSize(len(centerCoord))
    center.ref()[:] = centerCoord
    pose_l = m.vectorn()
    pose_l.setSize(len(pose))
    pose_l.ref()[:] = pose
    l.push(center)
    l.push(pose_l)
    l.call(2,1)
    return l.popvectorn()

def changeroatation(tracker_data):
     tracker_data = np.array(tracker_data)
     l=m.getPythonWin()
     l.getglobal('changeTrackerRotation')
     tracker = m.matrixn()
     tracker.setSize(tracker_data.shape[-2],tracker_data.shape[-1])
     tracker.ref()[:]=tracker_data
     l.push(tracker)
     l.call(1,1)
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
    vae_train = True
    latent_size = 512 
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    #device = torch.device('cpu')
    condition_frame =  30
    #tracker_data = torch.from_numpy(Tonumpy(All_tracker_data_load())).float().to(device)
    #if torch.isnan(tracker_data).any() :
    #    pdb.set_trace()
    mocap_data = torch.from_numpy(Tonumpy(All_mocap_data_load())).float().to(device)
    discont = Tonumpy(Discontinuity())
    discont = list(map(int,discont))
    frame_size = mocap_data.shape[0]-100
    #tracker_size = tracker_data.shape[1]
    mocap_size = mocap_data.shape[1]
    print("mocap_size:", frame_size)
    print("pose_size:", mocap_size)
    #print("tracker size:" , tracker_size )
    mini_batch = 512
    teacher_epochs = 100
    ramping_epochs = 200 
    student_epochs = 100
    output_size = mocap_size
    iteration = 5
    vae_epochs = teacher_epochs+ramping_epochs+student_epochs
    
    ae = TrackerAuto(mocap_size,30,1024,latent_size,1024,output_size*30).to(device)
    ae_optimizer = optim.Adam(ae.parameters(),lr=1e-4)
    scheduler_lr_ae = optim.lr_scheduler.StepLR(optimizer=ae_optimizer,step_size=20,gamma=0.95)
    
    all_indices = np.linspace(0,frame_size-1,frame_size)
    bad_indices = []
    for i in range(len(discont)-1):
        for j in range(65):
            bad_indices.append(discont[i+1] - j)
    bad_indices.sort()
    bad_indices = np.unique(bad_indices)
    print("bad_indices:",bad_indices)
    good_mask = np.isin(all_indices,bad_indices,assume_unique=True,invert=True)
    selectable_indices = all_indices[good_mask]
    prediction_frame = 30 
    # print(selectable_indices)

    if vae_train:
        ae.train()
        shape = (mini_batch,condition_frame,mocap_size)
        history = torch.empty(shape).to(device)
        start = time.time()
        for ep in range(1,vae_epochs+1):
            sampler = BatchSampler(SubsetRandomSampler(selectable_indices),mini_batch,drop_last=True)
            ep_recon_loss = 0
            for step,indices in enumerate(sampler):
                t_indices = torch.LongTensor(indices)
                condition_range = (t_indices.repeat((30, 1)).t()
                + torch.arange(0, -1, -1).long()
                )
                for i in range(condition_frame):
                    condition_range[:,i] += i
                    history[:,i,:].copy_(mocap_data[condition_range[:,i]])
                
                #t_indices += 4
                for offset in range(iteration):
                    ground_truth = mocap_data[condition_range + prediction_frame,:]
                    output = ae(history)
                    output = output.view(-1,condition_frame,output_size)
                    recon_loss = F.mse_loss(output,ground_truth)
                    ae_optimizer.zero_grad()
                    (recon_loss).backward()
                    ae_optimizer.step()
                    ep_recon_loss += float(recon_loss)/iteration

            end = time.time()
            avg_ae_loss = ep_recon_loss/mini_batch
            scheduler_lr_ae.step()
            print("epochs : {ep} , ae_loss : {avg_ae_loss:0.08f} ,  lr : {lr:0.08f},  FPS : {FPS}".format(ep = ep,avg_ae_loss = avg_ae_loss,lr=ae_optimizer.param_groups[0]['lr'],FPS=int((ep/(end-start))*100)))
            torch.save(copy.deepcopy(ae).cpu(),"trained_models/MotionPrediction.pt")

if __name__ == "__main__":
    main()

