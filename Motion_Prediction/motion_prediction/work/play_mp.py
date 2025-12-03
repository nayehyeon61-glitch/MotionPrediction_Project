import copy
from doctest import OutputChecker
from multiprocessing import Condition
import os
from random import randint
import sys
import platform
import pdb
from unittest import loader # use pdb.set_trace() for debugging
sys.path.append(os.getcwd())
import libmainlib as m   
import luamodule as lua  # see luamodule.py
import numpy as np 
import torch 
from gym_mp.models import *
import torch.optim as optim
import settings

# ----------------trained models-------------
settings.hand_model = torch.load("trained_models/hand_model.pt")
#settings.TrackerAE10 = torch.load("trained_models/TrackerAE10.pt")
settings.MotionPrediction = torch.load("trained_models/MotionPrediction.pt")
#settings.TrackerAE5 = torch.load("trained_models/TrackerAE5.pt")
settings.VAE10 = torch.load("trained_models/vae_model_10.pt")
#settings.VAE10 = torch.load("trained_models/vae_model_10_physics.pt")
#settings.VAE8 = torch.load("trained_models/vae_model_8.pt")
#settings.TrackerCNN = torch.load("trained_models/TrackerCNN20.pt")
#settings.Tracker_matching = torch.load("trained_models/tracker_matching.pt")
###########################################3
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
    l.getglobal("getIframeDOF")
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
    if data.ref().ndim == 1 :
        return np.array(data.ref())
    else :
        numpy_data = np.empty((data.rows(),data.cols()))
        numpy_data[:data.rows(),:data.cols()]=data.ref()
        return numpy_data

def test_AutoEncoder(tracker_data,output):
    with torch.no_grad():
        settings.TrackerAE10.eval()
        data = torch.tensor(Tonumpy(tracker_data)).float()
        data = data.unsqueeze(0)
        model_output = settings.TrackerAE10(data)
        model_output = model_output.view(10,-1)
        model_output=model_output.detach().numpy()
        output.ref()[:]=model_output
        return output

def test_MotionPrediction(tracker_data,output):
    with torch.no_grad():
        settings.MotionPrediction.eval()
        data = torch.tensor(Tonumpy(tracker_data)).float()
        data = data.unsqueeze(0)
        model_output = settings.MotionPrediction(data)
        model_output = model_output.view(30,-1)
        model_output=model_output.detach().numpy()
        output.ref()[:]=model_output
        return output

def test_TrackerMatching(tracker_data,output):
    with torch.no_grad():
        settings.Tracker_matching.eval()
        data = torch.tensor(Tonumpy(tracker_data)).float()
        data = data.unsqueeze(0)
        model_output = settings.Tracker_matching(data)
        model_output = model_output.detach().numpy()
        output.ref()[:] = model_output
        return output

def test_HandModel(tracker_data,output):
    with torch.no_grad():
        settings.hand_model.eval()
        data = torch.tensor(Tonumpy(tracker_data)).float()
        data = data.unsqueeze(0)
        model_output = settings.hand_model(data)
        model_output = model_output.detach().numpy()
        output.ref()[:] = model_output
        return output

def feed_tracker_AE5(tracker_data,output):
    with torch.no_grad():
        settings.TrackerAE5.eval()
        data = torch.tensor(Tonumpy(tracker_data)).float()
        data = data.unsqueeze(0)
        model_output = settings.TrackerAE5(data)
        model_output = model_output.detach().numpy()
        output.ref()[:] = model_output
        return output

def test_tracker_cnn(tracker_data,output):
    with torch.no_grad():
        settings.TrackerCNN.eval()
        data = torch.tensor(Tonumpy(tracker_data)).float()
        data = data.unsqueeze(0)
        model_output = settings.TrackerCNN(data)
        model_output = model_output.detach().numpy()
        output.ref()[:] = model_output
        return output

def play_mvae_seok(latent_vector,condition,vae_output):
    with torch.no_grad():
        settings.VAE8.eval()
        latent_vector = torch.tensor(Tonumpy(latent_vector)).float().to(device="cpu")
        condition = torch.tensor(Tonumpy(condition)).float().to(device="cpu")
        #######################
        condition=settings.VAE8.normalize(condition)
        condition = condition.view(1,-1)
        latent_vector = latent_vector.view(1,-1)
        output = settings.VAE8.sample(latent_vector,condition)
        output=settings.VAE8.denormalize(output) 
        output = output.detach().numpy()
        #######################
        vae_output.ref()[:] = output
        return vae_output

def test_vae(latent,condition_l,vae_output):
    with torch.no_grad():
        settings.VAE10.eval()
        latent_vector = torch.tensor(Tonumpy(latent)).float()
        condition = torch.tensor(Tonumpy(condition_l)).float()
        condition=settings.VAE10.normalize(condition)
        condition=condition.view(1,-1)
        latent_vector=latent_vector.view(1,-1)
        output = settings.VAE10.sample(latent_vector,condition)
        output = settings.VAE10.denormalize(output)
        output = output.detach().numpy()
        vae_output.ref()[:] = output
        return vae_output


def noise_data(x_t,t,output):
    device = "cpu"
    x_t = torch.tensor(Tonumpy(x_t)).float().to(device="cpu") 
    t = torch.tensor(t).long().to(device="cpu") 
    model = GaussianDiffusion(x_t.shape[0],100,x_t.shape[0])
    latent,_ = model.q_sample(x_t,t)
    latent = latent.detach().numpy()
    output.ref()[:] = latent
    return output

def make_latent_vector(latent_output):
    with torch.no_grad():
        latent = torch.empty(latent_output.size())
        #latent.normal_(0,1.0)
        latent = torch.randn(latent_output.size())
        latent = latent.detach().numpy()
        latent_output.ref()[:] = latent
        return latent_output

def make_random_indices(indice):
    indice.setSize(1)
    indice.ref()[0] = randint(108,26000)
    return indice

def main():
    device="cpu"
    scriptFile=sys.argv[1]
    option=''
    if len(sys.argv)==2:
        scriptFile=sys.argv[1]
    elif len(sys.argv)==3:
        option=sys.argv[1]
        scriptFile=sys.argv[2]
    uiscale=1.8

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
    m.startMainLoop() # this finishes when program finishes

if __name__=="__main__":
    main()

