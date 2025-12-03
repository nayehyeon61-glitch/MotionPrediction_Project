# this file contains a line-by-line python port of showSkinningWeights.lua (taesooLib/Samples/classification/lua/)
import os
import sys
import pdb # use pdb.set_trace() for debugging
#import code # or use code.interact(local=dict(globals(), **locals())) for debugging. see below.
import math
import random
from multiprocessing import Queue

from pathlib import Path

workPath=Path(os.getcwd()).parent.joinpath("work")

# script paths should be in path
sys.path.append(os.getcwd())
sys.path.append(Path(os.getcwd()).joinpath("ETRI_viewer"))
sys.path.append(Path(os.getcwd()).joinpath("ETRI_viewer").joinpath("make_120_bvh"))
sys.path.append(Path(os.getcwd()).joinpath("ETRI_viewer").joinpath("make_120_csv"))

os.chdir(workPath) # mainlib은 gui사용시 반드시 work폴더에서 실행되야해서
sys.path.append(os.getcwd())

import libmainlib as m   
import luamodule as lua  # see luamodule.py
import rendermodule as RE # see rendermodule.py
import controlmodule as control
import numpy as np
import copy
from ETRI_viewer.emulation import EmulationModeNoThread
from make_120_bvh.xsens_120_bvh import XsensBVH
from make_120_bvh.hierarchy import xsens_etri_hierarchy
from easydict import EasyDict as edict # pip3 install easydict
import re
from datetime import datetime
import pdb

def file_path_to_timestamp(file_path):
    match = re.search(r'(\d{4}_\d{2}_\d{2}_\d{2}_\d{2}_\d{2})', file_path)

    if match:
        date_time_str = match.group(1)
        date_time = datetime.strptime(date_time_str, "%Y_%m_%d_%H_%M_%S")
        return date_time
    else:
        print("file Path to timestamp err")


def onFrameChanged(iframe):
    global loader, skin, skinScale, emulation, xsens_BVH_queue, xsens_BVH, timeline

    try:
        data, message=emulation.getOneData()
        if data and 'xsens' in data:
            xsens_BVH_queue.put(data)
            pose=xsens_BVH.make_posedof(xsens_BVH_queue,loader)
            skin.setPoseDOF(pose)
    except Exception as e:
        print("Error", str(e))

#lua require should be in the ctor function
def ctor(this):
    global loader, skin, skinScale, emulation, xsens_BVH_queue, xsens_BVH, timeline
    bgnode=RE.ogreSceneManager().getSceneNode("BackgroundNode")
    #bgnode.setVisible(False)
    #gtk.chooseFile('choose a script file', '../Resource/motion/opensim/' ,'*.dof', false)
    #file_path='../python/ETRI_viewer/SoftSuit_LE_2024_05_30_16_35_37.bin'
    #file_path='../python/ETRI_viewer/SoftSuit_LE_2024_05_08_13_41_50.bin'
    #file_path='../python/ETRI_viewer/SoftSuit_LE_2023_11_15_09_29_14.bin'
    file_path=RE.fileChooser()
    
    emulation=EmulationModeNoThread(file_path)
    emulation.readBin()

    xsens_BVH_queue = Queue()
    xsens_BVH=XsensBVH()

    # change False to True to test file output (using ETRI code)
    if False:
        while True:
            data, message=emulation.getOneData()
            if 'xsens' in data:
                xsens_BVH_queue.put(data)
            click_timestamp=file_path_to_timestamp(file_path)
            xsens_BVH.make_bvh(xsens_BVH_queue, str(click_timestamp))

    loader=RE.BVHloader(xsens_etri_hierarchy()+'\nFrames: 0\n Frame Time: 0.008333\n', 'loadFromMemory')

    # set True to test bvh file output (using taesooLib)
    if False:
        mot=m.Motion (loader)
        mot.resize(100)
        for i in range(100):
            data, message=emulation.getOneData()
            if 'xsens' in data:
                xsens_BVH_queue.put(data)
                pose=xsens_BVH.make_posedof(xsens_BVH_queue,loader)
        lua.M(mot, 'exportBVH','test.bvh')


    timeline=RE.Timeline('timeline', 10000, 0.008333)

    skinScale=1

    this.create("Button", 'printHierarchy', 'printHierarchy')

    this.setWidgetHeight(100)
    this.create("Multi_Browser", "body parts", "body parts") # or you can instead use Select_Browser for single select.
    if True:
        mBrowser=this.widget(0)
        mBrowser.browserClear()
        for i in range(1, loader.numBone()-1 +1):
            mBrowser.browserAdd(loader.bone(i).name())

    this.updateLayout()

    skin=RE.createSkin(loader)
    skin.setScale(skinScale,skinScale,skinScale)
    skin.setMaterial('lightgrey_verytransparent')
    
    skin.setTranslation(0,0,0)

def onCallback(w, usrData):
    global skin

    if w.id()=="body parts":
        for i in range(1, loader.numBone()-1 +1):
            if w.browserSelected(i):
                print(loader.bone(i).name(),w.browserText(i))
                #skin.setBoneMaterial(i, 'red_transparent')
            else:
                pass
                #skin.setBoneMaterial(i, 'lightgrey_verytransparent')
    elif w.id()=='printHierarchy':
        loader.printHierarchy()

def main():
    RE.createMainWin(sys.argv)
    ctor(m.getPythonWin())
    print('ctor finished')
    m.startMainLoop() # this finishes when program finishes

if __name__=="__main__":
	main()
