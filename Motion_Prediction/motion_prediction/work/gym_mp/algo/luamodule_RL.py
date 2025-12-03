import pdb # use pdb.set_trace() for debugging
#import code # or use code.interact(local=dict(globals(), **locals())) for debugging. see below.
import numpy as np
import libmainlib as m 
import importlib

def lua_step(frameNum,inputdata):
    l=m.getPythonWin()
    if not l.isLuaReady() : return
    l.getglobal("step")
    actionvec=m.vectorn()
    actionvec.setSize(len(inputdata))
    for i in range(0,len(inputdata)):
        actionvec.set(i,np.float64(inputdata[i]))
    l.push(frameNum)
    l.push(actionvec)
    l.call(2,3)
    return l.popnumber(), l.popnumber(), l.popvectorn()

def lua_initenv():
    l=m.getPythonWin()
    if not l.isLuaReady() : return
    l.getglobal("init_env")
    l.call(0,0)

def lua_reset():
    l=m.getPythonWin()
    if not l.isLuaReady() : return
    l.getglobal("reset")
    l.call(0,1)
    return l.popvectorn()


def python_vectorn(vec):
    l=m.getPythonWin()
    if not l.isLuaReady() : return
    l.getglobal("python_vectorn")
    l.push(vec)
    l.call(1,1)
    return l.popvectorn()

def python_render(renderBool):
    l=m.getPythonWin()
    if not l.isLuaReady() : return
    l.getglobal("python_render")
    l.push(renderBool)
    l.call(1,0)
