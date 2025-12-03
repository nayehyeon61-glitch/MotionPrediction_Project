from importlib.metadata import entry_points
from gym.envs.registration import register
import gym_mp.luaEnv 

register( id='mvae-v0', entry_point='gym_mp.luaEnv:LuaEnv') 
register( id='seokmimic-v0', entry_point='gym_mp.luaEnv:LuaEnv') 
register( id='dance-v0', entry_point='gym_mp.luaEnv:LuaEnv') 
