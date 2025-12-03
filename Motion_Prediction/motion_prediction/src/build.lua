#!lua
EXE='QP_controller'

local f=io.open('../make.lua','r')
if f then
	f:close()
	luna_script_custom= 
	dofile('../make.lua')
else
	dofile('../../make.lua')
end
