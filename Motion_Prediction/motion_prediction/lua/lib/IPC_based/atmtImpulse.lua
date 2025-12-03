local impulseScript={}

local frameno=math.floor(200/4)
local impMag=impulseMagnitude or 150
impulseScript[frameno]=function(self) 
	print('impulse', impMag)
	rigidBodyWin("dostring", [[
	local impulseDuration=0.2
	local impulseMagnitude=]]..tostring(impMag)..[[
	mSynthesis.impulse=math.floor(impulseDuration*model.simulationFrameRate)
	mSynthesis.impulseDir=vector3(1,0,0)*impulseMagnitude
	mSynthesis.impulseGizmo=mSynthesis.objectList:registerEntity("arrow2", "arrow2.mesh")
	if mSynthesis.impulseGizmo then mSynthesis.impulseGizmo:setScale(2,2,2) end
	]])
end

return impulseScript
