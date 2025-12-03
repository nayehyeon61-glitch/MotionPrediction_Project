local turnScript={}

--local frameno=math.floor(102/4)
local frameno=math.floor(52/4)
local turnRatio= scriptParamTurn
print(scriptParamTurn)
turnScript[frameno]=function(self) 
	print('turn', turnRatio)
	rigidBodyWin("dostring","mSynthesis.desiredTurningSpeed="..tostring(turnRatio))
end

return turnScript
