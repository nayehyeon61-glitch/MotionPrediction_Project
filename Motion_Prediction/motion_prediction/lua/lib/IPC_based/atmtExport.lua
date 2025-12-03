local script={}

local startFrame=startFrame or 32 -- discard first 32 frames

script.before_ctor=function(self)
end

script.after_ctor = function(self)

	rigidBodyWin("dostring", [[

	local dofFile=chosenMotFile
	if dofFile then
		_exportBVH(string.sub(dofFile,1,-4).."bvh")
		local startF=]]..startFrame..[[
		local endF=mMotionDOFcontainer:numFrames()

		local mot=mMotionDOFcontainer.mot:copy()
		mMotionDOFcontainer:resize(endF-startF)
		mMotionDOFcontainer.mot:assign(mot:range(startF, endF))
		mSkin:applyMotionDOF(mMotionDOFcontainer.mot)

		RE.motionPanel():motionWin():detachSkin(mSkin)
		RE.motionPanel():motionWin():addSkin(mSkin)
	end
	]])
end
return script
