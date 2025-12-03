package.path=package.path..";../Samples/ysscripts/samples/?.lua" --;"..package.path
require("driverDynamicMuscle")

function frameMove(fElapsedTime)
	mSynthesis:oneStep()
	mSynthesis:oneStepSimul()
	if error_feedback_method~=EFM.NONE then	 
		mSynthesis:prepareNextStep()      
	end
end

function EVR:onFrameChanged(win, iframe)
end
