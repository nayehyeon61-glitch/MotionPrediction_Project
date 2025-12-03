require('console')
function ctor()

	skel=MainLib.VRMLloader('../Resource/motion/opensim/FullBody2_lee.wrl')
	for i=1,100 do
		simulator=Physics.DynamicsSimulator_gmbs() 
		--self.simulator=Physics.DynamicsSimulator_AIST_penalty() 
		simulator:registerCharacter(skel)
		simulator:setGVector(vector3(0,9.8,0)) 
		simulator:init(0.01, Physics.DynamicsSimulator.EULER) 
		simulator=nil
	end
	print('finished')
end


