
useCases.stand_common={
	k_p_ID=150,
	k_d_ID=10,
--	COPcontroller_kCoef=10,
--	COPcontroller_dampingCoef=50,
	excludeRoot=true,
	footPosFeedbackMethod=2, -- experimental
	noToeJoints=true,
	useQPsolver=true,
	useAnalyticIK=true,
	ddqObjWeight=10000,
	tauObjWeight=0.0001,
	EEobjWeight=60,
	EEobjWeightAngular=60,
	dotMomentumScale=1,
	k_d_momentum=10,
	lambdaObjWeight=0.0001,
	momentumWeight=8000,
	linearMomentumWeight=8000,
	momentumThr=150,
	velMarginOffset=0,
	maxPenetratingVel=0,
	useBulletColdet=false, -- bullet collision detector is faster but less accurate.
	contactMargin=0.01,
	invAccConeCoef=0,

	keyframes={
		desiredMomentum={numKey=2, default=vector3(0,0,0)},
		desiredDotMomentum={numKey=2, isContinuous=false,default=vector3(0,0,0)},
		--desiredLeaning={numKey=2, isContinuous=false,default=vector3(math.rad(5),0,math.rad(-1))},
		--desiredLeaning={numKey=2, isContinuous=false,default=vector3(math.rad(3),0,math.rad(-1))},
		desiredLeaning={numKey=2, isContinuous=false,default=vector3(0,0,0)},
		--desiredLeaning={numKey=2, isContinuous=false,default=vector3(0,0,0)}
	},
	useFootEncodingMethod2=
	{
		true,
		geom={
			L={ 
				{"lfoot", vector3(0,-0.03,0.02)},
				--{"ltoes", vector3(0,-0.03,0.02)},
				{"lfoot", vector3(0,-0.03,0.09)},
			},
			R={
				{"rfoot", vector3(0,-0.03,0.02)},
				--{"rtoes", vector3(0,-0.03,0.02)},
				{"rfoot", vector3(0,-0.03,0.09)},
			},
		}
	},
}
useCases.stand_common.init_globals = function ()
	outputSuperSample=1
	PDservoLatency=5-- for debugging. set 5 for normal use.
	IDservoLatency=0
	COPlatency=10
	PredictionLatency=0
	disableTouchDownFeedback=false
	model.simulationFrameRate=120
	model.penaltyForceStiffness=30000
	model.penaltyForceDamp=3000
	model.k_p_ID=200
	model.k_d_ID=20
	model.k_p_PD=100
	model.k_d_PD=5
end

function useCases.stand_common:createGraph()
	local graph=array:new()
	local nseg=table.getn(self.segmentation.firstFrames)-3
	
	graph:pushBack({"addInterpolatedSegment", 
	grpName="STP", 
	name="ts2", 
	seg0={"STP", "s"..nseg},
	seg1={"STP", "s1"},
	startWeight=0,
endWeight=1 -- 0 denotes seg0, 1 denotes seg1, 0.5 denotes inbetween.
	})

	graph:pushBack({"beginGroup", "STP"})

	for i=1, nseg-2 do
		graph:pushBack({"connect", "s"..i, "s"..(i+1)})
	end
	if nseg==2 then
		graph:pushBack(
		{"connect", "s"..(nseg-1), "ts2"},
		{"connect", "ts2", "ts2"},
		{"initialSegment", "STP", "s1"})
	else
		graph:pushBack(
		{"connect", "s"..(nseg-1), "ts2"},
		{"connect", "ts2", "s2"},
		{"initialSegment", "STP", "s1"})
	end
	return graph
end

useCases.stand_common.calcUpperBoundLocal =function(fixedLocal)
	return fixedLocal+0.1
end

useCases.stand_common.usePositionControl=true
useCases.stand_common.desiredPos=vector3(0,0,0)
useCases.stand_common.virtualForceCorrection=vector3(0,0,0)
useCases.stand_common.grpName='STP'

useCases.stand_common.controlParam={
}
useCases.stand1=table.merge(deepCopyTable(useCases.stand_common),{
	scenario=scenarios.STAND1,
	mot_file="../Resource/scripts/ui/RigidBodyWin/justin_jump.dof",
	segmentation={
		grpName="STP",
		firstFrames={ 0, 100, 200, 300, 400, },
		names={           's1', 's2'},
		swingL={           0,    0},
		swingR={           0,    0},
		footRefL= "convertFromSwingL",
		footRefR= "convertFromSwingR",
	},
	pendControlParam={velx_STP_s1=0, velz_STP_s1=0}
})
useCases.stand2=table.merge(deepCopyTable(useCases.stand_common),{
	scenario=scenarios.STAND2,
	mot_file="../Resource/scripts/ui/RigidBodyWin/justin_jump.dof",
	segmentation={
		grpName="STP",
		firstFrames={ 7441, 7666, 7886, 8121, 8380, 8674},
		names={               's1', 's2',  's3'},
		fillLeft=7441,
		swingL={           0,    0, 0},
		swingR={           0,    0,0},
		footRefL= "convertFromSwingL",
		footRefR= "convertFromSwingR",
	},
	pendControlParam={velx_STP_s1=0, velz_STP_s1=0, velx_STP_s2=0, velz_STP_s2=0},
--	keyframes={
--		desiredLeaning={numKey=2, isContinuous=false,default=vector3(math.rad(4),0,math.rad(14))}
--	}
})
useCases.stand3=table.merge(deepCopyTable(useCases.stand_common),{
	scenario=scenarios.STAND3,
	mot_file="../Resource/scripts/ui/RigidBodyWin/justin_jump.dof",
	segmentation={
		grpName="STP",
		firstFrames={ 11562, 11762, 11962, 12162, 12362, 12562}, 
		names={                  's1',   's2', 's3'},
		fillLeft=11562,
		swingL={           0,    0, 0},
		swingR={           0,    0,0},
		footRefL= "convertFromSwingL",
		footRefR= "convertFromSwingR",
	},
	pendControlParam={velx_STP_s1=0, velz_STP_s1=0, velx_STP_s2=0, velz_STP_s2=0}
})
local function setSelf(useCase)
	useCase.grpNameToUseCase={}
	useCase.grpNameToUseCase.STP=useCase
end
setSelf(useCases.stand1)
setSelf(useCases.stand2)
setSelf(useCases.stand3)

do
	local function accumulate(cp_mod)
		local useCase=useCases.stand3
		local cp_mod2={}
		for k,v in pairs(cp_mod) do
			cp_mod2[k]=v
		end
		useCase.controlParam=useCases.mergeCP(useCase.controlParam, cp_mod2)
	end
--	accumulate({['keyframe,1,desiredLeaning,STP,s1,x']=math.rad(7)}) 
--	accumulate({['keyframe,0,desiredLeaning,STP,s2,z']=math.rad(0)}) 
end
