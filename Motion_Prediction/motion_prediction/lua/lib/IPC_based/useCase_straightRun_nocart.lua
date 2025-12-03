do
local function makeOptParam(numKey,...)
	local gtarget=array:new()
	local prec=0.01
	for iid,id in ipairs({...}) do
		for i=0, numKey-1 do
			if string.sub(id,1,3)=="MAP" then
				gtarget:pushBack({"map,"..i..","..string.sub(id,4)..",x",prec})
				gtarget:pushBack({"map,"..i..","..string.sub(id,4)..",y",prec})
				gtarget:pushBack({"map,"..i..","..string.sub(id,4)..",z",prec})
			else
				gtarget:pushBack({"keyframe,"..i..","..id..",x",prec})
				gtarget:pushBack({"keyframe,"..i..","..id..",y",prec})
				gtarget:pushBack({"keyframe,"..i..","..id..",z",prec})
			end
		end
	end
	return gtarget
end


local function objCost(pos)
	local cost=0
	for i=0, pos:size()-1 do
		cost=cost+ math.SQR(math.max(math.abs(pos(i))-1.5,0)) -- correction larger than 90 centimeters are penalized
	end

	return cost
end
objCost=nil
objCost=nil
useCases.justinStraightRun_nocart={
	scenario=scenarios.STRAIGHT_RUN_NOCART, 
	useAnalyticIK=true, 
	grpName='run',
	grpNames={'STP','StR','run'},
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
	lfootpos=vector3(0,-0.03,0.09),
	rfootpos=vector3(0,-0.03,0.09),
	useQPsolver=true,
	footPosFeedbackMethod=2,
	--IKspeedBoost =1,
	useMomentumPreservingIK=false,
	maxContactForceY=980,
	mot_file=package.projectPath.."lua/IPC_based/justin_straight_run_gen.dof",
	desiredPos=vector3(0,0,0),
	pendControlParam={
		['keyframe,0,pendDesiredVel,StR,SR,z']= 1.020344774403, ['keyframe,0,pendDesiredVel,ignore,3,x']= -0.10905512894596, ['keyframe,0,pendDesiredVel,StR,RL,z']= 3.2686560923666, ['keyframe,0,pendDesiredVel,ignore,0,z']= 0.70411361293796, ['keyframe,0,pendDesiredVel,StR,RL,x']= 0.052155023803249, ['keyframe,0,pendDesiredVel,run,LR,x']= -0.17272239572564, ['keyframe,0,pendDesiredVel,ignore,2,z']= 3.1020832829171, ['keyframe,0,pendDesiredVel,StR,LR,x']= -0.046640894059674, ['keyframe,0,pendDesiredVel,run,RL,x']= -0.051987710912684, ['keyframe,0,pendDesiredVel,ignore,1,x']= -0.075849071555617, ['keyframe,0,pendDesiredVel,run,RL,z']= 3.1270298333636, ['keyframe,0,pendDesiredVel,StR,LR,z']= 3.0491768499573, ['keyframe,0,pendDesiredVel,ignore,2,x']= -0.10490981238368, ['keyframe,0,pendDesiredVel,ignore,1,z']= 3.4718132684552, ['keyframe,0,pendDesiredVel,run,LR,z']= 3.1380549224854, ['keyframe,0,pendDesiredVel,ignore,3,z']= 3.0407905321378, ['keyframe,0,pendDesiredVel,StR,SR,x']= -0.17688730551357, ['keyframe,0,pendDesiredVel,ignore,0,x']= -0.34828261551377,
		--['keyframe,0,pendDesiredVel,StR,SR,z']= 1.0020164003019, ['keyframe,0,pendDesiredVel,ignore,3,x']= -0.10438978669485, ['keyframe,0,pendDesiredVel,StR,RL,z']= 3.1811625404291, ['keyframe,0,pendDesiredVel,ignore,0,z']= 0.70556762848892, ['keyframe,0,pendDesiredVel,StR,RL,x']= 0.04459141487781, ['keyframe,0,pendDesiredVel,run,LR,x']= -0.16119394542466, ['keyframe,0,pendDesiredVel,ignore,2,z']= 3.0970010822767, ['keyframe,0,pendDesiredVel,StR,LR,x']= -0.056016466188483, ['keyframe,0,pendDesiredVel,run,RL,x']= -0.040159496989321, ['keyframe,0,pendDesiredVel,ignore,1,x']= -0.086018607468234, ['keyframe,0,pendDesiredVel,run,RL,z']= 3.1600240621885, ['keyframe,0,pendDesiredVel,ignore,0,x']= -0.34149528745167, ['keyframe,0,pendDesiredVel,ignore,2,x']= -0.10877808894455, ['keyframe,0,pendDesiredVel,StR,SR,x']= -0.17459668894925, ['keyframe,0,pendDesiredVel,run,LR,z']= 3.1790398319278, ['keyframe,0,pendDesiredVel,ignore,3,z']= 3.0417811392314, ['keyframe,0,pendDesiredVel,ignore,1,z']= 3.4366885303746, ['keyframe,0,pendDesiredVel,StR,LR,z']= 2.9876228833698, 
		['keyframe,0,pendDesiredVel,ignore,0,x']= -0.36909862906975, ['keyframe,0,pendDesiredVel,ignore,0,z']= 0.69433442347349, ['keyframe,0,pendDesiredVel,StR,SR,x']= -0.18041478706592, ['keyframe,0,pendDesiredVel,StR,SR,z']= 1.00868114971, ['keyframe,0,pendDesiredVel,StR,RL,x']= 0.056024006035327, ['keyframe,0,pendDesiredVel,StR,RL,z']= 3.1830462757453, ['keyframe,0,pendDesiredVel,StR,LR,x']= -0.0085317009315436, ['keyframe,0,pendDesiredVel,StR,LR,z']= 3.0211956114509, ['keyframe,0,pendDesiredVel,ignore,1,x']= -0.020817211067501, ['keyframe,0,pendDesiredVel,ignore,1,z']= 3.4497573158816, ['keyframe,0,pendDesiredVel,ignore,2,x']= -0.05201671375586, ['keyframe,0,pendDesiredVel,ignore,2,z']= 3.0920673285581, ['keyframe,0,pendDesiredVel,ignore,3,x']= -0.058920621805398, ['keyframe,0,pendDesiredVel,ignore,3,z']= 3.0411705062702, ['keyframe,0,pendDesiredVel,run,LR,x']= -0.17272239572564, ['keyframe,0,pendDesiredVel,run,LR,z']= 3.1380549224854, ['keyframe,0,pendDesiredVel,run,RL,x']= -0.051987710912679, ['keyframe,0,pendDesiredVel,run,RL,z']= 3.1270298333636, ['keyframe,0,pendDesiredVel,run,LR,x']= -0.13111716729959, ['keyframe,0,pendDesiredVel,run,LR,z']= 3.1389407422365, ['keyframe,0,pendDesiredVel,run,RL,x']= -0.031113761615861, ['keyframe,0,pendDesiredVel,run,RL,z']= 3.1287500313202, 
		-- setInertia
		['keyframe,0,pendDesiredVel,ignore,0,x']= -0.36693976506499, ['keyframe,0,pendDesiredVel,ignore,0,z']= 0.71236026718257, ['keyframe,0,pendDesiredVel,StR,SR,x']= -0.17590984017681, ['keyframe,0,pendDesiredVel,StR,SR,z']= 1.0223852437142, ['keyframe,0,pendDesiredVel,StR,RL,x']= 0.060881292730629, ['keyframe,0,pendDesiredVel,StR,RL,z']= 3.1923639393178, ['keyframe,0,pendDesiredVel,StR,LR,x']= 0.001666305526301, ['keyframe,0,pendDesiredVel,StR,LR,z']= 3.0303638826085, ['keyframe,0,pendDesiredVel,ignore,1,x']= -0.013533938978674, ['keyframe,0,pendDesiredVel,ignore,1,z']= 3.459254959755, ['keyframe,0,pendDesiredVel,ignore,2,x']= -0.046128250074543, ['keyframe,0,pendDesiredVel,ignore,2,z']= 3.1002889343032, ['keyframe,0,pendDesiredVel,ignore,3,x']= -0.054950738446712, ['keyframe,0,pendDesiredVel,ignore,3,z']= 3.0493001997701, ['keyframe,0,pendDesiredVel,run,LR,x']= -0.13111716729959, ['keyframe,0,pendDesiredVel,run,LR,z']= 3.1389407422365, ['keyframe,0,pendDesiredVel,run,RL,x']= -0.031113761615861, ['keyframe,0,pendDesiredVel,run,RL,z']= 3.1287500313202, ['keyframe,0,pendDesiredVel,run,LR,x']= -0.12803273275152, ['keyframe,0,pendDesiredVel,run,LR,z']= 3.1438578777517, ['keyframe,0,pendDesiredVel,run,RL,x']= -0.029818449018952, ['keyframe,0,pendDesiredVel,run,RL,z']= 3.1309849911601, 

	}, 
	controlParamSet={
		default={},
		fast={},
	},
	conservativePrediction=1.0,
	muscleActiveness=0.8,
	keyframes={
		pendDesiredVel={numKey=1, default=vector3(0,0,0)},
		swingFootForce={numKey=3, default=vector3(0,10,0)},
		footLmod={numKey=3, numKeyFrom='key', default=vector3(0,0,0)},
		footRmod={numKey=3, numKeyFrom='key', default=vector3(0,0,0)},
		footLmocapMod={numKey=3, numKeyFrom='key', default=vector3(0,0,0)},
		footRmocapMod={numKey=3, numKeyFrom='key', default=vector3(0,0,0)},
		desiredHeadAcc={numKey=3, numKeyFrom='key', default=vector3(0,0,0)},
		desiredMomentum={numKey=3, numKeyFrom='key', default=vector3(0,0,0)},
		desiredDotMomentum={numKey=3, numKeyFrom='key', isContinuous=false,default=vector3(0,0,0)},
		desiredLeaning={numKey=1, isContinuous=false,default=vector3(0,0,0)},
		head_mod={numKey=3, numKeyFrom='key', default=vector3(0,0,0)},
		spprtImpL={numKey=2, default=1},
		spprtImpR={numKey=2, default=1},
		--footLmod={numKey=3, default=vector3(0,0,0)},
		--footRmod={numKey=3, default=vector3(0,0,0)},
		-- x= accY_compensation (corresponds to jumping height, and compliance when landing), 
		-- y= COMacc_compensation (corresponds to jumping distance)
		mocapCompensationCoef={numKey=1, default=vector3(1, 1.0, 1)},
		--feedforwardTorque={numKey=10, default={"fromFile", "fft_straightRun_nocart.tbl"}}
	},
	segProperties={
		noCOPcontrol={default=false}
	},
	stageParamOld=
	{
		--{startSeg=1, endSeg=4, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,SR","footRmod,StR,SR"), objCost=objCost, baseStage=1},
		--{startSeg=1, endSeg=4, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,StR","MAPspprtFootMod,StR"), objCost=objCost,baseStage=1},
	},
}

useCases.jrun=useCases.justinStraightRun_nocart -- shortcut
useCases.STP=useCases.jrun
useCases.StR=useCases.jrun
useCases.run=useCases.jrun
useCases.run.attachCamera=true
do
	local slope=0
	--local slope=-5
	useCases.STP.slope=math.rad(slope)
	useCases.StR.slope=math.rad(slope)
	useCases.run.slope=math.rad(slope)
	if slope<0 then
		useCases.run.parameterManualAdjust=function(modify, accumulate)
			accumulate({ ['map,0,spprtFootMod,run,x']= 0.085944830331823, ['map,0,spprtFootMod,run,y']= -0.14585976821812, ['map,0,spprtFootMod,run,z']= 0.049486709953354, ['map,1,spprtFootMod,run,x']= -0.0082808788500458, ['map,1,spprtFootMod,run,y']= -0.22699881337415, ['map,1,spprtFootMod,run,z']= 0.012683722864974, ['map,2,spprtFootMod,run,x']= -0.024890480440672, ['map,2,spprtFootMod,run,y']= -0.030356012493423, ['map,2,spprtFootMod,run,z']= -0.15627054790776, ['map,0,swingFootMod,run,x']= 0.036734406458112, ['map,0,swingFootMod,run,y']= 0.21911109666827, ['map,0,swingFootMod,run,z']= -0.15001799300965, ['map,1,swingFootMod,run,x']= 0.044779553071535, ['map,1,swingFootMod,run,y']= 0.073456103273684, ['map,1,swingFootMod,run,z']= 0.21682111275372, ['map,2,swingFootMod,run,x']= -0.076856804868675, ['map,2,swingFootMod,run,y']= 0.14871788879271, ['map,2,swingFootMod,run,z']= 0.2065861557549, })
			local modAmt=0.05
			modify({
				['map,0,swingFootMod,run,y']=modAmt*0,
				['map,1,swingFootMod,run,y']=modAmt,
				['map,2,swingFootMod,run,y']=modAmt*0.5,
				['map,2,swingFootMod,run,z']=modAmt*0.5,
			})
			useCases.unmapControlParam(useCases.run)
			--useCases.run.maxContactForceY=480
			--accumulate({['useCase,contactMargin']=0.001})
		end
	end
end

function  useCases.run:genStageParamInitial()
	self.stageParamInitial=
	{
		----{startSeg=1, endSeg=2, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,SR","footRmod,StR,SR"), objCost=objCost, baseStage=1},
		--{startSeg=1, endSeg=7, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,SR","footRmod,StR,SR","MAPswingFootMod,StR","MAPspprtFootMod,StR"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=4, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPspprtFootMod,StR","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=5, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPspprtFootMod,StR","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=6, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=6, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,StR","MAPspprtFootMod,StR"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=7, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPspprtFootMod,StR","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=7, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,StR","MAPswingFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=8, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=8, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,StR","MAPspprtFootMod,StR"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=9, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPspprtFootMod,StR","MAPspprtFootMod,StR"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=9, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPswingFootMod,run"), objCost=objCost,baseStage=1},
		----{startSeg=1, endSeg=7, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPspprtFootMod,StR","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		----{startSeg=1, endSeg=8, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPspprtFootMod,StR","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		----{startSeg=1, endSeg=9, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPspprtFootMod,StR","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=5, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,SR","footRmod,StR,SR","MAPspprtFootMod,StR","MAPswingFootMod,StR"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=5, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPspprtFootMod,StR","MAPspprtFootMod,StR"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=5, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPspprtFootMod,StR","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=6, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=6, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,SR","footRmod,StR,SR","MAPspprtFootMod,StR","MAPswingFootMod,StR"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=6, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPspprtFootMod,StR","MAPswingFootMod,StR"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=7, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,SR","footRmod,StR,SR","MAPspprtFootMod,StR","MAPswingFootMod,StR"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=7, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPspprtFootMod,StR","MAPswingFootMod,StR"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=7, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPspprtFootMod,run","MAPswingFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=7, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,StR","MAPswingFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=8, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,SR","footRmod,StR,SR","MAPspprtFootMod,StR","MAPswingFootMod,StR"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=8, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPspprtFootMod,StR","MAPswingFootMod,StR"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=8, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPspprtFootMod,run","MAPswingFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=8, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,StR","MAPswingFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=9, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,SR","footRmod,StR,SR","MAPspprtFootMod,StR","MAPswingFootMod,StR"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=9, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPspprtFootMod,StR","MAPswingFootMod,StR"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=9, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPspprtFootMod,run","MAPswingFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=9, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,StR","MAPswingFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=10, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,SR","footRmod,StR,SR","MAPspprtFootMod,StR","MAPswingFootMod,StR"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=12, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,SR","footRmod,StR,SR","MAPspprtFootMod,StR","MAPswingFootMod,StR"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=12, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPspprtFootMod,run","MAPswingFootMod,run"), objCost=objCost,baseStage=1},
		----{startSeg=1, endSeg=11, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPspprtFootMod,StR","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=12, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPspprtFootMod,StR","MAPswingFootMod,StR"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=12, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPspprtFootMod,run","MAPswingFootMod,run"), objCost=objCost,baseStage=1},
		----{startSeg=1, endSeg=13, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPspprtFootMod,StR","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		--useCases.genOpt(self, 1,5,2,3,'StR', nil,{'footLmod','footRmod'},'StR',{'RL','LR'}),  -- L and R
		--{startSeg=1, endSeg=7, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPspprtFootMod,run","MAPswingFootMod,run"), objCost=objCost,baseStage=1},
		useCases.genOpt(self, 1,10,2,3,'StR', nil,{'footLmod','footRmod'},'StR',{'RL','LR'}),  -- L and R
		{startSeg=1, endSeg=24, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPspprtFootMod,run","MAPswingFootMod,run"), objCost=objCost,baseStage=1},
		useCases.genOpt(self, 1,24,1,2,'run', nil,{'head_mod'},'run',{'RL','LR'}),  -- L and R
		useCases.genOpt(self, 1,24,1,2,'run', nil,{'footLmod'},'run',{'RL','LR'}),  -- L and R
		useCases.genOpt(self, 1,24,1,2,'run', nil,{'footRmod'},'run',{'RL','LR'}),  -- L and R
		useCases.genOpt(self, 1,24,1,3,'StR', nil,{'footLmod'},'StR',{'SR','RL','LR'}),  -- L and R
		useCases.genOpt(self, 1,24,1,2,'run', {'x'},{'footLmod','footRmod'},'run',{'RL','LR'}),  -- L and R
		useCases.genOpt(self, 1,24,1,2,'run', {'z'},{'footLmod','footRmod'},'run',{'RL','LR'}),  -- L and R
		useCases.genOpt(self, 1,24,1,2,'StR', {'x'},{'footLmod','footRmod'},'StR',{'RL','LR'}),  -- L and R
		useCases.genOpt(self, 1,24,1,2,'StR', {'z'},{'footLmod','footRmod'},'StR',{'RL','LR'}),  -- L and R
		useCases.genOpt(self, 1,24,1,2,'run', {'x'},{'footLmod','head_mod'},'run',{'RL','LR'}),  -- L and R
		useCases.genOpt(self, 1,24,1,2,'run', {'z'},{'footRmod','head_mod'},'run',{'RL','LR'}),  -- L and R

		useCases.genOpt(self, 1,24,1,2,'run', nil,{'head_mod'},'run',{'RL','LR'}),  -- L and R
		useCases.genOpt(self, 1,24,1,2,'StR', nil,{'head_mod'},'StR',{'RL','LR'}),  -- L and R
		{startSeg=1, endSeg=24, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPspprtFootMod,run","MAPswingFootMod,run"), objCost=objCost,baseStage=1},
		{startSeg=1, endSeg=24, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPspprtFootMod,StR","MAPswingFootMod,StR"), objCost=objCost,baseStage=1},
		{startSeg=1, endSeg=30, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPspprtFootMod,run","MAPswingFootMod,run"), objCost=objCost,baseStage=1},
		----{startSeg=1, endSeg=15, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPspprtFootMod,StR","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		----{startSeg=1, endSeg=16, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPspprtFootMod,StR","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=16, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=17, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPspprtFootMod,StR","MAPswingFootMod,StR"), objCost=objCost,baseStage=1},
		----{startSeg=1, endSeg=17, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPspprtFootMod,StR","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		----{startSeg=1, endSeg=18, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPspprtFootMod,StR","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=19, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		----{startSeg=1, endSeg=20, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPspprtFootMod,StR","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		----{startSeg=1, endSeg=21, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPspprtFootMod,StR","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=22, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		----{startSeg=1, endSeg=23, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPspprtFootMod,StR","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		----{startSeg=1, endSeg=24, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPspprtFootMod,StR","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=25, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=14, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,StR","MAPspprtFootMod,StR"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=25, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=14, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,StR","MAPspprtFootMod,StR"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=5, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=6, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,SR","footRmod,StR,SR","MAPspprtFootMod,StR"), objCost=objCost, baseStage=1},
		--{startSeg=1, endSeg=7, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,StR","MAPspprtFootMod,StR"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=7, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=8, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=9, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=10, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=11, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=12, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=13, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=13, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,StR","MAPspprtFootMod,StR"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=15, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=15, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,StR","MAPspprtFootMod,StR"), objCost=objCost,baseStage=1},
		----{startSeg=1, endSeg=5, endFrac=0.5, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,StR","MAPspprtFootMod,StR"), objCost=objCost,baseStage=1},
		----{startSeg=1, endSeg=5, endFrac=0.5, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		----{startSeg=1, endSeg=5, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,SR","footRmod,StR,SR","MAPswingFootMod,StR","MAPspprtFootMod,StR"), objCost=objCost,baseStage=1},
		----{startSeg=1, endSeg=5, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,StR","MAPspprtFootMod,StR"), objCost=objCost,baseStage=1},
		----{startSeg=1, endSeg=6, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,StR","MAPspprtFootMod,StR"), objCost=objCost,baseStage=1},
		------{startSeg=1, endSeg=4, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		----{startSeg=1, endSeg=4, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		----{startSeg=1, endSeg=5, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,SR","footRmod,StR,SR","footLmod,StR,RL","footRmod,StR,RL","footLmod,StR,LR","footRmod,StR,LR"), objCost=objCost, baseStage=1},
		----{startSeg=1, endSeg=5, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		----{startSeg=1, endSeg=6, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,SR","footRmod,StR,SR","footLmod,StR,RL","footRmod,StR,RL","footLmod,StR,LR","footRmod,StR,LR"), objCost=objCost, baseStage=1},
		----{startSeg=1, endSeg=7, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		----{startSeg=1, endSeg=7, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=8, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,SR","footRmod,StR,SR","footLmod,StR,RL","footRmod,StR,RL","footLmod,StR,LR","footRmod,StR,LR"), objCost=objCost, baseStage=1},
		--{startSeg=1, endSeg=8, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=9, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,SR","footRmod,StR,SR","footLmod,StR,RL","footRmod,StR,RL","footLmod,StR,LR","footRmod,StR,LR"), objCost=objCost, baseStage=1},
		--{startSeg=1, endSeg=9, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=10, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=12, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=14, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=16, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=18, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=9, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,run,RL","footRmod,run,RL","footLmod,run,LR","footRmod,run,LR"), objCost=objCost, baseStage=1},
		--{startSeg=1, endSeg=10, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,run,RL","footRmod,run,RL","footLmod,run,LR","footRmod,run,LR"), objCost=objCost, baseStage=1},
		--{startSeg=1, endSeg=10, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,SR","footRmod,StR,SR","footLmod,StR,RL","footRmod,StR,RL","footLmod,StR,LR","footRmod,StR,LR"), objCost=objCost, baseStage=1},
		--{startSeg=1, endSeg=12, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,run,RL","footRmod,run,RL","footLmod,run,LR","footRmod,run,LR"), objCost=objCost, baseStage=1},
		--{startSeg=1, endSeg=12, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,SR","footRmod,StR,SR","footLmod,StR,RL","footRmod,StR,RL","footLmod,StR,LR","footRmod,StR,LR"), objCost=objCost, baseStage=1},
		----{startSeg=1, endSeg=4, endFrac=0.4, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,RL","footRmod,StR,RL"), objCost=objCost, baseStage=1},
		----{startSeg=1, endSeg=4, endFrac=0.4, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,LR","footRmod,StR,LR"), objCost=objCost, baseStage=1},
		----{startSeg=1, endSeg=4, endFrac=0.4, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,SR","footRmod,StR,SR"), objCost=objCost, baseStage=1},
		----{startSeg=1, endSeg=5, endFrac=0.4, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,SR","footRmod,StR,SR","footLmod,StR,RL","footRmod,StR,RL","footLmod,StR,LR","footRmod,StR,LR"), objCost=objCost, baseStage=1},
		----{startSeg=1, endSeg=5, endFrac=0.4, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,RL","footRmod,StR,RL"), objCost=objCost, baseStage=1},
		----{startSeg=1, endSeg=5, endFrac=0.4, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,LR","footRmod,StR,LR"), objCost=objCost, baseStage=1},
		----{startSeg=1, endSeg=5, endFrac=0.4, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,SR","footRmod,StR,SR"), objCost=objCost, baseStage=1},
		----{startSeg=1, endSeg=5, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,SR","footRmod,StR,SR","footLmod,StR,RL","footRmod,StR,RL","footLmod,StR,LR","footRmod,StR,LR"), objCost=objCost, baseStage=1},
		----{startSeg=1, endSeg=5, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		----{startSeg=1, endSeg=6, endFrac=0.4, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,SR","footRmod,StR,SR","footLmod,StR,RL","footRmod,StR,RL","footLmod,StR,LR","footRmod,StR,LR"), objCost=objCost, baseStage=1},
		----{startSeg=1, endSeg=6, endFrac=0.4, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		----{startSeg=1, endSeg=6, endFrac=0.4, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,RL","footRmod,StR,RL"), objCost=objCost, baseStage=1},
		----{startSeg=1, endSeg=6, endFrac=0.4, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,LR","footRmod,StR,LR"), objCost=objCost, baseStage=1},
		----{startSeg=1, endSeg=6, endFrac=0.4, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,SR","footRmod,StR,SR"), objCost=objCost, baseStage=1},
		----{startSeg=1, endSeg=6, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,SR","footRmod,StR,SR","footLmod,StR,RL","footRmod,StR,RL","footLmod,StR,LR","footRmod,StR,LR"), objCost=objCost, baseStage=1},
		----{startSeg=1, endSeg=6, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		----{startSeg=1, endSeg=6, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,SR","footRmod,StR,SR"), objCost=objCost, baseStage=1},
		----{startSeg=1, endSeg=6, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,RL","footRmod,StR,RL"), objCost=objCost, baseStage=1},
		----{startSeg=1, endSeg=6, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,LR","footRmod,StR,LR"), objCost=objCost, baseStage=1},
		----{startSeg=1, endSeg=6, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		----{startSeg=1, endSeg=7, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		----{startSeg=1, endSeg=7, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,SR","footRmod,StR,SR","footLmod,StR,RL","footRmod,StR,RL","footLmod,StR,LR","footRmod,StR,LR"), objCost=objCost, baseStage=1},
		----{startSeg=1, endSeg=8, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,SR","footRmod,StR,SR"), objCost=objCost, baseStage=1},
		----{startSeg=1, endSeg=8, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,RL","footRmod,StR,RL"), objCost=objCost, baseStage=1},
		----{startSeg=1, endSeg=8, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,LR","footRmod,StR,LR"), objCost=objCost, baseStage=1},
		----{startSeg=1, endSeg=8, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		----{startSeg=1, endSeg=8, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,SR","footRmod,StR,SR","footLmod,StR,RL","footRmod,StR,RL","footLmod,StR,LR","footRmod,StR,LR"), objCost=objCost, baseStage=1},
		----{startSeg=1, endSeg=9, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		----{startSeg=1, endSeg=10, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		----{startSeg=1, endSeg=11, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		----{startSeg=1, endSeg=12, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		----{startSeg=1, endSeg=13, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		----{startSeg=1, endSeg=13, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,SR","footRmod,StR,SR","footLmod,StR,RL","footRmod,StR,RL","footLmod,StR,LR","footRmod,StR,LR"), objCost=objCost, baseStage=1},
		----{startSeg=1, endSeg=14, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		----{startSeg=1, endSeg=15, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		----{startSeg=1, endSeg=16, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		----{startSeg=1, endSeg=16, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,StR,SR","footRmod,StR,SR","footLmod,StR,RL","footRmod,StR,RL","footLmod,StR,LR","footRmod,StR,LR"), objCost=objCost, baseStage=1},
		----{startSeg=1, endSeg=17, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,run","MAPspprtFootMod,run"), objCost=objCost,baseStage=1},
		--{startSeg=1, endSeg=15, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,run,RL","footRmod,run,RL","footLmod,run,LR","footRmod,run,LR"), objCost=objCost, baseStage=1},
		--{startSeg=1, endSeg=20, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,run,RL","footRmod,run,RL","footLmod,run,LR","footRmod,run,LR"), objCost=objCost, baseStage=1},
		--{startSeg=1, endSeg=30, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,run,RL","footRmod,run,RL","footLmod,run,LR","footRmod,run,LR"), objCost=objCost, baseStage=1},
		--{startSeg=1, endSeg=40, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"footLmod,run,RL","footRmod,run,RL","footLmod,run,LR","footRmod,run,LR"), objCost=objCost, baseStage=1},
	}
	if false then -- check
		local tbl=self.stageParamInitial
		for i=1,#tbl do
			printTable(tbl[i].param)
			dbg.console()
		end
	end
	return self.stageParamInitial
end

useCases.justinStraightRun_nocart.mapControlParam=function(graph, title, param)

	local out={}

	assert(string.sub(title, 1,4)=="map,")
	local tokens=string.tokenize(title,',')
	local idx=tonumber(tokens[2])
	local name=tokens[3]
	local convertMap=
	{
		swingFootMod={',footLmod,', ',RL'},
		spprtFootMod={',footRmod,', ',RL'},
	}
	local convertMapM=
	{
		swingFootMod={',footRmod,', ',LR'},
		spprtFootMod={',footLmod,', ',LR'},
	}

	local grp=tokens[4]
	local axis=tokens[5]
	local name2='keyframe,'..idx..convertMap[name][1]..grp..convertMap[name][2]
	local name2_mirror='keyframe,'..idx..convertMapM[name][1]..grp..convertMapM[name][2]

	if axis~=nil then
		array.pushBack(out, {name2..","..axis,param}) 
		if axis=='x' then
			array.pushBack(out, {name2_mirror..","..axis,param*-1}) 
		else
			array.pushBack(out, {name2_mirror..","..axis,param}) 
		end
	else
		local param2=param:copy()
		array.pushBack(out, {name2, param2:copy()})
		param2.x=param2.x*-1
		array.pushBack(out, {name2_mirror, param2:copy()})
	end
		
	return out
end

useCases.justinStraightRun_nocart.funcUpdateConstraints=function(graph)

	useCases.defaultFuncUpdateConstraints(useCase, graph)

	if false then
		-- check continuity
		print('lfoot')
		print(graph:getControlParameterFromGraph('keyframe,0,footLmod,run,RL,y')..'RL')
		print(graph:getControlParameterFromGraph('keyframe,1,footLmod,run,RL,y'))
		print(graph:getControlParameterFromGraph('keyframe,2,footLmod,run,RL,y'))
		print(graph:getControlParameterFromGraph('keyframe,3,footLmod,run,RL,y'))
		print(graph:getControlParameterFromGraph('keyframe,0,footLmod,run,LR,y')..'LR')
		print(graph:getControlParameterFromGraph('keyframe,1,footLmod,run,LR,y'))
		print(graph:getControlParameterFromGraph('keyframe,2,footLmod,run,LR,y'))
		print(graph:getControlParameterFromGraph('keyframe,3,footLmod,run,LR,y'))
		print('rfoot')
		print(graph:getControlParameterFromGraph('keyframe,0,footRmod,run,RL,y')..'RL')
		print(graph:getControlParameterFromGraph('keyframe,1,footRmod,run,RL,y'))
		print(graph:getControlParameterFromGraph('keyframe,2,footRmod,run,RL,y'))
		print(graph:getControlParameterFromGraph('keyframe,3,footRmod,run,RL,y'))
		print(graph:getControlParameterFromGraph('keyframe,0,footRmod,run,LR,y')..'LR')
		print(graph:getControlParameterFromGraph('keyframe,1,footRmod,run,LR,y'))
		print(graph:getControlParameterFromGraph('keyframe,2,footRmod,run,LR,y'))
		print(graph:getControlParameterFromGraph('keyframe,3,footRmod,run,LR,y'))
		dbg.console()
	end
end

useCases.justinStraightRun_nocart.registerContactPair=function(model, loader, floor, simulator)

	param=vectorn ()
	param:setValues(0.5,0.5, model.penaltyForceStiffness, model.penaltyForceDamp)
	for i=1,loader:numBone()-1 do

		local bone_i=loader:VRMLbone(i)
		if string.find(bone_i:name(), "foot")
			or string.find(bone_i:name(), "toe") then
			simulator:registerCollisionCheckPair(loader:name(),bone_i.NameId, floor:name(), floor:bone(1):name(), param)
		end
	end
end


useCases.justinStraightRun_nocart.init_globals=function()
	outputSuperSample=1
	PDservoLatency=5-- for debugging. set 5 for normal use.
	IDservoLatency=0
	COPlatency=10
	PredictionLatency=0
	disableTouchDownFeedback=false
	model.simulationFrameRate=120
	model.simulationSubTimesteps=1 -- has to be 1 for unknown reason
	model.penaltyForceStiffness=20000
	model.penaltyForceDamp=2000
	model.k_p_ID=250
	model.k_d_ID=25
	model.k_p_PD=200
	model.k_d_PD=15
	model.k_ID_torque_scale=1
	--model.k_p_PD=10
	--model.k_d_PD=5
	model.k_p_JTfoot=500
	model.k_d_JTfoot=50
	model.k_scale_active_pd.knee={0.5,0.5,1}
	model.k_scale_id.knee={1,1,1}
	model.k_scale_id.ankle={1,1,1}
	model.k_scale_id.toes={1,1,1}
	model.k_scale_id.shoulder={1,1,1}
	model.initialHeight=0.05
end

useCases.justinStraightRun_nocart.graphParam={
	STP={
		seg={"ss","st"},
		num_key={2, 2},
		key_first={0,0},
		key_last={0,0},
	},
	StR={
		seg={"SR","RL","LR"},
		num_key={4,  4,  4},
		key_first={0,0,0},
		key_last={"RL","LR","run:RL:0"},
	},
	run={
		seg={"RL","LR"},
		num_key={ 4,  4},
		key_first={0,0},
		key_last={"LR","RL:0"},
	},
}
useCases.justinStraightRun_nocart.segmentations=
{
	STP={
		firstFrames={0, 50, 95, 135, 169},
		names={"ss", "st"},
		swingL={0  ,  0},        
		swingR={0  ,  0},        
		footRefL= "convertFromSwingL",
		footRefR= "convertFromSwingR",
		usePositionControl=true,
	},
	StR={
		firstFrames={95, 135, 169, 234, 275,323}, -- SR:135->169, RL:169->234, LR:234->275 
		names={            "SR", "RL", "LR1"},
		swingL={            0,    1   ,  0  },
		swingR={            0,    0   ,  1  },
		footRefL= "convertFromSwingL",
		footRefR= "convertFromSwingR",
		usePositionControl=false,
	},
	run={
		names={           'LR', 'RL', 'LR2'},
		swingL={            0,    1   ,  0  },
		swingR={            1,    0   ,  1  },
		footRefL= "convertFromSwingL",
		footRefR= "convertFromSwingR",
		--firstFrames={661,697, 735, 772, 808},
		firstFrames={366, 412, 454, 500, 543, 588},--, 632...
		usePositionControl=false,
		noCOPcontrol=true,
	},
}
useCases.justinStraightRun_nocart.graph={
	{
		"addInterpolatedSegment",
		grpName="StR",
		name="LR",
		seg0={"StR", "LR1"},
		seg1={"run", "LR"},
		startWeight=0, endWeight=1
	},
	{"connect", "STP", "ss", "STP", "st"}, 
	{"connect", "STP", "st", "StR", "SR"}, 
	{"connectMulti", "StR",
	"SR", "RL", "LR"},
	{"connect", "StR", "LR", "run", "RL"},
	{"connectMulti", "run",
	"LR", "RL", "LR"},
	--{"initialSegment", "STP", "ss"}
	{"initialSegment", "StR", "SR"}
}
useCases.justinStraightRun_nocart.pendOptimizationPath=
{
	firstFrames={ 95,    135,      169,     234,       275,     323,        366,     412,     454,     500,     543,    588},
	segments={   'ignore,0', 'StR,SR', 'StR,RL', 'StR,LR', 'ignore,1','ignore,2','ignore,3','run,LR','run,RL','run,LR','run,RL'},
}

do -- controlParamSet.default

	local function copyPendDesiredVel(a,b)
		local prefix='keyframe,0,pendDesiredVel,'
		local pc=useCases.justinStraightRun_nocart
		pc.pendControlParam[prefix..b..',x']=pc.pendControlParam[prefix..a..',x']
		pc.pendControlParam[prefix..b..',z']=pc.pendControlParam[prefix..a..',z']
	end
	copyPendDesiredVel('StR,LR', 'StR,LR1')
	copyPendDesiredVel('run,LR', 'run,LR2')
	-- conservative prediction
	useCases.justinStraightRun_nocart.controlParamSet.default=table.mult(useCases.justinStraightRun_nocart.pendControlParam, useCases.justinStraightRun_nocart.conservativePrediction)

	useCases.justinStraightRun_nocart.controlParam=useCases.justinStraightRun_nocart.controlParamSet.default

	useCases.justinStraightRun_nocart.controlParam=useCases.justinStraightRun_nocart.controlParamSet.default
	local function accumulate(cp_mod)
		local useCase=useCases.justinStraightRun_nocart
		useCases.accumulate(useCase, cp_mod)
	end

	useCases.run.noCOMjoint=true
	useCases.run.spprtImpFromImp=true
	table.mergeInPlace(useCases.run.controlParam, 
	{
		['keyframe,0,desiredMomentum,StR,SR,x']=2.6343654232147, 
		['keyframe,0,footLmod,StR,LR,x']=0.10963224745488, 
		['keyframe,0,footLmod,StR,LR,y']=0.092171839140473, 
		['keyframe,0,footLmod,StR,LR,z']=0.010760895141585, 
		['keyframe,0,footLmod,StR,RL,x']=0.089611863818617, 
		['keyframe,0,footLmod,StR,RL,y']=0.032154487895041, 
		['keyframe,0,footLmod,StR,RL,z']=-0.076902321990632, 
		['keyframe,0,footLmod,StR,SR,x']=0.10827977853068, 
		['keyframe,0,footLmod,StR,SR,y']=-0.18777713781116, 
		['keyframe,0,footLmod,StR,SR,z']=-0.19471057961221, 
		['keyframe,0,footLmod,run,LR,x']=-0.085944830331823, 
		['keyframe,0,footLmod,run,LR,y']=-0.14585976821812, 
		['keyframe,0,footLmod,run,LR,z']=0.049486709953354, 
		['keyframe,0,footLmod,run,RL,x']=0.036734406458112, 
		['keyframe,0,footLmod,run,RL,y']=0.21911109666827, 
		['keyframe,0,footLmod,run,RL,z']=-0.15001799300965, 
		['keyframe,0,footRmod,StR,LR,x']=-0.093442155506505, 
		['keyframe,0,footRmod,StR,LR,y']=0.03184612135112, 
		['keyframe,0,footRmod,StR,LR,z']=-0.073338407774113, 
		['keyframe,0,footRmod,StR,RL,x']=-0.10890771865146, 
		['keyframe,0,footRmod,StR,RL,y']=0.092843923086524, 
		['keyframe,0,footRmod,StR,RL,z']=-0.0079081154178355, 
		['keyframe,0,footRmod,StR,SR,x']=0.18815637763281, 
		['keyframe,0,footRmod,StR,SR,y']=0.022487906212195, 
		['keyframe,0,footRmod,StR,SR,z']=0.022028851272596, 
		['keyframe,0,footRmod,run,LR,x']=-0.036734406458112, 
		['keyframe,0,footRmod,run,LR,y']=0.21911109666827, 
		['keyframe,0,footRmod,run,LR,z']=-0.15001799300965, 
		['keyframe,0,footRmod,run,RL,x']=0.085944830331823, 
		['keyframe,0,footRmod,run,RL,y']=-0.14585976821812, 
		['keyframe,0,footRmod,run,RL,z']=0.049486709953354, 
		['keyframe,0,head_mod,run,LR,x']=-0.00077469253225931, 
		['keyframe,0,head_mod,run,LR,y']=0.00011682844400421, 
		['keyframe,0,head_mod,run,LR,z']=-0.002025628455806, 
		['keyframe,0,head_mod,run,RL,x']=0.0037319788104053, 
		['keyframe,0,head_mod,run,RL,y']=0.0002421959608493, 
		['keyframe,0,head_mod,run,RL,z']=-0.0023759075272499, 
		['keyframe,0,pendDesiredVel,StR,LR,x']=0.001666305526301, 
		['keyframe,0,pendDesiredVel,StR,LR,z']=3.0303638826085, 
		['keyframe,0,pendDesiredVel,StR,LR1,x']=0.001666305526301, 
		['keyframe,0,pendDesiredVel,StR,LR1,z']=3.0303638826085, 
		['keyframe,0,pendDesiredVel,StR,RL,x']=0.060881292730629, 
		['keyframe,0,pendDesiredVel,StR,RL,z']=3.1923639393178, 
		['keyframe,0,pendDesiredVel,StR,SR,x']=-0.17590984017681, 
		['keyframe,0,pendDesiredVel,StR,SR,z']=1.0223852437142, 
		['keyframe,0,pendDesiredVel,ignore,0,x']=-0.36693976506499, 
		['keyframe,0,pendDesiredVel,ignore,0,z']=0.71236026718257, 
		['keyframe,0,pendDesiredVel,ignore,1,x']=-0.013533938978674, 
		['keyframe,0,pendDesiredVel,ignore,1,z']=3.459254959755, 
		['keyframe,0,pendDesiredVel,ignore,2,x']=-0.046128250074543, 
		['keyframe,0,pendDesiredVel,ignore,2,z']=3.1002889343032, 
		['keyframe,0,pendDesiredVel,ignore,3,x']=-0.054950738446712, 
		['keyframe,0,pendDesiredVel,ignore,3,z']=3.0493001997701, 
		['keyframe,0,pendDesiredVel,run,LR,x']=-0.12803273275152, 
		['keyframe,0,pendDesiredVel,run,LR,z']=3.1438578777517, 
		['keyframe,0,pendDesiredVel,run,LR2,x']=-0.12803273275152, 
		['keyframe,0,pendDesiredVel,run,LR2,z']=3.1438578777517, 
		['keyframe,0,pendDesiredVel,run,RL,x']=-0.029818449018952, 
		['keyframe,0,pendDesiredVel,run,RL,z']=3.1309849911601, 
		['keyframe,1,footLmod,StR,LR,x']=0.059829877837199, 
		['keyframe,1,footLmod,StR,LR,y']=-0.2103781902947, 
		['keyframe,1,footLmod,StR,LR,z']=-0.10562343243406, 
		['keyframe,1,footLmod,StR,RL,x']=0.022312335616711, 
		['keyframe,1,footLmod,StR,RL,y']=0.00034885300953766, 
		['keyframe,1,footLmod,StR,RL,z']=0.049411812919613, 
		['keyframe,1,footLmod,StR,SR,x']=0.023118510881842, 
		['keyframe,1,footLmod,StR,SR,y']=-0.068299275101603, 
		['keyframe,1,footLmod,StR,SR,z']=-0.13077451486626, 
		['keyframe,1,footLmod,run,LR,x']=0.0082808788500458, 
		['keyframe,1,footLmod,run,LR,y']=-0.22699881337415, 
		['keyframe,1,footLmod,run,LR,z']=0.012683722864974, 
		['keyframe,1,footLmod,run,RL,x']=0.044779553071535, 
		['keyframe,1,footLmod,run,RL,y']=0.073456103273684, 
		['keyframe,1,footLmod,run,RL,z']=0.21682111275372, 
		['keyframe,1,footRmod,StR,LR,x']=-0.015843086434742, 
		['keyframe,1,footRmod,StR,LR,y']=0.00095716206365868, 
		['keyframe,1,footRmod,StR,LR,z']=0.0264089653765, 
		['keyframe,1,footRmod,StR,RL,x']=-0.10057702607233, 
		['keyframe,1,footRmod,StR,RL,y']=-0.20045656297392, 
		['keyframe,1,footRmod,StR,RL,z']=-0.1767431905039, 
		['keyframe,1,footRmod,StR,SR,x']=0.11005321194008, 
		['keyframe,1,footRmod,StR,SR,y']=-0.029066484722099, 
		['keyframe,1,footRmod,StR,SR,z']=-0.19021178198299, 
		['keyframe,1,footRmod,run,LR,x']=-0.044779553071535, 
		['keyframe,1,footRmod,run,LR,y']=0.073456103273684, 
		['keyframe,1,footRmod,run,LR,z']=0.21682111275372, 
		['keyframe,1,footRmod,run,RL,x']=-0.0082808788500458, 
		['keyframe,1,footRmod,run,RL,y']=-0.22699881337415, 
		['keyframe,1,footRmod,run,RL,z']=0.012683722864974, 
		['keyframe,1,head_mod,run,LR,x']=0.0040031892714917, 
		['keyframe,1,head_mod,run,LR,y']=0.0035807794842721, 
		['keyframe,1,head_mod,run,LR,z']=0.0032375474375666, 
		['keyframe,1,head_mod,run,RL,x']=0.00018168437104957, 
		['keyframe,1,head_mod,run,RL,y']=0.00062418392547108, 
		['keyframe,1,head_mod,run,RL,z']=-0.0014161249957725, 
		['keyframe,2,footLmod,StR,LR,x']=0.063326030484086, 
		['keyframe,2,footLmod,StR,LR,y']=-0.20781259945373, 
		['keyframe,2,footLmod,StR,LR,z']=-0.17248589540236, 
		['keyframe,2,footLmod,StR,RL,x']=-0.024524166249329, 
		['keyframe,2,footLmod,StR,RL,y']=0.20131420335378, 
		['keyframe,2,footLmod,StR,RL,z']=0.22771773210638, 
		['keyframe,2,footLmod,StR,SR,x']=-0.021460199490206, 
		['keyframe,2,footLmod,StR,SR,y']=-0.20480856039841, 
		['keyframe,2,footLmod,StR,SR,z']=-0.0042095353015122, 
		['keyframe,2,footLmod,run,LR,x']=0.024890480440672, 
		['keyframe,2,footLmod,run,LR,y']=-0.030356012493423, 
		['keyframe,2,footLmod,run,LR,z']=-0.15627054790776, 
		['keyframe,2,footLmod,run,RL,x']=-0.076856804868675, 
		['keyframe,2,footLmod,run,RL,y']=0.14871788879271, 
		['keyframe,2,footLmod,run,RL,z']=0.2065861557549, 
		['keyframe,2,footRmod,StR,LR,x']=-0.011462857733707, 
		['keyframe,2,footRmod,StR,LR,y']=0.18676740761493, 
		['keyframe,2,footRmod,StR,LR,z']=0.33163130961269, 
		['keyframe,2,footRmod,StR,RL,x']=-0.060652363831335, 
		['keyframe,2,footRmod,StR,RL,y']=-0.20742792872782, 
		['keyframe,2,footRmod,StR,RL,z']=-0.17721500491444, 
		['keyframe,2,footRmod,StR,SR,x']=-0.06517460966799, 
		['keyframe,2,footRmod,StR,SR,y']=-0.18823401330652, 
		['keyframe,2,footRmod,StR,SR,z']=0.037308878214956, 
		['keyframe,2,footRmod,run,LR,x']=0.076856804868675, 
		['keyframe,2,footRmod,run,LR,y']=0.14871788879271, 
		['keyframe,2,footRmod,run,LR,z']=0.2065861557549, 
		['keyframe,2,footRmod,run,RL,x']=-0.024890480440672, 
		['keyframe,2,footRmod,run,RL,y']=-0.030356012493423, 
		['keyframe,2,footRmod,run,RL,z']=-0.15627054790776, 
		['keyframe,2,head_mod,run,LR,x']=0.0024714047373671, 
		['keyframe,2,head_mod,run,LR,y']=-0.0013328773480261, 
		['keyframe,2,head_mod,run,LR,z']=0.0023939960695219, 
		['keyframe,2,head_mod,run,RL,x']=0.0019355854836656, 
		['keyframe,2,head_mod,run,RL,y']=-0.0001117545984862, 
		['keyframe,2,head_mod,run,RL,z']=0.0092193319141004, 
		['useCases,run,COMobjWeight']=0, 
		['useCases,run,EEobjWeight']=6000, 
		['useCases,run,EEobjWeightAngular']=6000, 
		['useCases,run,QPservoDScaleCoef']=0.85, 
		['useCases,run,bigJointsAccCoef']=4, 
		['useCases,run,bigJointsTorqueCoef']=4, 
		['useCases,run,conservativeW']=1, 
		['useCases,run,contactMargin']=0.01, 
		['useCases,run,ddqObjWeight']=10000, 
		['useCases,run,desiredAccThr']=800, 
		['useCases,run,dotMomentumScale']=0.3, 
		['useCases,run,excludeRoot']=true, 
		['useCases,run,footPosFeedbackMethod']=2, 
		['useCases,run,headControlWeight']=6000, 
		['useCases,run,invAccConeCoef']=1, 
		['useCases,run,invFricCoef']=0.8, 
		['useCases,run,k_d_EE']=34, 
		['useCases,run,k_d_HEAD']=14, 
		['useCases,run,k_d_ID']=25, 
		['useCases,run,k_d_momentum']=0, 
		['useCases,run,k_p_EE']=150, 
		['useCases,run,k_p_HEAD']=150, 
		['useCases,run,k_p_ID']=250, 
		['useCases,run,lambdaObjWeight']=100000, 
		['useCases,run,lambdaObjWeight2']=1e-05, 
		['useCases,run,maxPenetratingVel']=0, 
		['useCases,run,maxTorque']=350, 
		['useCases,run,modifyFootSim']=true, 
		['useCases,run,momentumThr']=10, 
		['useCases,run,momentumWeight']=0, 
		['useCases,run,noCOMjoint']=true, 
		['useCases,run,noComvelDependentFootAdjustment']=true, 
		['useCases,run,noIntersectionPrevenction']=true, 
		['useCases,run,noStrideAdjust']=true, 
		['useCases,run,numericalDerivDmot']=true, 
		['useCases,run,pendulumK']=30000, 
		['useCases,run,perClassContactMargin']=1, 
		['useCases,run,tauObjWeight']=0.0001, 
		['useCases,run,upperBoundLocal']=3, 
		['useCases,run,useToePos']=true, 
		['useCases,run,velMarginStrength']=1, 
		['useCases,run,zmpOriScale']=0, 
	}, true)
	if useCases.run.parameterManualAdjust then
		local function modify(cp_mod)
			local useCase=useCases.run
			for k,v in pairs(cp_mod) do
				if(useCase.controlParam[k]==nil) then
					--print('warning: setting controlParam['..k..']=0')
					useCase.controlParam[k]=0
				end
				useCase.controlParam[k]=useCase.controlParam[k]+v
			end
		end
		useCases.run.parameterManualAdjust (modify, accumulate)
	end

	if true then
		-- trying new optimization
		accumulate({['useCase,lambdaObjWeight']=0.0001}) -- no effect
		accumulate({['useCase,EEobjWeight']=60000}) --walk:60000
		accumulate({['useCase,headControlWeight']=0}) -- walk: 0
		--accumulate({['useCase,momentumWeight']=0}) -- walk: 8000
		accumulate({['useCase,momentumWeight']=8000})
		accumulate({['useCase,footPosFeedbackMethod']=1})

		accumulate({ ['keyframe,0,footLmod,StR,RL,x']= 0.089328051402823, ['keyframe,0,footLmod,StR,RL,y']= 0.032814728386379, ['keyframe,0,footLmod,StR,RL,z']= -0.076881606425021, ['keyframe,1,footLmod,StR,RL,x']= 0.0221034405683, ['keyframe,1,footLmod,StR,RL,y']= 0.001059382990082, ['keyframe,1,footLmod,StR,RL,z']= 0.049519445495179, ['keyframe,2,footLmod,StR,RL,x']= -0.024576806656452, ['keyframe,2,footLmod,StR,RL,y']= 0.20102645336916, ['keyframe,2,footLmod,StR,RL,z']= 0.22790060805533, ['keyframe,0,footRmod,StR,RL,x']= -0.10982990619734, ['keyframe,0,footRmod,StR,RL,y']= 0.0923593334772, ['keyframe,0,footRmod,StR,RL,z']= -0.0084578499765452, ['keyframe,1,footRmod,StR,RL,x']= -0.10088762743161, ['keyframe,1,footRmod,StR,RL,y']= -0.20024886846597, ['keyframe,1,footRmod,StR,RL,z']= -0.17684030924314, ['keyframe,2,footRmod,StR,RL,x']= -0.060550640357129, ['keyframe,2,footRmod,StR,RL,y']= -0.20699256142586, ['keyframe,2,footRmod,StR,RL,z']= -0.17748989445956, ['keyframe,0,footLmod,StR,LR,x']= 0.10937230946072, ['keyframe,0,footLmod,StR,LR,y']= 0.091930044534136, ['keyframe,0,footLmod,StR,LR,z']= 0.011089387219785, ['keyframe,1,footLmod,StR,LR,x']= 0.059600767864059, ['keyframe,1,footLmod,StR,LR,y']= -0.2106684691971, ['keyframe,1,footLmod,StR,LR,z']= -0.1056507088915, ['keyframe,2,footLmod,StR,LR,x']= 0.064041326958074, ['keyframe,2,footLmod,StR,LR,y']= -0.20761618151308, ['keyframe,2,footLmod,StR,LR,z']= -0.17298817841744, ['keyframe,0,footRmod,StR,LR,x']= -0.093766320552194, ['keyframe,0,footRmod,StR,LR,y']= 0.031824628486666, ['keyframe,0,footRmod,StR,LR,z']= -0.073498583296931, ['keyframe,1,footRmod,StR,LR,x']= -0.015928903682876, ['keyframe,1,footRmod,StR,LR,y']= 0.0010331938278616, ['keyframe,1,footRmod,StR,LR,z']= 0.026494188089694, ['keyframe,2,footRmod,StR,LR,x']= -0.011671050460694, ['keyframe,2,footRmod,StR,LR,y']= 0.18708822775972, ['keyframe,2,footRmod,StR,LR,z']= 0.33162498811511, })
		accumulate({ ['map,0,spprtFootMod,run,x']= 0.080372374226618, ['map,0,spprtFootMod,run,y']= -0.15725061969431, ['map,0,spprtFootMod,run,z']= 0.39711163532769, ['map,1,spprtFootMod,run,x']= -0.19488859162836, ['map,1,spprtFootMod,run,y']= -0.24845711552279, ['map,1,spprtFootMod,run,z']= 0.013398380085484, ['map,2,spprtFootMod,run,x']= 0.052026815090632, ['map,2,spprtFootMod,run,y']= -0.1683832238221, ['map,2,spprtFootMod,run,z']= -0.045135548231626, ['map,0,swingFootMod,run,x']= -0.036085724819295, ['map,0,swingFootMod,run,y']= -0.066059540301871, ['map,0,swingFootMod,run,z']= 0.06214353616132, ['map,1,swingFootMod,run,x']= 0.01132808638145, ['map,1,swingFootMod,run,y']= 0.067345854589079, ['map,1,swingFootMod,run,z']= 0.19576656942676, ['map,2,swingFootMod,run,x']= -0.059020396997524, ['map,2,swingFootMod,run,y']= 0.088307894298958, ['map,2,swingFootMod,run,z']= 0.16065939419074, })
		useCases.unmapControlParam(useCases.run)
		--1.000 
		--Wed Jun 12 04:19:10 2013 setStage3 startSeg:1 endSeg:24 : [3.7035847588964e-06,3.7035847588964e-06] -> oeval:3.7035847588964e-06 dim:18}) -- skipped oeval:3.7035847588964e-06 

		accumulate({ }) -- skipped iter :-1 ['keyframe,0,head_mod,run,RL,x']= 0.0039263687155504, ['keyframe,0,head_mod,run,RL,y']= 0.00051399159448586, ['keyframe,0,head_mod,run,RL,z']= -0.0024915442708919, ['keyframe,1,head_mod,run,RL,x']= 0.00025267366043416, ['keyframe,1,head_mod,run,RL,y']= 0.00054263475791376, ['keyframe,1,head_mod,run,RL,z']= -0.001469147847773, ['keyframe,2,head_mod,run,RL,x']= 0.0018244490507892, ['keyframe,2,head_mod,run,RL,y']= -0.00016317799467343, ['keyframe,2,head_mod,run,RL,z']= 0.0091420844076305, ['keyframe,0,head_mod,run,LR,x']= -0.00073857724039926, ['keyframe,0,head_mod,run,LR,y']= -0.00017384120422515, ['keyframe,0,head_mod,run,LR,z']= -0.0021419055658981, ['keyframe,1,head_mod,run,LR,x']= 0.0033571663188455, ['keyframe,1,head_mod,run,LR,y']= 0.0035802750404307, ['keyframe,1,head_mod,run,LR,z']= 0.0033859430852185, ['keyframe,2,head_mod,run,LR,x']= 0.0020862206674022, ['keyframe,2,head_mod,run,LR,y']= -0.001317211537607, ['keyframe,2,head_mod,run,LR,z']= 0.0015524063095364, 
		--1.006 --1.001 --1.001 --1.000 --1.001 --1.000 --1.001 --1.000 --1.001 --1.001 --1.000 --1.000 --1.000 --0.999 --1.000 --1.000 --1.000 --1.000 --1.000 --0.999 
		--Wed Jun 12 08:44:23 2013 setStage4 startSeg:1 endSeg:24 : [3.7014285905672e-06,3.7033235376216e-06] -> oeval:3.7012518295708e-06 dim:18}) -- skipped oeval:3.7012518295708e-06 

		accumulate({ ['keyframe,0,footLmod,run,RL,x']= -0.036230967461486, ['keyframe,0,footLmod,run,RL,y']= -0.066081310637357, ['keyframe,0,footLmod,run,RL,z']= 0.062309431395304, ['keyframe,1,footLmod,run,RL,x']= 0.011354202096761, ['keyframe,1,footLmod,run,RL,y']= 0.067319169795729, ['keyframe,1,footLmod,run,RL,z']= 0.19579140266181, ['keyframe,2,footLmod,run,RL,x']= -0.059487455489592, ['keyframe,2,footLmod,run,RL,y']= 0.088497939578003, ['keyframe,2,footLmod,run,RL,z']= 0.16063498708937, ['keyframe,0,footLmod,run,LR,x']= -0.080773756789092, ['keyframe,0,footLmod,run,LR,y']= -0.15727551467194, ['keyframe,0,footLmod,run,LR,z']= 0.39735717046353, ['keyframe,1,footLmod,run,LR,x']= 0.1949736462783, ['keyframe,1,footLmod,run,LR,y']= -0.24845337720853, ['keyframe,1,footLmod,run,LR,z']= 0.013460508958267, ['keyframe,2,footLmod,run,LR,x']= -0.051939846888325, ['keyframe,2,footLmod,run,LR,y']= -0.16840957882938, ['keyframe,2,footLmod,run,LR,z']= -0.0449601956395, })
		--1.001 --1.001 --1.002 --1.002 --1.001 --1.001 --1.000 --1.000 --1.000 --1.000 --1.000 --1.000 --1.000 --1.000 --0.999 --0.999 --0.999 --0.999 --0.999 --0.999 
		--Wed Jun 12 13:11:12 2013 setStage5 startSeg:1 endSeg:24 : [3.6984853288788e-06,4.047189649969e-06] -> oeval:3.6984853288788e-06 dim:27}) -- skipped oeval:3.6984853288788e-06 

		accumulate({ ['keyframe,0,footRmod,run,RL,x']= 0.081122700859696, ['keyframe,0,footRmod,run,RL,y']= -0.15727011888622, ['keyframe,0,footRmod,run,RL,z']= 0.3982165918893, ['keyframe,1,footRmod,run,RL,x']= -0.19644954569743, ['keyframe,1,footRmod,run,RL,y']= -0.24990469062297, ['keyframe,1,footRmod,run,RL,z']= 0.013448695005133, ['keyframe,2,footRmod,run,RL,x']= 0.052135712146472, ['keyframe,2,footRmod,run,RL,y']= -0.16831639941438, ['keyframe,2,footRmod,run,RL,z']= -0.045542362798381, ['keyframe,0,footRmod,run,LR,x']= 0.0364005048213, ['keyframe,0,footRmod,run,LR,y']= -0.066017452105883, ['keyframe,0,footRmod,run,LR,z']= 0.06220196501082, ['keyframe,1,footRmod,run,LR,x']= -0.012009891591176, ['keyframe,1,footRmod,run,LR,y']= 0.066653096555139, ['keyframe,1,footRmod,run,LR,z']= 0.19642491132824, ['keyframe,2,footRmod,run,LR,x']= 0.05917525434926, ['keyframe,2,footRmod,run,LR,y']= 0.089023424353693, ['keyframe,2,footRmod,run,LR,z']= 0.16193020922652, })
		--1.079 --1.006 --1.095 --1.110 --1.028 --1.045 --1.120 --0.993 --1.061 --1.045 --1.045 --0.989 --0.990 --1.063 --0.989 --0.988 --1.011 --0.988 --0.985 --1.011 
		--Wed Jun 12 17:37:00 2013 setStage6 startSeg:1 endSeg:24 : [3.6363293538653e-06,4.437715976937e-06] -> oeval:3.6321864844063e-06 dim:12}) -- skipped oeval:3.6321864844063e-06 

		accumulate({ ['keyframe,0,footLmod,StR,SR,x']= 0.10795224861103, ['keyframe,0,footLmod,StR,SR,y']= -0.18778192037831, ['keyframe,0,footLmod,StR,SR,z']= -0.19446439530616, ['keyframe,1,footLmod,StR,SR,x']= 0.02258213986851, ['keyframe,1,footLmod,StR,SR,y']= -0.068307517277463, ['keyframe,1,footLmod,StR,SR,z']= -0.13031878131686, ['keyframe,2,footLmod,StR,SR,x']= -0.02171932525239, ['keyframe,2,footLmod,StR,SR,y']= -0.20469026637192, ['keyframe,2,footLmod,StR,SR,z']= -0.0041536036625628, ['keyframe,0,footLmod,StR,RL,x']= 0.089390880208901, ['keyframe,0,footLmod,StR,RL,y']= 0.032440953830281, ['keyframe,0,footLmod,StR,RL,z']= -0.077368523527263, ['keyframe,1,footLmod,StR,RL,x']= 0.022010796630026, ['keyframe,1,footLmod,StR,RL,y']= 0.00084896343256038, ['keyframe,1,footLmod,StR,RL,z']= 0.049874859379894, ['keyframe,2,footLmod,StR,RL,x']= -0.024671199983437, ['keyframe,2,footLmod,StR,RL,y']= 0.20187355697077, ['keyframe,2,footLmod,StR,RL,z']= 0.2283693132182, ['keyframe,0,footLmod,StR,LR,x']= 0.10954674763717, ['keyframe,0,footLmod,StR,LR,y']= 0.092103531823404, ['keyframe,0,footLmod,StR,LR,z']= 0.01079344201172, ['keyframe,1,footLmod,StR,LR,x']= 0.059828053575169, ['keyframe,1,footLmod,StR,LR,y']= -0.21090545060131, ['keyframe,1,footLmod,StR,LR,z']= -0.10434892687634, ['keyframe,2,footLmod,StR,LR,x']= 0.064761490474885, ['keyframe,2,footLmod,StR,LR,y']= -0.20667671860796, ['keyframe,2,footLmod,StR,LR,z']= -0.16332673870269, })
		--1.063 --1.001 --1.000 --1.000 --1.001 --1.000 --0.999 
	useCases.clampParam(useCases.run, 'footRmod,StR', 0.2)
	useCases.clampParam(useCases.run, 'footLmod,StR', 0.2)
		accumulate({
			['useCase,momentumWeight']=0,
			['useCases,run,headControlWeight']=36000, 
		['useCases,run,k_d_HEAD']=24, 
		['useCases,run,k_p_HEAD']=0, 
			['useCases,run,numericalDerivDmot']=true, 
			['useCases,run,QPservoDScaleCoef']=1, 
		['useCase,EEobjWeight']=33300, --walk:60000 -- the lower, better motion quality, but becomes unsteady. needs some research.
		['useCases,run,ddqObjWeight']=2000, 
		['useCases,run,desiredAccThr']=200, 
		['useCases,run,k_d_ID']=25, 
		['useCases,run,k_p_ID']=150, 
		})
	useCases.clampParam(useCases.run, 'footRmod,run', 0.2)
	useCases.clampParam(useCases.run, 'footLmod,run', 0.2)
	useCases.clampParam(useCases.run, 'footRmod,run,RL,x', 0)
	useCases.clampParam(useCases.run, 'footLmod,run,LR,x', 0)


-- tropt.lua: 	Thu Jun 13 15:46:24 2013

--	------------------New optimization-------- [1e+100,-1e+100] -> oeval:1.5124375648545e-05 dim:18}) -- skipped oeval:1.5124375648545e-05 
--0.842 --0.894 --0.834 --0.798 --0.800 --0.885 --0.814 --0.769 --0.776 --0.771 --0.833 --0.754 --0.754 --0.753 --0.833 --0.753 --0.753 --0.754 --0.753 --0.754 
--Thu Jun 13 00:42:01 2013 setStage2 startSeg:1 endSeg:10 : [1.1385990805265e-05,1.2911789547373e-05] -> oeval:2.4979323183231e-05 dim:36}) -- skipped oeval:2.4979323183231e-05 

accumulate({ ['keyframe,0,footLmod,StR,SR,x']= 0.10672381008113, ['keyframe,0,footLmod,StR,SR,y']= -0.19128469640075, ['keyframe,0,footLmod,StR,SR,z']= -0.19229388545682, ['keyframe,1,footLmod,StR,SR,x']= 0.026916376031262, ['keyframe,1,footLmod,StR,SR,y']= -0.067626846660222, ['keyframe,1,footLmod,StR,SR,z']= -0.13014663443125, ['keyframe,2,footLmod,StR,SR,x']= -0.030418907842928, ['keyframe,2,footLmod,StR,SR,y']= -0.19862676268526, ['keyframe,2,footLmod,StR,SR,z']= 0.0015589183324728, ['keyframe,0,footRmod,StR,SR,x']= 0.19057519774235, ['keyframe,0,footRmod,StR,SR,y']= 0.018837949726794, ['keyframe,0,footRmod,StR,SR,z']= 0.028777959559518, ['keyframe,1,footRmod,StR,SR,x']= 0.11574335165701, ['keyframe,1,footRmod,StR,SR,y']= -0.027533155958707, ['keyframe,1,footRmod,StR,SR,z']= -0.18089142022712, ['keyframe,2,footRmod,StR,SR,x']= -0.063770446843251, ['keyframe,2,footRmod,StR,SR,y']= -0.19270180280327, ['keyframe,2,footRmod,StR,SR,z']= 0.037595591033979, })
--0.809 --0.538 --0.405 --0.349 --0.308 --0.319 --0.287 --0.307 --0.305 --0.318 --0.287 --0.285 --0.269 --0.259 --0.342 --0.270 --0.317 --0.254 --0.254 --0.309 
--Thu Jun 13 02:44:59 2013 setStage3 startSeg:1 endSeg:10 : [6.3065250667658e-06,8.5224916429342e-06] -> oeval:6.2316910760161e-06 dim:18}) -- skipped oeval:6.2316910760161e-06 

accumulate({ ['keyframe,0,footLmod,StR,SR,x']= 0.12575793494673, ['keyframe,0,footLmod,StR,SR,y']= -0.17391738239396, ['keyframe,0,footLmod,StR,SR,z']= -0.2086037887986, ['keyframe,1,footLmod,StR,SR,x']= 0.096859512686518, ['keyframe,1,footLmod,StR,SR,y']= 0.0037597734217982, ['keyframe,1,footLmod,StR,SR,z']= -0.15214839865388, ['keyframe,2,footLmod,StR,SR,x']= -0.009752095911215, ['keyframe,2,footLmod,StR,SR,y']= -0.23240023522101, ['keyframe,2,footLmod,StR,SR,z']= 0.022719861190996, ['keyframe,0,footRmod,StR,SR,x']= 0.14969775469772, ['keyframe,0,footRmod,StR,SR,y']= 0.036349914565915, ['keyframe,0,footRmod,StR,SR,z']= 0.035916547262817, ['keyframe,1,footRmod,StR,SR,x']= 0.11394073498905, ['keyframe,1,footRmod,StR,SR,y']= 0.021222241994481, ['keyframe,1,footRmod,StR,SR,z']= -0.21770820272559, ['keyframe,2,footRmod,StR,SR,x']= -0.13240119643205, ['keyframe,2,footRmod,StR,SR,y']= -0.10225399718162, ['keyframe,2,footRmod,StR,SR,z']= 0.0006495452905632, ['map,0,swingFootMod,StR,x']= 0.0790146356909, ['map,0,swingFootMod,StR,y']= 0.025146674257723, ['map,0,swingFootMod,StR,z']= -0.076890486325767, ['map,1,swingFootMod,StR,x']= 0.0025525316263696, ['map,1,swingFootMod,StR,y']= -0.089571449560588, ['map,1,swingFootMod,StR,z']= 0.0067172270381037, ['map,2,swingFootMod,StR,x']= -0.076113310827803, ['map,2,swingFootMod,StR,y']= 0.1666859075828, ['map,2,swingFootMod,StR,z']= 0.20338004060148, ['map,0,spprtFootMod,StR,x']= -0.075727474924402, ['map,0,spprtFootMod,StR,y']= -0.11773578159512, ['map,0,spprtFootMod,StR,z']= -0.072731793281042, ['map,1,spprtFootMod,StR,x']= -0.09443135591906, ['map,1,spprtFootMod,StR,y']= -0.19477920721416, ['map,1,spprtFootMod,StR,z']= -0.26256595711538, ['map,2,spprtFootMod,StR,x']= -0.0085010359649363, ['map,2,spprtFootMod,StR,y']= -0.20714106599792, ['map,2,spprtFootMod,StR,z']= -0.13923919823697, })
--1.010 --1.004 --1.011 --1.007 --1.004 --1.013 --1.003 --1.003 --1.003 --1.002 --1.003 --1.002 --1.003 --1.000 --1.000 --1.000 --1.000 --1.000 --1.000 --0.999 
--Thu Jun 13 04:46:56 2013 setStage4 startSeg:1 endSeg:10 : [6.2283174454304e-06,6.2624003701996e-06] -> oeval:6.1833569250234e-06 dim:36}) -- skipped oeval:6.1833569250234e-06 

accumulate({ ['map,0,swingFootMod,StR,x']= 0.081180075861036, ['map,0,swingFootMod,StR,y']= 0.026205657068552, ['map,0,swingFootMod,StR,z']= -0.07827681983044, ['map,1,swingFootMod,StR,x']= 0.0058832646716082, ['map,1,swingFootMod,StR,y']= -0.088237051752205, ['map,1,swingFootMod,StR,z']= 0.0030093187313751, ['map,2,swingFootMod,StR,x']= -0.077538377198496, ['map,2,swingFootMod,StR,y']= 0.16581418580322, ['map,2,swingFootMod,StR,z']= 0.20304704572301, ['map,0,spprtFootMod,StR,x']= -0.074482402345901, ['map,0,spprtFootMod,StR,y']= -0.11708287026834, ['map,0,spprtFootMod,StR,z']= -0.067059283586435, ['map,1,spprtFootMod,StR,x']= -0.090450364437996, ['map,1,spprtFootMod,StR,y']= -0.19474202776283, ['map,1,spprtFootMod,StR,z']= -0.26189772688646, ['map,2,spprtFootMod,StR,x']= -0.007481394808176, ['map,2,spprtFootMod,StR,y']= -0.2085102040561, ['map,2,spprtFootMod,StR,z']= -0.13885268473116, })
--1.000 --1.140 --1.000 --0.999 --1.149 --1.000 --1.000 --1.148 --1.146 --1.148 --0.998 --0.997 --0.997 --0.997 --0.998 --0.997 --0.997 --0.996 --0.997 --0.996 
--Thu Jun 13 06:49:02 2013 setStage5 startSeg:1 endSeg:24 : [6.1596993333247e-06,7.106999491012e-06] -> oeval:8.4175585683588e-06 dim:18}) -- skipped oeval:8.4175585683588e-06 

accumulate({ ['keyframe,0,footLmod,StR,RL,x']= 0.081428675916785, ['keyframe,0,footLmod,StR,RL,y']= 0.026013437010635, ['keyframe,0,footLmod,StR,RL,z']= -0.0785390806826, ['keyframe,1,footLmod,StR,RL,x']= 0.0052889342374268, ['keyframe,1,footLmod,StR,RL,y']= -0.088712410683799, ['keyframe,1,footLmod,StR,RL,z']= 0.0031096713849021, ['keyframe,2,footLmod,StR,RL,x']= -0.078966816160793, ['keyframe,2,footLmod,StR,RL,y']= 0.1663459515026, ['keyframe,2,footLmod,StR,RL,z']= 0.20332634749314, ['keyframe,0,footRmod,StR,RL,x']= -0.074629615313809, ['keyframe,0,footRmod,StR,RL,y']= -0.11763797799032, ['keyframe,0,footRmod,StR,RL,z']= -0.066626826042889, ['keyframe,1,footRmod,StR,RL,x']= -0.090414285111729, ['keyframe,1,footRmod,StR,RL,y']= -0.19419506895341, ['keyframe,1,footRmod,StR,RL,z']= -0.2623140139967, ['keyframe,2,footRmod,StR,RL,x']= -0.0070053860263938, ['keyframe,2,footRmod,StR,RL,y']= -0.20835696206064, ['keyframe,2,footRmod,StR,RL,z']= -0.13880589898765, ['keyframe,0,footLmod,StR,LR,x']= 0.074395409444699, ['keyframe,0,footLmod,StR,LR,y']= -0.11692471261298, ['keyframe,0,footLmod,StR,LR,z']= -0.066777480886274, ['keyframe,1,footLmod,StR,LR,x']= 0.089877627465697, ['keyframe,1,footLmod,StR,LR,y']= -0.19445305034215, ['keyframe,1,footLmod,StR,LR,z']= -0.2614965962328, ['keyframe,2,footLmod,StR,LR,x']= 0.0073800500981526, ['keyframe,2,footLmod,StR,LR,y']= -0.20836039743032, ['keyframe,2,footLmod,StR,LR,z']= -0.13874132493239, ['keyframe,0,footRmod,StR,LR,x']= -0.080733690931744, ['keyframe,0,footRmod,StR,LR,y']= 0.025886875059175, ['keyframe,0,footRmod,StR,LR,z']= -0.078238294835512, ['keyframe,1,footRmod,StR,LR,x']= -0.0055959193255715, ['keyframe,1,footRmod,StR,LR,y']= -0.08802028893349, ['keyframe,1,footRmod,StR,LR,z']= 0.0037625334595219, ['keyframe,2,footRmod,StR,LR,x']= 0.078154311479167, ['keyframe,2,footRmod,StR,LR,y']= 0.16570894358215, ['keyframe,2,footRmod,StR,LR,z']= 0.2033864267459, })
--0.966 --0.356 --0.324 --0.280 --0.272 --0.261 --0.259 --0.258 --0.260 --0.255 --0.255 --0.256 --0.256 --0.256 --0.256 --0.257 --0.252 --0.253 --0.255 --0.251 
--Thu Jun 13 11:29:21 2013 setStage6 startSeg:1 endSeg:24 : [2.1050605451507e-06,2.3030724202882e-06] -> oeval:2.0928313695702e-06 dim:18}) -- skipped oeval:2.0928313695702e-06 

accumulate({ ['map,0,spprtFootMod,run,x']= 0.072508075053583, ['map,0,spprtFootMod,run,y']= 0.021048324654539, ['map,0,spprtFootMod,run,z']= 0.29714964512515, ['map,1,spprtFootMod,run,x']= -0.11241035701057, ['map,1,spprtFootMod,run,y']= -1.2694886952624, ['map,1,spprtFootMod,run,z']= 0.037385589759893, ['map,2,spprtFootMod,run,x']= -0.10383817400215, ['map,2,spprtFootMod,run,y']= -0.1877854792767, ['map,2,spprtFootMod,run,z']= -0.096749255433058, ['map,0,swingFootMod,run,x']= 0.10459249440861, ['map,0,swingFootMod,run,y']= -0.018055083449515, ['map,0,swingFootMod,run,z']= 0.02736455411146, ['map,1,swingFootMod,run,x']= 0.12899301040779, ['map,1,swingFootMod,run,y']= -0.083579196217407, ['map,1,swingFootMod,run,z']= 0.2268348000318, ['map,2,swingFootMod,run,x']= -0.22943766128901, ['map,2,swingFootMod,run,y']= 0.057171739328703, ['map,2,swingFootMod,run,z']= 0.2076718259816, })
--1.013 --1.012 --1.017 --1.013 --1.013 --1.011 --1.006 --1.010 --1.009 --1.010 --1.013 --1.006 --1.012 --1.016 --1.014 --1.014 --1.011 --1.017 --1.009 
	end

end
--useCases.justinStraightRun_nocart.controlParam=useCases.justinStraightRun_nocart.controlParamSet.fast
end
