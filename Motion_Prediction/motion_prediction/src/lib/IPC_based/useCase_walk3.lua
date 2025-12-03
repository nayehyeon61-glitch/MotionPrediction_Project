require('IPC_based/useCase_roundoff')
--genOptAll=select(1,...)

do -- walk3

	local turning=false
	local function makeOptParam(numKey,...)
		local gtarget=array:new()
		local prec=0.001
		local symmetricMapUsed=true
		for iid,id in ipairs({...}) do
			for i=0, numKey-1 do
				if string.sub(id,1,3)=="MAP" then
					gtarget:pushBack({"map,"..i..","..string.sub(id,4)..",x",prec})
					gtarget:pushBack({"map,"..i..","..string.sub(id,4)..",y",prec})
					gtarget:pushBack({"map,"..i..","..string.sub(id,4)..",z",prec})
					symmetricMapUsed=true
				else
					gtarget:pushBack({"keyframe,"..i..","..id..",x",prec})
					gtarget:pushBack({"keyframe,"..i..","..id..",y",prec})
					gtarget:pushBack({"keyframe,"..i..","..id..",z",prec})
				end
			end
		end
		if symmetricMapUsed then
			-- gtarget:pushBack({"useCases,walk3,asymmetricFoot,x",prec})
			-- gtarget:pushBack({"useCases,walk3,asymmetricFoot,y",prec})
			-- gtarget:pushBack({"useCases,walk3,asymmetricFoot,z",prec})
			--if turning then
			if true then
				-- gtarget:pushBack({"useCases,walk3,asymmetricFootTimeDependent,x",prec})
				-- gtarget:pushBack({"useCases,walk3,asymmetricFootTimeDependent,y",prec})
				----gtarget:pushBack({'useCases,walk3,turnGain',0.4})
				----gtarget:pushBack({'useCases,walk3,turnFeedbackTorque',1})
			end
			-- gtarget:pushBack({"useCases,walk3,asymmetricFootTimeDependent,z",prec}) -- 
		end

		return gtarget
	end
	local function makeOptParamAxis(numKey,axis,...)
		local gtarget=array:new()
		local prec=0.001
		local symmetricMapUsed=false
		for iid,id in ipairs({...}) do
			for i=0, numKey-1 do
				if string.sub(id,1,3)=="MAP" then
					gtarget:pushBack({"map,"..i..","..string.sub(id,4)..","..axis,prec})
					--symmetricMapUsed=true
				else
					gtarget:pushBack({"keyframe,"..i..","..id..","..axis,prec})
				end
			end
		end
		--[[if symmetricMapUsed then
			gtarget:pushBack({"useCases,walk3,asymmetricFoot,"..axis,prec})
			if turning then
				gtarget:pushBack({"useCases,walk3,asymmetricFootTimeDependent,x",prec})
				gtarget:pushBack({"useCases,walk3,asymmetricFootTimeDependent,y",prec})
				--gtarget:pushBack({'useCases,walk3,turnGain',0.4})
				--gtarget:pushBack({'useCases,walk3,turnFeedbackTorque',1})
			end
			gtarget:pushBack({"useCases,walk3,asymmetricFootTimeDependent,z",prec})
		end]]--

		return gtarget
	end
	useCases.walk3=deepCopyTable(useCases.roundoff_common)
	useCases.walk3._makeOptParam=makeOptParam
	useCases.walk3._makeOptParamAxis=makeOptParamAxis

	local useCase=useCases.walk3


	useCase.useQPsolver=true
	--useCase.footPosFeedbackMethod=1 -- best straight walking
	useCase.footPosFeedbackMethod=2 -- best straight walking
	useCase.turnGain=10
	useCase.turnFeedbackTorque=601
	useCase.grpName='walk3'
	useCase.segNames=useCase.graphParam.walk3.seg
	useCase.segmentation=useCase.segmentations.walk3
	--useCase.slope=math.rad(15)
	--useCase.slope=math.rad(5)
	--useCase.slope=math.rad(-5)
	--useCase.slope=math.rad(-3) useCase.prepareSlope=-0.1
	--useCase.slope=math.rad(-3) 
	useCase.slope=math.rad(0) 
	if useCase.slope<0 then
		useCase.parameterManualAdjust=function(modify, accumulate)
			--accumulate({ ['map,0,swingFootMod,walk3,y']= -0.05429470295023, ['map,1,swingFootMod,walk3,y']= 0.15496428386918, ['map,2,swingFootMod,walk3,y']= -0.12308168288883, ['map,0,spprtFootMod,walk3,y']= -0.010791226247806, ['map,1,spprtFootMod,walk3,y']= -0.10758404516053, ['map,2,spprtFootMod,walk3,y']= -0.44327213842703, })
			local modAmt=0.05
			modify({
				['keyframe,0,footRmod,walk3,l,y']=modAmt, 
				['keyframe,1,footRmod,walk3,l,y']=modAmt, 
				['keyframe,2,footRmod,walk3,l,y']=modAmt, 
				['keyframe,0,footLmod,walk3,r,y']=modAmt, 
				['keyframe,1,footLmod,walk3,r,y']=modAmt, 
				['keyframe,2,footLmod,walk3,r,y']=modAmt, 
				['keyframe,0,footRmod,walk3,lr,y']=modAmt, 
				['keyframe,2,footLmod,walk3,lr,y']=modAmt,
				['map,0,swingFootMod,walk3,y']=modAmt*0,
				['map,1,swingFootMod,walk3,y']=modAmt,
				['map,2,swingFootMod,walk3,y']=modAmt*0.5,
			})
			useCases.unmapControlParam(useCases.walk3)
			useCases.walk3.maxContactForceY=4800
			accumulate({['useCase,contactMargin']=0.001})
		end
	end
	if useCase.slope>0 then
		useCase.footlocalori_modification=quater(useCase.slope, vector3(1,0,0)) 
	end

	useCase.graph={
		{
			"addInterpolatedSegment",
			grpName="walk3",
			name="L",
			seg0={"walk3", "L2"},
			seg1={"walk3", "L1"},
			startWeight=0, endWeight=1
		},
		{
			"addInterpolatedSegment",
			grpName="walk3",
			name="r",
			seg0={"walk3", "r1"},
			seg1={"walk3", "R"},
			startWeight=0, endWeight=1
		},
		{"connectMulti", "walk3", "stand", "l", "lr", "r", "RL", "L", "LR", "R", "RL"},
		{"initialSegment", "walk3", "stand"},
	}

	useCase.lfootModScale=vector3(1,1,1)
	useCase.rfootModScale=vector3(1,1,1)
	useCase.measureOptCost=useCases.measureOptCost
	useCase.amt_sideway_vel=0.94 -- 0.94 is optimal for kinematic controller
	--useCase.amt_sideway_vel=2.35 -- 
	useCase.amt_sideway_vel=0.3 -- 
	useCase.amt_sideway_vel_negative=0.94
	--useCase.amt_sideway_vel=5.45 -- 

	useCase.keyframes.mocapOriModL={numKey=3, numKeyFrom='key', isContinuous=false,default=vector3(0,0,0)}
	useCase.keyframes.mocapOriModR={numKey=3, numKeyFrom='key', isContinuous=false,default=vector3(0,0,0)}
	useCase.keyframes.footLmod={numKey=3, numKeyFrom='key', isContinuous=false,default=vector3(0,0,0)}
	useCase.keyframes.footRmod={numKey=3, numKeyFrom='key', isContinuous=false,default=vector3(0,0,0)}
	useCase.keyframes.footLmocapMod={numKey=3, numKeyFrom='key', isContinuous=false,default=vector3(0,0,0)}
	useCase.keyframes.footRmocapMod={numKey=3, numKeyFrom='key', isContinuous=false,default=vector3(0,0,0)}
	useCase.keyframes.desiredHeadAcc={numKey=3, numKeyFrom='key', isContinuous=false,default=vector3(0,0,0)}
	useCase.keyframes.desiredMomentum={numKey=3, numKeyFrom='key', isContinuous=false,default=vector3(0,0,0)}
	useCase.keyframes.desiredDotMomentum={numKey=3, numKeyFrom='key', isContinuous=false,default=vector3(0,0,0)}
	--useCase.keyframes.asymmetricFootBasis={numKey=3, isContinuous=false,default=0}
	--useCase.keyframes.asymmetricFootMask={numKey=3, isContinuous=false,default=0}

	useCases.walk3.keyframes.EEplanningCoefL={numKey=2, isContinuous=false,default={0,0}}
	useCases.walk3.keyframes.EEplanningCoefR={numKey=2, isContinuous=false,default={0,0}}
	local function setKeyframes(useCase, key, path, values)
		local cp_mod={}

		local numKey=useCase.keyframes[key].numKey
		assert(#path*numKey==#values)
		for iseg, seg in ipairs(path) do
			for j=1, numKey do
				local vidx=(iseg-1)*numKey+j
				if type(values[vidx])=='table' then
					for i=1, #values[vidx] do
						local k='keyframe,'..(j-1)..','..key..','..string.gsub(seg,'%.',',')..','..tostring(i-1)
						cp_mod[k]=values[vidx][i]
					end
				end
			end
		end

		--print('asdfasdf')
		--for k, v in pairs(cp_mod) do
		--print(k,v)
		--end
		--dbg.console()
		useCase.controlParam=useCases.mergeCP(useCase.controlParam, cp_mod)
	end
	-- 0 means predicted position, 1 means simulated position key0={ , }, key1={ , }, ...
	-- setKeyframes(useCases.walk3, 'EEplanningCoefL', 
	-- {"walk3.stand",   "walk3.r", "walk3.rl","walk3.l" ,"walk3.LR" , "walk3.R" , "walk3.RL", "walk3.L"},
	-- {{0,0}, {0,0}, {0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{1,1},{1,0},{1,0},{1,0},{0,0},{1,1},{1,1},{1,1}})

	-- setKeyframes(useCases.walk3, 'EEplanningCoefR', 
	-- {"walk3.stand",   "walk3.r", "walk3.rl","walk3.l" ,"walk3.LR" , "walk3.R" , "walk3.RL", "walk3.L"},
	-- {{0,0}, {0,0}, {0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{1,1},{1,1},{1,1},{1,1},{1,0},{1,0},{1,0}})

	--useCase.asymmetricFoot=vector3(0,0,0)
	--useCase.asymmetricFootTimeDependent=vector3(0,0,0)
	useCase.attachCamera=true
	-- useCase.cartPoleFN="../Resource/scripts/ui/RigidBodyWin/gymnist_wald_d.zmpcom"
	if true then
		useCase.useAnalyticIK=true
		useCase.useMomentumPreservingIK=false
	else
		useCase.useAnalyticIK=false
		useCase.useMomentumPreservingIK=true
	end

	useCase.funcUpdateConstraints=function(graph)

		--[[
		local function con(axis,mirror, footmod)
			local landingFoot0=graph:getControlParameterFromGraph('map,2,swing'..footmod..',walk3,'..axis)
			local landingFoot2=graph:getControlParameterFromGraph('map,0,spprt'..footmod..',walk3,'..axis)*mirror
			graph:changeControlParameter('map,0,landing'..footmod..',walk3,'..axis,landingFoot0)
			graph:changeControlParameter('map,1,landing'..footmod..',walk3,'..axis,(landingFoot0+landingFoot2)*0.5)
			graph:changeControlParameter('map,2,landing'..footmod..',walk3,'..axis,landingFoot2)
			local kickingFoot0=graph:getControlParameterFromGraph('map,2,spprt'..footmod..',walk3,'..axis)
			local kickingFoot2=graph:getControlParameterFromGraph('map,0,swing'..footmod..',walk3,'..axis)*mirror
			graph:changeControlParameter('map,0,kicking'..footmod..',walk3,'..axis,kickingFoot0)
			graph:changeControlParameter('map,1,kicking'..footmod..',walk3,'..axis,(kickingFoot0+kickingFoot2)*0.5)
			graph:changeControlParameter('map,2,kicking'..footmod..',walk3,'..axis,kickingFoot2)
		end

		for a=1,3 do
			local axis
			local mirror=1
			if a==1 then
				axis='x'
				mirror=-1
			elseif a==2 then
				axis='y'
			else
				axis='z'
			end
			con(axis, mirror,'FootMod')
			con(axis, mirror,'FootMocapMod')
		end
		]]--
		useCases.defaultFuncUpdateConstraints(useCase, graph)

		if false then
			-- check continuity
			print(graph:getControlParameterFromGraph('keyframe,0,footLmod,walk3,stand,x'))
			print(graph:getControlParameterFromGraph('keyframe,1,footLmod,walk3,stand,x'))
			print(graph:getControlParameterFromGraph('keyframe,0,footLmod,walk3,l,x'))
			print(graph:getControlParameterFromGraph('keyframe,1,footLmod,walk3,l,x'))
			print(graph:getControlParameterFromGraph('keyframe,2,footLmod,walk3,l,x'))
			print(graph:getControlParameterFromGraph('keyframe,0,footLmod,walk3,lr,x'))
			print(graph:getControlParameterFromGraph('keyframe,1,footLmod,walk3,lr,x'))
			print(graph:getControlParameterFromGraph('keyframe,0,footLmod,walk3,r,x')..'r')
			print(graph:getControlParameterFromGraph('keyframe,1,footLmod,walk3,r,x'))
			print(graph:getControlParameterFromGraph('keyframe,2,footLmod,walk3,r,x'))
			print(graph:getControlParameterFromGraph('keyframe,0,footLmod,walk3,RL,x')..'RL')
			print(graph:getControlParameterFromGraph('keyframe,1,footLmod,walk3,RL,x'))
			print(graph:getControlParameterFromGraph('keyframe,0,footLmod,walk3,L,x')..'L')
			print(graph:getControlParameterFromGraph('keyframe,1,footLmod,walk3,L,x'))
			print(graph:getControlParameterFromGraph('keyframe,2,footLmod,walk3,L,x'))
			print(graph:getControlParameterFromGraph('keyframe,0,footLmod,walk3,LR,x'))
			print(graph:getControlParameterFromGraph('keyframe,1,footLmod,walk3,LR,x'))
			print(graph:getControlParameterFromGraph('keyframe,0,footLmod,walk3,R,x'))
			print(graph:getControlParameterFromGraph('keyframe,1,footLmod,walk3,R,x'))
			print(graph:getControlParameterFromGraph('keyframe,2,footLmod,walk3,R,x'))
			dbg.console()
		end

		local function copyPendDesiredVel(a,b)
			local pf='keyframe,0,pendDesiredVel,walk3,'
			graph:changeControlParameter(pf..a..",x", graph:getControlParameterFromGraph(pf..b..",x"))
			graph:changeControlParameter(pf..a..",y", graph:getControlParameterFromGraph(pf..b..",y"))
			graph:changeControlParameter(pf..a..",z", graph:getControlParameterFromGraph(pf..b..",z"))
		end
		local function avgParam(out,keyout,b,kb,c,kc)
			local pf=',pendDesiredVel,walk3,'
			local axes={',x',',y',',z'}
			for a,axis in ipairs(axes) do
				local bb=graph:getControlParameterFromGraph('keyframe,'..kb..pf..b..axis)
				local cc=graph:getControlParameterFromGraph('keyframe,'..kc..pf..c..axis)
				graph:changeControlParameter('keyframe,'..keyout..pf..out..axis, 0.5*(bb+cc))
			end
		end
		copyPendDesiredVel('lr', 'l')
		copyPendDesiredVel('rl', 'r1')
		copyPendDesiredVel('lr2', 'l2')
		copyPendDesiredVel('rl2', 'r2')
		copyPendDesiredVel('LR', 'L1')
		copyPendDesiredVel('RL', 'R')
		copyPendDesiredVel('LR2', 'L2')
		--copyPendDesiredVel('RL2', 'R1')
		--avgParam('LR' ,0, 'LR1', 0, 'lr2',0)
		--copyPendDesiredVel('LR', 'L')
	end
	useCases.walk3.mapControlParam=function(graph, title, param)
		local out={}

		assert(string.sub(title, 1,4)=="map,")
		local tokens=string.tokenize(title,',')
		local idx=tonumber(tokens[2])
		local name=tokens[3]

		local convertMap=function(name, mirror)
			local FootIdx=string.find(name, "Foot")
			local key=string.sub(name, 1, FootIdx+3)
			local id=string.lower(string.sub(name, FootIdx+4, FootIdx+4))..string.sub(name, FootIdx+5)
			local convertMap
			if mirror then
				convertMap=
				{
					swingFoot={',footR', ',L', useCase.rfootModScale}, 
					spprtFoot={',footL', ',L', useCase.lfootModScale},
					landingFoot={',footR', ',LR', useCase.rfootModScale},
					kickingFoot={',footL', ',LR', useCase.lfootModScale},
				}
			else
				convertMap=
				{
					swingFoot={',footL', ',R', useCase.lfootModScale}, 
					spprtFoot={',footR', ',R', useCase.rfootModScale},
					landingFoot={',footL', ',RL', useCase.lfootModScale },
					kickingFoot={',footR', ',RL', useCase.rfootModScale},
				}
			end
			return {convertMap[key][1]..id..",", convertMap[key][2], convertMap[key][3]}
		end

		local grp=tokens[4]
		local axis=tokens[5]
		local cmap=convertMap(name)
		local cmapm=convertMap(name,true)
		local name2='keyframe,'..idx..cmap[1]..grp..cmap[2]
		local name2_mirror='keyframe,'..idx..cmapm[1]..grp..cmapm[2]

		if axis~=nil then
			array.pushBack(out, {name2..","..axis,param*cmap[3][axis]}) 
			if axis=='x' then
				array.pushBack(out, {name2_mirror..","..axis,param*-1*cmapm[3][axis]}) 
			else
				array.pushBack(out, {name2_mirror..","..axis,param*cmapm[3][axis]}) 
			end
		else
			local param2=param:copy()
			array.pushBack(out, {name2, param2*cmap[3]})
			param2.x=param2.x*-1
			array.pushBack(out, {name2_mirror, param2*cmapm[3]})
		end

		return out
	end

	-- the initial values for the follwing two were automatically generated using l ipco_prep
	-- l ipco_prep "grpName='walk3'"
	useCases.walk3.pendControlParam=useCases.roundoff_common.pendControlParam_walk3 -- share
	--useCases.walk3.stateEstimationMarker="stateEstimation_upperbody_marker.lua"

	do
		useCases.walk3.init_globals= function()
			useCases.roundoff_common.init_globals()
			model.clampTorqueID=800
			-- Feb 14 11:06PM
			--model.k_scale_active_pd.ankle={1,2,1}
			--model.k_scale_active_pd.ankle={1,1,1}
			model.k_scale_active_pd.ankle={1,1,1}
			model.penaltyForceStiffness=20000
			model.penaltyForceDamp=2000

			model.k_scale_active_pd.toes={1,1,1}


			--May 23
			model.k_scale_active_pd.default={1,1,1}
			model.k_scale_active_pd.collar={1,1,1}
			model.k_scale_active_pd.shoulder={1,1,1}
			model.k_scale_active_pd.elbow={1,1,1}
			model.k_scale_active_pd.knee={1,1,1}
			model.k_scale_active_pd.hip={1,1,1}
			model.k_scale_active_pd.chest={1,1,1}
			model.k_scale_active_pd.ankle={1,1,1}
			model.k_scale_active_pd.toes={1,1,1}
		end
		useCases.walk3.maxContactForceY=480
	end

	-- the initial values for the follwing two were automatically generated using l ipco_prep
	useCases.walk3.pendOptimizationPath=
	{
		firstFrames={3745, 3766,3786,3853,3877,3928,3948,4000,4020,4067,4088,4136,4157,4204,4226,4275,4296,4345,},
		segments={'ignore,0','walk3,stand','walk3,l','walk3,lr','walk3,r1','walk3,rl','walk3,l2','walk3,lr2','walk3,r2','walk3,rl2','walk3,L1','walk3,LR','walk3,R','walk3,RL','walk3,L2','walk3,LR2','ignore,1'},
	}

	useCases.walk3.pendOptExcludePattern= { '.+rl.+','.+lr.+','.+RL.+','.+LR.+'}
	--	local gains={{'useCases,walk3,comGain', 1}, {'useCases,walk3,comGain_y',1}, {'useCase,k_p_ID',2}, {'useCase,k_d_ID',0.2}, {'useCase,k_p_PD', 1}, {'useCase,k_d_PD', 0.1}, }
	
	local function genGain(name, axis, strength)
		return 
		{
			{'keyframe,0,'..name..',walk3,L,'..axis,strength},
			{'keyframe,0,'..name..',walk3,R,'..axis,strength},
			{'keyframe,1,'..name..',walk3,L,'..axis,strength},
			{'keyframe,1,'..name..',walk3,R,'..axis,strength},
			{'keyframe,2,'..name..',walk3,L,'..axis,strength},
			{'keyframe,2,'..name..',walk3,R,'..axis,strength},
		}
	end

	useCases.rectifyGraph(useCases.walk3)
	if useCases.walk3.useQPsolver then
		function useCases.walk3:updateStageParam(stage, stageParam)
			local gains_x, gains_y, gains_z, gains
			if self.optDotMomentum then
				gains_x=genGain('desiredDotMomentum','x', 20)
				gains_y=genGain('desiredDotMomentum','y', 20)
				gains_z=genGain('desiredDotMomentum','z', 20)
			else
				gains_x=genGain('desiredMomentum','x', 2)
				gains_y=genGain('desiredMomentum','y', 2)
				gains_z=genGain('desiredMomentum','z', 2)
			end
			--gains_x=genGain('desiredHeadAcc','x', 2)
			--gains_y=genGain('desiredHeadAcc','y', 2)
			--gains_z=genGain('desiredHeadAcc','z', 2)
			gains=array.concatMulti(gains_x,gains_y, gains_z)	
			gains={}
			local endSegW=80
			local genOpt=function(starts, ends, ps, pe, name, axis, others)
				local out=useCases.genOpt(useCase, starts, ends, ps, pe, name, axis,others)
				assert(out)
				local objCost='objCost_walk'
				--local objCost=nil
				out.objCost=objCost
				return out
			end
			local tgt_all={'footLmod','footRmod'}
			local tgt_L={'footLmod', 'mocapOriModL'}
			local tgt_R={'footRmod', 'mocapOriModR'}
			if false and stage<11 then -- before starting to walk
				local tbl={ 
					{2,2, {'footLmod', 'footRmod'}}, 
					{1,5, {'footLmod_all', 'footRmod_all'}},
					{1,1, {'footLmod', 'footRmod'}},
					{2,2, {'footLmod', 'footRmod'}},
					{4,4, {'footLmod', 'footRmod'}}, 
					{2,4, {'footLmod'}}, 
					{2,4, {'footRmod'}}, 
					{2,5, {'footLmod_all', 'footRmod_all'}},
					{2,2, {'footLmod', 'footRmod'}},
					{4,4, {'footLmod', 'footRmod'}},
					{2,4, {'footLmod'}},
					{2,4, {'footRmod'}},
				}
				
				local tb=tbl[stage]
				local endSegW=7
				stageParam[stage]=genOpt( 1, endSegW, tb[1], tb[2],'walk3', nil, tb[3])
			elseif false then
				-- doesn't assume symmetry
				local mod=math.mod(stage-1,4)
				if mod==0 then -- footall
					stageParam[stage]=genOpt( 8, endSegW, 6, 8,'walk3', nil, {'footLmod_all','footRmod_all'})
				elseif mod==1 then
					stageParam[stage]=genOpt( 8, endSegW, 6, 8,'walk3',{'x','y'})
				elseif mod==2 then
					stageParam[stage]=genOpt( 8, endSegW, 6, 8,'walk3',{'x','z'})
				else
					stageParam[stage]=genOpt( 8, endSegW, 6, 8,'walk3',{'y','z'})
				end
			else  -- optimize walk cycles (assuming symmetry)
				--local endSegW=stage+23
				local mod=math.mod(stage-1,4)
				if mod==0 then
					stageParam[stage]={startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), objCost=objCost,baseStage=1}
				elseif mod==1 then
					stageParam[stage]={startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", param=makeOptParamAxis(3,'z',"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), objCost=objCost,baseStage=1}
				elseif mod==2 then
					stageParam[stage]={startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", param=makeOptParamAxis(3,'x',"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), objCost=objCost,baseStage=1}
				elseif mod==3 then
					stageParam[stage]={startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", param=makeOptParamAxis(3,'y',"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), objCost=objCost,baseStage=1}
				end
			end
			--stageParam[stage+1]={baseStage=1, startSeg=1}
		end

		function useCases.walk3:genStageParamInitial()
			if true then
				local stageParam={}
				self:updateStageParam(1, stageParam)
				return stageParam
			else
				self.updateStageParam=nil
			end
			local gains_x, gains_y, gains_z, gains
			if self.optDotMomentum then
				gains_x=genGain('desiredDotMomentum','x', 20)
				gains_y=genGain('desiredDotMomentum','y', 20)
				gains_z=genGain('desiredDotMomentum','z', 20)
			else
				gains_x=genGain('desiredMomentum','x', 2)
				gains_y=genGain('desiredMomentum','y', 2)
				gains_z=genGain('desiredMomentum','z', 2)
			end
			--gains_x=genGain('desiredHeadAcc','x', 2)
			--gains_y=genGain('desiredHeadAcc','y', 2)
			--gains_z=genGain('desiredHeadAcc','z', 2)
			gains=array.concatMulti(gains_x,gains_y, gains_z)	
			gains={}
			local endSegW=40
			local genOpt=function(starts, ends, ps, pe, name, axis, others)
				local out=useCases.genOpt(useCase, starts, ends, ps, pe, name, axis,others)
				assert(out)
				out.objCost=objCost
				return out
			end
			local tgt_all={'footLmod','footRmod'}
			local tgt_L={'footLmod', 'mocapOriModL'}
			local tgt_R={'footRmod', 'mocapOriModR'}
			self.stageParamInitial=
			{
				--genOptAll(1,5,1,2,'walk3'),
				--genOptAll(1,5,2,3,'walk3'),
				--genOptAll(1,5,3,4,'walk3'),
				--{startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), objCost=objCost,baseStage=1},
				--{startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", param=array.concatMulti(makeOptParamAxis(3,'z',"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), gains_z),objCost=objCost,baseStage=1},
				--{startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", param=array.concatMulti(makeOptParamAxis(3,'x',"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), gains_x),objCost=objCost,baseStage=1},
				--{startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", param=array.concatMulti(makeOptParamAxis(3,'y',"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), gains_y),objCost=objCost,baseStage=1},
				--{startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), objCost=objCost,baseStage=1},
				--table.merge(genOpt(1,endSegW,5,8,nil,nil,tgt_all),{objCost=objCost}), -- L and R
				--table.merge(genOpt(1,endSegW,5,7,nil,nil,tgt_all),{objCost=objCost}), -- L
				--table.merge(genOpt(1,endSegW,6,8,nil,nil,tgt_all),{objCost=objCost}), -- R
				--genOpt(1,endSegW,5,8,nil,nil,tgt_all), -- L and R
				--genOpt(1,endSegW,5,7,nil,nil,tgt_all), -- L
				--genOpt(1,endSegW,6,8,nil,nil,tgt_all), -- R
				--table.merge(genOptAll(1,5,1,5,'walk3'),{objCost=objCost}),
				--genOptAll(1,6,1,3,'walk3'),
				--genOptAll(1,6,2,3,'walk3'),
				--genOptAll(1,6,2,4,'walk3'),
				--genOpt(1,10,5,7,nil,nil,tgt_all), -- L
				--genOpt(1,10,5,8,nil,nil,tgt_all), -- L and R
				--genOpt(1,10,6,8,nil,nil,tgt_all), -- R
				--genOpt(1,10,2,3,nil,nil,tgt_all),
				--genOpt(1,10,2,4,nil,nil,tgt_all),
				--{startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), objCost=objCost,baseStage=1},
				--genOpt(1,endSegW,2,3,nil,nil,tgt_all), 
				--{startSeg=1, endSeg=20, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), objCost=objCost,baseStage=1},
				--genOpt(1,endSegW,5,7,nil,nil,tgt_all), -- L
				--genOpt(1,10,2,3,nil,nil,tgt_all), 
				--genOpt(1,10,3,4,nil,nil,tgt_all), 
				--genOpt(1,endSegW,5,7,nil,nil,tgt_L), -- L
				--genOpt(1,endSegW,6,8,nil,nil,tgt_R), -- R
				--genOpt(1,endSegW,5,7,nil,nil,tgt_all), -- L
				--genOpt(1,endSegW,6,8,nil,nil,tgt_all), -- R
				--genOpt(1,endSegW,5,8,nil,nil,tgt_all), -- L and R
				--genOpt(1,endSegW+10,5,7,nil,nil,tgt_all), -- L
				--genOpt(1,endSegW+10,6,8,nil,nil,tgt_all), -- R
				--genOpt(1,endSegW+10,5,8,nil,nil,tgt_all), -- L and R
				--{startSeg=1, endSeg=10, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), objCost=objCost,baseStage=1},
				--{startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", param=gains,objCost=objCost,baseStage=1},
				--{startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), objCost=objCost,baseStage=1},
				--{startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", param=array.concatMulti(makeOptParamAxis(3,'z',"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), gains_z),objCost=objCost,baseStage=1},
				--{startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", param=array.concatMulti(makeOptParamAxis(3,'x',"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), gains_x),objCost=objCost,baseStage=1},
				--genOptAll(1,5,1,2,'walk3'),
				--genOptAll(1,5,2,3,'walk3'),
				--genOptAll(1,5,3,4,'walk3'),
				--{startSeg=1, endSeg=30, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), objCost=objCost,baseStage=1},
				--{startSeg=1, endSeg=10, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), objCost=objCost,baseStage=1},
				--{startSeg=1, endSeg=10, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), objCost=objCost,baseStage=1},
				-- {startSeg=1, endSeg=30, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), objCost=objCost,baseStage=1},
				--genOptAll(1,3,1,2,'walk3'),
				--genOptAll(1,5,2,4,'walk3'),
				--genOptAll(1,5,3,4,'walk3'),
				--genOptAll(1,5,1,2,'walk3'),
				{startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), objCost=objCost,baseStage=1},
				useCases.genOpt(self, 1,24,1,2,'walk3', nil,{'head_mod'},'walk3',{'L','R'}),  -- L and R
				{startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", param=array.concatMulti(makeOptParamAxis(3,'z',"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), gains_z),objCost=objCost,baseStage=1},
				{startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", param=array.concatMulti(makeOptParamAxis(3,'x',"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), gains_x),objCost=objCost,baseStage=1},
				{startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", param=gains,objCost=objCost,baseStage=1},
				genOptAll(1,10,1,2,'walk3'),
				genOptAll(1,10,3,4,'walk3'),
				{startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), objCost=objCost,baseStage=1},
				{startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", param=gains,objCost=objCost,baseStage=1},
				{startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", param=array.concatMulti(makeOptParamAxis(3,'x',"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), gains),objCost=objCost,baseStage=1},
				-- {startSeg=1, endSeg=30, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), objCost=objCost,baseStage=1},
				-- {startSeg=1, endSeg=30, nvar=2,setStageFunc="setStage_param", param=array.concatMulti(makeOptParamAxis(3,'x',"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), gains),objCost=objCost,baseStage=1},
				-- {startSeg=1, endSeg=30, nvar=2,setStageFunc="setStage_param", param=array.concatMulti(makeOptParamAxis(3,'y',"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), gains),objCost=objCost,baseStage=1},
				-- {startSeg=1, endSeg=30, nvar=2,setStageFunc="setStage_param", param=array.concatMulti(makeOptParamAxis(3,'z',"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), gains),objCost=objCost,baseStage=1},
				genOptAll(1,30,1,3,'walk3'),
				genOptAll(1,30,2,4,'walk3'),
				{startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", param=array.concatMulti(makeOptParamAxis(3,'y',"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), gains),objCost=objCost,baseStage=1},
				{startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", param=array.concatMulti(makeOptParamAxis(3,'z',"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), gains),objCost=objCost,baseStage=1},
				-- {startSeg=1, endSeg=30, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), objCost=objCost,baseStage=1},
				-- {startSeg=1, endSeg=30, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), objCost=objCost,baseStage=1},
				-- {startSeg=1, endSeg=30, nvar=2,setStageFunc="setStage_param", param=makeOptParamAxis(3,'x',"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), objCost=objCost,baseStage=1},
				-- {startSeg=1, endSeg=30, nvar=2,setStageFunc="setStage_param", param=makeOptParamAxis(3,'y',"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), objCost=objCost,baseStage=1},
				-- {startSeg=1, endSeg=30, nvar=2,setStageFunc="setStage_param", param=makeOptParamAxis(3,'z',"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), objCost=objCost,baseStage=1},
				-- genOptAll(1,30,2,4,'walk3'),
				--{startSeg=1, endSeg=30, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), objCost=objCost,baseStage=1},
				--genOptAll(1,10,1,3,'walk3'),
				--genOptAll(1,10,2,4,'walk3'),
				--{startSeg=1, endSeg=30, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), objCost=objCost,baseStage=1},
				--{startSeg=1, endSeg=30, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), objCost=objCost,baseStage=1},
				--{startSeg=1, endSeg=30, nvar=2,setStageFunc="setStage_param", param=array.concatMulti(makeOptParamAxis(3,'x',"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), gains),objCost=objCost,baseStage=1},
				--{startSeg=1, endSeg=30, nvar=2,setStageFunc="setStage_param", param=array.concatMulti(makeOptParamAxis(3,'y',"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), gains),objCost=objCost,baseStage=1},
				--{startSeg=1, endSeg=30, nvar=2,setStageFunc="setStage_param", param=array.concatMulti(makeOptParamAxis(3,'z',"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), gains),objCost=objCost,baseStage=1},
				--{startSeg=1, endSeg=60, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), objCost=objCost,baseStage=1},
				--{startSeg=1, endSeg=60, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), objCost=objCost,baseStage=1},
				--{startSeg=1, endSeg=60, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), objCost=objCost,baseStage=1},
				--{startSeg=1, endSeg=40, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPlandingFootMod,walk3", "MAPkickingFootMod,walk3"), objCost=objCost,baseStage=1},
				--{startSeg=1, endSeg=40, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPlandingFootMod,walk3", "MAPkickingFootMod,walk3"), objCost=objCost,baseStage=1},

				--genOptAll(1,5,2,5,'walk3'),
			}
			useCases.cleanupStageParam(self.stageParamInitial)
			if false then -- check
				local tbl=self.stageParamInitial
				for i=1,#tbl do
					printTable(tbl[i].param)
					dbg.console()
				end
			end
			local tbl=self.stageParamInitial
			useCases.cleanupStageParam(tbl)

			return self.stageParamInitial
		end
	end
	if turning then
		useCases.walk3.stageParamInitial=
		{
			genOptAll(1,10,1,3,'walk3'),
			{startSeg=1, endSeg=50, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), objCost=objCost,baseStage=1},
			genOptAll(1,10,2,4,'walk3'),
			{startSeg=1, endSeg=50, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), objCost=objCost,baseStage=1},
			--{startSeg=1, endSeg=30, nvar=2,setStageFunc="setStage_param", param=array.concatMulti(makeOptParamAxis(3,'x',"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), gains),objCost=objCost,baseStage=1},
			--{startSeg=1, endSeg=60, nvar=2,setStageFunc="setStage_param", param=array.concatMulti(makeOptParamAxis(3,'x',"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), gains),objCost=objCost,baseStage=1},
			genOptAll(1,20,1,4,'walk3'),
			{startSeg=1, endSeg=50, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), objCost=objCost,baseStage=1},
			{startSeg=1, endSeg=60, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), objCost=objCost,baseStage=1},
			{startSeg=1, endSeg=60, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingFootMod,walk3","MAPspprtFootMod,walk3"), objCost=objCost,baseStage=1},
			--{startSeg=1, endSeg=40, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPlandingFootMod,walk3", "MAPkickingFootMod,walk3"), objCost=objCost,baseStage=1},
			--{startSeg=1, endSeg=40, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPlandingFootMod,walk3", "MAPkickingFootMod,walk3"), objCost=objCost,baseStage=1},

			--genOptAll(1,5,2,5,'walk3'),
		}
		for ii,tbl in ipairs(useCases.walk3.stageParamInitial) do
			table.mergeInPlace(tbl,{desiredTurningSpeed={0,0,0.9,1.3}})
		end
	end
	do
		local function accumulate(cp_mod)
			local useCase=useCases.walk3
			useCases.accumulate(useCase, cp_mod)
		end

		local function modify(cp_mod)
			local useCase=useCases.walk3
			for k,v in pairs(cp_mod) do
				if(useCase.controlParam[k]==nil) then
					--print('warning: setting controlParam['..k..']=0')
					useCase.controlParam[k]=0
				end
				useCase.controlParam[k]=useCase.controlParam[k]+v
			end
		end

		local function setZero(substr)
			local useCase=useCases.walk3
			for k,v in pairs(useCase.controlParam) do
				if string.find(k, substr, nil, true) then
					useCase.controlParam[k]=0
				end
			end
		end
		local function overwrite(cp_mod)
			local useCase=useCases.walk3
			useCase.controlParam=cp_mod
		end

		useCases.walk3.controlParam.sequentialControlParam={}
		local function accumulateAt(index, cp_mod)
			local useCase=useCases.walk3
			local scp=useCase.controlParam.sequentialControlParam[index] or {}
			useCase.controlParam.sequentialControlParam[index]=useCases.mergeCP(scp, cp_mod)
		end

		function modifySwingFoot(key, id, mod,inc,postfix)
			postfix=postfix or 'MocapMod'
			local useCase=useCases.walk3
			local c='map,'..key..',swingFoot'..postfix..','..id
			if inc then
				useCase.controlParam[c]=useCase.controlParam[c]+mod
			else
				useCase.controlParam[c]=mod
			end
		end
		function modifyKey(key,id, seg, mod)
			local useCase=useCases.walk3
			local c='keyframe,'..key..','..id..',walk3,'..seg..',y'
			useCase.controlParam[c]=useCase.controlParam[c]+mod
		end
		function fill_mid(key, id, mod)
			local useCase=useCases.walk3
			local c1=string.gsub(key, ',0,', ',0,')
			local c2=string.gsub(key, ',0,', ',1,')
			local c3=string.gsub(key, ',0,', ',2,')

			useCase.controlParam[c2]=(useCase.controlParam[c1]+useCase.controlParam[c3])*0.5
		end

		useCases.walk3.optDotMomentum=false
		useCases.walk3.initialHeight=0.1
		accumulate({ ['map,0,swingFootMod,walk3,x']= 0.099250856714659, ['map,0,swingFootMod,walk3,y']= -0.11641806291409, ['map,0,swingFootMod,walk3,z']= 0.17881384388533, ['map,1,swingFootMod,walk3,x']= -0.0089718073652387, ['map,1,swingFootMod,walk3,y']= 0.24714261875708, ['map,1,swingFootMod,walk3,z']= -0.06037111695253, ['map,2,swingFootMod,walk3,x']= 0.023644782794604, ['map,2,swingFootMod,walk3,y']= -0.30661051737098, ['map,2,swingFootMod,walk3,z']= -0.040893354331592, ['map,0,spprtFootMod,walk3,x']= -0.0066585263676615, ['map,0,spprtFootMod,walk3,y']= -0.17127559714192, ['map,0,spprtFootMod,walk3,z']= 0.048358920584557, ['map,1,spprtFootMod,walk3,x']= 0.054426083268682, ['map,1,spprtFootMod,walk3,y']= -0.071880444499683, ['map,1,spprtFootMod,walk3,z']= -0.086887537100599, ['map,2,spprtFootMod,walk3,x']= -0.041362149728043, ['map,2,spprtFootMod,walk3,y']= -0.27425407045801, ['map,2,spprtFootMod,walk3,z']= 0.071445792133375, })
		useCases.unmapControlParam(useCases.walk3)
	end
end
