
--require("RigidBodyWin/subRoutines/PDservo")
--require("RigidBodyWin/subRoutines/IDservo")
require("subRoutines/QPservo_switchingRootJoint")
require("subRoutines/CompareChain")
require("IPC_based/LocoSynthesis")
require("subRoutines/pickle")
require("IPC_based/debug_config")
--require('RigidBodyWin/testRoutines/testmw_save') dbg_testmw=true preciseComparisonSimul={0,5} preciseComparison={0,5}


-- See MotToVRML.lua for detailed information.
EFM={NONE=1, FOOT=2, PointClouds=3,UpperBodyOrientation=4, Momentum=5, ZMP=6  }
EFT={LeftFoot=1, RightFoot=2}

--dbg.startDebug()
--dbg.startTrace()
-- options are listed in a decreasing-importance order.
-- scenarios are defined in subRoutines/MotionGraph.lua

function init_globals()
	init_debug_globals()

	if cleanVisualize==true then
		RE.viewpoint():setFOVy(45.000002)
		RE.viewpoint().vpos:set(-31.603369, 97.652362, 422.504089)
		RE.viewpoint().vat:set(-39.439721, 97.466525, -14.588023)
		RE.viewpoint():update()
	else
		RE.viewpoint():setFOVy(45.000002)
		RE.viewpoint().vpos:set(-31.603369, 167.652362, 622.504089)
		RE.viewpoint().vat:set(-39.439721, 167.466525, -14.588023)
		RE.viewpoint():update()
	end

	scenario=useCase.scenario

	--   OnlineSynthesis.init_globals()

	error_feedback_method=EFM.Momentum

	PredictionLatency=5

	model.initialHeight=0

	renderDownSample=4  
	-- SCENARIO_DEPENDENT put all scenario dependent parameters here 
	if useCase.init_globals then
		useCase.init_globals()
	elseif scenario==scenarios.JUMP then
		model.simulationFrameRate=12000
		outputSuperSample=1
		--PDservoLatency=3*outputSuperSample
		PDservoLatency=5
		PredictionLatency=5
		disableTouchDownFeedback=true

	elseif model==model_files.hyunwoo_real_cart or model==model_files.hyunwoo_full_cart  then
		outputSuperSample=1
		PDservoLatency=5
		PredictionLatency=5
		renderDownSample=4
		--      model.initialHeight=0.5
		disableTouchDownFeedback=false
		--   outputSuperSample=1
		--   PDservoLatency=2
	elseif model==model_files.justin_straight_run_cart or model==model_files.justin_straight_run or model==model_files.justin_runf3_cart then
		outputSuperSample=1
		PDservoLatency=5
		PredictionLatency=0
		renderDownSample=4
		model.initialHeight=0.03
		disableTouchDownFeedback=true
		--   outputSuperSample=1
		--   PDservoLatency=2
	elseif model==model_files.justin_run_cart or model==model_files.justin_runf3_cart then
		outputSuperSample=1
		PDservoLatency=10-- for debugging. set 5 for normal use.
		PredictionLatency=0
		renderDownSample=4
		disableTouchDownFeedback=false
	elseif scenario>=scenarios.STAND1 and scenario<=scenarios.STAND4 then
		model.simulationFrameRate=12000
		--   model.simulationFrameRate=24000
		outputSuperSample=1
		--PDservoLatency=3*outputSuperSample
		PDservoLatency=5
		disableTouchDownFeedback=true
		renderDownSample=4
		useCartPoleCalculator=false
		model.initialHeight=0
	else
		assert(false)
	end

	if model==model_files.justin_runf3_cart then
		model.initialHeight=0
	end

	if renderSuperSample==true or renderDownSample==nil then
		renderDownSample=1
	end

	assert(math.mod(model.simulationFrameRate, model.motionFrameRate)==0)
	assert(math.mod(model.simulationFrameRate, model.zmpSamplingRate)==0)
	assert(math.mod(model.simulationFrameRate, model.motionFrameRate*outputSuperSample)==0)
	model.timestep=1/model.simulationFrameRate
	useCartPolClass=true
	usePDservoPos=false
	useID_C1=false
	useFootMaintain=false
	k_contact=0.0001
	k_push=0   -- k_push meter per second


	assert(useCase.useQPsolver)
	useQPsolver=true
	useInverseDynamics=false
	usePenaltyMethod=false
	showDesiredPose=true
	drawCOM=false
	drawZMP=false
	--simulator=simulators.UT
	--simulator=simulators.AIST
	simulator=simulators.gmbs
	--simulator=simulators.VP
	--simulator=simulators.SDFAST
	collisionTestOnlyAnkle =false
	debugContactParam={20, 0, 0.01, 0.6, 0}-- size, tx, ty, tz, tfront
	drawControlForceScale=0.000
	debugContactForceVis={drawControlForceScale,0,drawControlForceScale}
	print("h1")

	--[[ implementations ]]--
	k_p=model.k_p
	k_d=model.k_d
	model.timestep=1/((math.floor((1.0/model.timestep)/30))*30)
	print(model.timestep)
	initialHeight=model.initialHeight


	if cleanVisualize then
		print("h2")
		debugContactParam={20,0,0.01,0,0}
		debugContactForceVis={0,0,0}
		boolean_options.drawControlForce=false
		boolean_options.drawPredictedZMP=false
	end

end

gTimer=util.PerfTimer2()--(1,"timer1")
gTimer2=util.PerfTimer2()--(1,"timer2")
gTimer3=util.PerfTimer2()--(1,"timer3")
gTimer4=util.PerfTimer2()--(1,"timer3")

 --gTimer=util.PerfTimer(100,"timer1")
 --gTimer2=util.PerfTimer(100,"timer2")
 --gTimer3=util.PerfTimer(100,"timer3")

function util.PerfTimer:stopMsg(str)
	-- self:stop()
	-- print(str)
end

function util.PerfTimer2:stopMsg(str)

	if true then
		-- RE.output(str, tostring(self:stop()))
		RE.output2("time_"..str, tostring(self:stop()/1000).."ms")
		--self.count=self.count or {}
		--self.total=self.total or {}
		--self.count[str]=(self.count[str] or 0)+1
		--if self.count[str]==100 then
			--print(str, tostring(self.total[str]).."ms/100 calls")
			--self.total[str]=0
			--self.count[str]=0
		--end
		--self.total[str]=(self.total[str] or 0)+self:stop()/1000
	end
end

function setPoseDOFRot90(skin, pose)
	if skin~=nil then
		local root=MotionDOF.rootTransformation(pose)
		root.rotation:leftMult(quater(math.rad(90), vector3(0,1,0)))
		local pose2=vectorn()
		pose2:assign(pose)
		MotionDOF.setRootTransformation(pose2, root)
		skin:setPoseDOF(pose2)
	end
end


--class 'OnlineLocoSynthesis' (OnlineSynthesis2)
OnlineLocoSynthesis=LUAclass(OnlineSynthesis2)

function OnlineLocoSynthesis:__finalize()
	self.outputMotion=nil
--	self.outputMotion_pdtarget=nil
	self.simulator=nil
	OnlineSynthesis2.__finalize(self)
end

function OnlineLocoSynthesis:setFrameTrigger(f, func, param)
	assert(self.frameTrigger==nil)
	self.frameTrigger={f, func, param}
end

function OnlineLocoSynthesis:clearFrameTrigger()
	assert(self.frameTrigger~=nil)
	self.frameTrigger=nil
end

function OnlineLocoSynthesis:setTouchDownTrigger(func, param)
	assert(self.touchDownTrigger==nil)
	self.touchDownTrigger={func, param}
end

function OnlineLocoSynthesis:clearTouchDownTrigger()
	assert(self.touchDownTrigger~=nil)
	self.touchDownTrigger=nil
end

function OnlineLocoSynthesis:setFloor(sim)
	local mesh=Mesh()

	mesh:createBox(380, 0.2, 380)

	local a=transf();
	a:identity()
	a.rotation:setRotation(vector3(1,0,0), useCase.slope)
	mesh:transform(matrix4(a))

	if false and useCase.slope<0 then
		a:identity()
		a.translation.y=useCase.prepareSlope or -1
		mesh:transform(matrix4(a))
		local mesh2=Mesh()
		mesh2:createBox(380, 0.2, 380)
		a:identity()
		mesh2:transform(matrix4(a))

		mesh:merge(mesh, mesh2)
	end
	-- obstacle is a single node rigidbody with fixed joint.
	sim:createObstacle(mesh)
	sim:getWorldState(1):localFrame(1).translation.y=-0.1-0.01 --collision margin: 0.01
	sim:getWorldState(1):forwardKinematics()
	
	--[[
	-- the skeleton of the obstacle should be directly manipulated because fixed joint has 0 dof.
	sim:getWorldState(1):localFrame(1).translation.y=-0.1-0.01 --collision margin: 0.01
	sim:getWorldState(1):localFrame(1).rotation:setRotation(vector3(1,0,0), useCase.slope)
	sim:getWorldState(1):forwardKinematics()

	if useCase.slope<0 then
		sim:getWorldState(1):localFrame(1).translation.y=-0.1-0.01-1 --collision margin: 0.01
		sim:getWorldState(1):forwardKinematics()

		sim:createObstacle(mesh)
		sim:getWorldState(2):localFrame(1).translation.y=-0.1-0.01 --collision margin: 0.01
		sim:getWorldState(2):localFrame(1).rotation:setRotation(vector3(1,0,0), 0)
		sim:getWorldState(2):forwardKinematics()
	end
	]]--
	local floors={sim:skeleton(1)}
	--[[
	if useCase.slope<0 then
		floors[2]=sim:skeleton(2)
	end
	]]
	for i,floor in ipairs(floors) do
		--	 registerContactPairJump(model, self.skel, floor, self.simulator)
		if useCase.registerContactPair then
			useCase.registerContactPair(model, sim:skeleton(0), floor, sim)
		else
			registerContactPairAll(model, sim:skeleton(0), floor, sim)
		end
	end
	sim:setSimulatorParam("debugContact", debugContactParam)   
	sim:setSimulatorParam("contactForceVis", debugContactForceVis)
	sim:setSimulatorParam("penaltyDepthMax", model.penaltyDepthMax) 
	--   convertJointNameToBone(model.bones, self.skel_withoutCOMjoint)

	--   printTable(model.bones)

	local skel_withoutCOMjoint=sim:skeleton(0)
	if true then	
		self.contactQueryBoneInfo={}
		sim:registerContactQueryBone(0,
		MainLib.VRMLloader.upcast(
		skel_withoutCOMjoint:getBoneByVoca(MotionLoader.LEFTANKLE)))
		sim:registerContactQueryBone(1,
		MainLib.VRMLloader.upcast(
		skel_withoutCOMjoint:getBoneByVoca(MotionLoader.RIGHTANKLE)))
		self.contactQueryBoneInfo[0]='L'
		self.contactQueryBoneInfo[1]='R'
		if not useCase.noToeJoints then
			sim:registerContactQueryBone(2,
			MainLib.VRMLloader.upcast(
			skel_withoutCOMjoint:getBoneByVoca(MotionLoader.LEFTTOES)))
			sim:registerContactQueryBone(3,
			MainLib.VRMLloader.upcast(
			skel_withoutCOMjoint:getBoneByVoca(MotionLoader.RIGHTTOES)))
			self.contactQueryBoneInfo[2]='L'
			self.contactQueryBoneInfo[3]='R'
		end
		if useCase.keyframes.importanceLH then
			sim:registerContactQueryBone(4, MainLib.VRMLloader.upcast(
			skel_withoutCOMjoint:getBoneByVoca(MotionLoader.LEFTWRIST)))
			sim:registerContactQueryBone(5, MainLib.VRMLloader.upcast(
			skel_withoutCOMjoint:getBoneByVoca(MotionLoader.RIGHTWRIST)))
			self.contactQueryBoneInfo[4]='LH'
			self.contactQueryBoneInfo[5]='RH'
		end
	end
	return floors
end

function OnlineLocoSynthesis:__init()
	self.externalForce=vector3(0,0,0)
	self.impulse=0

	self.theta=vectorn()
	self.dtheta=vectorn()
	OnlineSynthesis2.__init(self)

	self:initContactState('R')

	if mrd_info.outputMotion==true then
		mrd_info.outputMotion=MotionDOFcontainer(self.skel_withoutCOMjoint.dofInfo)
	end

	if mrd_info.outputContactForce then
		mrd_info.outputContactForce={matrixn(), {vector3(0,0,0), vector3(0,0,0), vector3(0,0,0)}}
	end

	if exportDebugInfo then
		mrd_info.numChannels=table.getn(mrd_channel_strings)
		mrd_channels={}
		for i,v in ipairs(mrd_channel_strings) do
			mrd_channels[v]=i-1
		end

		mrdplot=MRDplot()

		mrdplot:initMRD(mrd_info.numChannels, 0, 1)


		mrd_info.tempv=vectorn(mrd_info.numChannels)
		mrd_info.tempv2=vectorn(1)

		if mrd_channels.rclavicle_theta~=nil then
			mrd_info.rclavicle_dof=self.skel_withoutCOMjoint.dofInfo:startR(self.skel_withoutCOMjoint:getBoneByVoca(MotionLoader.RIGHTCOLLAR):treeIndex())
		end

		for k,v in pairs(mrd_channels) do
			mrdplot.names:set(v, k)
		end
		RE.output("mrdplot", "started")


		-- mrdplotcf=MRDplot()
		-- mrdplotcf:initMRD(controlforce:size(), 0,1)

		-- for i=1,self.skel_withoutCOMjoint:numBones()-1 do
		-- 	 local dofIndex=self.skel_withoutCOMjoint.dofInfo:startT(i)
		-- 	 mrdplotcf.names:set(dofIndex,
		-- 			     tostring(dofIndex)..self.skel_withoutCOMjoint:bone(i):name())
		-- end
	end
	if cleanVisualize==true then
		self.simulSkin=RE.createVRMLskin(self.skel_withoutCOMjoint, false)
		self.simulSkin:scale(100,100,100)
		self.simulSkin:setMaterial("lightblue")
		self.skin2:setVisible(false)
		self.skin2:setTranslation(-100,0,0)
		self.simulSkin:setMaterial("lightgrey_transparent")
	else
		self.skin2:setTranslation(100,0,0)
		self.simulSkin=RE.createVRMLskin(self.skel_withoutCOMjoint, false)
		self.simulSkin:scale(100,100,100)
		self.simulSkin:setMaterial("lightgrey_transparent")

		self.skin3=RE.createVRMLskin(self.skel_withoutCOMjoint, false )
		self.skin4=RE.createVRMLskin(self.skel_withoutCOMjoint, false)
		self.targetSkin=RE.createVRMLskin(self.skel_withoutCOMjoint, false)
		--self.targetSkin_pd=RE.createVRMLskin(self.skel_withoutCOMjoint, false)

		-- 90 degree-rotated skins
		self.simulSkinRot=RE.createVRMLskin(self.skel_withoutCOMjoint, false)
		self.targetSkinRot=RE.createVRMLskin(self.skel_withoutCOMjoint, false)

		if usePDservoPos then
			self.targetPosSkin=RE.createVRMLskin(self.skel, false)
			self.targetPosSkinRot=RE.createVRMLskin(self.skel, false)
			self.targetPosSkin:scale(100,100,100)
			self.targetPosSkin:setTranslation(-150,0,0)
			self.targetPosSkinRot:scale(100,100,100)
			self.targetPosSkinRot:setTranslation(-150,200,0)
		end


		--   self.targetSkin:setVisible(false) 
		--   self.simulSkin:setVisible(false) 

		self.skin3:scale(100,100,100)  
		self.skin3:setTranslation(150,0,0)  
		self.skin4:scale(100,100,100)
		self.skin4:setTranslation(200,0,0)
		self.targetSkin:scale(100,100,100)  
		self.targetSkin:setTranslation(-100,0,0) 
		--self.targetSkin_pd:scale(100,100,100)  
		--self.targetSkin_pd:setTranslation(-200,0,0) 
		self.simulSkinRot:scale(100,100,100)
		self.simulSkinRot:setTranslation(0,200,0)
		self.targetSkinRot:scale(100,100,100)
		self.targetSkinRot:setTranslation(-100,200,0)

	end

--	self.pendulum.skin:setTranslation(100,0,0)  
	self.pendulum:setVisualTranslation(100,0,0)


	self.numSimulFrames=0  

	print("Physics start")  


	-- physics start 
	self.outputMotion=MotionDOF(self.skel_withoutCOMjoint.dofInfo) 
--	self.outputMotion_pdtarget=MotionDOF(self.skel_withoutCOMjoint.dofInfo)
	self.simulator=createSimulator(simulator)
	self.simulator:registerCharacter(self.skel_withoutCOMjoint)

	--   self.floor=VRMLloader("../Resource/mesh/floor_y.wrl")
	--   self.floor:printDebugInfo()

	self.floors=self:setFloor(self.simulator)



--	self.simulator:createObstacle(mesh)
	-- -- the skeleton of the obstacle should be directly manipulated because fixed joint has 0 dof.
	-- self.simulator:getWorldState(2):localFrame(1).translation:assign(vector3(0, -0.1, 1))
	-- self.simulator:getWorldState(2):localFrame(1).rotation:setRotation(vector3(1,0,0), math.rad(-5))
	-- self.simulator:getWorldState(2):forwardKinematics()

	-- self.floors={self.simulator:skeleton(1), self.simulator:skeleton(2)}

	if useCase.slope~=0 then
	--if true then
		-- draw obstacles
		-- remove background so that obstacles can be seen.
		local rootnode =RE.ogreRootSceneNode()
		if rootnode~=nil then
			local bgnode=RE.createChildSceneNode(rootnode , "BackgroundNode")
			self.skinFloors={}
			for i=1,table.getn(self.floors) do
				self.skinFloors[i]=RE.createVRMLskin(self.floors[i], false)
				self.skinFloors[i]:scale(100,100,100)
				self.skinFloors[i]:setPose(self.simulator, i)
			end
		end
	end


	-- self.box=VRMLloader("../Resource/scripts/ui/RigidBodyWin/box1.wrl")
	-- self.skinBox=RE.createVRMLskin(self.box, false)
	-- self.skinBox:scale(100,100,100)
	-- self.simulator:registerCharacter(self.box)


	if model.simulationSubTimesteps==nil then
		model.simulationSubTimesteps=1
	end

	local timestep=model.timestep/model.simulationSubTimesteps
	self.simulator:init(timestep, Physics.DynamicsSimulator.EULER) 





	local initialState, initialVel=self:calcInitialState()

	--initialState:set(1, initialState:get(1)+0.01)
	if self.skel.dofInfo:numSphericalJoint()==2 then
		initialState= MotionDOF.mergeRoot(initialState)
		-- print('a',initialState)
		-- print('a',initialVel)
	else
		-- print('b', initialState)
		-- print('b', initialVel)
	end
	self.simulator:setLinkData(0,Physics.DynamicsSimulator.JOINT_VALUE, initialState) 
	self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, initialVel)
	self.simulator:setGVector(vector3(0,9.8,0)) 
	self.simulator:initSimulation()

	if saveDebugInfo then


		local tt=vectorn()
		self.simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, tt)
		local ee=matrixn()
		if self.simulator.test then
			self.simulator:test('getExactStateSpherical', ee)
		end
		saveDebugInfo(self.simulator, "debugInfo_nocomp0.tbl", {initialState:copy(), initialVel:copy(),tt, ee})
	end

	self.simulSkin:setPoseDOF(initialState)  

	setPoseDOFRot90(self.simulSkinRot,initialState)

	assert(useQPsolver )
	self.pdservo=QPservo(self.skel_withoutCOMjoint.dofInfo, timestep, Physics.DynamicsSimulator.EULER, self.simulator) 
	--self.pdservo=QPservo_switchingRootJoint(self.skel_withoutCOMjoint.dofInfo,timestep, Physics.DynamicsSimulator.EULER, self.simulator, self) 
	self.pdservo:initQPservo(nil, nil, nil, nil)   




--	self.comTracker=PointTracker(self.outputFrameRate, 0.01,50,1)
--	self.zmpTracker=PointTracker(self.outputFrameRate, 0.01,100,1)

	print("constructor end")
	local p=CT.vec(0.01,0.01, 0.01,0.01,0.01,0.01)

	local zmpNew, zmpOriNew=self.pendulum:_getPosition( lastFrame)
	local calcCOMpos=self.calcCOMpos
	local comNew=calcCOMpos(self,zmpNew, zmpOriNew)

	--self.comTracker:init(CT.vec(comNew.x, comNew.y, comNew.z, 0, 0, 0),p)
	--self.zmpTracker:init(CT.vec(zmpNew.x, zmpNew.y, zmpNew.z, 0, 0, 0),p)

	if saveDebugInfo and dbg_testmw then
		saveDebugInfo(self.simulator, "debugInfo_restoreState1.tbl")
		self:saveStates("debugStates_ar1.tbl")
	end
end

function OnlineLocoSynthesis:isNowTheMoment(frameRate)   
	local simulFrame=self.numSimulFrames   
	local freq=model.simulationFrameRate/frameRate
	if math.mod(simulFrame, freq)==0 then      
		return true   
	else      
		return false   
	end

end

function OnlineLocoSynthesis:oneStepSimul()   
	--   print("oneStepSimul")   

	gTimer2:start()
	-- produce one frame of motion and pendulum motion.   
	--           -1' 0'  1'  2'  3'   
	--onestep    .   .   .   .   .   
	--               . depends on 012 (use smoothing to estimate pos and vel at 0')   
	--           .........   
	--           012345678   
	--   	. depends on 0' (and a good estimate of 1' for velocity)   


	-- how story goes?
	-- init() : produce -1' 

	-- oneStep(): predict 0' and 1'' (numFrames=1)
	-- oneStepSimul() : produce 0,1,2,3 based on 0' (samplePose)  and 1'' (samplePose2)
	-- prePareNextStep() : update 0' 
	-- oneStep(): predict 1' and 2'' (numFrames=2)
	-- oneStepSimul() : produce 4,5,6,7 based on 1' and 2''
	-- ...

	local frameRate=self.outputFrameRate
	local simulFrameRate=model.simulationFrameRate   

	local simulFrames=simulFrameRate/frameRate   
	local simulStart=(self.numFrames-1)*simulFrames   
	local simulEnd=(self.numFrames)*simulFrames  
	--   print("[",simulStart, ",",simulEnd,"]")   
	if self.pdservo.dmotionDOF==nil then     
		self.pdservo.dmotionDOF=matrixn()  
		self.pdservo.ddmotionDOF=matrixn()
	end   

	local dmotionDOF=self.pdservo.dmotionDOF   
	local ddmotionDOF=self.pdservo.ddmotionDOF   

	-- copy motionDOF, update dmotionDOF  
	self.pdservo.motionDOF=self.outputMotion  
--	self.pdservo.motionDOF_pdtarget=self.outputMotion_pdtarget

	if usePDservoPos then
		self.pdservo.conPosL=self.outputGlobal.footLglobal
		self.pdservo.conPosR=self.outputGlobal.footRglobal
	end
	gTimer:reset()
	gTimer4:reset()
	--   assert(self.outputMotion:numFrames()==self.numFrames+1) 
	-- one additional frame stores the predicted pose.
	dmotionDOF:resize(self.outputMotion:numFrames(),self.outputMotion:numDOF())   
	ddmotionDOF:resize(self.outputMotion:numFrames(),self.outputMotion:numDOF())   
	local mrdMotion=mrd_info.outputMotion
	if mrdMotion then
		mrd_info.avgTheta=CT.zeros(self.skel_withoutCOMjoint.dofInfo:numDOF())
	end
	self.contactState.contactDepth=vectorn(self.simulator:queryContactDepths():size())
	self.contactState.contactDepth:setAllValue(0)
	-- calc derivative   
	assert(simulStart>=0)
	if simulStart>=0 then   

		if useCase.numericalDerivDmot then
			assert(math.mod(simulFrameRate,frameRate)==0) 
			assert(self.numSimulFrames==simulStart)    
			calcDerivative_row_fd(self.numFrames-1, dmotionDOF, self.outputMotion)  
			local dtheta_d=dmotionDOF:row(dmotionDOF:rows()-2)
			if true then
				-- limb importance에 비례해서 예제 동작의 속도를 사용할지, desired motion의 속도를 사용할지 결정
				local importance=self.ikInfo[1].limbImportance
				local temp=vectorn()
				self.outputLocal:sampleVel(self.outputGlobal:getRefTime(), temp)
				importance.O=1
				importance.LH=importance.LH or 0
				importance.RH=importance.RH or 0
				RE.output2('limbimportance', table.tostring2(importance))
				local dofToLimb=self.pdservo.dofToLimb
				local invmap=self.pdservo.dofToLimbInvMap
				assert(dtheta_d:size()==dofToLimb:size())
				local scale=useCase.limbDScaleCoef or 1
				for i=7,dtheta_d:size()-1 do
					local imp= importance[invmap[dofToLimb(i)]] 
					dtheta_d:set(i, sop.map(imp, 0, 1, temp(i), dtheta_d(i)*scale))
				end
			end

			dmotionDOF:row(dmotionDOF:rows()-1):assign(dtheta_d) 
		else
			self.outputLocal:sampleVel(self.outputGlobal:getRefTime(), dmotionDOF:row(dmotionDOF:rows()-1))
		end
		self.outputLocal:sampleAcc(self.outputGlobal:getRefTime(), ddmotionDOF:row(ddmotionDOF:rows()-1))


		-- if       debugRestoreStates then
		--    debugRestoreStates.dmot=dmotionDOF:row(dmotionDOF:rows()-1):copy()

		-- end

		
		local mocapCompensationCoef=vector3(0.5,0.8,1)
		if useCase.mocapCompensationCoef_x then
			mocapCompensationCoef.x=useCase.mocapCompensationCoef_x
		end

		if useCase.useQPsolver then
			mocapCompensationCoef.y=1
		end
		if useCase.keyframes and useCase.keyframes.mocapCompensationCoef then
			local ri=self.outputGlobal:getRefTime()
			mocapCompensationCoef=self.outputLocal:sampleKey(ri, 'mocapCompensationCoef')
		end


		local currFrame=self.numFrames-1

		local rotY=self:calcRotY(currFrame)
		local refTimeKinematic=self.outputGlobal.refTime(currFrame)
		local comacc_y=self.outputLocal:sampleVal(refTimeKinematic, 'comacc_y' )
		local lcomacc=self.outputLocal:sampleV3(refTimeKinematic, 'lcomacc')
		local comacc=rotate(lcomacc, rotY)
		local landingPosError
		if useCase.calcDesiredFootPosFlightPhase then
			landingPosError=self.outputLocal:sampleVal(refTimeKinematic, 'landingPosError' )
		end
		local currSegIndex=math.floor(refTimeKinematic)
		local currSeg=self.outputLocal.segInfo(currSegIndex).seg


		for currSimulFrame=simulStart,simulEnd-1 do	 
			-- simulation steps	 
			local refTime=currSimulFrame/simulFrames
			assert(0<=refTime and refTime<=dmotionDOF:rows()-1)
			RE.output2("comacc_y", comacc_y, comacc.y)

			--##dos if g_debugOneStep then --##dos g_debugOneStep:pushBack({"controlForce",fixedLocal, leaning, pendvel, cartvel, dvz, ts}) --##dos end -- local mrdCF=mrd_info.outputContactForce -- if mrdCF then --    local currFrame=mrdCF[1]:rows() --    mrmdCF[1]:resize(currFrame+1,6) --    mrdCF[1]:row(currFrame):setVec3(0, mrdCF[2][1]) --    mrdCF[1]:row(currFrame):setVec3(3, mrdCF[2][2]) --    mrdCF[2][1]:assign(vector3(0,0,0)) --    mrdCF[2][2]:assign(vector3(0,0,0)) --    if math.mod(self.numFrames, mrd_info.export_freq)==0 then -- 	 local binaryFile=util.BinaryFile() -- 	 binaryFile:openWrite("debug_plot.cf") -- 	 binaryFile:pack(mrdCF[1]) -- 	 binaryFile:close() --    end	 -- end -- if       debugRestoreStates and currSimulFrame<simulStart+5 then --    if debugRestoreStates.simulll==nil then --       debugRestoreStates.simulll=array:new() --    end --    local dr={} --    local ttt=vectorn() --    local ttt2=vectorn() --            self.simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, ttt) --            self.simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, ttt2) -- 	   dr.ttt=ttt -- 	   dr.ttt2=ttt2 -- 	   dr.cff=controlforce:copy() -- 	   debugRestoreStates.simulll:pushBack(dr) -- end
			gTimer:start()


			self.pdservo:sampleCurrPose(self.simulator)
			local downSampleTargetPoseGen=math.round(model.simulationFrameRate/120)
			if math.mod(currSimulFrame,downSampleTargetPoseGen)==0 then
				local refTimeD=(currSimulFrame+downSampleTargetPoseGen-1)/simulFrames
				self.pdservo:sampleTargetPoses( refTimeD)
			end

			local useQPsolver=useCase.useQPsolver
			local maxContactForceY
			local cf
			if useQPsolver then
				self.pdservo:computeContact(self.simulator, currSeg.swingFoot)
			end

			local function computeTorque()
				do -- position controller
					local idservo=self.pdservo
					if math.mod(currSimulFrame,downSampleTargetPoseGen)==0 then
						-- regenerate target poses

						do
							local cstate=self.pdservo.state -- contact state
							-- too long 
							--RE.output2("cstate", table.tostring(cstate))
							if useCase.calcDesiredFootPosFlightPhase then
								local foot, desiredfootPos, desiredfootPosGlobal, flightTime=useCase.calcDesiredFootPosFlightPhase(self)
								if foot~='N' then
									desiredfootPos:radd(rotate(landingPosError, rotY ))
									self.desiredFootPosFlightPhase=
									{
										foot=foot,
										desiredfootPosLocal=desiredfootPos,
										desiredfootPosGlobal=desiredfootPosGlobal,
										flightTime=flightTime
									}
								end
							end
						end
					end
					if useQPsolver then
						local graph=self.graph
						local footInfo=self.ikInfo[1]
						local footPosL=footInfo[1][1]
						local footPosR=footInfo[2][1]
						local footOriL=footInfo[1][2]
						local footOriR=footInfo[2][2]
						local handPosL,handPosR, handOriL, handOriR
						if footInfo[4] then
							handPosL=footInfo[3][1]
							handPosR=footInfo[4][1]
							handOriL=footInfo[3][2]
							handOriR=footInfo[4][2]
						end

						local headPos, headOri
						local headControlWeight=useCase.headControlWeight
						if headControlWeight~=0 then
							if footInfo[4] then
								headPos=footInfo[5][1]
								headOri=footInfo[5][2]
							else
								headPos=footInfo[3][1]
								headOri=footInfo[3][2]
							end
						end

						local method=useCase.useFootEncodingMethod2
						local fk=self.simulator:getWorldState(0)
						local lprojPos=method:calcProjPos(fk,'L')
						local rprojPos=method:calcProjPos(fk,'R')

						if false then
							dbg.namedDraw('Sphere', lprojPos*100, 'lprojposL')
							dbg.namedDraw('Sphere', rprojPos*100, 'lprojposR')
						end

						local lfoot=self.simulator:getWorldState(0):globalFrame(graph.bone2.lfoot):toGlobalPos(graph.lfootpos)   
						local rfoot=self.simulator:getWorldState(0):globalFrame(graph.bone2.rfoot):toGlobalPos(graph.rfootpos)   
						local lfootori=self.simulator:getWorldState(0):globalFrame(graph.bone2.lfoot).rotation
						local rfootori=self.simulator:getWorldState(0):globalFrame(graph.bone2.rfoot).rotation
						local lfootvel= self.simulator:getWorldVelocity(0, graph.bone2.lfoot, graph.lfootpos)
						local rfootvel= self.simulator:getWorldVelocity(0, graph.bone2.rfoot, graph.rfootpos)
						local lfootavel= self.simulator:getWorldAngVel(0, graph.bone2.lfoot)
						local rfootavel= self.simulator:getWorldAngVel(0, graph.bone2.rfoot)
						local lhand, lhandori
						local rhand, rhandori
						local lhandvel
						local rhandvel
						local impL=footInfo[1][3]
						local impR=footInfo[2][3]
						local impLH,impRH
						if handPosL then
							lhand=self.simulator:getWorldState(0):globalFrame(graph.bone2.lhand):toGlobalPos(graph.lhandpos)   
							rhand=self.simulator:getWorldState(0):globalFrame(graph.bone2.rhand):toGlobalPos(graph.rhandpos)   
							lhandori=self.simulator:getWorldState(0):globalFrame(graph.bone2.lhand).rotation
							rhandori=self.simulator:getWorldState(0):globalFrame(graph.bone2.rhand).rotation
							lhandvel= self.simulator:getWorldVelocity(0, graph.bone2.lhand, graph.lhandpos)
							rhandvel= self.simulator:getWorldVelocity(0, graph.bone2.rhand, graph.rhandpos)
							lhandavel= self.simulator:getWorldAngVel(0, graph.bone2.lhand)
							rhandavel= self.simulator:getWorldAngVel(0, graph.bone2.rhand)
							impLH=footInfo[3][3]
							impRH=footInfo[4][3]
						end
						local head, headori, headvel, headavel
						
						local lheadpos=graph.bone2.head:localCOM()
						if headPos then
							head=self.simulator:getWorldState(0):globalFrame(graph.bone2.head):toGlobalPos(lheadpos)   
							headori=self.simulator:getWorldState(0):globalFrame(graph.bone2.head).rotation
							headvel= self.simulator:getWorldVelocity(0, graph.bone2.head, lheadpos)
							headavel= self.simulator:getWorldAngVel(0, graph.bone2.head)
						end

						local footPosL2, footPosR2, footOriL2, footOriR2
						local handPosL2, handPosR2, handOriL2, handOriR2
						local headPos2, headOri2

						do
							local footInfo=self.ikInfo[2]
							footPosL2=footInfo[1][1]
							footPosR2=footInfo[2][1]
							footOriL2=footInfo[1][2]
							footOriR2=footInfo[2][2]
							if footInfo[4] then
								handPosL2=footInfo[3][1]
								handPosR2=footInfo[4][1]
								handOriL2=footInfo[3][2]
								handOriR2=footInfo[4][2]
							end
							if headPos then
								if footInfo[4] then
									headPos2=footInfo[5][1]
									headOri2=footInfo[5][2]
								else
									headPos2=footInfo[3][1]
									headOri2=footInfo[3][2]
								end
							end

						end
						--print(footPosL, footPosR, impL, impR)
						-- actually qpservo
						idservo:_calcDesiredAcceleration(self)
						local supportFootImportance=footInfo.supportFootImportance
						local limbImportance=footInfo.limbImportance
						idservo:QPsolve(self.simulator, idservo.theta, idservo.dtheta, supportFootImportance, limbImportance )
						local function diffRot(q1, q2, frameRate)
							--local q=quater()
							--q:difference(q2,q1)
							----local v=rotate(q:rotationVector(), q2)
							----
							----local mat1=matrix4()
							----mat1:setRotation(q1)
							----local mat2=matrix4()
							----mat2:setRotation(q2)
							----print('diffRot', q1, q2) print((mat1-mat2):invSkew()*120,q:rotationVector()*120, v*120)
							--return q:rotationVector()
							local tf=transf(q2,vector3(0,0,0))
							local tf2=transf(q1,vector3(0,0,0))
							return rotate(tf:twist(tf2,1/frameRate).w, tf.rotation)
						end
						local k_p=useCase.k_p_EE or 120
						local k_d=useCase.k_d_EE or 12
						local k_pr=k_p
						local k_dr=k_d
						--##dos if g_debugOneStep then --##dos g_debugOneStep:pushBack({"addb",footPosL:copy(), lfoot:copy(), k_p, footPosL2:copy(), footPosL:copy(), lfootvel:copy(), k_d,impL}) --##dos end
						local EEobjWeight=useCase.EEobjWeight or 50000
						local EEobjWeightAngular=useCase.EEobjWeightAngular
						if EEobjWeightAngular then
							do
								local function velAtConstraint(q1,t1,q2,t2,gain1,gain2)
									local tf=transf(q2,t2)
									local tf2=transf(q1,t1)
									local V=tf:twist(tf2,1/120)
									return rotate(V.w,tf.rotation)*gain1, rotate(V.v,tf.rotation)*gain2
								end
								do
									local w_p,v_p=velAtConstraint(footOriL,footPosL,lfootori,lfoot,k_pr/120, k_p/120)
									local w_d,v_d=velAtConstraint(footOriL2,footPosL2,footOriL,footPosL,k_dr,k_d)
									idservo:addBoneObjective2(self.simulator, graph.bone2.lfoot:treeIndex(), graph.lfootpos,w_p+w_d- lfootavel*k_dr, v_p+v_d-lfootvel*k_d, impL*EEobjWeightAngular, impL*EEobjWeight)
								end
								do
									local w_p,v_p=velAtConstraint(footOriR,footPosR,rfootori,rfoot,k_pr/120, k_p/120)
									local w_d,v_d=velAtConstraint(footOriR2,footPosR2,footOriR,footPosR,k_dr,k_d)
									idservo:addBoneObjective2(self.simulator, graph.bone2.rfoot:treeIndex(), graph.rfootpos,w_p+w_d- rfootavel*k_dr, v_p+v_d-rfootvel*k_d, impR*EEobjWeightAngular, impR*EEobjWeight)
								end
								if handPosL then
									if not cleanVisualize then
										dbg.namedDraw('Sphere', handPosL*100+vector3(100,0,0), 'handposL')
										dbg.namedDraw('Sphere', handPosR*100+vector3(100,0,0), 'handposR')
									end
									do
										local w_p,v_p=velAtConstraint(handOriL,handPosL,lhandori,lhand,k_pr/120, k_p/120)
										local w_d,v_d=velAtConstraint(handOriL2,handPosL2,handOriL,handPosL,k_dr,k_d)
										idservo:addBoneObjective2(self.simulator, graph.bone2.lhand:treeIndex(), graph.lhandpos,w_p+w_d- lhandavel*k_dr, v_p+v_d-lhandvel*k_d, impLH*EEobjWeightAngular, impLH*EEobjWeight)
									end
									do
										local w_p,v_p=velAtConstraint(handOriR,handPosR,rhandori,rhand,k_pr/120, k_p/120)
										local w_d,v_d=velAtConstraint(handOriR2,handPosR2,handOriR,handPosR,k_dr,k_d)
										idservo:addBoneObjective2(self.simulator, graph.bone2.rhand:treeIndex(), graph.rhandpos,w_p+w_d- rhandavel*k_dr, v_p+v_d-rhandvel*k_d, impRH*EEobjWeightAngular, impRH*EEobjWeight)
									end
								end
								if headPos then
									do
										local k_p=useCase.k_p_HEAD or k_p
										local k_d=useCase.k_d_HEAD or k_d
										k_pr=k_p
										k_dr=k_d
										local w_p,v_p=velAtConstraint(headOri,headPos,headori,head,k_pr/120, k_p/120)
										local w_d,v_d=velAtConstraint(headOri2,headPos2,headOri,headPos,k_dr,k_d)
										idservo:addBoneObjective2(self.simulator, graph.bone2.head:treeIndex(), lheadpos,w_p+w_d- headavel*k_dr, v_p+v_d-headvel*k_d, headControlWeight, headControlWeight)
									end
								end
							end
						else
							idservo:addBoneObjective(self.simulator, graph.bone2.lfoot:treeIndex(), graph.lfootpos, (footPosL-lfoot)*k_p+((footPosL2-footPosL)*120-lfootvel)*k_d, impL*EEobjWeight)
							idservo:addBoneObjective(self.simulator, graph.bone2.rfoot:treeIndex(), graph.rfootpos, (footPosR-rfoot)*k_p+((footPosR2-footPosR)*120-rfootvel)*k_d, impR*EEobjWeight)
							if handPosL then
								idservo:addBoneObjective(self.simulator, graph.bone2.lhand:treeIndex(), graph.lhandpos, (handPosL-lhand)*k_p+((handPosL2-handPosL)*120-lhandvel)*k_d, impLH*EEobjWeight)
								idservo:addBoneObjective(self.simulator, graph.bone2.rhand:treeIndex(), graph.rhandpos, (handPosR-rhand)*k_p+((handPosR2-handPosR)*120-rhandvel)*k_d, impRH*EEobjWeight)
							end
						end
						local momentumWeight=useCase.momentumWeight or 10000
						local linearMomentumWeight=useCase.linearMomentumWeight

						if self.footStates.usePositionControl==false then
							linearMomentumWeight=nil
						end
						--RE.output2('momentum weight', momentumWeight, linearMomentumWeight)
						if momentumWeight~=0 then
							local desiredAngMomentum=self.outputLocal:sampleV3(refTimeKinematic,'langMomentum')
							+self.outputLocal:sampleKey(refTimeKinematic, "desiredMomentum")
							desiredAngMomentum:rotate(rotY)
							local ldmmt= self.outputLocal:sampleV3(refTimeKinematic,'lpenddotangMomentum')
							local desiredDotAngMomentum=self.outputLocal:sampleV3(refTimeKinematic,'ldotangMomentum') -ldmmt
							
							--util.appendFile('pend3.txt', table.tostring2({i, iinterval, ldmmt})..'\n')

							--desiredDotAngMomentum:zero()
							desiredDotAngMomentum:rotate(rotY)
							desiredDotAngMomentum:radd(self.footStates.desiredDotAngMomentum*useCase.dotMomentumScale)

							local desiredLinMomentum=self.outputLocal:sampleV3(refTimeKinematic,'llinMomentum')
							desiredLinMomentum:rotate(rotY)
							local desiredDotLinMomentum=self.outputLocal:sampleV3(refTimeKinematic,'ldotlinMomentum')
							-self.outputLocal:sampleV3(refTimeKinematic,'lpenddotlinMomentum')
							desiredDotLinMomentum:rotate(rotY)
							desiredDotLinMomentum:radd(self.footStates.desiredDotLinMomentum*useCase.dotMomentumScale)
							local MMM=self.simulator:calcMomentumCOM(0)
							--RE.output2('momentum', MMM)
							-- momentum damping
							
							local k_d=useCase.k_d_momentum or useCase.k_d_EE or 12
							local thr=useCase.momentumThr or 100
							local dmmt=math.smoothClampVec3((desiredAngMomentum-MMM:M())*k_d , thr)+math.smoothClampVec3(desiredDotAngMomentum, thr)
							dmmt:radd(rotate(
							self.outputLocal:sampleKey(refTimeKinematic, "desiredDotMomentum"),rotY))

							local lm=math.smoothClampVec3((desiredLinMomentum-MMM:F())*k_d,thr)+math.smoothClampVec3(desiredDotLinMomentum, thr)
							--RE.output2('desiredDotMomentum', lm, dmmt)
							--RE.output2('desiredMomentum', desiredAngMomentum, MMM:M(), desiredDotAngMomentum)
							idservo:addMomentumObjective(self.simulator,dmmt, lm , momentumWeight, linearMomentumWeight) -- desired linear momentum ignored.
						end
					end
					return idservo.controlforce
				end
			end

			local impulse 
			if self.impulse>0 then
				RE.output2("impulse",self.impulse)
				local chest=self.skel_withoutCOMjoint:VRMLbone(self.skel_withoutCOMjoint:getBoneByVoca(MotionLoader.CHEST2):treeIndex())

				local gf=self.impulseDir
				local frame=self.simulator:getWorldState(0):globalFrame(chest)
				local lf=frame:toLocalDir(gf)

				local dir=gf:copy()
				dir:normalize()
				if self.impulseGizmo then
					local t=transf()
					t:axisToAxis(vector3(0,0,0), vector3(0,1,0), frame.translation*100-10*dir, dir)
					RE.output2("gizmo", t.rotation,t.translation)
					self.impulseGizmo:transform(t)
					dbg.namedDraw('Sphere', t.translation*100, 'impulse',"red", 5)
				else
					local pos=frame:toGlobalPos(chest:localCOM())
					dbg.namedDraw('Arrow',pos*100-50*dir,pos*100,'impulseGizmo')
				end
				print('impulse')
				impulse={chest=chest, lf=lf}
				self.impulse=self.impulse-1
			end

			if useCase.keyframes and useCase.keyframes.feedforwardTorque then
				local ri=self.outputGlobal:getRefTime()
				feedforwardTorque=self.outputLocal:sampleKey(ri, 'feedforwardTorque')
			end
				--##dos if g_debugOneStep then --##dos local qd=self.simulator:queryContactDepths() g_debugOneStep:pushBack({"queryContactDepth",qd:copy(), self.contactState.contactDepth:copy()}) g_debugOneStep:pushBack({"bfullstate",deepCopyTable(self:getStates())}) --##dos end
			local computedTorque=computeTorque()
			for i=1, model.simulationSubTimesteps do
				if self.pdservo.stepSimul then
					self.pdservo:stepSimul(self.simulator, impulse)
				else
					computedTorque:range(0,7):setAllValue(0)
					self.simulator:setLinkData(0,Physics.DynamicsSimulator.JOINT_TORQUE, computedTorque)
					if impulse then
						self.simulator:addForceToBone(0, impulse.chest, impulse.chest:localCOM(), impulse.lf)
					end
					--	 dbg.console()
					if self.simulator:stepSimulation()==false then
						self.errorOccurred=true
						return
					end
				end
			end
			if mrdMotion then
				mrd_info.avgTheta:radd(self.simulator._lastSimulatedPose)		
			end

			local qd=self.simulator:queryContactDepths()
			self.contactState.contactDepth:radd(qd)
				--##dos if g_debugOneStep then --##dos g_debugOneStep:pushBack({"queryContactDepth",qd:copy(), self.contactState.contactDepth:copy()}) --##dos g_debugOneStep:pushBack({"fullstate",deepCopyTable(self:getStates())}) --##dos end
			gTimer:pause()
			self.numSimulFrames=self.numSimulFrames+1




			--	 if true then
			--	    -- only for extremely slow debugging
			--	    self.simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, self.theta)
			--	    self.simulSkin:setPoseDOF(self.theta)
			--	    setPoseDOFRot90(self.simulSkinRot,self.theta)
			--	    renderOneFrame()
			--	 end

			--	 if self:isNowTheMoment(model.renderingFrameRate)==true 
			--	 and self:isNowTheMoment(model.motionFrameRate)==false then
			--	    self.targetSkin:setVisible(true)
			--	    self.simulSkin:setVisible(true)	
			--	    self.targetSkin:setPoseDOF(self.pdservo.theta_d)
			--	    self.simulSkin:setPose(self.simulator,0)	
			--	    noFrameMove=true
			--	    RE.renderOneFrame(false)	
			--	    noFrameMove=nil
			--	 end  
		end     
		--      self.targetSkin:setVisible(true)   
		--      self.simulSkin:setVisible(true)  

		if cleanVisualize~=true then
			self.targetSkin:setPoseDOF(self.pdservo.theta_d)  
--			self.targetSkin_pd:setPoseDOF(self.pdservo.theta_d_pd)  
			setPoseDOFRot90(self.targetSkinRot, self.pdservo.theta_d)

			if usePDservoPos then
				self.targetPosSkin:setPoseDOF(self.pdservo.theta_o)
				setPoseDOFRot90(self.targetPosSkinRot, self.pdservo.theta_o)
			end
			--      self.simulSkin:setPose(self.simulator,0)
		end

		self.simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, self.theta)
		self.simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, self.dtheta)
		self.simulSkin:setPoseDOF(self.theta)
		setPoseDOFRot90(self.simulSkinRot,self.theta)

		-- if       debugRestoreStates then
		--    debugRestoreStates.theta=self.theta:copy()
		-- end
	end

	if self.impulse<=0 and self.impulseGizmo~=nil then
		self.objectList:erase("arrow2")
		self.impulseGizmo=nil
	end


	if mrdMotion then
		local compactSize=true
		local currFrame=mrdMotion:numFrames()
		mrdMotion:resize(currFrame+1)
		mrd_info.avgTheta:rdiv(simulFrames)		
		mrd_info.avgTheta:setQuater(3, self.theta:toQuater(3):Normalize())
		--mrd_info.avgTheta:setQuater(3, mrd_info.avgTheta:toQuater(3):Normalize())
		mrdMotion:row(currFrame):assign(mrd_info.avgTheta)
		--      pr(self.theta(1))
		local conL, conR
		do
			local cs=self.contactState
			if cs.contactDepth:size()==2 then
				conL=cs.contactDepth(0)>0 
				conR=cs.contactDepth(1)>0
			else
				conL=cs.contactDepth(0)>0 or cs.contactDepth(2)>0
				conR=cs.contactDepth(1)>0 or cs.contactDepth(3)>0
			end
		end

		mrdMotion.conL:set(currFrame, conL)
		mrdMotion.conR:set(currFrame, conR)

		if not mrdMotion.signals.state then
			mrdMotion.signals.state=matrixn()
			mrdMotion.signals.cf=matrixn()
		end
		if not compactSize then
			local tgt={MotionLoader.RIGHTANKLE, MotionLoader.RIGHTTOES}
			local tgtt={0,0}
			local cdof=0
			for i,tgt in ipairs(tgt) do
				local treeindex=self.skel_withoutCOMjoint:getBoneByVoca(tgt):treeIndex()
				tgtt[i]=treeindex
				local sdof=self.skel_withoutCOMjoint.dofInfo:startR(treeindex)
				local edof=self.skel_withoutCOMjoint.dofInfo:endR(treeindex)
				cdof=cdof+edof-sdof
			end
			local signals=mrdMotion.signals
			if signals.desiredAcc==nil then
				signals.desiredAcc=CT.zeros(currFrame+1, cdof*3)
			end
			local dim=cdof
			signals.desiredAcc:resize(currFrame+1,cdof*3)

			cdof=0
			for i,tgt in ipairs(tgt) do
				local treeindex=tgtt[i]
				local sdof=self.skel_withoutCOMjoint.dofInfo:startR(treeindex)
				local edof=self.skel_withoutCOMjoint.dofInfo:endR(treeindex)
				signals.desiredAcc:row(currFrame):range(cdof, cdof+edof-sdof):assign(self.pdservo.desiredacceleration:range(sdof,edof))
				signals.desiredAcc:row(currFrame):range(dim+cdof, dim+cdof+edof-sdof):assign(self.pdservo.theta:range(sdof,edof))
				signals.desiredAcc:row(currFrame):range(dim*2+cdof, dim*2+cdof+edof-sdof):assign(self.pdservo.theta_d:range(sdof,edof))
				cdof=cdof+edof-sdof
			end
		end
		local signals=mrdMotion.signals
		if not compactsize then 
			local poses={'pose1','pose2'}
			for i,v in ipairs(poses) do
				if signals[v]==nil then
					signals[v]=CT.zeros(currFrame+1, mrdMotion.mot:cols())
				end
				signals[v]:resize(currFrame+1,mrdMotion.mot:cols())
			end
			signals.pose1:row(currFrame):assign(self.samplePose)
			signals.pose2:row(currFrame):assign(self.pdservo.theta_d) 
			if dbg._coordinates then	
				for k,v in pairs(dbg._coordinates) do
					if signals['coord_'..k]==nil then
						signals['coord_'..k]=CT.zeros(currFrame+1,7)
					end
					local mat=signals['coord_'..k]
					mat:resize(currFrame+1,7)
					mat:row(currFrame):setVec3(0, v.translation)
					mat:row(currFrame):setQuater(3, v.rotation)
				end
			end
		end
		do 
			local state=mrdMotion.signals.state 
			local refTime=self.outputGlobal.refTime
			local refTimeKinematic=refTime(math.min(currFrame,refTime:size()-1))
			local currSeg=self.outputLocal.segInfo(math.floor(refTimeKinematic))
			state:resize(currFrame+1,1)
			local segName=string.upper(currSeg.seg.name)
			if segName=='LR' or segName=='RL' then
				state:set(currFrame,0, 0.5)
			elseif segName=="L" then
				state:set(currFrame,0, 0)
			elseif segName=="R" then
				state:set(currFrame,0, 1)
			else
				state:set(currFrame,0, -0.2)
			end
			local mrdCF=mrd_info.outputContactForce
			if mrdCF then
				local cf=mrdMotion.signals.cf
				cf:resize(currFrame+1,9)
				local frameRate=self.outputFrameRate
				local simulFrameRate=model.simulationFrameRate   
				local simulFrames=simulFrameRate/frameRate   
				cf:row(currFrame):setVec3(0, mrdCF[2][1]/simulFrames)
				cf:row(currFrame):setVec3(3, mrdCF[2][2]/simulFrames)
				cf:row(currFrame):setVec3(6, mrdCF[2][3]/simulFrames)

				mrdCF[2][1]:assign(vector3(0,0,0))
				mrdCF[2][2]:assign(vector3(0,0,0))
				mrdCF[2][3]:assign(vector3(0,0,0))
			end
		end

		if math.mod(self.numFrames, mrd_info.export_freq)==0 then
			mrdMotion:exportMot(str.left(mrd_info.filename,-3).."dof")
		end	 
	end

	--RE.output2("debug_computedTorque", self.pdservo.controlforce)
	--RE.output2("debug_theta", self.pdservo.theta)
	--RE.output2("debug_dtheta", self.pdservo.dtheta)
	if exportDebugInfo and self.numFrames>=mrd_info.min_frame then

		if mrd_channels.rclavicle_theta~=nil then
			local dofIndex=mrd_info.rclavicle_dof
			mrd_info.tempv:set(mrd_channels.rclavicle_theta, self.pdservo.theta(dofIndex))
			mrd_info.tempv:set(mrd_channels.rclavicle_theta_d, self.pdservo.theta_d(dofIndex))
			mrd_info.tempv:set(mrd_channels.rclavicle_dtheta, self.pdservo.dtheta(dofIndex))
			mrd_info.tempv:set(mrd_channels.rclavicle_dtheta_d, self.pdservo.dtheta_d(dofIndex))
			mrd_info.tempv:set(mrd_channels.rclavicle_torque, self.pdservo.controlforce(dofIndex))
			mrd_info.tempv2:set(0, mrd_info.tempv(mrd_channels.rclavicle_theta_d)-mrd_info.tempv(mrd_channels.rclavicle_theta))
			VRMLloader.projectAngles(mrd_info.tempv2)
			mrd_info.tempv:set(mrd_channels.delta, mrd_info.tempv2(0))
		end

		if mrd_channels.COMx~=nil then
			local com=self.simulator:calculateCOM(0)

			mrd_info.tempv:set(mrd_channels.COMx, com.x)
			mrd_info.tempv:set(mrd_channels.COMy, com.y)
			mrd_info.tempv:set(mrd_channels.COMz, com.z)
		end

		mrdplot:addPoint(mrd_info.tempv)

		RE.output("mrdplot" , tostring(mrdplot:numPoints()))

		if math.mod(mrdplot:numPoints(), mrd_info.export_freq)==0 then
			mrdplot:save(mrd_info.filename)
			RE.output("mrdplot", "exported"..self.numFrames)
		end
	end

	-- local controlforce=self.pdservo.controlforce

	-- if mrdplotcf~=nil then

	--    mrdplotcf:addPoint(controlforce)
	--    if math.mod(mrdplotcf:numPoints(),900)==0 then
	-- 	 mrdplotcf:save("controlforce.mrd")
	--    end
	-- end

	gTimer:stopMsg("onestepSimul::stepSimulation")
	gTimer2:stopMsg("onestepSimul")
	gTimer4:stopMsg("onestepSimul::stepSimul")

end

function OnlineLocoSynthesis:prepareNextSimulStep()

	--------------------------------------------------------------------------------------



	-- feedback timing error:
	--------------------------------------------------------------------------------------
	-- 1. check if touch-down occurs in target motion

end


function FootInfo(pos, time)
	local out={}
	out.pos=vector3()
	out.pos:assign(pos)
	out.time=time
	return out
end


function OnlineLocoSynthesis:errorFeedbackFootState2(footTransition)
	-- foot transition == "LT" means L to T transition. Prevent LR or TT transition from happening.
	RE.output("transition", footTransition..tostring(self.numFrames-1))

	if disableTouchDownFeedback then return end

	-- currFrame corresponds to the last frame of the simulated motions
	--                       and self.samplePose 
	local currFrame=self.numFrames-1 

	if currFrame==0 then return end

	-- detected transition. search nearby





	if true then


		-- method 0 : flight phase에만 동작. 
		local cs=self.contactState


		if not cs.L and not cs.R then

			local transitionTime=self.outputGlobal:searchTransition(currFrame,footTransition)

			local landingTime

			if footTransition=="LT" then
				landingTime=self.outputGlobal:searchTransition(currFrame, "TR")
			else
				assert(footTransition=="RT")
				landingTime=self.outputGlobal:searchTransition(currFrame, "TL")
			end

			if transitionTime~=-1 and landingTime~=-1 then
				-- assert(transitionTime~=-1)
				-- assert(landingTime~=-1)

				print(footTransition)


				local landingHeightDiff=self.outputGlobal.synRoot(landingTime,1)-self.outputGlobal.synRoot(transitionTime,1)
				RE.output2("flight", transitionTime,landingTime,landingHeightDiff)

				self:predict_landing_trigger_run(transitionTime, landingTime, landingHeightDiff)
			end	 
		elseif self.touchDownTrigger~=nil and (footTransition =="TL" or footTransition=="TR") then
			self.touchDownTrigger[1](self,self.touchDownTrigger[2])
			self.touchDownTrigger=nil
		end

	end
end


function OnlineLocoSynthesis:errorFeedbackFootState(state, time)
	--   if true then return end
	-- adjust timeScale
	local currFrame=self.numFrames-1

	local targetTime=self.outputGlobal:search(time, state)

	if targetTime==-1 then
		RE.output("timingAdjust", time.."->???")
		return
	end
	local error=targetTime-time
	local alpha=0.5*120

	-- (T2'  -   T1)/(     T2         -    T1    )
	-- (alpha+error)/(alpha+targetTime-targetTime)

	if targetTime~=0 then
		self.timeScale=(alpha+error)/alpha
		if self.timeScale<0.1 then
			self.timeScale=0.1
		end
	end
	RE.output2("timingAdjust", time,targetTime,self.timeScale)

end

function OnlineLocoSynthesis:initContactState(supportFoot)
	self.contactState={}

	local cs=self.contactState

	cs.supportFoot=supportFoot
	cs.swingFoot="L"
	cs.refTime=0
	if supportFoot=="L" then
		cs.swingFoot="R"
	end
	cs.transition={}
	cs.transition.type=supportFoot..supportFoot
	cs.transition.time=0
	self.prevContactState=deepCopyTable(cs)
end

function OnlineLocoSynthesis:encodeContactState(ps,cs, supportFoot, refTime)

	ps.supportFoot=cs.supportFoot
	ps.refTime=cs.refTime
	ps.swingFoot=cs.swingFoot

	cs.supportFoot=supportFoot
	cs.refTime=refTime
	cs.swingFoot="L"
	if cs.supportFoot=="L" then
		cs.swingFoot="R"
	end

	if ps.supportFoot~=cs.supportFoot then
		if cs.transition then
			ps.transition=deepCopyTable(cs.transition)
		end
		cs.transition={}
		cs.transition.time=self.numFrames-1
		cs.transition.type=ps.supportFoot..cs.supportFoot
	end 
end

function calcMinHeight(bone, tf)

	local mesh=bone:getMesh()

	local minY=1000000
	for v=0, mesh:numVertex()-1 do
		local lpos=mesh:getVertex(v)
		local gpos=tf:toGlobalPos(lpos)
		if gpos.y<minY then minY=gpos.y end
	end

	return minY
end

function OnlineLocoSynthesis:prepareNextStep() 

	if self.errorOccurred or error_feedback_method==EFM.NONE then 
		-- character falled down.
		RE.output2('falled down')
	else
		gTimer2:start()

		local depth={}
		do
			local skel=self.skel_withoutCOMjoint
			skel:setPoseDOF(self.theta)
			-- assuming self.graph.skel_withoutCOMjoint:setPoseDOF(..) was called.
			local unconvert=self.unconvertConPos
			local projpos={}
		--	dbg.draw('Coordinate', lprojpos, 'lprojpos', vector3(0,0.05,0))
			local cs=self.contactState

			local limbs={'L','R'}

			if cs.contactDepth:size()==2 then
				depth.L=cs.contactDepth(0)
				depth.R=cs.contactDepth(1)
			elseif cs.contactDepth:size()==4 then
				depth.L=math.min(cs.contactDepth(0),cs.contactDepth(2))
				depth.R=math.min(cs.contactDepth(1),cs.contactDepth(3))
			else
				depth.L=math.min(cs.contactDepth(0),cs.contactDepth(2))
				depth.R=math.min(cs.contactDepth(1),cs.contactDepth(3))
				depth.LH=cs.contactDepth(4)
				depth.RH=cs.contactDepth(5)
				limbs={'L','R','LH','RH'}
			end
			projpos.L=unconvert("L",self.graph.lfootpos)
			projpos.R=unconvert("R",self.graph.rfootpos)
			if #limbs==4 then
				projpos.LH=unconvert("LH",self.graph.lhandpos)
				projpos.RH=unconvert("RH",self.graph.rhandpos)
			end
			for i,isL in ipairs(limbs) do
				self.footStates['prevContact'..isL]=self.footStates['isContact'..isL]
			end

			for i,isL in ipairs(limbs) do
				self.footStates[isL]=projpos[isL]
				if depth[isL]>0 then
					self.footStates['isContact'..isL]=true
				else
					--self.footStates[isL]=nil
					self.footStates['isContact'..isL]=false
				end
			end
		end

		assert(self.numSimulFrames>0, self.numSimulFrames)   
		--------------------------------------------------------------------------------------
		-- feedback timing error:
		--------------------------------------------------------------------------------------
		-- 1. check if touch-down occurs in simulated motion
		local cs=self.contactState

		local contactDepth=depth

		-- query contact가 깊이를 리턴하도록 고친다.

		-- visualize contact state

		do 
			local ps=self.prevContactState   
			local currFrame=self.numFrames-1
			local outputG=self.outputGlobal
			local outputL=self.outputLocal
			local refTime=outputG.refTime(currFrame)
			local currSegIndex=math.floor(refTime)
			local currSeg=outputL.segInfo(currSegIndex)
			local prevSeg=outputL.segInfo(currSegIndex-1)
			local currSupportFoot=string.sub(currSeg.seg.name, 1,1)
			local prevSupportFoot=string.sub(prevSeg.seg.name, 1,1)

			RE.output2("trackingInfo2", currFrame, outputG.refTime(currFrame), currSeg.seg.name)
			if cs.transition.time==self.numFrames-1 then

				local latestChange=prevSupportFoot..currSupportFoot
				if latestChange==cs.transition.type then
					-- late touch down	 
					--RE.output("timingFeedback", "late touch down")

					-- releaseLock(currSeg) -- lock was set during predictGlobal (when segIndex was about to change, len was modified so that segIndex doesn't change

				elseif currSupportFoot == ps.supportFoot then

					local supposedTime=outputL.segInfo(currSegIndex+1).invRefTime
					local delta=supposedTime-currFrame
					if delta<currSeg.len then

						-- early touch down
						--RE.output2("timingFeedback", "early touch down")
						outputG.refTime:set(currFrame, currSegIndex+1)
						local nextSeg=outputL.segInfo(currSegIndex+1)

						assert(string.sub(nextSeg.seg.name,1,1)==cs.supportFoot)
						-- -- timewarping for preventing discontinuity
						currSeg.len=currSeg.len-delta
						currSeg.nextLen=currSeg.nextLen+delta
						prevSeg.nextLen=prevSeg.nextLen-delta
					else
						print("Error?")
					end
				else
					--RE.output2("timingFeedback", "supportFootChangeIgnored", currSupportFoot, ps.supportFoot, cs.supportFoot)
				end
			elseif currSegIndex~=math.floor(outputG.refTime(currFrame-1)) and cs.supportFoot==prevSupportFoot then
				--RE.output2("timingFeedback", "lock late touch down", currFrame)
				lockLateTouchDown=true
			end
		end

		-- update self.outputGlobal.synRoot   
		-- update self.pendulum   
		-- 1. calc actual ZMP and COM from the synthesized motion: ZMPcalculator online(online smoothing)   
		-- low-level to high-level: calc ZMP from footPosition:   
		-- coordinate :   
		--   translation=center of Lfoot and Rfoot   
		--   rotation=axisToAxis(Lfoot-Rfoot, vector3(0,0,1)),   
		--   scale=1/(Lfoot-Rfoot):length()   
		-- analysis step will be implemented in ZMPgraph.   
		-- 2. feedback to pendulum state.   
		-- in the oneStep() function, the inverse operation will be performed.   
		-- high-level to low-level: calc footPosition from ZMPtrajectory   
		-- produce one frame of motion and pendulum motion.   


		do

			local examplePose=self.samplePoseBeforeIK:copy()
			local lastFrame=self.numFrames-1 
			local refTime=self.outputGlobal.refTime(lastFrame)
			--print(refTime)
			local examplePoseNoMerge=vectorn()
			self.outputLocal:samplePose(refTime, examplePoseNoMerge)
			--print(examplePose-MotionDOF.mergeRoot(examplePoseNoMerge))
			
			local sim=self.simulator
			if true then
				-- align root transformation
				local ctf=MotionDOF.rootTransformation(sim._lastSimulatedPose)
				local etf=MotionDOF.rootTransformation(examplePose)

				local diff=quater()
				diff:difference(etf.rotation, ctf.rotation)
				local coffsetQ=quater() local crotY=quater()
				diff:decompose(crotY, coffsetQ )
				etf.rotation:mult(crotY, etf.rotation:copy())
				MotionDOF.setRootTransformation(examplePose, etf)
			end
			self.skel_withoutCOMjoint:setPoseDOF(examplePose)-- self.samplePose corresponds to the last frame

			--local delta_t=1.0/self.outputFrameRate
			local delta_t=1.0
			local sign=1.0


			local mm
			local dbgPose
			if true then
				mm=sim:calcMomentumCOMfromPose(0, delta_t, sim._lastSimulatedPose, examplePose) sign=-1.0
				--mm=sim:calcMomentumCOMfromPose(0, delta_t, examplePose, sim._lastSimulatedPose)
			else
				local pose2=vectorn()
				pose2:interpolate(delta_t, examplePose,sim._lastSimulatedPose)
				local rootOri2=quater()
				rootOri2:interpolate(delta_t,examplePose:toQuater(3),sim._lastSimulatedPose:toQuater(3))
 
				pose2:setQuater(3, rootOri2)
				mm=sim:calcMomentumCOMfromPose(0, delta_t*delta_t, examplePose, pose2)

				dbgPose=pose2
			end

			if false then --sim._lastSimulatedPose(1)<examplePose(1)*0.7 then
				error_feedback_method=EFM.NONE -- fall down.
				print("falldown")
				return
			end

			local I=vectorn()
			sim:calcInertia(0, sim._lastSimulatedPose,I);
			--sim:calcInertia(0, examplePose,I);
			--dbg.console()
			local refTime=self.outputGlobal:getRefTime()
			local zmpToPelvis=transf(self.outputLocal:sampleQ(refTime, 'zmpToPelvisR'),
			self.outputLocal:sampleV3(refTime, 'zmpToPelvisT'))

			-- zmpToPelvis=zmpCoord:inverse()*pelvis
			--> zmpCoord=pelvis*zmpToPelvis:inverse()
			local examplePendulum

			if examplePose:size()~=examplePoseNoMerge:size() then
				local tf12=MotionDOF.transformation(examplePose,0)
				local tf2=MotionDOF.transformation(examplePoseNoMerge,7)
				local tf1=tf12*tf2:inverse()
				examplePendulum=tf1*zmpToPelvis:inverse()
			else
				examplePendulum=MotionDOF.rootTransformation(examplePose)*zmpToPelvis:inverse()
			end

			if false then
				dbg.namedDraw('Coordinate', MotionDOF.rootTransformation(examplePose), "examplePose")
				dbg.namedDraw('Coordinate', examplePendulum, "examplePendlum")
			end
			local posErr
			local w
			local totalmass=I(6)
			local invI
			do
				-- use 6 by 6
				local m=totalmass
				local r=vector3(I(7), I(8), I(9))	
				local I6=CT.mat(6,6,I(0),I(3), I(4), 0, -r.z, r.y,
				I(3),I(1), I(5), r.z, 0,  -r.x,
				I(4),I(5), I(2), -r.y, r.x, 0 ,
				0, r.z, -r.y, m,   0 , 0,
				-r.z,0,  r.x, 0,  m,  0,
				r.y, -r.x, 0, 0,  0,  m)

				invI=CT.inverse(I6)
				local mM=mm:M()
				local mF=mm:F()
				local m=CT.mat(6,1,mM.x, mM.y, mM.z, mF.x, mF.y, mF.z)
				local ww=invI*m

				w=vector3(ww(0,0), ww(1,0), ww(2,0))*(delta_t*sign*(useCase.conservativeW or 1))
				posErr=vector3(ww(3,0), ww(4,0), ww(5,0))*(delta_t*sign)
			end

			local wq=quater()
			wq:setRotation(w)

			--dbg.console()

			-- modify examplePendulum configuration based on the detected error = mm/I to obtain the estimated pendulum configuration
			local zmpNew=examplePendulum.translation:copy()
			local comNew=self:calcCOMpos(zmpNew, examplePendulum.rotation)
			
			zmpNew:assign(comNew+rotate(zmpNew-comNew, wq))
			comNew:radd(posErr)
			zmpNew:radd(posErr)
			-- project to the ground
			comNew.y=comNew.y-zmpNew.y
			zmpNew.y=0

			local dbgMode=false

			if dbgMode then
				if dbgskin==nil then
					dbgskin=RE.createVRMLskin(self.skel_withoutCOMjoint, false)
					dbgskin:scale(100,100,100)
					--dbgskin:setTranslation(100,100,0)
				end
				dbgskin:setPoseDOF(dbgPose or examplePose)
				RE.output2("momentumEst", mm:M(), mm:F())
				dbg.namedDraw('Sphere', zmpNew*100+vector3(100,0,0), 'zmpNew')
				dbg.namedDraw('Sphere', comNew*100+vector3(100,0,0), 'comNew')
				dbg.namedDraw('Line', comNew*200-zmpNew*100+vector3(0,0,0), comNew*100+vector3(0,0,0), "ZMPCOM2")
				dbg.namedDraw('Line',  comNew*100+vector3(100,0,0), comNew*100+vector3(100,0,0) +mm:M(),'mmmm')
				dbg.namedDraw('Line',  comNew*100+vector3(100,0,0), comNew*100+vector3(100,0,0) +mm:F(),'ffff','solidblue')
			end

			local rotAxis_y

			do

				local rotY=quater()
				local q=quater()
				if true then
					------------------------------------------------------------  
					-- feedback rot_y  (update self.outputGlobal.synRoot(self.numFrames-1)
					------------------------------------------------------------  

					local offset=quater()
					q:mult(wq, examplePendulum.rotation)
					q:decompose(rotY, offset)
				end
				local MMM=self.simulator:calcMomentumCOM(0)
				--local cv=self.simulator:calculateCOMvel( 0)
				--print('zero?',cv*totalmass-MMM:F())

				-- need to convert MMM into a pendulum momentum!
				--


				local MMM_refM=self.outputLocal:sampleV3(refTime, 'langMomentum')
				local MMM_refF=self.outputLocal:sampleV3(refTime, 'llinMomentum')
				local MMM_refPendM=self.outputLocal:sampleV3(refTime, 'lpendangMomentum')
				local MMM_refPendF=self.outputLocal:sampleV3(refTime, 'lpendlinMomentum')
				local MMM_delta_M=MMM_refPendM-MMM_refM
				local MMM_delta_F=MMM_refPendF-MMM_refF

				local mM=MMM:M()+rotate(MMM_delta_M,rotY)
				local mF=MMM:F()+rotate(MMM_delta_F,rotY) -- character's momentum has converted to pendulum's.
				local m=CT.mat(6,1,mM.x, mM.y, mM.z, mF.x, mF.y, mF.z)
				local ww=invI*m
				local w=vector3(ww(0,0), ww(1,0), ww(2,0))
				local comV=vector3(ww(3,0), ww(4,0), ww(5,0))
				w.y=0 -- todo: feedback this angular momentum error along vertical axis
				local v=w:cross(self.pendulum.pole:localCOM())
				


				local cartV=comV-v

				--print('w,v=',w,v)
				--
				self.pendulum:setStateRaw(zmpNew,q, cartV, w)
				do
					local synRoot=self.outputGlobal.synRoot      
					self.graph.id_rotY.setRotY(synRoot, lastFrame, rotY)

					local actualCOM=self.simulator:calculateCOM(0)
					self.graph.id_rotY.setCOM_height(synRoot, lastFrame, actualCOM.y)


					local vpend=self.verticalPend
					vpend:setState(actualCOM.y, MMM:F().y/vpend.M)
					vpend:setDesiredState(comNew.y, MMM_refF.y/vpend.M)

					--print('actCOM', actualCOM.y, comNew.y)
				end
			end
		end
		------------------------------------------------------------ 
		-- (TODO) feedback cart - pole_com distance   
		------------------------------------------------------------
	end

	gTimer2:stopMsg("prepareNextStep")
end
-- input: outputGlobal, self.pendulum.
function OnlineLocoSynthesis:oneStep()

	--   print("oneStep", self.numFrames, self.numSimulFrames)


	local currFrame=self.numFrames

	-- predict one more frame (temporalily. for estimating velocity)

	OnlineSynthesis2.oneStep(self)


	if self.frameTrigger~=nil then
		if self.frameTrigger[1]>currFrame then
			self[self.frameTrigger[2]](self,self.frameTrigger[3])
			self.frameTrigger=nil
		end

		-- if g_debugOneStep then
		-- 	 g_debugOneStep:pushBack("frametrigger")
		-- end

	end

	gTimer2:start()

	if cleanVisualize~=true then
		self.skin3:setPoseDOF(self.samplePose2)
		self.skin4:setPoseDOF(self.samplePoseBeforeIK)
	end
	self.outputMotion:resize(currFrame+2)   
	self.outputMotion:row(currFrame):assign(self.samplePose) 
	self.outputMotion:row(currFrame+1):assign(self.samplePose2)
--	self.outputMotion_pdtarget:resize(currFrame+2)
--	self.outputMotion_pdtarget:row(currFrame):assign(self.samplePose_pdtarget)
--	self.outputMotion_pdtarget:row(currFrame+1):assign(self.samplePose_pdtarget)
	--
	--    debugRestoreStates.samplePose=self.samplePose:copy()
	--    debugRestoreStates.samplePose2=self.samplePose2:copy()
	-- end


	-- a good prediction for the next frame. (will be overwritten next frame)
	gTimer2:stopMsg("oneStep2")

end
