
require("config")

require("RigidBodyWin/IPC_based/common")
require("RigidBodyWin/IPC_based/useCases")

if useCase.unoptimized~=nil then
	useCase=useCase.unoptimized
end

require("RigidBodyWin/subRoutines/Optimizer")

require("RigidBodyWin/motiongraph")
require("RigidBodyWin/IPC_based/LocoGraph")
require("RigidBodyWin/IPC_based/CompareChain")

require('RigidBodyWin/IPC_based/trajGenerator')

scenario=useCase.scenario
model=scenarios.toModel(scenario)


require("RigidBodyWin/IPC_based/LocoSynthesis")
useContactCentroidAsBase =false
require("RigidBodyWin/IPC_based/LocoSimulation")

PDservoLatency=0
PredictionLatency=0
outputSuperSample=1
useSmoothnessTerm=false -- at first, use true. Later use false to further refine the optimized result. 

useCartPole2=false
useID_C1=false
useCartPolClass=true
usePDservoPos=false

gTimer=util.PerfTimer2()--(1,"timer1")
gTimer2=util.PerfTimer2()--(1,"timer2")
gTimer3=util.PerfTimer2()--(1,"timer3")

function util.PerfTimer:stopMsg(str)
	--self:stop()
	--print(str)
end

function util.PerfTimer2:stopMsg(str)
	--RE.output(str, tostring(self:stop()))
end

CartPoleController=LUAclass(OnlineSynthesis2)

-- 할일. 
-- 특정 상황에서 밸런스를 잡는 방법은 다양하다.
-- ZMP를 어떤 경로로 움직일건지.. 미래에 대한 플래닝도 포함한다.
-- 우리는 간단한 컨트롤러를 사용하고, 그 예측값의 에러를 저장 재사용하는 방식을 취한다.
-- 물론 컨트롤러는 예측값의 에러가 작아지도록 설계되어야 된다.


function OnlineSynthesis2:sampleRotY(frame)
	--override
	return MotionDOF.rootTransformation(self.graph.motions[1].mot:row(frame)).rotation:rotationY()
end


function CartPoleController:printSeq()
	for i,v in ipairs(self.sequence) do
		print(v.grpName, v.name)
	end
end
function CartPoleController:__init(segmentationName)


	RE.renderer():fixedTimeStep(false)


	OnlineSynthesis2.__init(self)


	--   self.controlskin=RE.createConnectedVRMLskin(self.skel, self.motionDOF)

	self.skin1=RE.createVRMLskin(self.skel_withoutCOMjoint, false)

	self.skin1:setMaterial("lightgrey_transparent")

	self.skin2=nil

	self.skin1:scale(100,100,100)
	--   self.skin1:setTranslation(0,100,0)

	self.fk=BoneForwardKinematics(self.skel_withoutCOMjoint)
	self.fk:init()
	self.compareChain=CompareChain(self.skel_withoutCOMjoint, self.fk, self.skel_withoutCOMjoint:fkSolver())

	self.controlforce=matrixn()

	do
		-- standing motions
		--	   local prev_n=table.getn(self.firstFrames)
		--	   self.firstFrames[prev_n+1]=self.firstFrames[prev_n]+self.firstFrames[prev_n]-self.firstFrames[prev_n-1]
		self.sequence={}

		local segmentation=useCase.segmentations[segmentationName]

		assert(segmentation.grpName and segmentation.names and segmentation.swingFoot and segmentation.usePositionControl)-- already rectified

		self.firstFrames=shallowCopyTable(segmentation.firstFrames)
		for k,v in pairs(useCase.segProperties) do
			self[k]=shallowCopyTable(segmentation[k])
		end

		local numSeg=LocoGraph.getNumSeg(self.firstFrames)
		for i=1, numSeg do
			local grpName=segmentation.grpName 
			local segName=segmentation.names[i]
			self.sequence[i]=self.graph[grpName][segName]
			assert(self.sequence[i])
		end

		self.lastFrame=self.firstFrames[table.getn(self.firstFrames)]
		print("segmentation", self.firstFrames[1], self.lastFrame)
	end

	self.theta=vectorn()
	self.dtheta=vectorn()

	if self.sequence~=nil then
		local n_seq=table.getn(self.sequence)

		self.outputLocal.segInfo:set(1, LocoSegInfo(self.sequence[1])) -- override
		print("override", self.sequence[1].name, 1)
		for i=2,n_seq do
			self.currSegment=self.sequence[i]
			print("append", self.currSegment.name, self.outputLocal.segInfo:size())

			self.outputLocal:append(self.currSegment)
		end      

		local n_seq=LocoGraph.getNumSeg(self.firstFrames)
		assert(n_seq==table.getn(self.sequence))
		assert(self.outputLocal.segInfo:size()==n_seq+1)
		for i=1, n_seq do
			assert(self.outputLocal.segInfo(i).seg==self.sequence[i])
		end



		local lff=self.firstFrames[1]
		self.firstFrame=lff
		-- set refTime though unused
		self.outputGlobal.refTime:setSize(self.firstFrames[n_seq+3]-self.firstFrames[2]+1)
		for i=1, n_seq do
			local ff=self.firstFrames[i+1]-self.firstFrames[2]
			local fl=self.firstFrames[i+2]-self.firstFrames[2]

			self.outputGlobal.refTime:range(ff,fl+1):linspace(i, i+1, fl-ff+1)
			self.outputLocal.segInfo(i).invRefTime=ff
		end

		self.refTimeAll=vectorn(array.back(self.firstFrames)+1-self.firstFrames[1])
		for i=1, table.getn(self.firstFrames)-1 do
			local ff=self.firstFrames[i]-self.firstFrames[1]
			local fl=self.firstFrames[i+1]-self.firstFrames[1]
			self.refTimeAll:range(ff, fl+1):linspace(i, i+1, fl-ff+1)
		end
	end

	-- remove background so that obstacles can be seen.
	local rootnode =RE.ogreRootSceneNode()
	local bgnode=RE.createChildSceneNode(rootnode , "BackgroundNode")
	self.simulator=Physics.DynamicsSimulator_gmbs()
	self.simulator:registerCharacter(self.skel_withoutCOMjoint)


	--self.pdservo=IDservo(self.skel_withoutCOMjoint.dofInfo, model.timestep, Physics.DynamicsSimulator.EULER)
	self.pdservo=QPservo(self.skel_withoutCOMjoint.dofInfo, model.timestep, Physics.DynamicsSimulator.EULER, self.simulator) 
	self.pdservo:initQPservo(nil, nil, nil, nil)   

	self.COPcontroller=COPcontroller(self.skel_withoutCOMjoint)

	local mesh=Mesh()

	mesh:createBox(380, 0.2, 380)
	local tf=matrix4(quater(1,0,0,0), vector3(0,0.02,0))
	mesh:transform(tf)
	self.simulator:createObstacle(mesh)

	self.floor=self.simulator:skeleton(1)
	self.skinFloor=RE.createVRMLskin(self.floor, false)
	self.skinFloor:scale(100,100,100)
	registerContactPairAll(model, self.skel_withoutCOMjoint, self.floor, self.simulator)
	self.simulator:init(0.001, Physics.DynamicsSimulator.EULER)

	self.simulator:setSimulatorParam("penaltyDepthMax", model.penaltyDepthMax)
	debugContactParam={20, 0, 0.01, 0, 0}-- size, tx, ty, tz, tfront
	drawControlForceScale=0
	debugContactForceVis={drawControlForceScale,drawControlForceScale,drawControlForceScale}
	self.simulator:setSimulatorParam("debugContact", debugContactParam)   
	self.simulator:setSimulatorParam("contactForceVis", debugContactForceVis)
	self.simulator:setSimulatorParam("penaltyDepthMax", model.penaltyDepthMax) 
	self.simulator:registerContactQueryBone(0, MainLib.VRMLloader.upcast( self.skel_withoutCOMjoint:getBoneByVoca(MotionLoader.LEFTANKLE)))
	self.simulator:registerContactQueryBone(1, MainLib.VRMLloader.upcast( self.skel_withoutCOMjoint:getBoneByVoca(MotionLoader.RIGHTANKLE)))
end


-- use contact centroid
function CartPoleController:predictCOM_v2(predictedCOM, predictedHead, predictedHip, predictedBase)

	-- calculate COM, base positions and desired COM accelerations to be compensated.
	local graph=self.graph

	-- proceed first two frames seperately. This is to avoid immediate trigger.

	predictedCOM:setSize(self.lastFrame-self.firstFrame+1)--self.graph.mot:numFrames())
	predictedHead:setSize(self.lastFrame-self.firstFrame+1)--self.graph.mot:numFrames())
	predictedHip:setSize(self.lastFrame-self.firstFrame+1)--self.graph.mot:numFrames())
	predictedBase:setSize(self.lastFrame-self.firstFrame+1)

	MotionLoader.setVoca(self.skel_withoutCOMjoint, model.bones)

	local mot=self.graph.motions[1].mot_withoutCOMjoint
	local dmot=self.graph.motions[1].lvel
	local ddmot=self.graph.motions[1].lacc

	self.pdservo.motionDOF=mot
	self.pdservo.dmotionDOF=dmot
	self.pdservo.ddmotionDOF=ddmot
	self.pdservo.motionDOF_pdtarget=mot

	local errorCode=10
	for i=0, predictedCOM:size()-1 do
		self.skel_withoutCOMjoint:setPoseDOF(mot:row(self.firstFrame+i))
		self.skin1:setPoseDOF(mot:row(self.firstFrame+i))
		predictedCOM(i):assign(self.skel_withoutCOMjoint:calcCOM())
		predictedHead(i):assign(self.skel_withoutCOMjoint:getBoneByVoca(MotionLoader.HEAD):getTranslation())
		predictedHip(i):assign(self.skel_withoutCOMjoint:bone(1):getTranslation())
		local state=mot:row(self.firstFrame+i)
		local dstate=dmot:row(self.firstFrame+i)

		self.simulator:setLinkData(0,Physics.DynamicsSimulator.JOINT_VALUE, state)
		self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dstate)
		self.simulator:setGVector(vector3(0,9.8,0)) 
		self.simulator:initSimulation()

		-- find out corresponding interval
		local iinterval=math.min(math.floor(self.refTimeAll(i)), table.getn(self.firstFrames)-1)
		--self.pdservo:calcDesiredAcceleration(self.simulator, self.firstFrame+i, state, dstate) 
		--self.pdservo:_computeHDtorque(self.simulator, state, dstate,nil, self.swingFoot[iinterval]) 
		
		self.pdservo:sampleCurrPose(self.simulator)
		self.pdservo:sampleTargetPoses( self.firstFrame+i)
		self.pdservo:computeContact(self.simulator, self.swingFoot[iinterval])
		self.pdservo:_calcDesiredAcceleration()
		self.pdservo:calcContactCentroid(self.simulator, graph, self.swingFoot[iinterval])
		RE.output2("swingFoot", self.swingFoot[iinterval], iinterval, ":", self.pdservo.contactHull.N)
		if swingFoot=="B" then
			predictedBase(i):assign(vector3(errorCode,errorCode,errorCode))
		else
			local lframe, rframe
			lframe=graph.lfoot:getFrame()
			rframe=graph.lfoot:getFrame()
			local desiredpos=(
			lframe:toGlobalPos(graph.lfootpos)+
			rframe:toGlobalPos(graph.rfootpos))*0.5
			predictedBase(i):assign(desiredpos)
		end

		dbg.namedDraw('Sphere', predictedBase(i), 'base')
		self.simulator:drawDebugInformation()
		renderOneFrame()
	end
	local errors=boolN()
	errors:setSize(predictedCOM:size())
	for i=0, errors:size()-1 do
		errors:set(i, predictedBase(i).y==errorCode)
	end

	local errorInterval=intIntervals()
	errorInterval:runLengthEncode(errors:bit())

	if errorInterval:size()==1 then
		assert(errorInterval:startI(0)~=0 or errorInterval:endI(0)~=predictedCOM:size())
	end

	-- fill gap
	for i=0, errorInterval:size()-1 do
		local f=errorInterval:startI(i)
		local l=errorInterval:endI(i)-1

		local fv, lv
		if f==0 then
			fv=predictedBase(l+1)
		else
			fv=predictedBase(f-1)
		end

		if l==errorInterval:size()-1 then
			lv=predictedBase(f-1)
		else
			lv=predictedBase(l+1)
		end


		for j=f,l do
			predictedBase(j):assign(vector3(
			sop.map(j,f-1, l+1, fv.x, lv.x),
			sop.map(j,f-1, l+1, fv.y, lv.y),
			sop.map(j,f-1, l+1, fv.z, lv.z)))
		end
	end
end
function CartPoleController:predictCOM(predictedCOM, predictedHead, predictedHip, predictedBase)

	-- calculate COM, base positions and desired COM accelerations to be compensated.
	local graph=self.graph

	-- proceed first two frames seperately. This is to avoid immediate trigger.

	predictedCOM:setSize(self.lastFrame-self.firstFrame+1)--self.graph.mot:numFrames())
	predictedHead:setSize(self.lastFrame-self.firstFrame+1)--self.graph.mot:numFrames())
	predictedHip:setSize(self.lastFrame-self.firstFrame+1)--self.graph.mot:numFrames())
	predictedBase:setSize(self.lastFrame-self.firstFrame+1)

	MotionLoader.setVoca(self.skel_withoutCOMjoint, model.bones)

	local mot=self.graph.motions[1].mot_withoutCOMjoint
	local dmot=self.graph.motions[1].lvel
	local ddmot=self.graph.motions[1].lacc

	self.pdservo.motionDOF=mot
	self.pdservo.dmotionDOF=dmot
	self.pdservo.ddmotionDOF=ddmot
	self.pdservo.motionDOF_pdtarget=mot

	local errorCode=10
	for i=0, predictedCOM:size()-1 do
		self.skel_withoutCOMjoint:setPoseDOF(mot:row(self.firstFrame+i))
		self.skin1:setPoseDOF(mot:row(self.firstFrame+i))
		predictedCOM(i):assign(self.skel_withoutCOMjoint:calcCOM())
		predictedHead(i):assign(self.skel_withoutCOMjoint:getBoneByVoca(MotionLoader.HEAD):getTranslation())
		predictedHip(i):assign(self.skel_withoutCOMjoint:bone(1):getTranslation())
		local state=mot:row(self.firstFrame+i)
		local dstate=dmot:row(self.firstFrame+i)

		self.simulator:setLinkData(0,Physics.DynamicsSimulator.JOINT_VALUE, state)
		self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dstate)
		self.simulator:setGVector(vector3(0,9.8,0)) 
		self.simulator:initSimulation()

		-- find out corresponding interval
		local iinterval=math.min(math.floor(self.refTimeAll(i)), table.getn(self.firstFrames)-1)
		self.pdservo:calcDesiredAcceleration(self.simulator, self.firstFrame+i, state, dstate) 
		self.pdservo:_computeHDtorque(self.simulator, state, dstate,nil, self.swingFoot[iinterval]) 
		RE.output2("swingFoot", self.swingFoot[iinterval], iinterval, ":", self.pdservo.contactHull.N)
		if self.pdservo.contactHull.N>=1 then
			predictedBase(i):assign(self.pdservo:calcContactCentroid(self.simulator, self.graph))
		else
			predictedBase(i):assign(vector3(errorCode,errorCode,errorCode))
		end

		dbg.draw('Sphere', predictedBase(i), 'base')
		self.simulator:drawDebugInformation()
		renderOneFrame()
	end
	local errors=boolN()
	errors:setSize(predictedCOM:size())
	for i=0, errors:size()-1 do
		errors:set(i, predictedBase(i).y==errorCode)
	end

	local errorInterval=intIntervals()
	errorInterval:runLengthEncode(errors:bit())

	if errorInterval:size()==1 then
		assert(errorInterval:startI(0)~=0 or errorInterval:endI(0)~=predictedCOM:size())
	end

	-- fill gap
	for i=0, errorInterval:size()-1 do
		local f=errorInterval:startI(i)
		local l=errorInterval:endI(i)-1

		local fv, lv
		if f==0 then
			fv=predictedBase(l+1)
		else
			fv=predictedBase(f-1)
		end

		if l==errorInterval:size()-1 then
			lv=predictedBase(f-1)
		else
			lv=predictedBase(l+1)
		end


		for j=f,l do
			predictedBase(j):assign(vector3(
			sop.map(j,f-1, l+1, fv.x, lv.x),
			sop.map(j,f-1, l+1, fv.y, lv.y),
			sop.map(j,f-1, l+1, fv.z, lv.z)))
		end
	end
end
function CartPoleController:calcDesiredVel(i)
	local mot=self.graph.motions[1].mot_withoutCOMjoint
	local iinterval=math.min(math.floor(self.refTimeAll(math.min(i, self.refTimeAll:size()-1))), table.getn(self.firstFrames)-1)
	local iseg=self:calcIseg(iinterval)
	local rot_y=MotionDOF.rootTransformation(mot:row(math.min(self.firstFrame+i, self.lastFrame))).rotation:rotationY()
	self.pendulum:setOrientation2(rot_y)
	assert(self.sequence[iseg])
	local desiredVel=self.sequence[iseg].pendDesiredVel(0):copy()
	return desiredVel
end
function CartPoleController:calcIseg(iinterval)
	local iseg
	if iinterval==1 then
		iseg=1
	elseif iinterval==table.getn(self.firstFrames)-1 then
		iseg=iinterval-2
	else
		iseg=iinterval-1
	end
	return iseg
end
function CartPoleController:predictDesiredCOMacc(predictedCOM, predictedBase,predictedComAcc, landingPosError)

	local jcacheProxy={}
	--JacobianCache.setLocalFootPos(jcacheProxy)
	--local jtConFoot=JTcontrollerFoot(self.simulator:skeleton(0), jcacheProxy)
	local mot=self.graph.motions[1].mot_withoutCOMjoint
	local dmot=self.graph.motions[1].lvel
	predictedComAcc:setSize(self.lastFrame-self.firstFrame+1)
	landingPosError:setSize(self.lastFrame-self.firstFrame+1)
	landingPosError:setAllValue(vector3(0,0,0))
	local predictedBaseVel=vector3N()
	predictedBaseVel:derivative(predictedBase, self.outputFrameRate)
	local predictedCOMvel=vector3N()
	predictedCOMvel:derivative(predictedCOM, self.outputFrameRate)
	RE.output2("drawTraj", "true")
	dbg.draw('Traj', predictedCOM:matView()*100)
	dbg.draw('Traj', predictedBase:matView()*100)
	--	local desiredCOPcorrection=vector3N(self.lastFrame-self.firstFrame+1)
	-- predict COM acceleration
	for i=0, predictedCOM:size()-1 do
		self.skel_withoutCOMjoint:setPoseDOF(mot:row(self.firstFrame+i))
		self.skin1:setPoseDOF(mot:row(self.firstFrame+i))
		local state=mot:row(self.firstFrame+i)
		local dstate=dmot:row(self.firstFrame+i)
		self.simulator:setLinkData(0,Physics.DynamicsSimulator.JOINT_VALUE, state)
		self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dstate)
		self.simulator:initSimulation()
		--measure errors:zero

		--		local opt=PendPosOptimizer(self, predictedBase, predictedCOM, predictedBaseVel, predictedCOMvel, i)
		--		opt:optimize()
		--
		--		local correctionDesiredCOP=vector3(opt.opt_dimension[1].curval,
		--		0, opt.opt_dimension[2].curval)

		--		self:predictPendulumTrajectoryPos(predictedBase(i)+correctionDesiredCOP)
		local iinterval=math.min(math.floor(self.refTimeAll(i)), table.getn(self.firstFrames)-1)
		local iseg=self:calcIseg(iinterval)
		local usePositionControl=self.usePositionControl[iseg]
		assert(usePositionControl~=nil)
		local desiredVel
		if usePositionControl then
			local com=self.simulator:calculateCOM(0)
			local comVel=self.simulator:calculateCOMvel(0)
			self.pendulum:setState(predictedBase(i), com, vector3(0,0,0), comVel)
		else
			self.pendulum:setState(predictedBase(i), predictedCOM(i), predictedBaseVel(i), predictedCOMvel(i)) 
			desiredVel=self:calcDesiredVel(i)
		end

		util.appendFile("pendStateN.txt", util.tostring(self.firstFrame+i,self.pendulum.theta, self.pendulum.dtheta).."\n")
		--util.appendFile("pendStateN.txt", util.tostring(predictedCOM(i), self.pendulum:calcCOMpos(), self.pendulum:calcCartPos()).."\n")
		util.appendFile("pendStateN.txt", util.tostring(usePositionControl, predictedBase(i), desiredVel).."\n")

		local nextFootStepTime=math.floor((self.firstFrames[iinterval]+self.firstFrames[iinterval+1])/2-(i+self.firstFrames[1]))
		nextFootStepTime=math.max(0, nextFootStepTime)
		
		assert(usePositionControl~=nil)
		if usePositionControl then
			RE.output2("predictPendTraj", i+self.firstFrames[1], iseg, iinterval, usePositionControl, predictedBase(i), nextFootStepTime)
			self:predictPendulumTrajectory(true, predictedBase(i), math.max(nextFootStepTime, COPlatency))
		else
			RE.output2("predictPendTraj", i+self.firstFrames[1], iseg, iinterval, usePositionControl, desiredVel, nextFootStepTime)
			self:predictPendulumTrajectory(false, i, math.max(nextFootStepTime, COPlatency))
		end

		local desiredCOP=self.pendulum:getPos(COPlatency)
		local nextFootStep=self.pendulum:getPos(nextFootStepTime)
		assert(desiredCOP.x==desiredCOP.x)
		assert(nextFootStep.x==nextFootStep.x)
		dbg.namedDraw("Sphere", nextFootStep*100, "nextFootStep")
		dbg.namedDraw("Sphere", predictedCOM(i)*100, "predictedCOM")
		dbg.namedDraw("Sphere", predictedBase(i)*100, "predictedBase")

		local pend=self.pendulum
		--self.objectList:registerObject("zmpPredict", "LineList2D_ZX", "solidred", pend.Y:range(0, pend.Y:rows(), 0, pend.Y:cols())*100, 0)
		self.simulator:setGVector(vector3(0,9.8,0)) 
		self.simulator:initSimulation()

		if self.COPcontroller_dampingCoef then
			useCase._COPcontroller_dampingCoef =self.COPcontroller_dampingCoef[iseg]
		else
			useCase._COPcontroller_dampingCoef= useCase.COPcontroller_dampingCoef
		end
		if false then -- if you change this, change spprtFootCalculator.lua too
			predictedComAcc(i):assign(
			self.COPcontroller:calcDesiredComAcc(self.simulator, desiredCOP))
		else
			local currentCOP=self.pendulum:getPos(0)
			local com=self.simulator:calculateCOM(0)
			--local ref_yg=predictedCOM(i).y -- pendulum COM height
			local ref_yg=com.y
			--ref_yg=nil
			local acc1=self.COPcontroller:calcDesiredComAcc(self.simulator,desiredCOP, ref_yg) 
			local acc2=self.COPcontroller:calcDesiredComAcc(self.simulator,currentCOP, ref_yg) 
			predictedComAcc(i):assign(acc1-acc2)
			predictedComAcc(i).y=acc1.y
		end
		 
		--[[
		if useCase.calcDesiredFootPosFlightPhase then
			local isJumping, JumpElapsed=useCase.isFlightPhase(self.firstFrame+i,self.graph)

			if isJumping then
				local locosyn={ 
					simulator=self.simulator,     -- for calculating com, comVel
					graph=self.graph,       -- for retrieving segments
					pdservo={
						state={flightPhaseElapsed=JumpElapsed}
					}
				}
				local currCOM=self.simulator:calculateCOM(0)
				local foot, desiredfootPos, desiredfootVel, flightTime=useCase.calcDesiredFootPosFlightPhase(locosyn)
				local currPos=jtConFoot:calcFootPos(self.simulator:getWorldState(0), 'B')-currCOM
				landingPosError(i):assign(currPos-desiredfootPos)
				dbg.namedDraw("Sphere", (desiredfootPos+currCOM)*100, "desiredFootPos")
				dbg.namedDraw("Sphere", (currPos+currCOM)*100, "currFootPos")
			end
		end
		]]--
		--		desiredCOPcorrection(i):assign(correctionDesiredCOP)
		renderOneFrame()
	end

	RE.output2("drawTraj", "false")
--	self.objectList:registerObject("pelvis", "LineList", "solidgreen", predictedCOM:matView()*100, 0)
	--	self.desiredCOPcorrection=desiredCOPcorrection
end


function OnlineSynthesis2:predictPendulumTrajectory(usePosControl, desiredParam, amount)

	assert(usePosControl~=nil)
	-- unused
	local pend=self.pendulum
	local theta=vectorn()
	local dtheta=vectorn()
	local numFrames=pend:numFrames()

	pend:_saveStates(theta, dtheta)

	local frameRate=self.outputFrameRate
	assert(numFrames==0)

	do
		
		local iframe
		if usePosControl then
			self.pendulum:setDesiredPosition(desiredParam)
		else
			iframe=desiredParam	
		end
		local i=0
		pend:oneStep()
		while i<amount+1 do
			if not usePosControl then
				self.pendulum:setDesiredVelocity(self:calcDesiredVel(iframe+i))
			end
			pend:oneStep()
			--pend:fourSteps()
			--pend:draw() renderOneFrame()
			if pend.errorOccurred==true then
				assert(false)
			end
			--i=i+4
			i=i+1
		end
	end

	pend:_restoreStates(theta, dtheta, numFrames)
end

screenshot=false
captureUniqueId=0


PendPosOptimizer=LUAclass (Optimizer)
-- unused now 

function PendPosOptimizer:__init(cpno, base, com, base_vel, com_vel, curr_frame)
	local stepSize=5
	--	local method=Optimizer.methods.CMAes
	local method=Optimizer.methods.ConjugateGradient
	--local method=Optimizer.methods.NRconjugateGradient
	method.tol=0.0001
	--local method=Optimizer.methods.GSLBFGS
	local opt_dimension={
		{title="x", curval=0, max_step=0.05, grad_step=0.015},
		{title="z", curval=0, max_step=0.05, grad_step=0.015},
	}
	Optimizer.__init(self, stepSize, opt_dimension, method) 
	self.cpno=cpno
	self.cpno=cpno
	self.base=base
	self.com=com
	self.base_vel=base_vel
	self.com_vel=com_vel
	self.curr_frame=curr_frame
end

function PendPosOptimizer:objectiveFunction(pos)
	local pend=self.cpno.pendulum
	local i=self.curr_frame
	pend:setState(self.base(i), self.com(i), self.base_vel(i), self.com_vel(i))

	self.cpno:predictPendulumTrajectoryPos(self.base(i)+ vector3(pos(0), 0, pos(1)))

	local dist=0
	for j=i, math.min(self.base:size()-1, i+COPlatency) do
		local theta=pend.Y:row(j-i)
		local pcom=pend:__getCOMpos( theta)
		dist=dist+pcom:distance(self.com(j))
		dist=dist+pend:getPos(j-i):distance(self.base(j))
	end
	RE.output2("pos", pos)
--	self.objectList:registerObject("zmpPredict", "LineList2D_ZX", "solidred", pend.Y:range(0, pend.Y:rows(), 0, pend.Y:cols())*100, 0)
	renderOneFrame()
	return dist
end

CartPoleNoOptimizer=LUAclass()

function CartPoleNoOptimizer:__finalize()
	self.synthesis:__finalize()
end
function CartPoleNoOptimizer:__init(segmentationName)
	print('0')
	local dofFN=useCase.mot_file or model.mot_file

	local graphFN=useCase.cartPoleFN or str.left(dofFN,-3).."zmpcom"

	self.backupCartPole=nil
	-- TODO: need to backup zmpcom.
	if util.isFileExist(graphFN) then
		self.backupCartPole=util.loadTable(graphFN)
	end

	self.synthesis = CartPoleController:new(segmentationName)

	do 
		print('a')
		local predictedCOM=vector3N()
		local predictedBase=vector3N()
		if postProcess==true then
			predictedCOM:assign(self.backupCartPole.com:range(self.synthesis.firstFrame, self.synthesis.lastFrame+1))
			predictedBase:assign(self.backupCartPole.zmp:range(self.synthesis.firstFrame, self.synthesis.lastFrame+1))
		else
			local predictedHead=vector3N()
			local predictedHip=vector3N()
			--self.synthesis:predictCOM(predictedCOM, predictedHead, predictedHip, predictedBase)
			self.synthesis:predictCOM_v2(predictedCOM, predictedHead, predictedHip, predictedBase)
		end

		local predictedComAcc=vector3N()
		local landingPosError=vector3N()
		
		print('b')
		self.synthesis:predictDesiredCOMacc(predictedCOM, predictedBase, predictedComAcc,landingPosError)
		self.synthesis.predictedCOM=predictedCOM
		self.synthesis.predictedBase=predictedBase
		self.synthesis.predictedComAcc=predictedComAcc
		if useCase.calcDesiredFootPosFlightPhase then
			self.synthesis.landingPosError=landingPosError
		end
	end

	if debug_pendulum then debug_pendulum=nil end
	--  export ZMP file.
	do

		print('c')
		local initFront=false
		if self.backupCartPole==nil then
			self.backupCartPole={}
			self.backupCartPole.zmp=vector3N(self.synthesis.graph.motions[1].mot:numFrames())
			self.backupCartPole.com=vector3N(self.synthesis.graph.motions[1].mot:numFrames())
			self.backupCartPole.plannedComacc=vector3N(self.synthesis.graph.motions[1].mot:numFrames())
			--			self.backupCartPole.desiredCOPcorrection=vector3N(self.synthesis.graph.motions[1].mot:numFrames())
			initFront=true
		end

		if useCase.calcDesiredFootPosFlightPhase then
			self.backupCartPole.landingPosError=vector3N(self.synthesis.graph.motions[1].mot:numFrames())
			self.backupCartPole.landingPosError:setAllValue(vector3(0,0,0))
		end
		local lff=self.synthesis.firstFrame
		local numFrames=self.synthesis.lastFrame-lff+1
		local com=self.backupCartPole.com
		local zmp=self.backupCartPole.zmp
		local plannedComacc=self.backupCartPole.plannedComacc
		--		local desiredCOPcorrection=self.backupCartPole.desiredCOPcorrection
		local landingPosError=self.backupCartPole.landingPosError
		local pend=self.synthesis.pendulum

		plannedComacc:resize(com:rows())

		local zz
		for i=0, numFrames-1 do
			local cc=self.synthesis.predictedCOM(i)
			com(i+lff):assign(cc)

			-- use predicted base zmp(i+lff):assign(zz)
			-- use COM position
			if useContactCentroidAsBase then
				zmp(i+lff):assign(self.synthesis.predictedBase(i))
			else
				zz=cc
				zz.y=0
				zmp(i+lff):assign(zz)
			end

			plannedComacc(i+lff):assign(self.synthesis.predictedComAcc(i))
			--			desiredCOPcorrection(i+lff):assign(self.synthesis.desiredCOPcorrection(i))
			if useCase.calcDesiredFootPosFlightPhase then
				landingPosError(i+lff):assign(self.synthesis.landingPosError(i))
			end
		end

		--self.synthesis.objectList:registerObject("com", "LineList", "solidblue", com:matView():range( lff, com:size(), 0, 3)*100,0)

		-- control force to the cart represented in global coordinate.
		-- This will later be converted into local coordinate in 
		-- ZMPgraphSegment:calcLocalPelvisAndFoot

		print('d')

		if initFront then
			for i=0, lff -1 do
				com(i):assign(com(lff))
				zmp(i):assign(zmp(lff))
				--				desiredCOPcorrection(i):zero()
				plannedComacc(i):zero()
			end
		end
		--		util.saveTable(graphFN, {zmp=zmp, com=com, plannedComacc=plannedComacc, desiredCOPcorrection=desiredCOPcorrection})
		util.saveTable( {zmp=zmp, com=com, plannedComacc=plannedComacc, landingPosError=landingPosError}, graphFN)
	end
end

RE.viewpoint():setFOVy(45.000002)
RE.viewpoint().vpos:set(-31.603369, 177.652362, 622.504089)
RE.viewpoint().vat:set(-39.439721, 177.466525, -14.588023)
RE.viewpoint():update()

function ctor()

	--require('RigidBodyWin/subRoutines/fastMode') -- not much faster
	util.writeFile("pendStateN.txt", " ")
	if useCase.pendControlParam_all then
		print('all')
		useCase.pendControlParam=useCase.pendControlParam_all
	end
	if useCase.segmentations then

		for segmentationName, segmentation in pairs(useCase.segmentations) do

			print('f')
			mSynthesis=CartPoleNoOptimizer(segmentationName)
			mSynthesis:__finalize()
			mSynthesis=nil
			collectgarbage() 
		end
	else
		mSynthesis=CartPoleNoOptimizer("segmentation")
	end

	this("exit",0)

end

function dtor()
	dbg.finalize()
end

function onCallback(w, userData)
end

function renderOneFrame()
	noFrameMove=true
	RE.renderOneFrame(true)	
	noFrameMove=nil
end

function frameMove(fElapsedTime)
	--	debug.debug()

	if mSynthesis~=nil then
		if state==states.stop then
			this("exit",0)
		end
	end
end

