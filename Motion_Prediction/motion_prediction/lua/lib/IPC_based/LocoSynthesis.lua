require("IPC_based/LocoGraph")
require("IPC_based/runMapping")
require("IPC_based/cartPoleBall")
OnlineSynthesis2=LUAclass()
function OnlineSynthesis2.convertConPos(isL, projPos, lconpos, conOri)
	local toepos, heelpos
	local method=useCase.useFootEncodingMethod2
	local lposinfo=method.geom[isL]
	heelpos=lposinfo[1][3]:getFrame():toGlobalPos(lposinfo[1][2])
	toepos=lposinfo[2][3]:getFrame():toGlobalPos(lposinfo[2][2])
	local origFrame=lposinfo[1][3]:getFrame():copy()
	local lheelpos=origFrame:toLocalPos(heelpos)
	local ltoepos=origFrame:toLocalPos(toepos)
	origFrame.rotation:assign(conOri)
	local heelpos=origFrame:toGlobalPos(lheelpos)
	local toepos=origFrame:toGlobalPos(ltoepos)
	local conpos=origFrame:toGlobalPos(lconpos)

	local currProjPos=LocoGraph.projectFoot(heelpos,toepos)
	local delta=vector3()
	delta:difference(currProjPos, projPos)
	return conpos+delta
end

--      |        -----------+
--      |                   |
--      +-------------------+
--     						lconpos
--     			projpos
-- self.skel_withoutCOMjoint:setPoseDOF(self.theta) is necessary before calling unconvert
function OnlineSynthesis2.unconvertConPos(isL, lconpos)
	local toepos, heelpos
	local method=useCase.useFootEncodingMethod2
	local lposinfo=method.geom[isL]
	heelpos=lposinfo[1][3]:getFrame():toGlobalPos(lposinfo[1][2])
	toepos=lposinfo[2][3]:getFrame():toGlobalPos(lposinfo[2][2])
	local projPos=LocoGraph.projectFoot(heelpos,toepos)
	local tf=transf()
	tf.translation:assign(projPos)
	tf.rotation:assign(lposinfo[1][1]:getFrame().rotation)
	tf.rotation:identity()
	return tf
end
function OnlineSynthesis2:projectToTerrain(x, ori)
	if useCase.slope and useCase.slope~=0 then
		local r=Physics.RayTestResult ()
		local cd=self.simulator:getCollisionDetector()
		local s=x+vector3(0,100,0)
		local t=x-vector3(0,100,0)
		cd:rayTest(1, 1,s ,t , r)
		if r:hasHit() then
			local o=vector3()
			o:interpolate(r.m_closestHitFraction, s,t)

			--[[
			if useCase.slope<0 then
				local o2=vector3()
				cd:rayTest(2, 1,s ,t , r)
				o2:interpolate(r.m_closestHitFraction, s,t)

				o.y=math.max(o.y, o2.y)
			end
			]]--
			x:assign(o)
			x.y=x.y-0.0056
		end
	end
end
function OnlineSynthesis2:adjustStrideRatio(currSeg, val)
	local ts=1
	local name=self.outputLocal.segInfo(currSeg).seg.name
	if val>0 then
		if name=="L" then
			ts=(1-val)
		end
	else
		if name=="R" then
			ts=(1+val)
		end
	end
	return ts
end
function OnlineSynthesis2:initializeControlParameters()


	if useCase.pendControlParam~=nil then
		-- prin("a")
		-- debug.debug()
		self:changeControlParameters(useCase.pendControlParam,true)

		-- out(self.graph.StR.LR.transitions["RL"].footLcorrection[1])
		-- out(self.graph.StR.LR.transitions["RL"].footLcorrection[2])
		-- debug.debug()
	end

	if useCase.controlParam~=nil then
		-- prin("a")
		-- debug.debug()
		self:changeControlParameters(useCase.controlParam)

		--out(self.graph.StR.LR.swingFootForce)
		--dbg.console()
		-- out(self.graph.StR.LR.transitions["RL"].footLcorrection[2])
		-- debug.debug()
	end
end

function createIKsolver_swingFoot(self)
	local ik={}
	ik.effectors=MotionUtil.Effectors()
	ik.effectors:resize(2)

	local lfoot=self.lfoot
	local rfoot=self.rfoot
	local lknee=self.lknee
	local rknee=self.rknee
	ik.effectors(0):init(lfoot,self.lfootpos)
	ik.effectors(1):init(rfoot,self.rfootpos)

	--   ik.solver=MotionUtil.createFullbodyIkDOF_limbIK(skel.dofInfo, ik.effectors, lknee, rknee)
	ik.solver=DIPC_IKsolver(self.skel,ik.effectors, lknee, rknee)
	ik.conPos=vector3N(2)
	ik.conDelta=quaterN(2)

	--if iterativeIK~=nil then  
	ik.fk=BoneForwardKinematics(self.skel)
	ik.fk:init()
	ik.compareChain=CompareChain(self.skel, ik.fk, self.skel:fkSolver())
	ik.skel=self.skel
	--end

	-- rotY is used for defining foot global orientations.
	function ik:solve(rotY, roottf, lfoot, rfoot, pose)
		assert(pose~=nil)
		self.conPos(0):assign(lfoot)
		self.conPos(1):assign(rfoot)
		self.conDelta(0):assign(quater(1,0,0,0))
		self.conDelta(1):assign(quater(1,0,0,0))
		self.solver:IKsolve(pose, rotY, roottf, self.conDelta, self.conPos, 0, vector3(0,0,0))
	end

	return ik
end

function OnlineSynthesis2:calcInitialState(useCapturedPosition)
	local initialState=vectorn()
	--   initialState:assign(self.currSegment:mot():row(0)) 

	-- local initialPose=self.currSegment:mot():row(PDservoLatency/outputSuperSample)
	-- local initialVel=self.dmotionDOF:row(self.currSegment.first+PDservoLatency/outputSuperSample)
	local initialPose=self.currSegment:mot():row(0)
	initialState:assign(initialPose)

	local modified

	do -- move to (0,0,0) and rotate to face z. pendulum initial state have to moved to.

		local initialPosePelvis=MotionDOF.transformation(initialPose,0)
		if not useCapturedPosition then
			initialState:setQuater(3, initialPosePelvis.rotation:offsetQ()) -- make face forward
		end

		local roottf= MotionDOF.transformation(initialState,0)
		roottf.rotation:leftMult( quater(math.rad(0),vector3(0,1,0)))
		MotionDOF.setTransformation(initialState, 0, roottf) 
		--
		if not useCapturedPosition then
			-- move to (0,0,0)
			initialState:set(0,0) 
			initialState:set(2,0) 
		end
		initialState:set(1,initialPose:get(1)) 

		modified=MotionDOF.transformation(initialState,0)*initialPosePelvis:inverse()
		modified.translation.y=0
		if saveDebugInfo then
			util.saveTable( {initialPose, self.currSegment:mot(), initialPosePelvis:toTable(), initialPosePelvis:inverse():toTable(), model.initialHeight, initialState, initialVel},'debugInfo_nocomp2.tbl')
		end
	end



	local initialVel=self.currSegment:dmot():row(0):copy()

--##dos if g_debugOneStep then --##dos g_debugOneStep:pushBack({"pendInit0",initialPose:copy(), initialState:copy(), initialVel:copy(), modified.rotation:copy(), modified.translation:copy()}) --##dos end
	return initialState, initialVel,modified
end

function OnlineSynthesis2:createPendulum()
	return LocoGraph.createPendulum(self.outputFrameRate)
end
function OnlineSynthesis2:initPendState(f, useCapturedPosition, useCapturedPositionForPendToo)
	--local zf=self.currSegment.locomot.ZMPcalculatorFullbody
	local zf=self.currSegment.locomot.ZMPcalculator
	if useCapturedPositionForPendToo then
		zf=self.currSegment.locomot.ZMPcalculatorFullbody
	end
	if useCase.enableStartFrameBug==1 then
		assert(false)
		zf=self.currSegment.locomot.ZMPcalculatorFullbody
	end
	local com=zf.com(f)
	local zmp=zf.zmp(f)
	local comvel=zf.comvel(f)
	local zmpvel=vector3(0,0,0)
	if zf.zmpvel then
		zmpvel=zf.zmpvel(f)
	end
	--print('initPendState', f, useCapturedPosition, useCapturedPositionForPendToo, comvel, zmpvel)
	--dbg.console()
	-- dbg.console(util.tostring(com,zmp, comvel))
	local initialState, initialVel, tf=self:calcInitialState(useCapturedPosition)
	
	if not useCase.noTimeVaryingInertia then
		local linertia=self.currSegment.linertia:sampleRow(0)
		self.pendulum.pole:setLocalCOM(vector3(0, linertia.y,0))
		self.pendulum:setInertia(linertia)
	end
	-- hoihoi
	if useCase.speedMod then
		self.pendulum:setState(tf*zmp, tf*com, tf.rotation*zmpvel+useCase.speedMod, tf.rotation*comvel+useCase.speedMod)   
	else
		self.pendulum:setState(tf*zmp, tf*com, tf.rotation*zmpvel, tf.rotation*comvel)   
	end
	self.pendulum:setOrientation2(MotionDOF.rootTransformation(initialState).rotation:rotationY())
	self.pendulum:setDesiredVelocity(vector3(0,0,0))

	if false then
		print('currFrame', self.currSegment.first)
		local pendvel=self.pendulum:calcCOMvel()
		local cartvel=self.pendulum:calcCartVel()
		print('cart', tf*zmp, self.pendulum:calcCartPos())
		print('com', tf*com, self.pendulum:calcCOMpos())
		print('cartvel',tf.rotation*zmpvel,cartvel)
		print('comvel', tf.rotation*comvel,pendvel)
		dbg.console()
	end
	if false then
		dbg.namedDraw('Sphere', (tf*com)*100, 'initialCOM')
		dbg.namedDraw('Sphere', MotionDOF.rootTransformation(initialState).translation*100, 'initialPelvis')
	end
	--##dos if g_debugOneStep then --##dos g_debugOneStep:pushBack({"pendInit",com:copy(), zmp:copy(), comvel:copy(), zmpvel:copy(), tf.rotation:copy(), tf.translation:copy()}) --##dos g_debugOneStep:pushBack({"pendState",self.pendulum:getStates()}) end
end
function OnlineSynthesis2:__init()

	scenario=useCase.scenario
	-- init globals
	model=scenarios.toModel(scenario)

	--dbg.startDebug()
	-- iterativeIK={ drawDebugPose=false, nIter=5 }

	model.motionFrameRate=model.frame_rate
	model.zmpSamplingRate=120
	renderSuperSample=false
	predictPendulumDownSample=true

	predictedControlForceWeight=0
	useShearedTransform=true

	init_globals()

	--   RE.renderer():fixedTimeStep(false)
	self.skel_withoutCOMjoint=MainLib.VRMLloader(useCase.model_file_name or model.file_name)
	self.skel_withoutCOMjoint.dofInfo:setFrameRate(model.frame_rate)
	MotionLoader.setVoca(self.skel_withoutCOMjoint, model.bones)

	model.mot_file= useCase.mot_file or model.mot_file -- override
	self.graph=LocoGraph:new(self.skel_withoutCOMjoint, model.mot_file)
	self.skel=self.graph.skel

	assert(self.graph.motions[1].mot.dofInfo:frameRate()==model.frame_rate)
	assert(self.graph.motions[1].lmot.dofInfo:frameRate()==model.frame_rate)

	self.desiredSpeedX=0			
	self.desiredSpeedZ=0


	self.desiredTurningSpeed=0

	self.trackingErrorRot=quater()
	self.trackingErrorRot:identity()
	self.pendControlForce=vector3(0,0,0)
	local skel=self.skel



	self.outputFrameRate=model.frame_rate*outputSuperSample
	self.pendulum=self:createPendulum()
	self.pendulum.skin:setVisible(false)
	local mass=skel:calcTotalMass()
	self.verticalPend=SDS(mass,0.01,1,1000000,0.01,1/self.outputFrameRate)
	self.graph.pendLocalCOM=self.pendulum.pole:localCOM()
	self.graph.pendulum=self.pendulum
	self.graph:createGraph(graphFile )
	--
	assert(skel)
	--   self.LQRlookahead=5*outputSuperSample
	self.skin2=RE.createVRMLskin(self.skel_withoutCOMjoint, false)
	self.skin2:scale(100,100,100)

	if useCase.hookAfterGraphConstruction then
		useCase.hookAfterGraphConstruction(self)
	end

	--   RE.motionPanel():scrollPanel():addPanel(self.graph.leftFoot:bit(), CPixelRGB8(255,255,0))
	--   RE.motionPanel():scrollPanel():addPanel(self.graph.rightFoot:bit(), CPixelRGB8(255,255,0))

	local useHand=useCase.keyframes.importanceLH ~=nil
	if useCase.useAnalyticIK then
		self.ik=self.graph:createIKsolver(useHand, self.skel_withoutCOMjoint)
	else
		self.ik=self.graph:createIKsolver2()
	end
	--self.ik_COM=self.graph:createIKsolver_COM(useHand, self.skel_withoutCOMjoint)

	assert(skel.dofInfo:frameRate()==model.frame_rate)

	-- graph, spread, lookahead
	self.outputLocal=LocoGraphOutputLocal:new(self.graph)
	self.outputGlobal=LocoGraphOutputGlobal:new(self.outputLocal, outputSuperSample)

	self.footStates={LfootStride=1, RfootStride=1, LHfootStride=1, RHfootStride=1}

	--   self.outputFrameRate=skel.dofInfo:frameRate()*outputSuperSample

	self.objectList=Ogre.ObjectList ()
	g_objectList=self.objectList

	self.numFrames=0 -- with respect to outputFrameRate (=mocap frame rate * outputSuperSample)
	self.currSegment=self.graph.initialSegment
	self.prevFrac=1

	-- warning! code duplication: see trajGenerator.lua and movingWindowOptimizerSubroutines4.lua
	--self:produceSegment() -- the first segment becomes initialSegment.next
	self.outputLocal:append(self.currSegment) -- the first segment becomes initialSegment
	self.outputLocal.segInfo(1).invRefTime=0
	self.samplePose=vectorn()
	self.samplePose2=vectorn()
	self.samplePoseBeforeIK=vectorn()
--	self.samplePose_pdtarget=vectorn()

	self.pelvisError=quater()
	self.pelvisError:identity()
	self.prevSegment=self.graph.initialSegment
	self.prevLocalFirst=0

	-- pendulum init state:
	do
		local f=self.currSegment.first	
		self:initPendState(f)
	end

	self:initializeControlParameters()

--	self.desiredCOP=useCase.desiredPos:copy()

	--self.graph:logKeyframes()
end

function OnlineSynthesis2:__finalize()
	self.skel=nil
	self.prevSegment=nil

	if self.graph then
		self.graph:__finalize()
		self.graph=nil
	end
end

function OnlineSynthesis2:calcRotY(frame)
	local synRoot=self.outputGlobal.synRoot

	if frame>= synRoot:rows() then
		print("warning.. calcRotY")
		frame=synRoot:rows()-1
	end

	return self.graph.id_rotY.rotY(synRoot:row(frame))
end

function OnlineSynthesis2:calcRotY2(frame)
	local synRoot=self.outputGlobal.synRoot

	if frame>= synRoot:rows() then
		print("warning.. calcRotY")
		frame=synRoot:rows()-1
	end

	return self.graph.id_rotY.rotY2(synRoot:row(frame))
end

function OnlineSynthesis2:sampleRotY(frame)
	local f1=math.floor(frame)
	local f2=math.ceil(frame)
	if f1==f2 then
		return self:calcRotY(f1)
	end

	local q1=self:calcRotY(f1)
	local q2=self:calcRotY(f2)
	local q3=quater()
	q3:safeSlerp(q1, q2, sop.map(frame, f1, f2, 0, 1))
	return q3
end

function OnlineSynthesis2:outputMotion() -- kinematic controller
	return self.outputMotion
end
function OnlineSynthesis2:sampleSmoothRotY(ltime)

	local f1=self.outputGlobal:invRefTime(ltime)
	local f2=self.outputGlobal:invRefTime(math.max(ltime-1,1))

	local q1=self:sampleRotY(f1)
	local q2=self:sampleRotY(f2)

	local q3=quater()
	q3:safeSlerp(q1, q2, 0.5)
	return q3
end

function OnlineSynthesis2:getStates()
	-- pdservo..? self.numSimulFrames..? self.outputMotion, error_feedback_method,..
	-- store 2 previous segments.
	-- have to be called just before calling oneStep()
	local storeInfo={}
	local outputL=self.outputLocal
	local outputG=self.outputGlobal
	assert(self.numFrames==self.outputGlobal.numFrames)
	local currFrame=self.numFrames-1

	local lcurrFrame

	if currFrame==-1 then
		currFrame=0
		lcurrFrame=0
	else
		lcurrFrame=outputG.refTime(currFrame)
	end

	local settings={ nhistory=2}

	-- outputLocal: store prev two segments + current segment
	local curSeg=math.floor(lcurrFrame)
	--   local startSeg=math.max(0, curSeg-settings.nhistory) -- buggy
	local startSeg=0

	if self.deltaSeq==nil then
		self.deltaSeq=0
	end

--	storeInfo.desiredCOP=self.desiredCOP
	storeInfo.useCaseGrpName=useCase.grpName
	storeInfo.useCase_desiredPos=useCase.desiredPos
	storeInfo.deltaSeq=self.deltaSeq+startSeg
	storeInfo.prevDeltaAngle=  self.prevDeltaAngle
	-- startSeg becomes seg 0

	local startL=startSeg
	-- outputGlobal: corresponding
	local refTime=vectorn()
	local synRoot=matrixn()
	storeInfo.pendTrajectory=matrixn()
	storeInfo.outputMotion=matrixn()
	storeInfo.prevAmt=self.prevAmt

	local startG=0
	if currFrame~=0 then 

		startG=math.ceil(outputL.segInfo(startSeg).invRefTime)

		refTime:assign(outputG.refTime:range(startG, outputG.refTime:size()))
		refTime:rsub(startL)

		synRoot:assign(outputG.synRoot:range(startG, outputG.synRoot:rows(), 0, outputG.synRoot:cols()))

		local Y=self.pendulum.Y
		storeInfo.outputMotion=self.outputMotion:matView():range(startG, self.outputMotion:rows(), 0, self.outputMotion:cols())
		storeInfo.pendTrajectory:assign(Y:range(startG, Y:rows(), 0, Y:cols()))

		local dmot=self.pdservo.dmotionDOF
		storeInfo.dmotionDOF=dmot:range(startG, dmot:rows(), 0, dmot:cols())
		local ddmot=self.pdservo.ddmotionDOF
		storeInfo.ddmotionDOF=ddmot:range(startG, ddmot:rows(), 0, ddmot:cols())
--		local mot_pd=self.pdservo.motionDOF_pdtarget
--		storeInfo.motionDOF_pdtarget=mot_pd:matView():range(startG, mot_pd:rows(), 0, mot_pd:cols())
	end
	storeInfo.scale_lleg=self.pdservo.scale_lleg
	storeInfo.scale_rleg=self.pdservo.scale_rleg
	storeInfo.scale_upperbody=self.pdservo.scale_upperbody
	storeInfo.pdservo_state=self.pdservo.state


	storeInfo.refTime=refTime
	storeInfo.synRoot=synRoot
	storeInfo.seq={}

	for i=startSeg, outputL.segInfo:size()-1 do
		local segInfo=outputL.segInfo(i)
		storeInfo.seq[i-startSeg]=
		{name=segInfo.seg.name, grpName=segInfo.seg.grpName,
		len=segInfo.len, nextLen=segInfo.nextLen, timeScale=segInfo.timeScale, 
		invRefTime=segInfo.invRefTime-startG}
	end

	storeInfo.currSegment={self.currSegment.name, self.currSegment.grpName}


	-- self
	storeInfo.prevLocalFirst=self.prevLocalFirst

	storeInfo.desiredSpeedX=  self.desiredSpeedX
	storeInfo.desiredSpeedZ=  self.desiredSpeedZ
	storeInfo.desiredTurningSpeed= self.desiredTurningSpeed

	if self.trackingInfo then
		storeInfo.trackingInfo={}

		storeInfo.trackingInfo.currSeg={grpName=self.trackingInfo.currSeg.grpName, name=self.trackingInfo.currSeg.name}
		storeInfo.trackingInfo.frac=self.trackingInfo.frac
	end

	storeInfo.prevFrac=self.prevFrac
	storeInfo.desiredFootPosFlightPhase  =self.desiredFootPosFlightPhase   
	storeInfo.prevDomain_seg=self.prevDomain_seg
	storeInfo.prevDomain_edge=self.prevDomain_edge
	-- states
	storeInfo.states=vectorn()
	storeInfo.dstates=vectorn()
	self.simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, storeInfo.states)
	self.simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, storeInfo.dstates)
	if simulator==simulators.gmbs then 
		storeInfo.gmbs_root_state=matrixn()
		self.simulator:test('getExactStateSpherical', storeInfo.gmbs_root_state)
	end

	storeInfo.collMargin=vectorn()
	self.simulator:getAllCollisionMargin(storeInfo.collMargin)


	storeInfo.theta=self.theta
	storeInfo.dtheta=self.dtheta
	storeInfo.contactState=self.contactState
	storeInfo.prevContactState=self.prevContactState
	storeInfo.pendControlforce=self.pendControlforce
	storeInfo.error_feedback_method=error_feedback_method
	assert(not self.errorOccurred)
	assert(storeInfo.error_feedback_method~=nil)
	-- self.outputMotion, self.pdservo.dmotionDOF
	storeInfo.timeScaleOverride=self.timeScaleOverride
	storeInfo.timescale=self.timescale
	storeInfo.targetHeight=self.targetHeight

	storeInfo.pendState=self.pendulum:getStates()
	storeInfo.pendState.numFrames=self.pendulum:numFrames()-startG
	storeInfo.vpendState=self.verticalPend:getStates()

	storeInfo.numFrames=self.numFrames-startG
	storeInfo.startG=startG
	--assert(storeInfo.pendState.numFrames==storeInfo.numFrames)

	-- prevZMP, prevCOM, ...
	if self._prevZMP==nil then
		storeInfo.prevZMP=nil
		storeInfo.prevCOM=nil
		storeInfo.prevCOMvel=nil
		storeInfo.prevZMPvel	=nil
	else
		storeInfo.prevZMP=self._prevZMP:copy()
		storeInfo.prevCOM=self._prevCOM:copy()
		storeInfo.prevCOMvel=self._prevCOMvel:copy()
		storeInfo.prevZMPvel=self._prevZMPvel:copy()
	end
	storeInfo.startSearch=outputG.startSearch

	storeInfo.trackingErrorRot=self.trackingErrorRot
	-- tracker
--	storeInfo.comTracker=self.comTracker:getAllState()
--	storeInfo.zmpTracker=self.zmpTracker:getAllState()
	storeInfo.pelvisError= self.pelvisError

	-- globals
	storeInfo.regResult=regResult
	storeInfo.controlParam=useCase.controlParam
	--storeInfo.asymmetricFoot=useCase.asymmetricFoot
	-- regResult modifies control parameters (footLcorrection and footRcorrection). So they should also be saved.

	storeInfo.segProperties=array:new()
	--  storeInfo.edgeProperties=array:new()
	for i,grpName in ipairs(self.graph.groups.tf) do
		for k,v in pairs(self.graph[grpName]) do

			local custumParam={}
			if useCase.keyframes then
				for kk, vv in pairs(useCase.keyframes) do
					custumParam[kk]=v[kk]
				end
			end
			if useCase.segProperties then
				for kk, vv in pairs(useCase.segProperties) do
					custumParam[kk]=v[kk]
				end
			end
			assert(v.name~=nil)
			storeInfo.segProperties:pushBack({grpName=grpName, name=v.name, 
			keyframes=custumParam})
		end

	end     
	storeInfo.footStates=util.convertToLuaNativeTable(self.footStates)

	return storeInfo

end

function OnlineSynthesis2:saveStates(fn)
	local storeInfo=self:getStates()
	util.saveTable(storeInfo, fn)
end

function OnlineSynthesis2:restoreModelSpecific(storeInfo)
	self._prevZMP=storeInfo.prevZMP
	self._prevCOM=storeInfo.prevCOM
	self._prevCOMvel=storeInfo.prevCOMvel
	self._prevZMPvel=storeInfo.prevZMPvel
end
function OnlineSynthesis2:restoreStates(fn)

	-- store 2 previous segments.
	-- have to be called just before calling oneStep()

	if fineLog then fineLog("restoreStates"..fn) end
	local storeInfo=util.loadTable(fn)
	if fineLog then fineLog("loadTable finished"..fn) end
	self:_restoreStates(storeInfo)
end

function OnlineSynthesis2:_restoreStates(storeInfo)

	local outputL=self.outputLocal
	local outputG=self.outputGlobal


	self.numFrames=storeInfo.numFrames
	if self.outputFrameRate==nil then dbg.console() end
	self.numSimulFrames=(self.numFrames)*(model.simulationFrameRate/self.outputFrameRate)
	local currFrame=self.numFrames-1
	outputG.numFrames=storeInfo.numFrames

	outputG.refTime:assign(storeInfo.refTime)
	outputG.synRoot:assign(storeInfo.synRoot)

--	self.desiredCOP=storeInfo.desiredCOP
	useCase=useCases[storeInfo.useCaseGrpName]
	useCase.desiredPos=storeInfo.useCase_desiredPos
	self.deltaSeq=storeInfo.deltaSeq
	self.prevDeltaAngle= storeInfo.prevDeltaAngle
	local lcurrFrame=0

	if currFrame<0 then
		currFrame=0
		lcurrFrame=0
	else
		lcurrFrame=outputG.refTime(currFrame)
	end

	outputL:__init(self.graph)
	outputL.segInfo:resize(0)
	for i=0, table.getn(storeInfo.seq) do
		local seg=storeInfo.seq[i]

		outputL:append(self.graph[seg.grpName][seg.name])

		local segInfo=outputL.segInfo(i)
		assert(segInfo.seg.name==seg.name)
		segInfo.len=seg.len
		segInfo.nextlen=seg.nextLen
		segInfo.timeScale=seg.timeScale
		segInfo.invRefTime=seg.invRefTime      
	end

	-- self

	self.desiredSpeedX= storeInfo.desiredSpeedX
	self.desiredSpeedZ= storeInfo.desiredSpeedZ
	self.desiredTurningSpeed= storeInfo.desiredTurningSpeed

	self.prevDomain_seg=storeInfo.prevDomain_seg
	self.prevDomain_edge=storeInfo.prevDomain_edge

	self.trackingInfo=storeInfo.trackingInfo
	self.prevFrac=storeInfo.prevFrac
	self.desiredFootPosFlightPhase = storeInfo.desiredFootPosFlightPhase  
	self.prevLocalFirst=storeInfo.prevLocalFirst


	-- states
	self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, storeInfo.states)
	self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, storeInfo.dstates)
	if simulator==simulators.gmbs then
		-- restore exact state/dstate of the root joint (gmbs specific)
		self.simulator:test('setExactStateSpherical', storeInfo.gmbs_root_state)
	end
	self.simulator:setAllCollisionMargin( storeInfo.collMargin)
	self.simulator:initSimulation()

	self.outputMotion:resize(storeInfo.outputMotion:rows())
	if storeInfo.outputMotion:rows()~=0 then
		self.outputMotion:matView():assign(storeInfo.outputMotion)
	end
	self.prevAmt=storeInfo.prevAmt

	if storeInfo.dmotionDOF then
		if self.pdservo.dmotionDOF==nil then
			self.pdservo.dmotionDOF=matrixn()  
			self.pdservo.ddmotionDOF=matrixn()  
--			self.pdservo.motionDOF_pdtarget=matrixn()
		end   

		self.pdservo.dmotionDOF:assign(storeInfo.dmotionDOF)
		self.pdservo.ddmotionDOF:assign(storeInfo.ddmotionDOF)
--		self.pdservo.motionDOF_pdtarget:resize(storeInfo.motionDOF_pdtarget:rows())
--		self.pdservo.motionDOF_pdtarget:matView():assign(storeInfo.motionDOF_pdtarget)
	else
		self.pdservo.dmotionDOF=nil
		self.pdservo.ddmotionDOF=nil
--		self.pdservo.motionDOF_pdtarget=nil
	end
	self.pdservo.scale_lleg= storeInfo.scale_lleg
	self.pdservo.scale_rleg= storeInfo.scale_rleg
	self.pdservo.scale_upperbody= storeInfo.scale_upperbody
	self.pdservo.state=storeInfo.pdservo_state

	self.theta=storeInfo.theta
	self.dtheta=storeInfo.dtheta
	self.contactState=storeInfo.contactState
	self.prevContactState=storeInfo.prevContactState
	self.pendControlforce=storeInfo.pendControlforce
	error_feedback_method = storeInfo.error_feedback_method
	assert(storeInfo.error_feedback_method~=nil)
	self.errorOccurred=false
	local Y=self.pendulum.Y
	Y:assign(storeInfo.pendTrajectory)

	self.timeScaleOverride=storeInfo.timeScaleOverride
	self.timescale=storeInfo.timescale
	self.targetHeight=storeInfo.targetHeight

	self.pendulum:restoreStates(storeInfo.pendState)
	self.verticalPend:restoreStates(storeInfo.vpendState)

	self:restoreModelSpecific(storeInfo)

	self.currSegment=self.graph[storeInfo.currSegment[2]][storeInfo.currSegment[1]]


	outputG.startSearch=storeInfo.startSearch

	self.trackingErrorRot=storeInfo.trackingErrorRot

--	self.comTracker:setAllState(storeInfo.comTracker)
--	self.zmpTracker:setAllState( storeInfo.zmpTracker)
	self.pelvisError=storeInfo.pelvisError

	--      debugRestoreStates={}

	-- globals
	regResult=storeInfo.regResult
	useCase.controlParam=storeInfo.controlParam
	--useCase.asymmetricFoot=storeInfo.asymmetricFoot

	for i,v in ipairs(storeInfo.segProperties) do
		assert(v.name~=nil)
		local seg=self.graph[v.grpName][v.name]
		--seg.refLoffset=v.refLoffset
		--seg.refRoffset=v.refRoffset

		if useCase.keyframes then
			for kk, vv in pairs(useCase.keyframes) do
				if(v.keyframes[kk]~=nil) then
					seg[kk]=v.keyframes[kk]
				end
			end
		end
	end


	self.footStates=util.convertFromLuaNativeTable(storeInfo.footStates)
end

function OnlineSynthesis2:oneStep()

	self:updateKeyframes()


	gTimer2:start()
	local currFrame = self.numFrames


	-- assert(self.numFrames==self.outputGlobal.numFrames)

	local frameRate=self.outputFrameRate
	local dvz=self.desiredSpeedZ

	local fixedLocal=self.outputGlobal:getRefTime()
	local leaning=self.pendulum:calcLeaning()
	local pendvel=self.pendulum:calcCOMvel()
	local cartvel=self.pendulum:calcCartVel()



	RE.output2("currFrame", currFrame, fixedLocal)
	RE.output2("leaning", leaning)

	-- if       debugRestoreStates then
	--    debugRestoreStates.oneStep1=util.tostring(fixedLocal, leaning, pendvel, cartvel, dvz, frameRate, currFrame, max_forward_ref)

	-- end



	gTimer:start()
	if self.simulator==nil then
		useCase.turnOffTurnGain=true
	else
		useCase.turnOffTurnGain=false
	end

	do 
		-- clear time-scale
		local currSeg=math.floor(fixedLocal)
		local segInfo=self.outputLocal.segInfo
		segInfo(currSeg).timeScale=1
		if segInfo(currSeg+1) then
			segInfo(currSeg+1).timeScale=1
		end
		if segInfo(currSeg+2) then
			segInfo(currSeg+2).timeScale=1
		end
	end

	-- 1. planned root (CAN BE REPLANNED. NO STITCH, NO TIMESCALE)
	self.outputGlobal:predictGlobal(self.desiredTurningSpeed, self.desiredSpeedX, dvz,10000,  self)


	gTimer:stopMsg("predictGlobal")

	gTimer:start()

	-- self.objectList:registerObject("predictRoot", "LineList", "solidred", self.outputGlobal.synRoot:range(0, self.outputGlobal.synRoot:rows(), 0, 3)*100, 0)

	-- simulate pendulum.
	--self.pendDbgFn='pend_'..tostring(currFrame )..'.txt'
	--util.writeFile(self.pendDbgFn,'')
	-- self.pendDbgFn=nil
	self:__updateDesiredVel(currFrame)

	-- if self.pendDbgFn then
	-- 	local pend=self.pendulum
	-- 	util.appendFile(self.pendDbgFn, util.tostring('before onestep', pend.theta, pend.dtheta)..'\n')
	-- end
	--##dos if g_debugOneStep then g_debugOneStep:pushBack({"pendStates1",self.pendulum:getStates()}) end
	self.pendulum:oneStep()
	--##dos if g_debugOneStep then g_debugOneStep:pushBack({"pendStates2",self.pendulum:getStates()}) end
	self.verticalPend:singleStep()

	self.footStates.desiredDotAngMomentum=self.pendulum:calcDotMomentum()
	--##dos if g_debugOneStep then g_debugOneStep:pushBack({"pendStates4",self.pendulum:calcZMPCOM()}) end 
	--##dos if g_debugOneStep then g_debugOneStep:pushBack({"pendStates3",self.footStates.desiredDotAngMomentum:copy()}) end
	do
		local v=self.pendulum:calcDotLinMomentum()
		self.footStates.desiredDotLinMomentum=v
		local u=self.verticalPend.cf(0,0)
		--print(u)
		v.y=v.y+u
--		v.x=0
--		v.z=0
--		v.y=u

		--##dos if g_debugOneStep then g_debugOneStep:pushBack({"footStates",self.footStates.desiredDotAngMomentum:copy(), v:copy(), u}) end
	end
	if not cleanVisualize then
		-- draw momentum
		local startPos=self.pendulum:getPosition()+vector3(0,2,0)
		dbg.namedDraw('Line', startPos*100, (startPos+self.footStates.desiredDotLinMomentum*0.1)*100, "dotlinmmt", "solidred")
		dbg.namedDraw('Line', startPos*100, (startPos+self.footStates.desiredDotAngMomentum*0.1)*100, "dotangmmt", "solidblue")
	end

	do -- position control 시 pendulum state 보정 
		local currSeg=math.floor(fixedLocal)
		local rot_y=self:calcRotY(currFrame)
		local seg=self.outputLocal.segInfo(currSeg).seg
		if seg.usePositionControl then
			local lpendDstate=vectorn()
			self.outputLocal:sampleVec(fixedLocal, 'lpendDState', lpendDstate)
			--##dos 			if g_debugOneStep then g_debugOneStep:pushBack({"lpendStates6",lpendDstate:copy()}) end
			--##dos 			if g_debugOneStep then g_debugOneStep:pushBack({"lpendStates7", fixedLocal, self.pendulum.theta:copy(), self.pendulum.dtheta:copy() }) end

			local dx=vector3(lpendDstate(0), 0, lpendDstate(1))
			local dq=vector3(lpendDstate(2), 0, lpendDstate(3))
			local ddx=vector3(lpendDstate(4), 0, lpendDstate(5))
			local ddq=vector3(lpendDstate(6), 0, lpendDstate(7))
			dx:rotate(rot_y)
			dq:rotate(rot_y)
			ddx:rotate(rot_y)
			ddq:rotate(rot_y)
			local theta=self.pendulum.theta	
			local dtheta=self.pendulum.dtheta	
			theta:set(1, theta(1)+dx.x)
			theta:set(0, theta(0)+dx.z)
			local dqq=quater()
			dqq:setRotation(dq)
			theta:setQuater(2, dqq*theta:toQuater(2))
			dtheta:setVec3(0, dtheta:toVector3(0)+ddx)
			dtheta:setVec3(3, dtheta:toVector3(3)+ddq)
			--print(lpendDstate)
		end
		--##dos 		if g_debugOneStep then g_debugOneStep:pushBack({"pendStates6",self.pendulum:getStates()}) end
	end
	-- if self.pendDbgFn then
	-- 	local pend=self.pendulum
	-- 	util.appendFile(self.pendDbgFn, util.tostring('after onestep', pend.theta, pend.dtheta)..'\n')
	-- end


	if self.pendulum.errorOccurred==true then
		self.errorOccurred=true
		return
	end

	self.pendulum:draw()

	local bclamp=false

	--self.pendControlForce= vector3(-1*self.pendulum.controlforce(0),0, self.pendulum.controlforce(1))

	--if useTorqueGenerationCoef~=nil then
	--local controltorque=self.pendControlForce -- - self.outputGlobal:predictControlForce(currFrame)*predictedControlForceWeight
	--
	--
	--
	--RE.output2("pendControlForce2", self.pendControlForce)
	--
	--local controlangle
	--controlangle, bclamp=math.clampVec3(controltorque*useTorqueGenerationCoef, math.rad(10))
	----if bclamp==true then
	--
	--self.trackingErrorRot:setRotation(controlangle)
	--
	--
	--else
	self.trackingErrorRot:identity()
	--end

	if self.pendulum:calcLeaning()>math.rad(50) then
		print("falldown")
		error_feedback_method=EFM.NONE -- fall down.
	end

	-- if       debugRestoreStates then
	--    debugRestoreStates.trackingErrorRot=self.trackingErrorRot
	-- end

	self:generatePose(currFrame, 0, self.samplePoseBeforeIK, false)

	--##dos if g_debugOneStep then --##dos g_debugOneStep:pushBack({"spbik",self.samplePoseBeforeIK:copy()}) --##dos end

	-- calc nextOrientation.
	--self.outputGlobal:predictGlobal(self.desiredTurningSpeed, self.desiredSpeedX, dvz,10000,  self)

	--self.controlforce=vector3(self.pendulum.controlforce(1),0,self.pendulum.controlforce(0))

	gTimer:stopMsg("predictGlobal2")

	gTimer:start()
	-- predict future ZMP trajectory so that foot position can be planned.
	RE.output2("pendnumFrame", (currFrame+1),self.pendulum:numFrames())
	--if fixedLocal-math.floor(fixedLocal)<0.05 then

	local currSeg=math.floor(fixedLocal)
	local ts=1
	local grpName=self.outputLocal.segInfo(currSeg).seg.grpName

	local impL, impR, impLH, impRH
	local sppimpL, sppimpR, sppimpLH, sppimpRH

	do -- calc importance
		local useHand=useCase.keyframes.importanceLH
		local refTime=self.outputGlobal:getRefTime(currFrame)
		impL=self.outputLocal:sampleKey(refTime,'importanceL')
		impR=self.outputLocal:sampleKey(refTime,'importanceR')
		sppimpL=self.outputLocal:sampleKey(refTime,'spprtImpL')
		sppimpR=self.outputLocal:sampleKey(refTime,'spprtImpR')
		if useHand then
			impLH=self.outputLocal:sampleKey(refTime,'importanceLH')
			impRH=self.outputLocal:sampleKey(refTime,'importanceRH')
			sppimpLH=self.outputLocal:sampleKey(refTime,'spprtImpLH')
			sppimpRH=self.outputLocal:sampleKey(refTime,'spprtImpRH')
		end
		if useCase.spprtImpFromImp then
			sppimpL=impL
			sppimpR=impR
			sppimpLH=impLH
			sppimpRH=impRH
		end
		RE.output2("importance","L,R,LH,RH", impL, impR,impLH,impRH)
		RE.output2("sppImp", sppimpL, sppimpR, sppimpLH, sppimpRH, refTime)	
		self.imp={impL,impR, impLH, impRH, sppimpL, sppimpR, sppimpLH, sppimpRH}
	end

	--ys
	--if (grpName=='walk3' or grpName=='handstand') and self.simulator then
	if self.simulator then
		if useCase.noStride==nil or useCase.noStride==false then
			-- stride estimation	
			local lbound=self:calcPredictionBound(currFrame+1, "L", IDservoLatency)
			local rbound=self:calcPredictionBound(currFrame+1, "R", IDservoLatency)

			local useHand=useCase.keyframes.importanceLH
			if useHand then
				lbound=math.max(lbound, self:calcPredictionBound(currFrame+1, "LH", IDservoLatency))
				rbound=math.max(rbound, self:calcPredictionBound(currFrame+1, "RH", IDservoLatency))
			end
			local pendcf=self:predictPendulumTrajectory(self.desiredTurningSpeed, self.desiredSpeedX, self.desiredSpeedZ, math.max(lbound,rbound)+1)

			local function calcTimeScale(L,R)
				local ts=1
				local lfootStride,lpos=self:strideEstimation(currFrame, L, IDservoLatency,1)
				local rfootStride,rpos=self:strideEstimation(currFrame, R, IDservoLatency,1)
				RE.output2('strideEstimation'..L..R, lfootStride, rfootStride)

				local strideMode='N'
				local keyName='stride'
				if L=='LH' then
					strideMode='NH'
					keyName='strideH'
				end
				local stride=0
				if lfootStride~=0 then
					strideMode=L
					stride=lfootStride
				elseif rfootStride~=0 then
					strideMode=R
					stride=rfootStride
				end

				if stride>0 then
					local ts_min=0.5
					local stride=lpos:distance(rpos)
					local desiredStride=self.outputLocal:sampleVal(self.outputGlobal.refTime(currFrame), keyName)
					if stride>0.1 and desiredStride>0.1 then -- and stride > desiredStride then
						ts=desiredStride/stride
					end
					self.footStates[strideMode..'footStride']=1/ts
					ts=math.max(ts,ts_min)
					ts=math.min(ts,1.0)

					local strideRatio=(self.footStates[R..'footStride']-self.footStates[L..'footStride'])
					local gain=2
					ts=ts*self:adjustStrideRatio(currSeg, gain*strideRatio)
					--print(strideRatio)
					RE.output2('strideRatio'..L, strideRatio)

					ts=math.max(ts,ts_min)
					ts=math.min(ts,1.0)
					--if currFrame>200 then ts=0.7 end
				else
					ts=((self.timescale or 1) +1)*0.5
				end
				return ts
			end

			local ts_foot=calcTimeScale("L","R")
			local useHand=useCase.keyframes.importanceLH
			if useHand then
				local ts_hand=calcTimeScale("LH","RH")
				RE.output2('timeScale', ts_foot, ts_hand, self.timeScaleOverride)
				ts=math.min(ts_foot, ts_hand)
			else
				RE.output2('timeScale', ts_foot, self.timeScaleOverride)
				ts=ts_foot
			end
		end
	end
	if false then -- manual adjustment
		if float_options.timescaleL then
			local val=float_options.timescaleL.val
			ts=self:adjustStrideRatio(currSeg, val)
		end
		--if useCase.noStrideAdjust then ts=1 end
	end
	if grpName=='handstand' and self.simulator then
		-- mocap was too fast (so it was ugly)
		-- 
		if useCase.graph[#useCase.graph][3]=="LRH0" then
			ts=ts*1.4
		else
			ts=ts*sop.clampMap(currFrame, 270, 390,1.0, 1.4)
		end

	end
	self.timescale=ts
	RE.output2('timeScale_f', ts, self.timeScaleOverride)
	do
		-- apply adjusted time scale
		local segInfo=self.outputLocal.segInfo
		segInfo(currSeg).timeScale=ts
		--if segInfo(currSeg+1) then segInfo(currSeg+1).timeScale=ts end
	end

	-- recalculate ref-time
	self.outputGlobal:predictGlobal(self.desiredTurningSpeed, self.desiredSpeedX, dvz,10000,  self)

	if true then
		RE.output2("desiredSpeedZ",self.desiredSpeedZ)
		local lbound=self:calcPredictionBound(currFrame+1, "L", IDservoLatency)
		local rbound=self:calcPredictionBound(currFrame+1, "R", IDservoLatency)

		local useHand=useCase.keyframes.importanceLH
		if useHand then
			lbound=math.max(lbound, self:calcPredictionBound(currFrame+1, "LH", IDservoLatency))
			rbound=math.max(rbound, self:calcPredictionBound(currFrame+1, "RH", IDservoLatency))
		end
		--print(math.max(currFrame, lbound,rbound)+1)
		local pendcf=self:predictPendulumTrajectory(self.desiredTurningSpeed, self.desiredSpeedX, self.desiredSpeedZ, math.max(lbound,rbound)+1)
		--      self:predictPelvisOrientations()
		if false then
			-- integral control
			print(pendcf)
			self.footStates.desiredDotAngMomentum=self.pendulum:calcDotMomentum2(pendcf)
			self.footStates.desiredDotLinMomentum=pendcf
			local u=self.verticalPend.cf(0,0)
			pendcf.y=pendcf.y+u
		end
	end

	-- save current root position and orientation:
	--   local synRoot=self.outputGlobal.synRoot  
	--   synRoot:row(currFrame):assign(self.samplePoseBeforeIK:range(0,7))
	self.outputGlobal.numFrames=currFrame+1
	if self.errorOccurred==true then
		return 
	end


	--if not useCase.ignoreSwingFootForce then

	gTimer:stopMsg("predictPendulumTrajectory")
	gTimer:start()
	-- predict one more frame (temporalily. for estimating velocity)
	local blendFootCorrectionCoef=useCase.blendFootCorrectionCoef or 1
	if self.simulator==nil then
--		blendFootCorrectionCoef=0
	end
	--assert(self.simulator==nil or blendFootCorrectionCoef==1)

	--##dos if g_debugOneStep then --##dos g_debugOneStep:pushBack({"bfc", useCase.grpName, blendFootCorrectionCoef, useCase.blendFootCorrectionCoef}) --##dos end
	self.ikInfo={}
	self.mapIndex=1
	self:generatePose(currFrame,IDservoLatency,  self.samplePose, true, blendFootCorrectionCoef)
	self.mapIndex=2
	self:generatePose(currFrame+1,IDservoLatency,  self.samplePose2, true, blendFootCorrectionCoef)
	if true then -- override ankle angles
		local pose1=vectorn()
		local pose2=vectorn()
		self.mapIndex=3
		self:generatePose(currFrame,IDservoLatency,  pose1, true, 0)
		self:generatePose(currFrame+1,IDservoLatency,  pose2, true, 0)

		local dofInfo=self.graph.skel_withoutCOMjoint.dofInfo

		local lt=self.graph.bone2.lfoot:treeIndex()
		local rt=self.graph.bone2.rfoot:treeIndex()

		for i=dofInfo:startT(lt), dofInfo:endR(lt)-1 do
			self.samplePose:set(i, pose1(i))
			self.samplePose2:set(i, pose2(i))
		end
		for i=dofInfo:startT(rt), dofInfo:endR(rt)-1 do
			self.samplePose:set(i, pose1(i))
			self.samplePose2:set(i, pose2(i))
		end
	end

--	if self.desiredCOP then
--		local zmp=self.pendulum:getPos(currFrame+COPlatency)
--		self.desiredCOP:assign(zmp)
--	end

	--##dos if g_debugOneStep then --##dos g_debugOneStep:pushBack({"sp",self.samplePose:copy()}) --##dos g_debugOneStep:pushBack({"sp2",self.samplePose2:copy()}) --##dos end


	gTimer:stopMsg("generatePose")

	self.skin2:setPoseDOF(self.samplePose)
	--   self.skin2:setPoseDOF(temp)

	--   local lfoot=self.outputGlobal.footLglobal(currFrame)
	--   local rfoot=self.outputGlobal.footRglobal(currFrame)
	--
	--   footLEntity=self.objectList:registerEntity("footL", "sphere1010.mesh")
	--   footLEntity:setScale(10,10,10)
	--   footLEntity:setPosition(lfoot*100)
	--   
	--   footREntity=self.objectList:registerEntity("footR", "sphere1010.mesh")
	--   footREntity:setScale(10,10,10)
	--   footREntity:setPosition(rfoot*100)

	self.numFrames=self.numFrames+1

	gTimer:start()
	self:debugDraw()
	gTimer:stopMsg("debugDraw")


	gTimer2:stopMsg("onestep")

	-- if       debugRestoreStates then
	--    debugRestoreStates.controlforce=self.controlforce

	-- end


	if self.simulator==nil then
		local synRoot=self.outputGlobal.synRoot      
		local targetPose=synRoot:row(currFrame)      
		local rotAxis_y=self.graph.id_rotY.rotY(targetPose)
		self.graph.id_rotY.setRotY(synRoot, currFrame, rotAxis_y)
	end

	if cleanVisualize==true then
		local skel=self.skel
		local info=self.trackingInfo


		if info~=nil then
			local currFrame=self.numFrames
			local seg, refTime, f, l, frac1=self:calcWeights(math.max(currFrame,0))
			local pose=seg:pose(frac1)
			local root=MotionDOF.rootTransformation(pose)
			local root2=MotionDOF.rootTransformation(self.samplePoseBeforeIK)

			root.rotation:mult(root2.rotation:rotationY(), 
			root.rotation:offsetQ())
			root.translation.x=root2.translation.x
			root.translation.z=root2.translation.z
			MotionDOF.setRootTransformation(pose, root)
			self.skin2:setPoseDOF(pose)


			-- if       debugRestoreStates then
			--    debugRestoreStates.info=util.tostring(info.frac, root2.rotation, root.rotation, root2.translation, root.translation)
			-- end

		end
	end


end
function OnlineSynthesis2:debugDraw()

	local currFrame=self.numFrames-1
	if boolean_options.drawPelvis then
		local root=self.outputGlobal.synRoot
		local lines=root:range(0, root:rows(), 0,3)*100
		self.objectList:registerObject("pelvis", "LineList", "solidgreen", lines, 0)


		-- dbg.drawCoordinate(self.objectList,       MotionDOF.rootTransformation(root:row(currFrame)), "pelvisCurr")
		-- dbg.drawCoordinate(self.objectList,       MotionDOF.rootTransformation(root:row(currFrame+10)), "pelvisCurr2")
	end	


	-- buggy
	--self.pendulum:setOrientation(self.rot_y:rotationVector().y)



	local ipf=math.floor(currFrame)
	-- local com=self.outputGlobal:predictCOM(ipf)
	-- local zmp=self.outputGlobal:predictZMP(ipf)



end

function OnlineSynthesis2:produceSegment()

	local newseg
	self.desiredType, newseg=self.graph:transitionFunction(self.desiredType, self.currSegment)
	assert(newseg)
	self.currSegment=newseg
	self.outputLocal:append(self.currSegment)


end

function OnlineSynthesis2:setDesiredVel(start, vel)
	--		pr("start" , start, vel)
	self.desiredVelStart=start
	self.desiredVel=vel
	--self:updateDesiredVel()
end








function OnlineSynthesis2:calcCOMpos(zmp, zmpOri)
	local com=vector3()
	com:rotate(zmpOri, self.pendulum.pole:localCOM())
	com:radd(zmp)
	return com
end


function setPoseDOFRot90(skin, pose)

	local root=MotionDOF.rootTransformation(pose)
	root.rotation:leftMult(quater(math.rad(90), vector3(0,1,0)))
	local pose2=vectorn()
	pose2:assign(pose)
	MotionDOF.setRootTransformation(pose2, root)
	skin:setPoseDOF(pose2)
end

function OnlineSynthesis2:setSegmentProperties(seq, name, valseq, replicate)
	local N_seq=table.getn(seq)

	if replicate==true then
		for i=1, N_seq do
			seq[i][name]=math.copy(valseq)
		end
	else
		assert(N_seq==table.getn(valseq))
		for i=1, N_seq do
			seq[i][name]=valseq[i]
		end
	end
end


function OnlineSynthesis2:__updateConstraints()
	local grpNames=useCase.grpNames or { useCase.grpName}
	for igrp,grpName in ipairs(grpNames) do
		local useCase=useCase.grpNameToUseCase[grpName]
		if useCase.keyframes then
			for kk, vv in pairs(useCase.keyframes) do
				if vv.isContinuous then
					-- for each segment
					for i,grpName in ipairs(self.graph.groups.tf) do
						for k,v in pairs(self.graph[grpName]) do
							local keyvalues=v[kk]
							assert(keyvalues:size()>2)
							keyvalues:row(0):assign(keyvalues:row(1))
							keyvalues:at(-1):assign(keyvalues:at(-2))
						end
					end
				end
			end
			if useCase.funcUpdateConstraints then
				useCase.funcUpdateConstraints(self.graph)
			end
		end
	end
	if self.pdservo then
		self.pdservo:updateCoef()
	end
end

function OnlineSynthesis2:changeControlParameters(tbl, noConUpd)
	if tbl==nil then return end

	self.graph:changeControlParameters(tbl)

	if noConUpd==nil then
		self:__updateConstraints()
	end
end


function OnlineSynthesis2:calcWeights(currFrame)

	local refTime=self.outputGlobal:getRefTime(currFrame)
	local segIndex=math.floor(refTime)

	local segInfo=self.outputLocal.segInfo(segIndex)
	local nextSegInfo=self.outputLocal.segInfo(segIndex+1)
	local seg=segInfo.seg

	local f=segIndex
	local l=segIndex+1
	local frac1=refTime-segIndex

	return seg, refTime, f, l, frac1
end

function OnlineSynthesis2:updateKeyframes()

	local currFrame=self.numFrames


	--      currFrame=currFrame+PDservoLatency 

	local seg, refTime, f, l, frac1=self:calcWeights(math.max(currFrame,0))

	if f~=self.prevLocalFirst then

		self.prevSegment=seg
		self.prevLocalFirst=f

		--	 if currFrame-PDservoLatency>0 then
		if currFrame>0 then

			if self.simulator~=nil and seg.calcDeltaAngle~=nil then
				local comVel=self.simulator:calculateCOMvel(0)
				local rotY=self:calcRotY(currFrame)
				local currDeltaAngle=seg.calcDeltaAngle(comVel, rotY)
				local desiredDeltaAngle=seg.deltaAngles(0)

				local pda=self.prevDeltaAngle
				self.prevDeltaAngle=currDeltaAngle:copy()

				if pda then
					currDeltaAngle:safeSlerp(pda, currDeltaAngle:copy(), 0.5)
				end

				self.pelvisError=quater()
				self.pelvisError:difference(desiredDeltaAngle, currDeltaAngle)
				self.pelvisError:setRotation(vector3(0,1,0), math.clamp(self.pelvisError:rotationAngleAboutAxis(vector3(0,1,0)),
				math.rad(-10), math.rad(10)))
				--##dos if g_debugOneStep then --##dos g_debugOneStep:pushBack({"pelvisError",self.pelvisError:copy()}) --##dos end
				-- do  -- method 2 

				--    -- back error correction into the lmot.

				--    if string.sub(seg.name,2)=="F" then
				-- 	  self.outputLocal:correctOrientation(f,math.round((f+l)/2), self.pelvisError)
				--    elseif string.sub(seg.name,1,1)=="F" then
				-- 	  self.outputLocal:correctOrientation(math.round((f+l)/2),l, self.pelvisError)
				--    end

				--    self.pelvisError:identity()
				-- end
			end
		end

	end


	if useCase.keyframes.footLmod then
		footLcorrection=self.outputLocal:sampleKey(refTime, 'footLmod')
		footRcorrection=self.outputLocal:sampleKey(refTime, 'footRmod')
	end
	if useCase.keyframes.head_mod then
		head_mod=self.outputLocal:sampleKey(refTime, 'head_mod')
	end

	if useCase.keyframes.footLmocapMod then
		footLmocapCorrection=self.outputLocal:sampleKey(refTime, 'footLmocapMod')
		footRmocapCorrection=self.outputLocal:sampleKey(refTime, 'footRmocapMod')
		RE.output2("footLcorrection", footLcorrection:copy(), footLmocapCorrection:copy())
		RE.output2("footRcorrection", footRcorrection:copy(), footRmocapCorrection:copy())
	end

	if useCase.keyframes.handLmod then

		handLcorrection=self.outputLocal:sampleKey(refTime, 'handLmod')
		handRcorrection=self.outputLocal:sampleKey(refTime, 'handRmod')
	end



	self.trackingInfo={currSeg=seg, frac=frac1}
	if seg.grpName~=useCase.grpName then
		useCase=useCases[seg.grpName]
	end
	RE.output2("trackingInfo", seg.grpName,seg.name,frac1,"reftime:",refTime)

	--      assert(frac1>-0.0001 and frac1<1.0001)



	--##dos if g_debugOneStep then --##dos g_debugOneStep:pushBack({"trackingInfo",tostring(seg.name).." "..tostring(frac1).." refTime:"..tostring(refTime).." "}) --##dos end





	--useTorqueGenerationR=0

	-- useTorqueGenerationL=1
	-- useTorqueGenerationR=1


	-- if self.contactState.E=="L" then
	-- 	 useTorqueGenerationR=0
	-- elseif self.contactState.E=="R" then
	-- 	 useTorqueGenerationL=0
	-- end




	-- if debugRestoreStates then
	-- 	 if debugRestoreStates.updateKF==nil then
	-- 	    debugRestoreStates.updateKF=array:new()
	-- 	 end

	-- 	 local dr={}
	-- 	 dr.seg=seg.grpName.."_"..seg.name
	-- 	 dr.segfootL=seg.footLcorrection:copy()
	-- 	 dr.footLcorrection=footLcorrection:copy()
	-- 	 dr.footRcorrection=footRcorrection:copy()
	-- 	 debugRestoreStates.updateKF:pushBack(dr)
	-- end

	disableTouchDownFeedback=seg.disableTouchDownFeedback
	-- iterativeIK.nIter=seg.iterativeIK

	-- assert( seg.iterativeIK~=nil )

	-- RE.output("seg", tostring(seg.name).." weight:"..tostring(segweight)
	-- 	  .." control"..tostring(seg.controlMethod))
	--   RE.output("useTorqueGeneration", tostring(useTorqueGenerationL).. " "..tostring(useTorqueGenerationR))
end
function OnlineSynthesis2:calcDesiredBasePos(seg, currFrame)
	local lframe, rframe
	if self.simulator then
		lframe=self.simulator:getWorldState(0):globalFrame(self.graph.bone2.lfoot)
		rframe=self.simulator:getWorldState(0):globalFrame(self.graph.bone2.rfoot)
	else
		if self.outputMotion:numFrames()==0 then
			return vector3(0,0,0)
		end
		local pose=self.outputMotion:row(currFrame-1)
		self.skel_withoutCOMjoint:setPoseDOF(pose)
		lframe=self.graph.bone2.lfoot:getFrame()
		rframe=self.graph.bone2.rfoot:getFrame()
	end

	local desiredpos=(
	lframe:toGlobalPos(self.graph.lfootpos)+
	rframe:toGlobalPos(self.graph.rfootpos))*0.5

	--dbg.namedDraw('Sphere', desiredpos*100, 'DesiredBasePos')
	return desiredpos
end

function OnlineSynthesis2:__updateDesiredVel(currFrame)
	local outputG=self.outputGlobal
	local seg,refTime,f,l,frac1=self:calcWeights(math.max(0,currFrame))

	-- use firstframe
	--local ff=outputG:invRefTime(f) 

	-- use ref frame
	--local pendRotRefTime=self.outputLocal:sampleKey(f, "pendRotRefTime")
	--local ff=outputG:invRefTime(f+pendRotRefTime)
	local ff=outputG:invRefTime(f)
	local rot_y=self:sampleRotY(ff)

	--if scenario>=scenarios.STAND1 and scenario<=scenarios.STAND4 then
	if currFrame==self.numFrames then
		--print('pendRefTime: ', currFrame, ff)
		self.usePositionControl=seg.usePositionControl
		if self.usePositionControl then
			local calcDesiredBasePos=useCase.calcDesiredBasePos or self.calcDesiredBasePos
			useCase.desiredPos=calcDesiredBasePos(self,seg, currFrame)
		end
		--print('positionControl',self.usePositionControl)
		--self.usePositionControl=false
		--if currFrame==300 then this('exit',1) end
	end

	self.pendulum:changeMode(self.usePositionControl)
	-- if self.pendDbgFn then
	-- 	util.appendFile(self.pendDbgFn, util.tostring(self.usePositionControl,currFrame,refTime, pendRotRefTime,ff,'A', rot_y, 'B', self:sampleRotY(ff+1) ))
	-- end
	self.pendulum:setOrientation2(rot_y)
	--if seg.usePositionControl then
	if self.usePositionControl then
		self.pendulum:setDesiredPosition(useCase.desiredPos)
		local desiredLeaning=self.outputLocal:sampleKey(refTime,'desiredLeaning')
		if float_options.desiredAngleX then
			desiredLeaning.x=desiredLeaning.x+float_options.desiredAngleX.val
			desiredLeaning.z=desiredLeaning.z+float_options.desiredAngleZ.val
		end
		if self.simulator==nil then
			desiredLeaning.x=0
			desiredLeaning.z=0
		end

		self.pendulum:setDesiredOrientation(desiredLeaning)	
		
		if currFrame==self.numFrames then
			RE.output2("desiredPParam", "Pos", useCase.desiredPos)
			RE.output2("desiredLeaning", "Leaning", desiredLeaning)
			dbg.namedDraw('Sphere', useCase.desiredPos*100, "targetPos")	
			--print('desiredleaning', desiredLeaning)
		end
		--##dos if g_debugOneStep then --##dos g_debugOneStep:pushBack({"setDesiredPos",useCase.desiredPos:copy()}) --##dos end
	else
		local tt
		if false then
			tt=seg.desiredVelocity:copy()
			tt:rotate(rot_y)

		else
			tt=self.outputLocal:sampleKey(refTime, "pendDesiredVel")
			RE.output2('pendDesiredVel', tt)

			if self.simulator then
				local scale_factor=useCase.pendVelScale or 1
				tt.z=tt.z*scale_factor
				tt.x=tt.x*scale_factor
			end

			tt.z=tt.z+self.desiredSpeedZ
			-- hoihoi
			tt.x=tt.x+self.desiredSpeedX

	if useCase.speedMod then
		tt=tt+useCase.speedMod
	end

			local amt_sideway_vel=useCase.amt_sideway_vel or 3
			local amt_sideway_vel_negative=useCase.amt_sideway_vel_negative or amt_sideway_vel

			local tt_len=tt:length()
			local dq_mod=self.desiredTurningSpeed
			-- if self.outputGlobal.synRoot:cols()==6 then
			-- 	dq_mod=self.outputGlobal.synRoot(currFrame,1)
			-- else
			-- 	assert(false)
			-- end
			-- dq_mod=0
			if dq_mod>0 then
				tt.x=tt.x+dq_mod/model.frame_rate*120*amt_sideway_vel*tt.z
			else
				tt.x=tt.x+dq_mod/model.frame_rate*120*amt_sideway_vel_negative*tt.z
			end

			-- preserver original speed
--			assert(tt:length()~=0)
			if tt:length()==0 then 
				print('warning - unoptimized pend param',tt,  seg.name, seg.grpName) 
			else
				tt:scale(tt_len/tt:length())
			end
			--	    if currFrame==self.numFrames then print(tt) end
			--	 end

			--tt.x=tt.x+seg.velModx
			--tt.z=tt.z+seg.velModz

			-- if self.pendDbgFn then
			-- 	util.appendFile(self.pendDbgFn, util.tostring('K',tt))
			-- end
			tt:rotate(rot_y)

			-- if self.pendDbgFn then
			-- 	util.appendFile(self.pendDbgFn, util.tostring('P',tt, self.desiredSpeedX, rot_y ))
			-- end
			----	 local alpha=0.7	 -- smoothing?
			--local alpha=1.0  -- no smoothing
			--tt=tt*alpha+self.pendulum:calcCOMvel()*(1-alpha)


		end
		--##dos if g_debugOneStep then --##dos g_debugOneStep:pushBack({"setDesiredVel",seg.name, seg.pendDesiredVel, refTime,tt:copy()}) --##dos end
		if not useCase.noTimeVaryingInertia then
			local linertia=self.outputLocal:sampleV3(refTime, "linertia")
			self.pendulum.pole:setLocalCOM(vector3(0, linertia.y,0))
			self.pendulum:setInertia(linertia)
		end
		self.pendulum:setDesiredVelocity(tt)
		-- if self.pendDbgFn then
		-- 	util.appendFile(self.pendDbgFn, util.tostring(currFrame, tt)..'\n')
		-- end
		if currFrame==self.numFrames then
			RE.output2("desiredPParam", "Vel", tt)
			--print(tt, self.pendulum.theta, self.pendulum.dtheta)
			--print(tt,  seg.name, seg.grpName) 
		end
	end

end

function OnlineSynthesis2:findSegment(currFrame)
	local r=self.outputGlobal.refTime
	assert(currFrame>=0 and currFrame<r:size(),
	currFrame..","..r:size())

	return self.outputLocal:findSegmentRef(r(currFrame))
end

function OnlineSynthesis2:predictPendulumTrajectory(dTurningSpeed, dvx, dvz, prediction_bound)

	local outputGlobal=self.outputGlobal
	local outputLocal=self.outputLocal

	-- pendulum:predict..
	local pend=self.pendulum
	local theta=vectorn()
	local dtheta=vectorn()
	local numFrames=pend:numFrames()

	pend:_saveStates(theta, dtheta)


	local fixed=outputGlobal.numFrames-0
	assert(fixed==numFrames-1)
	local fixedLocal=outputGlobal.refTime(fixed)

	local frameRate=self.outputFrameRate
	--local avgControlForce=vector3(0,0,0)  -- for integral control
	--local numC=0
	--local CFwindow_size=10

--	print(outputGlobal.synRoot:rows() - (fixed+1), prediction_bound )
	if predictPendulumDownSample==false then
		-- too slow
		for i=fixed+1, outputGlobal.synRoot:rows()-1 do

			-- update desired vel
			local currFrame=i

			self:__updateDesiredVel(currFrame)
			-- if self.pendDbgFn then
			-- 	util.appendFile(self.pendDbgFn, util.tostring(i, pend.theta, pend.dtheta)..'\n')
			-- end
			pend:oneStep()

			-- if self.pendDbgFn then
			-- 	local ee=pend.numFrame
			-- 	util.appendFile(self.pendDbgFn, util.tostring(i, pend.theta, pend.dtheta, pend.Y:row(ee-1))..'\n')
			-- end
			if pend.errorOccurred==true then
				self.errorOccurred=true
				return
			end

		end
	else
		i=fixed+1
		--      pr("predictPend", outputGlobal.synRoot:rows() )
		assert(prediction_bound<outputGlobal.synRoot:rows())
		while i<= prediction_bound do

			-- update desired vel
			local currFrame=i
			self:__updateDesiredVel(currFrame)
			-- if self.pendDbgFn then
			-- 	util.appendFile(self.pendDbgFn, util.tostring(pend.theta, pend.dtheta)..'\n')
			-- end
			pend:fourSteps()

			--for j=i, i+3 do
			--	if numC<CFwindow_size then
			--		avgControlForce:radd(pend.controlForce)
			--		numC=numC+1
			--	end
			--end
			if pend.errorOccurred==true then
				self.errorOccurred=true
				return
			end
			-- if self.pendDbgFn then
			-- 	local ee=pend.numFrame
			-- 	util.appendFile(self.pendDbgFn, util.tostring(pend.theta, pend.dtheta, pend.Y:row(ee-1))..'\n')
			-- end

			i=i+4
		end
	end


	if false and boolean_options.drawPredictedZMP then
		--self.objectList:registerObject("zmpPredict", "LineList2D", "solidred", pend.Y:range(numFrames, pend.Y:rows(), 0, pend.Y:cols())*100, 0)
		self.objectList:registerObject("zmpPredict", "LineList2D_ZX", "solidred", pend.Y:range(0, pend.Y:rows(), 0, pend.Y:cols())*100, 0)

		if true then -- draw predicted orientation too
			local out=matrixn(0,3)
			local tempVec=vectorn()
			local tempVec2=vectorn(3)
			for i=math.floor((fixed+40)/40)*40, prediction_bound, 40 do
				local r=pend.Y:row(i)
				local s=vector3(r(1),0.03,r(0))
				outputGlobal.synRoot:sampleRow(i, tempVec)
				local rot_y_ref=self.graph.id_rotY.rotY2(tempVec)
				local v=rotate( vector3(0.1,0,0),rot_y_ref)
				tempVec2:setVec3(0,s-v)
				out:pushBack(tempVec2);
				tempVec2:setVec3(0,s+v)
				out:pushBack(tempVec2);
			end
			self.objectList:registerObject("oriPredict", "LineList", "solidred", out*100 , 0)
		end
	end	


	--pr("fixed", fixed,numFrames)
	--   assert(fixed==numFrames-1)

	pend:_restoreStates(theta, dtheta, numFrames)

	-- if self.pendDbgFn then
	-- 	local ee=pend.numFrame
	-- 	util.appendFile(self.pendDbgFn, util.tostring('after restore', pend.theta, pend.dtheta, pend.Y:row(ee-1))..'\n')
	-- end
	--return avgControlForce*(1/numC)
end

function OnlineSynthesis2:predictPelvisOrientations()-- predict future pelvis orientations based on trajectory tangents
	local outputGlobal=self.outputGlobal
	local synRoot=outputGlobal.synRoot
	local pend=self.pendulum
	local fixedGlobal=math.max(outputGlobal.numFrames-2,0)
	local fixedLocal=outputGlobal.refTime(fixedGlobal)
	local iseg=math.floor(fixedLocal)

	local function calcTangentOri(iseg, _name)
		local firstF=outputGlobal:invRefTime(iseg)
		local lastF=outputGlobal:invRefTime(iseg+1)

		local comF=pend:__getCOMpos(pend.Y:row(firstF))
		local comE=pend:__getCOMpos(pend.Y:row(lastF))

		if _name then
			dbg.drawArrow(self.objectList,comF*100+vector3(0,100,0),comE*100+vector3(0,100,0),_name)
		end

		local speed=(comE-comF):length()/(lastF-firstF)*self.outputFrameRate

		local trot=quater()
		trot:setAxisRotation(vector3(0,1,0), vector3(0,0,1), comE-comF)
		local prot=quater()
		prot:safeSlerp(self:calcRotY(firstF), self:calcRotY(lastF), 0.5)

		local speedMin=0.5 --m/s
		local speedMax=1

		local tangentOri=quater()
		tangentOri:safeSlerp(prot, trot, sop.clampMap(speed, speedMin, speedMax, 0, 1))
		return firstF, lastF, tangentOri
	end
	local ff1, lf1, t1=calcTangentOri(iseg, "seg1")
	local ff2, lf2, t2=calcTangentOri(iseg+1, "seg2")
	local ff3, lf3, t3=calcTangentOri(iseg+2, "seg3")
	local m1=math.round((ff1+lf1)/2)
	local m2=math.round((ff2+lf2)/2)
	local m3=math.round((ff3+lf3)/2)


	dbg.drawArrow(self.objectList, vector3(0,0,0), vector3(0,0,100):Rotate(t1), "t1arrow")

	for i=fixedGlobal+1, lf3 do
		local rotY=self.graph.id_rotY.rotY(synRoot:row( i))

		local trot=quater()
		if i< m1 then
			trot=t1
		elseif i<m2 then
			trot:safeSlerp(t1,t2, sop.map(i, m1, m2, 0, 1))
		elseif i<m3 then
			trot:safeSlerp(t2,t3, sop.map(i, m2, m3, 0, 1))
		else
			trot=t3
		end

		if i<fixedGlobal+10 then
			rotYmod:safeSlerp(rotY, trot, sop.map(i, fixedGlobal, fixedGlobal+60, 0, 1))
		else
			rotYmod=trot
		end

		self.graph.id_rotY.setRotY(synRoot,i, rotYmod)
	end

end

function OnlineSynthesis2:timeScale()
	local currSeg=math.floor(self.outputGlobal.refTime(self.numFrames-1))
	return self.outputLocal.segInfo(currSeg).timeScale
end

function OnlineSynthesis2:strideEstimation( refFrame, isL, prediction_latency,importance)
	return self:predictFootPosition(nil, refFrame, isL, prediction_latency, importance, true)
end

function OnlineSynthesis2:calcPredictionBound(refFrame, isL, prediction_latency)
	local mapIndex=isL..tostring(self.mapIndex)
	local outputGlobal=self.outputGlobal
	local refTime=outputGlobal.refTime(refFrame)
	local outputLocal=outputGlobal.outputLocal
	local iseg=math.floor(refTime)
	local f=iseg
	local l=iseg+1
	local footref=outputLocal:getFootRef(iseg,isL)
	local weight=sop.map(refTime-f, 0, 1, footref(2), footref(3))
	--if useCase.smoothSwingFootTransition then
		--weight=math.smoothTransition(weight)
	--end

	local time0=f+footref(0)
	local time1=f+footref(1)
	local footlocal=outputLocal:sampleV3(refTime, 'foot'..isL)

	if time0<1 then time0=1 end
	if time1<1 then time1=1 end

	local frame0=self.outputGlobal:invRefTime(time0)
	local frame1=self.outputGlobal:invRefTime(time1)
	if frame0<0 then frame0=0 end
	if frame1<0 then frame1=0 end

	--print(time0, f, footref(0), frame0, frame1, refFrame, prediction_latency)
	return math.max(frame0, frame1, refFrame)+prediction_latency
end

function OnlineSynthesis2:predictFootPosition(pelvis, refFrame, isL, prediction_latency,importance, isStrideEstimation, usePositionControl, spprtImportance)

	local mapIndex=isL..tostring(self.mapIndex)
	local outputGlobal=self.outputGlobal
	local refTime=outputGlobal.refTime(refFrame)
	local outputLocal=outputGlobal.outputLocal
	local iseg=math.floor(refTime)
	local f=iseg
	local l=iseg+1
	local footref=outputLocal:getFootRef(iseg,isL)
	local weight=sop.map(refTime-f, 0, 1, footref(2), footref(3))
	--if useCase.smoothSwingFootTransition then
		--weight=math.smoothTransition(weight)
	--end

	local time0=f+footref(0)
	local time1=f+footref(1)
	local footlocal=outputLocal:sampleV3(refTime, 'foot'..isL)
	local footlocal1=outputLocal:sampleV3(time0, 'foot'..isL)
	local footlocal2=outputLocal:sampleV3(time1, 'foot'..isL)

	if time0<1 then time0=1 end
	if time1<1 then time1=1 end

	local frame0=self.outputGlobal:invRefTime(time0)
	local frame1=self.outputGlobal:invRefTime(time1)

	if self.footStates.usePositionControl then
	--if outputLocal.segInfo(iseg).seg.usePositionControl then
		frame0=refFrame
		frame1=refFrame
	end
	if frame0<0 then frame0=0 end
	if frame1<0 then frame1=0 end

	local pend=self.pendulum

	local tempVec=vectorn()
	local outputGlobal=self.outputGlobal
	--   outputGlobal.synRoot:sampleRow(refFrame+prediction_latency, tempVec)
	outputGlobal.synRoot:sampleRow(refFrame, tempVec)

	--local rot_y_ref=self.graph.id_rotY.rotY(tempVec)
	local rot_y_ref=self.graph.id_rotY.rotY2(tempVec)

	local zmp_ref=vector3()
	local com_ref=vector3()

	local vel
	-- foot decoding : see "foot encoding" for synthesis part.
	do
		local Y=pend.Y
		-- use interpolated zmp and com (using frame0 and frame1 and weight)
		local f0=frame0+prediction_latency
		local f1=frame1+prediction_latency
		--if refFrame==self.numFrames and isL=='L' then print(refFrame, self.numFrames, frame0, frame1, refFrame, weight) end

		Y:sampleRow(f0, tempVec)
		local zmp1, zmpo1=pend.__getPosition(tempVec)
		self:projectToTerrain(zmp1,zmpo1)
		Y:sampleRow(f1, tempVec)
		local zmp2, zmpo2=pend.__getPosition(tempVec)	 
		self:projectToTerrain(zmp2, zmpo2)
		--Y:sampleRow(refFrame+prediction_latency, tempVec)
		Y:sampleRow(f0*(1-weight)+f1*weight,tempVec)
		local zmp, zmpo=pend.__getPosition(tempVec)
		self:projectToTerrain(zmp, zmpo)

		-- local gamma=1.1
		-- weight=math.pow(weight, gamma)

		local rot_y1, rot_y2
		if true then -- use rot_y sampled at the ref frame for the foot.
			outputGlobal.synRoot:sampleRow(f0, tempVec)
			rot_y1=self.graph.id_rotY.rotY2(tempVec)
			outputGlobal.synRoot:sampleRow(f1, tempVec)
			rot_y2=self.graph.id_rotY.rotY2(tempVec)
			if self.mapIndex==1 then
				RE.output2('refFrame'..isL, refFrame, frame0, frame1, weight,f0*(1-weight)+f1*weight)--, rot_y1, rot_y2)
			end
		end

		local tcom
		if useShearedTransform then
			com1=zmp1+zmpo1*pend.pole:localCOM()
			com2=zmp2+zmpo2*pend.pole:localCOM()
			tcom=vector3()
			tcom:interpolate(weight, com1, com2)
			com_ref:assign(zmp+zmpo*self.pendulum.pole:localCOM())

		end

		if not useCase.noIntersectionPrevenction then
			zmp1=zmp1+com_ref-tcom
			zmp2=zmp2+com_ref-tcom
		end

		local simfoot=self.footStates[isL]

		if simfoot then
			--dbg.namedDraw('Coordinate', transf(rot_y1, simfoot.translation), 'simfoot'..isL, vector3(0,0.05,0))
			-- simfoot.translation=zmp_coord_ref*footlocal
			--
			-- (R t) l = f
			-- (0 1)
			local l1=footlocal1:copy()
			l1:rotate(rot_y1)
			local l2=footlocal2:copy()
			l2:rotate(rot_y2)
			local zmp1_actual=simfoot.translation-l1
			local zmp2_actual=simfoot.translation-l2
			zmp1_actual.y=0
			zmp2_actual.y=0
			local prevContact=self.footStates['prevContact'..isL]
			local isContact=self.footStates['isContact'..isL]
			if useCase.footPosFeedbackMethod==1 then

				--if isL=="L" then
				if not cleanVisualize then
					--dbg.namedDraw('Sphere',zmp1*100+vector3(0,0,0), "zmp1"..isL, "red")
					--dbg.namedDraw('Sphere',zmp2*100+vector3(0,0,0), "zmp2"..isL, "red")
					dbg.namedDraw('Sphere',zmp2_actual*100+vector3(0,0,0), "zmp2act"..isL, "green")
				end
				if refFrame>69 and isContact and not prevContact then
				end
				if isContact 
					and time0 == time1  -- support foot
					then
					--RE.output2('simfoot'..isL, self.footStates['isContact'..isL])
					zmp1.x=zmp1_actual.x
					zmp1.z=zmp1_actual.z
					zmp2.x=zmp2_actual.x
					zmp2.z=zmp2_actual.z
				else
					--RE.output2('simfoot'..isL, '_')
				end
			elseif useCase.footPosFeedbackMethod==2 then
				-- always use predicted positions
				if isL=="LH"  then
					if not finalRender then
						dbg.namedDraw('Sphere',zmp1_actual*100+vector3(0,0,0), "zmp1act"..isL, "green")
					end
				end
				-- strideEstimation에서는 time0==time1조건이 사용됨
				if time0>=time1 then
				--if time0==time1 then
					-- 발이 다면 더이상 플래닝하지 않는다.
					-- actualValid: 0 or 1 or -1. only 1 means zmp1_actual is valid.
					local av=self.footStates['actualValid'..isL]
					if (av==nil or av<=0) then
						if isContact then
							self.footStates['actualValid'..isL]=1
							self.footStates['lastSpprt'..isL]=zmp1_actual:copy()
							if isL=="LH"  then
								if not finalRender then
								dbg.namedDraw('Sphere',zmp1_actual*100+vector3(0,0,0), "lastspprt"..isL, "red")
							end
							end
						else
							self.footStates['actualValid'..isL]=0 -- initialize
						end
					end
					RE.output2('actualValid'..isL,'PlannedSpprt', time0, time1)
					if self.footStates['actualValid'..isL]==1 then
						local spprtZmp=self.footStates['lastSpprt'..isL]
						zmp1=spprtZmp:copy()
						zmp2=spprtZmp:copy()
						RE.output2('actualValid'..isL,'LastSpprt', time0, time1)
					end
				else
					if self.footStates['actualValid'..isL]==1 then 
						self.footStates['actualValid'..isL]=-1
					end

					if self.footStates['actualValid'..isL]==-1 then
						local spprt=self.footStates['lastSpprt'..isL]
						assert(spprt)
						zmp1=spprt:copy()
					end
					RE.output2('actualValid'..isL,'Swing', time0, time1)
				end
				self:projectToTerrain(zmp1)
				self:projectToTerrain(zmp2)

			elseif useCase.footPosFeedbackMethod==3 then

				if time0~=time1 then
					-- swing foot
					if refTime>(time0+time1)/2 then
						RE.output2('simfoot'..isL, 'case1', self.footStates['isContact'..isL])
						if isContact then
							-- early touch down
							--zmp1=zmp1_actual
							--zmp2=zmp2_actual
						else
							-- not yet touch down
							zmp1=zmp1_actual 
						end
					else
						RE.output2('simfoot'..isL, 'case2', self.footStates['isContact'..isL])
						if isContact then
							--  not yet touch off
							zmp1=zmp1_actual
						else
							zmp1=zmp1_actual
						end
					end
				else
					-- support foot
					if refTime>time0 then
						--dbg.namedDraw('Coordinate', transf(rot_y1, simfoot.translation), 'simfoot'..isL, vector3(0,0.05,0))
						RE.output2('simfoot'..isL, 'case3', self.footStates['isContact'..isL])
						if isContact then
							zmp1=zmp1_actual
							zmp2=zmp2_actual
						else
							-- early foot touch off
							zmp1=zmp1_actual
							zmp2=zmp2_actual
						end
					else
						RE.output2('simfoot'..isL, 'case4', self.footStates['isContact'..isL], f,l, time0)
						if isContact then
							-- early touch down: overwrite predicted with actual
							if true then
								if l>=time0 then -- R or L (not RL and LR)
									assert(f<=time0) 
									zmp1=zmp1_actual
								end
							else
								zmp1=zmp1_actual
							end
							zmp2=zmp2_actual	
						else
							-- use predicted pos
						end
					end
				end
			end
		end

		if isStrideEstimation then
			return zmp1:distance(zmp2), zmp2
		end
		
		rot_y_ref:safeSlerp(rot_y1,rot_y2, weight)
		zmp_ref:interpolate(weight, zmp1, zmp2)

		if frame1>frame0 then
			vel=(zmp2-zmp1)/(frame1-frame0)
		else
			vel=vector3(0,0,0)
		end
	end

	if isStrideEstimation then
		return 0, zmp
	end

	local zmp_coord_ref

	if useShearedTransform then
		zmp_coord_ref= matrix4(quater(1,0,0,0), zmp_ref)*CT.shearY(com_ref-zmp_ref)*matrix4(rot_y_ref, vector3(0,0,0))
		--assert(isSimilar(zmp_coord_ref:getTranslation(), zmp_ref))
	else
		zmp_coord_ref =transf(rot_y_ref, zmp_ref)
	end

	-- if outddd then
	--    util.outputToFile("ddd.txt", "b"..tostring(fixed)..":roty:"..tostring(rot_y_ref)..
	-- 		  ":zmp12:"..tostring(zmp1)..tostring(zmp2)..":w:"..tostring(weight)
	-- 	 .." "..time0.." "..frame1)
	-- end

	if not cleanVisualize then -- turn on visualization 
		if useShearedTransform then
			-- sheared tf mat
			local v1=zmp_coord_ref:getTranslation()
			local v2=zmp_coord_ref*vector3(0,0,1)
			local coordL=CT.transfIdentity() 
			coordL.translation:assign(v1)
			coordL.rotation:setAxisRotation(vector3(0,1,0),vector3(0,0,1), v2-v1)
			if importance==1 then
				if not finalRender then
				dbg.drawCoordinate(self.objectList, coordL, "zmp_coord_"..isL)
			end
			end
		else
			dbg.drawCoordinate(self.objectList, zmp_coord_ref, "zmp_coord_"..isL)
		end
	end


	-- foot position error feedback
	local method=useCase.useFootEncodingMethod2
	if pelvis and method and method[1] then
		local function calcFootOri(isL, zmp_ref)
			--return pelvis.rotation*outputLocal:sampleQ(refTime, 'zmpcomToFoot'..isL)
			--local comCurr=vector3()
			--do
				--local tempVec=vectorn()
				--self.pendulum.Y:sampleRow(refFrame, tempVec)
				--local zmp, zmpo=self.pendulum.__getPosition(tempVec)
				--comCurr:assign(zmp+zmpo*self.pendulum.pole:localCOM())
			--end
			local comCurr=com_ref
			local shearRot=quater()
			shearRot:axisToAxis(vector3(0,1,0), comCurr-zmp_ref)
			local footOriRefCoord=shearRot*rot_y_ref
			-- zmpcomToFoot=footOriRefCoord:inverse() * globalFootOri
			local footlocalori=outputLocal:sampleQ(refTime, 'zmpcomToFoot'..isL)

			local idntf='mocapOriMod'..isL
			if useCase.keyframes[idntf] then
				local footlocalori_mocapmod=outputLocal:sampleKey(refTime,idntf )
				local ori_mocapmodQ=quater()
				ori_mocapmodQ:setRotation(footlocalori_mocapmod)
				footlocalori:leftMult(ori_mocapmodQ)
			end
			if useCase.footlocalori_modification then
				footlocalori:leftMult(useCase.footlocalori_modification)
			end
			if false then --debug draw
				if(refFrame==self.numFrames and string.sub(isL,2,2)~="H") then
					dbg.namedDraw('Sphere',comCurr*100+vector3(30,0,0), "comCurr"..isL, "green")
					dbg.namedDraw('Sphere',zmp_ref*100+vector3(30,0,0), "zmp_ref"..isL, "green")
					local coord=transf()
					coord.translation:assign(zmp_ref)
					coord.rotation:assign(footOriRefCoord)
					dbg.namedDraw('Coordinate', coord, "foot_ori_coord_"..isL,vector3(0.3,0,0))
					coord.rotation:assign(outputLocal:sampleQ(refTime, 'zmpcomToFoot'..isL))

					dbg.namedDraw('Coordinate', coord, "foot_ori_lcoord_"..isL,vector3(0.4,0,0))
				end
			end
			return footOriRefCoord*footlocalori
		end
		return zmp_coord_ref* footlocal, calcFootOri(isL, zmp_ref)
	end
	return zmp_coord_ref*footlocal, quater(1,0,0,0)
	--return zmp_coord_ref*vector3(0,0,0)
end

  function OnlineSynthesis2:generatePose(currFrame, latency, outputPose, solveIK, footCorrection)

	if footCorrection==nil then footCorrection=1 end

	local prediction_latency
	if solveIK then
		prediction_latency=PredictionLatency
	else
		prediction_latency=0
	end

	local refFrame=currFrame+latency
	local refTime=self.outputGlobal.refTime(refFrame)
	local rot_y

	if solveIK then
		rot_y=self:calcRotY2(refFrame)
	else
		rot_y=self:calcRotY(refFrame)
	end

	local prevRefTime=self.outputGlobal.refTime(math.max(refFrame-1,0))
	
	--	if not solveIK then print('refTime:genPose', refTime) end
	self.outputLocal:samplePose(refTime, outputPose)

	--##dos if g_debugOneStep then	--##dos g_debugOneStep:pushBack({"gp",{rot_y, refTime, outputPose:copy()}})	--##dos end
	-- if       debugRestoreStates then	--    if debugRestoreStates.generatePose==nil then	--       debugRestoreStates.generatePose=array:new() 	--    end	--    local gp={}	--    gp.synLMot=self.outputLocal.synLMot:matView():copy()	--    gp.rot_y=rot_y:copy()	--    gp.refTime=refTime	--    gp.outputPose=outputPose:copy()	--    debugRestoreStates.generatePose:pushBack(gp)	-- end

	self.graph.id.recoverPose(outputPose) -- 

	--##dos if g_debugOneStep then	--##dos --      g_debugOneStep:pushBack({"synLmot",self.outputLocal.synLMot:matView():copy()})	--##dos g_debugOneStep:pushBack({"gp2",{rot_y:copy(), refTime, outputPose:copy()}})	--##dos end

	local zmp_coord -- for pelvis
	do
		local zmp, zmpOri
		zmp, zmpOri=self.pendulum:_getPosition(refFrame)
		self:projectToTerrain(zmp, zmpOri)
		--local zmp2
		--zmp2, zmpOri=self.pendulum:_getPosition(math.min(refFrame, self.pendulum.Y:rows()-1))
		--zmpOri:scale(0.8)
		zmp_coord=transf(zmpOri*rot_y, zmp)
	end


	local zmpToPelvis=transf(self.outputLocal:sampleQ(refTime, 'zmpToPelvisR'),
	self.outputLocal:sampleV3(refTime, 'zmpToPelvisT'))



	--##dos if g_debugOneStep then --##dos g_debugOneStep:pushBack({"zcc",{zmp_coord.rotation:copy(), zmp_coord.translation:copy(), zmpToPelvis.rotation:copy(), zmpToPelvis.translation:copy()}}) --##dos end


	--if self.outputLocal.segInfo(math.floor(refTime)).seg.usePositionControl then
	--useStateDependentDesiredPose=false
	--end


	--local useStateDependentDesiredPose=true
	--end
	if solveIK==true then

		local pelvis
		-- how to calculate pelvis position
		pelvis=zmp_coord:toGlobal(zmpToPelvis)

		do
			-- usePositionControl or not
			local refTime=self.outputGlobal.refTime(currFrame)
			self.footStates.usePositionControl= self.outputLocal.segInfo(math.floor(refTime)).seg.usePositionControl 
		end

		local impL,impR, impLH, impRH, sppimpL, sppimpR, sppimpLH, sppimpRH=unpack(self.imp)
		assert(impL)
		local lfootPos,lfootOri=self:predictFootPosition(pelvis,currFrame, "L", prediction_latency,impL, false, usePositionControl, sppimpL)
		local rfootPos,rfootOri=self:predictFootPosition(pelvis,currFrame, "R", prediction_latency,impR, false, usePositionControl, sppimpR)
		RE.output("footori",
		lfootOri:rotationAngleAboutAxis(vector3(0,1,0)),
		rfootOri:rotationAngleAboutAxis(vector3(0,1,0)))

		local lhandPos, rhandPos, lhandOri, rhandOri
		local useHand=useCase.keyframes.importanceLH
		if useHand then
			lhandPos,lhandOri=self:predictFootPosition(pelvis,currFrame, "LH", prediction_latency,impLH, false, usePositionControl, sppimpLH)
			rhandPos,rhandOri=self:predictFootPosition(pelvis,currFrame, "RH", prediction_latency,impRH, false, usePositionControl, sppimpRH)
		end
		local frameRate=self.outputGlobal:frameRate()
		local delta_t=1/frameRate      

		--      RE.output("lpos", "...")
		--      RE.output("rpos", "...")


		do
			
			--##dos if g_debugOneStep then --##dos g_debugOneStep:pushBack({"coord",timeL, timeR, refCoordL, refCoordR, fcL, iqL, fcR, iqR}) --##dos end


			local outputLocal=self.outputLocal

			--local lfn=fcL*lfootPos
			--local rfn=fcR*rfootPos
			--lfn.y=math.min(lfn.y, lfootPos.y)
			--rfn.y=math.min(rfn.y, rfootPos.y)
			local lfn=lfootPos
			local rfn=rfootPos

			local lhn=lhandPos
			local rhn=rhandPos

			local headn=vector3(0,0,0)

			--			if useCase.usePositionControl then

			if currFrame==self.numFrames then
				RE.output2("footPos", lfootPos, rfootPos)
			end

			local function modifyFoot()
				local function adjustFootPosition()
					if footCorrection ~=0 then

						local rot_y2=rot_y:copy()
						local scl=1

						--print('ts',self.timescale)
						local fc_weight=footCorrection*scl*self.timescale
						if footLcorrection~=nil then
							lfn=lfn+rot_y2*footLcorrection*fc_weight
						end
						if footRcorrection~=nil then
							rfn=rfn+rot_y2*footRcorrection*fc_weight
						end
						if lhn and handLcorrection~=nil then
							lhn=lhn+rot_y2*handLcorrection*fc_weight
						end
						if rhn and handRcorrection~=nil then
							rhn=rhn+rot_y2*handRcorrection*fc_weight
						end
						if head_mod then
							headn=headn+rot_y2*head_mod
						end
						--if useCase.useAnalyticIK then
						--self.ik:solve(rot_y, pelvis, outputPose, {lfn, impL}, {rfn,impR} )
						--else
						--if useHand then
						--self.ik.solver:_solve(IKsolver.modes.fixedRoot,rot_y, pelvis, outputPose, {lfn, impL}, {rfn,impR}, {lhn, impLH}, {rhn, impRH} )
						--else
						--self.ik.solver:_solve(IKsolver.modes.fixedRoot,rot_y, pelvis, outputPose, {lfn, impL}, {rfn,impR} )
						--end
						--end
						--[[if useCase.asymmetricFoot then
							local asf=useCase.asymmetricFoot * self.outputLocal:sampleKey(refTime, 'asymmetricFootMask')
							if useCase.asymmetricFootTimeDependent then
								asf=asf+(useCase.asymmetricFootTimeDependent* self.outputLocal:sampleKey(refTime, 'asymmetricFootBasis'))
							end
							lfn=lfn+rotate(asf, rot_y2)
							rfn=rfn+rotate(vector3(-asf.x, asf.y, asf.z), rot_y2)
						end]]--
					end
				end

				local mocapmod_coef=1
				--if self.simulator then

				if footLmocapCorrection~=nil then
					lfn=lfn+rot_y*footLmocapCorrection*mocapmod_coef
				end
				if footRmocapCorrection~=nil then
					rfn=rfn+rot_y*footRmocapCorrection*mocapmod_coef
				end
				--RE.output2("lfn_rfn", lfn:copy(), rfn:copy())
				-- additional inputs for flight phase IK
				useCase._tempIKvar=self.desiredFootPosFlightPhase 
				useCase._tempDState=self.dtheta

				if currFrame==self.numFrames then
					RE.output2("footPos2", lfn, rfn)
				end

				--if lfootOri then
				if lfootOri then
					-- lfn, rfn: projected global pos
					-- convert to body-fixed global pos

					MotionDOF.setRootTransformation(outputPose, pelvis)
					if not useCase.noCOMjoint then	
						-- remove rootjoint
						outputPose:assign(MotionDOF.mergeRoot(outputPose))
						pelvis=MotionDOF.rootTransformation(outputPose)
					end
					local skel=self.graph.skel_withoutCOMjoint 
					skel:setPoseDOF(outputPose)

					local convert=self.convertConPos

					local function analyticSolveIK(nosolve)
						local mlfn=convert("L",lfn, self.graph.lfootpos, lfootOri)
						local mrfn=convert("R",rfn, self.graph.rfootpos, rfootOri)

						if not cleanVisualize then
							if not nosolve then
								dbg.draw('Sphere', mlfn*100, "mlfn")
								dbg.draw('Sphere', mrfn*100,"mrfn")
							end
							--end
							--##dos if g_debugOneStep then --##dos g_debugOneStep:pushBack({"aIK",lfn:copy(), rfn:copy(), mlfn:copy(), mrfn:copy(), lfootOri:copy(), rfootOri:copy()}) end

							--RE.output2("mlfn_mrfn", mlfn:copy(), mrfn:copy())
						end
						local skel=self.skel_withoutCOMjoint
						local headindex=skel:getTreeIndexByVoca(MotionLoader.HEAD)
						local headBone=skel:VRMLbone(headindex)
						local head= headBone:getFrame()*headBone:localCOM()
						local headOri=headBone:getFrame().rotation:copy()
						--
						-- local mlfn=lfn
						-- local mrfn=rfn
						local useHand=useCase.keyframes.importanceLH
						if useHand then
							local useHandHeuristic={}
							local mlhn=convert("LH",lhn, self.graph.lhandpos, lhandOri)
							local mrhn=convert("RH",rhn, self.graph.rhandpos, rhandOri)

							if useHandHeuristic then
								local skel=self.graph.skel_withoutCOMjoint
								skel:setPoseDOF(outputPose)
								useHandHeuristic.rhip_ori=skel:getBoneByVoca(MotionLoader.RIGHTHIP):getFrame().rotation:copy()
								useHandHeuristic.lhip_ori=skel:getBoneByVoca(MotionLoader.LEFTHIP):getFrame().rotation:copy()
							end
							
							if not nosolve then
								self.ik:solve3(rot_y, pelvis, outputPose, 
								{mlfn, lfootOri, impL}, 
								{mrfn, rfootOri, impR},
								{mlhn, lhandOri, impLH},
								{mrhn, rhandOri, impRH}
								)
							end
							self.ikInfo[self.mapIndex]=
							{
								{mlfn, lfootOri, impL,sppimpL}, 
								{mrfn, rfootOri, impR,sppimpR},
								{mlhn, lhandOri, impLH,sppimpLH},
								{mrhn, rhandOri, impRH,sppimpRH},
								{head+headn, headOri}
							}

							if useHandHeuristic then
								local skel=self.graph.skel_withoutCOMjoint
								skel:setPoseDOF(outputPose)
								useHandHeuristic.rhip=skel:getBoneByVoca(MotionLoader.RIGHTHIP):getFrame().rotation:copy()
								useHandHeuristic.lhip=skel:getBoneByVoca(MotionLoader.LEFTHIP):getFrame().rotation:copy()

								local rdiff=quater()
								local ldiff=quater()
								rdiff:difference(useHandHeuristic.rhip_ori, useHandHeuristic.rhip)
								ldiff:difference(useHandHeuristic.lhip_ori, useHandHeuristic.lhip)
								--print(rdiff, ldiff)
							end
						else
							if not nosolve then
							--	RE.output2("imp", impL, impR)
								self.ik:solve3(rot_y, pelvis, outputPose, 
								{mlfn, lfootOri, impL}, 
								{mrfn, rfootOri, impR} )
								
								if false then --debug visualization
									local skel=self.graph.skel_withoutCOMjoint
									skel:setPoseDOF(outputPose)
									local lfootPos=self.graph.bone2.lfoot:getFrame()*self.graph.lfootpos
									local rfootPos=self.graph.bone2.rfoot:getFrame()*self.graph.rfootpos

									dbg.namedDraw('Sphere', lfootPos*100, 'mlfnpos2')
									dbg.namedDraw('Sphere', rfootPos*100, 'mrfnpos2')
								end
							end
							self.ikInfo[self.mapIndex]=
							{
								{mlfn, lfootOri, impL, sppimpL}, 
								{mrfn, rfootOri, impR, sppimpR},
								{head+headn, headOri}
							}
						end
					end


					local desiredCOM
					do -- calc desiredCOM
						--local pend=self.pendulum
						--local Y=pend.Y
						---- use zmp at refFrame
						--local tempVec=vectorn()
						--local refFrame=currFrame+latency
						--Y:sampleRow(refFrame, tempVec)
						--local zmp, zmpo=pend.__getPosition(tempVec)
						--self:projectToTerrain(zmp, zmpo)
						--local pendCOM=zmp+zmpo*pend.pole:localCOM()
						--local comToCOM=self.outputLocal:sampleV3(refTime, 'comToCOM')
						--desiredCOM=pendCOM+rotate(comToCOM,rot_y) 
						---- print(pendCOM.y,desiredCOM.y, comToCOM.y)
					end

					analyticSolveIK()
					----modifyCOM(outputPose,  desiredCOM, (1-footCorrection))
					--modifyCOM(outputPose,  desiredCOM, 1)
					if footCorrection ~=0 then
						adjustFootPosition()
						--assert(useCase.modifyFootSim) walk3 uses this option set off.
						if useCase.modifyFootSim==true then --and self.simulator then
							analyticSolveIK(false) 
						else
							analyticSolveIK(true) -- optimized modifications are not applied to the desired motion.
						end
					end
				else
					local useHand=useCase.keyframes.importanceLH
					if useHand then
						self.ik:solve(rot_y, pelvis, outputPose, {lfn, impL}, {rfn,impR}, {lhn, impLH}, {rhn, impRH} )
					else
						self.ik:solve(rot_y, pelvis, outputPose, {lfn, impL}, {rfn,impR} )
					end
				end
			end
			local function unmodifyFoot()
				if footCorrection ~=0 then
					local fc_weight=footCorrection
					if footLcorrection~=nil then
						lfn=lfn-rot_y*footLcorrection*fc_weight
					end
					if footRcorrection~=nil then
						rfn=rfn-rot_y*footRcorrection*fc_weight
					end
					if handLcorrection~=nil then
						lhn=lhn-rot_y*handLcorrection*fc_weight
					end
					if handRcorrection~=nil then
						rhn=rhn-rot_y*handRcorrection*fc_weight
					end
					if head_mod then
						headn=headn-rot_y*head_mod
					end
				end

				if footLmocapCorrection~=nil then
					lfn=lfn-rot_y*footLmocapCorrection
				end
				if footRmocapCorrection~=nil then
					rfn=rfn-rot_y*footRmocapCorrection
				end
				--[[if footCorrection ~=0 then
					if useCase.asymmetricFoot then
						local asf=useCase.asymmetricFoot 
						if useCase.asymmetricFootTimeDependent then
							asf=asf+(useCase.asymmetricFootTimeDependent* self.outputLocal:sampleKey(refTime, 'asymmetricFootBasis'))
						end
						lfn=lfn-rotate(asf, rot_y)
						rfn=rfn-rotate(vector3(-asf.x, asf.y, asf.z), rot_y)
					end
				end]]--
			end


			if self.desiredFootPosFlightPhase then
				--assert (not useHand)

				local lpelvis, frameL, frameR, frameLH, frameRH=self:calcOrigFootCoord(rot_y, pelvis, outputPose, useHand)
				--pelvis=lpelvis
				local orig_lfn=frameL:toGlobalPos(self.graph.lfootpos)
				local orig_rfn=frameR:toGlobalPos(self.graph.rfootpos)
				local origComPos=self.skel:calcCOM()-- simulator:calculateCOM(0)
				local orig_mid=(orig_lfn+orig_rfn)*0.5


				local tempPose=outputPose:copy()
				MotionDOF.setRootTransformation(tempPose, pelvis)
				self.skel:setPoseDOF(tempPose)
				local currComPos=self.skel:calcCOM()-- simulator:calculateCOM(0)
				local curr_lfn=frameL:toGlobalPos(self.graph.lfootpos)
				local curr_rfn=frameR:toGlobalPos(self.graph.rfootpos)
				local desiredComToFoot=self.desiredFootPosFlightPhase.desiredfootPosLocal
				local flightTime=self.desiredFootPosFlightPhase.flightTime
				if desiredComToFoot then

					--local cor=-orig_mid+currComPos+desiredComToFoot
					local cor=-orig_mid+self.desiredFootPosFlightPhase.desiredfootPosGlobal
					cor:scale(flightTime)
					lfn=orig_lfn+ cor
					rfn=orig_rfn+ cor
					lfn:interpolate(flightTime, curr_lfn, lfn)
					rfn:interpolate(flightTime, curr_rfn, rfn)
					-- preserve original (com-foot).y
					lfn.y=orig_lfn.y+currComPos.y-origComPos.y+0.05--, orig_lfn.y)
					rfn.y=orig_rfn.y+currComPos.y-origComPos.y+0.05--, orig_rfn.y)

					if false then
						dbg.namedDraw('Sphere', origComPos*100, 'origComPos')
						dbg.namedDraw('Sphere', currComPos*100, 'currComPos')
						dbg.namedDraw('Sphere', orig_lfn*100, 'orig_lfn')
						dbg.namedDraw('Sphere', lfn*100, 'lfn2')
						dbg.namedDraw('Sphere', rfn*100, 'rfn2')
					end
				else 
					lfn=orig_lfn
					rfn=orig_rfn
					pelvis=lpelvis
				end
				modifyFoot()
			else
				RE.output2('self.targetHeight', self.targetHeight)

				local outputPose1, outputPose_orig


				if true then --useStateDependentDesiredPose then
					modifyFoot()
				end


				if false then --not useStateDependentDesiredPose then

					local lpelvis, frameL, frameR, frameLH, frameRH=self:calcOrigFootCoord(rot_y, pelvis, outputPose, useHand)
					pelvis=lpelvis
					lfn=frameL:toGlobalPos(self.graph.lfootpos)
					rfn=frameR:toGlobalPos(self.graph.rfootpos)
					local useHand=useCase.keyframes.importanceLH
					if useHand then
						lhn=frameLH:toGlobalPos(self.graph.lhandpos)
						rhn=frameRH:toGlobalPos(self.graph.rhandpos)
					end
					if false then
						dbg.namedDraw('Sphere', lfn*100, 'lfn')
						dbg.namedDraw('Sphere', rfn*100, 'rfn')
						dbg.namedDraw('Sphere', pelvis.translation*100, 'pfn')
					end
					modifyFoot()
				end

			end

			do
				-- summarize importance
				local footInfo=self.ikInfo[1]
				local supportFootImportance
				local limbImportance
				local useHand=useCase.keyframes.importanceLH
				if useHand then
					supportFootImportance={L=footInfo[1][4],R=footInfo[2][4], LH=footInfo[3][4], RH=footInfo[4][4]}
					limbImportance={L=footInfo[1][3], R=footInfo[2][3], LH=footInfo[3][3], RH=footInfo[4][3]}
				else
					supportFootImportance={L=footInfo[1][4],R=footInfo[2][4],}
					limbImportance={L=footInfo[1][3], R=footInfo[2][3],}
				end
				footInfo.supportFootImportance=supportFootImportance
				footInfo.limbImportance=limbImportance
			end

		end
	else
		local pelvis
		pelvis=zmp_coord:toGlobal(zmpToPelvis)
		do
			pelvis2=MotionDOF.rootTransformation(outputPose)		
			local origRotY=quater()
			local offset=quater()
			local origRotY2=quater()
			local offset2=quater()
			pelvis2.rotation:decompose(origRotY2, offset2)
			pelvis.rotation:decompose(origRotY, offset)
			pelvis.rotation:mult(origRotY,offset2)
			MotionDOF.setRootTransformation(outputPose, pelvis)      
		end
		if not useCase.noCOMjoint then	
			-- remove rootjoint
			outputPose:assign(MotionDOF.mergeRoot(outputPose))
			pelvis=MotionDOF.rootTransformation(outputPose)
		end
	end
end

function OnlineSynthesis2:calcOrigFootCoord(rot_y, pelvis, outputPose, useHand)
	local frameL, frameR
	local tempPose=outputPose:copy()
	local origRootTF=MotionDOF.rootTransformation(tempPose)
	local origRotY=quater()
	local offset=quater()
	origRootTF.rotation:decompose(origRotY, offset)
	origRootTF.rotation:mult(rot_y, offset)
	MotionDOF.setRootTransformation(tempPose, origRootTF)

	self.skel:setPoseDOF(tempPose)
	if useHand then
		return origRootTF, self.graph.lfoot:getFrame(), self.graph.rfoot:getFrame(),
		self.graph.lhand:getFrame(), self.graph.rhand:getFrame()
	end
	return origRootTF, self.graph.lfoot:getFrame(), self.graph.rfoot:getFrame()
end


function OnlineSynthesis2:debugOut()
	if self.simulator then
		self.simulator:drawDebugInformation()
	end
	local mrdCF=mrd_info.outputContactForce

	if mrdCF then
		local cfCurrFrame=mrdCF[1]:rows()
		
		do 
			local fileCount=math.floor(self.numFrames/mrd_info.export_freq)

			-- mrdCF[1]:resize(cfCurrFrame+1,6)
			-- local frameRate=self.outputFrameRate
			-- local simulFrameRate=model.simulationFrameRate   
			-- local simulFrames=simulFrameRate/frameRate   
			-- mrdCF[1]:row(cfCurrFrame):setVec3(0, mrdCF[2][1]/simulFrames)
			-- mrdCF[1]:row(cfCurrFrame):setVec3(3, mrdCF[2][2]/simulFrames)

			-- mrdCF[2][1]:assign(vector3(0,0,0))
			-- mrdCF[2][2]:assign(vector3(0,0,0))

			if mrdCF[3]==nil then
				mrdCF[3]=array:new()
			end
			local dump=TStrings()
			RE.dumpOutput(dump,1)
			local pn=dump:size()
			local dump2=TStrings()
			RE.dumpOutput(dump2,2)
			RE.outputEraseAll(2) 

			dump:resize(dump:size()+dump2:size())
			for i=0, dump2:size()-1 do
				dump:set(i+pn, dump2(i))
			end
			mrdCF[3]:pushBack(dump)

			if math.mod(self.numFrames, mrd_info.export_freq)==0 then
				local binaryFile=util.BinaryFile()
				binaryFile:openWrite(string.sub(mrd_info.filename,1,-5).."_"..fileCount..".cf")
				binaryFile:pack(mrdCF[1])
				local nf=mrdCF[3]:size()
				binaryFile:packInt(nf)
				for i=1, nf do
					binaryFile:pack(mrdCF[3][i])
				end
				binaryFile:close()
				mrd_info.outputContactForce={matrixn(), {vector3(0,0,0), vector3(0,0,0), vector3(0,0,0)}}
			end	 
		end
	end

end
