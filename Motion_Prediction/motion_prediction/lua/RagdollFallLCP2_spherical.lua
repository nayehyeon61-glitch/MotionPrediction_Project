require("config")

package.projectPath='../Samples/QP_controller/'
package.resourcePath='../Resource/motion/'
package.path=package.path..";../Samples/QP_controller/lua/?.lua" --;"..package.path
package.path=package.path..";../Samples/classification/lua/?.lua" --;"..package.path
package.path=package.path..";../Resource/classification/lua/?.lua" --;"..package.path
require("module")
package.path=package.path..";../lua/?.lua" --;"..package.path
package.path=package.path..";../../taesooLib/Samples/scripts/RigidBodyWin/subRoutines/?.lua" --;"..package.path
require("Timeline")

--require("PDservo_spherical")
require("PDservo_spd")

require("IPC_based/common")
require("useCasesMuscle")

local MS=require('subRoutines/MuscleSim')

do 
	model=model_files.full_ipfrun_repeat
	model.rendering_step=1/30 -- real-time speed
	-- muscle is simulated at 840hz so use an integer multiple
	model.timestep=1/210 -- when require('PDservo_spd') that works only for physX
	model.frame_rate=30 -- mocap frame rate
	model.k_p_PD=0
	model.k_d_PD=1
	--model.numSolverIterations=2000
end
noControl=false





EVR=LUAclass(EventReceiver)
function EVR:__init()
end
function EVR:onFrameChanged(win, iframe)
	if mLoader~=nill and this:findWidget("simulation"):checkButtonValue() then
		local timer=util.PerfTimer2()--1, niter.."simulation")
		timer:start()
		local niter=math.floor(model.rendering_step/model.timestep+0.5)
		mRagdoll:frameMove(niter)
		RE.output2("framemoveTime", timer:stop(), niter)
	end
end
function ctor()
	RE.ogreSceneManager():setFogNone()
	mEventReceiver=EVR()
   --	this:create("Button", "Start", "Start")
   --	this:widget(0):buttonShortcut("FL_ALT+s")

   this:create("Check_Button", "simulation", "simulation", 0, 2,0)
   this:widget(0):checkButtonValue(1) -- 1 for imediate start
   this:widget(0):buttonShortcut("FL_ALT+s")
   
   this:create("Button", "single step", "single step", 2, 3,0)
   
   this:create("Check_Button", "draw skeleton", "draw skeleton", 0, 3,0)
   this:widget(0):checkButtonValue(0)

   this:create("Button", "debug LoaderToTree class", "debug LoaderToTree class")
   this:create("Button", "debug get/setlink", "debug get/setlink")
   this:create("Button", "debug get/setlink2", "debug get/setlink2")
   this:create("Button", "getSphericalQ", "getSphericalQ")
   this:create("Button", "throw a ball", "throw a ball")

   this:updateLayout()
   this:redraw()
   
   RE.viewpoint().vpos:assign(vector3(330.411743, 69.357635, 0.490963))
   RE.viewpoint().vat:assign(vector3(-0.554537, 108.757057, 0.477768))
   RE.viewpoint():update()
   RE.viewpoint():TurnRight(math.rad(viewRotate or 0))
   _start()
end

function dtor()
   -- remove objects that are owned by C++
   if mSkin~=nill then
      RE.remove(mSkin)
      mSkin=nil
   end
   if mSkin2~=nill then
      RE.remove(mSkin2)
      mSkin2=nil
   end
   if mTimeline then
	   mTimeline:dtor()
       mTimeline=nil
   end
   -- remove objects that are owned by LUA
   collectgarbage()
end

function _start()
	dtor()
	mTimeline=Timeline("Timeline", 1000000, 1/30)
	RE.motionPanel():motionWin():playFrom(0)
	print("start")
	mLoader=MainLib.VRMLloader(model.file_name)
	mLoaderFixedJoints={}
	for i=1, mLoader:numBone()-1 do
		local bone=mLoader:VRMLbone(i)
		if bone:HRPjointType(0)==MainLib.VRMLTransform.FIXED then
			mLoaderFixedJoints[bone:name()]={ bone:parent():name(), bone:getOffset() }
		end
	end
	mLoader:removeAllRedundantBones()
	mLoader:printHierarchy()
	if model.mot_file~=nill then
		local container=MotionDOFcontainer(mLoader.dofInfo, model.mot_file)
		mMotionDOF=container.mot
	end

	if model.useThreeDOFs then
		-- convert some joints to 3DOFs (YZX)
		local srcMotion=Motion( mMotionDOF)
		for i,k in ipairs(model.useThreeDOFs) do
			mLoader:setChannels(mLoader:getBoneByName(k), '', 'YZX')
		end
		mMotionDOF=MotionDOF(mLoader.dofInfo)
		mMotionDOF:set(srcMotion)
	end
	mFloor=MainLib.VRMLloader("../Resource/mesh/floor_y.wrl")

	drawSkeleton=this:findWidget("draw skeleton"):checkButtonValue()

	local simulator

	-- 360 hz
	--simulator= Physics.DynamicsSimulator_Trbdl_LCP('libccd') simulator:setParam_R_B_MA(0.05,1,0.05) -- always necessary.
	simulator= Physics.DynamicsSimulator_Trbdl_impulse('libccd') simulator:setParam_restitution_MA(0,0.05) -- always necessary.
	--simulator= Physics.DynamicsSimulator_Trbdl_impulse('libccd') simulator:setParam_restitution_MA(0,0.05) -- always necessary.
	-- need to use 1200 hz
	--simulator= Physics.DynamicsSimulator_physX()

	local simulatorParam={
		timestep=model.timestep,
		integrator=Physics.DynamicsSimulator.EULER,
		debugContactParam={10, 0, 0.01, 0, 0}, -- size, tx, ty, tz, tfront
	}
	mRagdoll= RagdollSim(mLoader, drawSkeleton, mMotionDOF, simulator, simulatorParam)
	mRagdoll.drawDebugInformation=true

	mSkin2=RE.createVRMLskin(mFloor, false)
	mSkin2:scale(100,100,100)
end

function onCallback(w, userData)
	if w:id()=="Start" then
		_start()
	elseif w:id()=="getSphericalQ" then
		local sim=mRagdoll.simulator
		local q=vectorn()
		local dq=vectorn()
		sim:getSphericalState(0, q, dq)

		sim:setSphericalState(0, q, dq)

		local q2=vectorn()
		local dq2=vectorn()
		sim:getSphericalState(0, q2, dq2)

	elseif w:id()=="debug get/setlink" then
		-- just for testing and debugging.
		local theta=vectorn()
		local dtheta=vectorn()
		local sim=mRagdoll.simulator
		sim:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, theta);
		sim:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dtheta);
		sim:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, theta);
		sim:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dtheta);
		sim:initSimulation()
		local theta2=vectorn()
		local dtheta2=vectorn()
		sim:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, theta2);
		sim:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dtheta2);
		local delta=dtheta-dtheta2
		assert(delta:maximum()<0.0001)
		assert(delta:minimum()>-0.0001)
		print('delta1:', delta)
		sim:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, theta2);
		sim:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dtheta2);
		sim:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, theta);
		sim:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dtheta);
		local delta=dtheta-dtheta2
		assert(delta:maximum()<0.0001)
		assert(delta:minimum()>-0.0001)
		print('test passed.')
	elseif w:id()=="debug LoaderToTree class" then
		-- just for testing and debugging.
		local theta=vectorn()
		local dtheta=vectorn()
		local theta2=vectorn()
		local dtheta2=vectorn()
		local sim=mRagdoll.simulator
		sim:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, theta);
		sim:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dtheta);

		sim:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, theta);
		sim:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dtheta);

		sim:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, theta2);
		sim:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dtheta2);

		sim:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, theta2);
		sim:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dtheta2);

		local e=MotionUtil.Effectors()
		local c=MotionUtil.Constraints()
		local tree=MotionUtil.LoaderToTree(mLoader, e,c, false,false)
		tree:setPoseDOF(mLoader.dofInfo, theta)
		print('hihi')
		print(theta)
		print(dtheta)
		tree:setVelocity(mLoader.dofInfo, dtheta)
		tree:Print()

		local testPassed=true
		for i=1, mLoader:numBone()-1 do
			local wldst=sim:getWorldState(0):globalFrame(i)
			local wldst2=tree:getLastNode(i)._global
			if (wldst.translation- wldst2.translation):length()>0.0001 then
				testPassed=false
				print('test1 failed')
				dbg.console()
			end
		end
		for i=1, mLoader:numBone()-1 do
			local wldst=sim:getWorldState(0):globalFrame(i)
			local wldst2=tree:getLastNode(i)._global
			local angVel=sim:getWorldAngVel(0, mLoader:VRMLbone(i))
			local linVel=sim:getWorldVelocity(0, mLoader:VRMLbone(i), vector3(0,0,0))
			local bangVel=tree:getLastNode(i):bodyAngVel():copy()
			local blinVel=tree:getLastNode(i):bodyLinVel():copy()
			angVel:rotate(wldst2.rotation:inverse())
			linVel:rotate(wldst2.rotation:inverse())
			print(i)
			print(wldst)
			print(wldst2)
			print(angVel, bangVel)
			print(linVel, blinVel)
			if( angVel- bangVel):length()>0.0001 then
				testPassed=false
				dbg.console()
			end
			if( linVel- blinVel):length()>0.0001 then
				testPassed=false
				dbg.console()
			end
		end
		local dtheta2=vectorn(dtheta:size())
		tree:getVelocity(mLoader.dofInfo, dtheta2)
		local delta=dtheta-dtheta2
		assert(delta:maximum()<0.0001)
		assert(delta:minimum()>-0.0001)
		if testPassed then
			print("Test passed")
		end
	elseif w:id()=='debug get/setlink2' then
		testGetSetLink()
	elseif w:id()=='throw a ball' then
		local sim=mRagdoll.simulator
		if not g_balls then
			g_balls={}
			local ball=Geometry()
			ball:initEllipsoid(vector3(0.2,0.2,0.2))
			g_balls.skel=MainLib.VRMLloader(ball, false)
			g_balls.skel:setTotalMass(10)

			local box=Geometry()
			box:initBox(vector3(0.2,1,2))
			g_wall=MainLib.VRMLloader(box, true)

			g_wallSkin=RE.createVRMLskin(g_wall, false)
			g_wallSkin:setScale(100,100,100)
			sim:registerDynamicCharacter('wall', g_wall, CT.vec(-1,0.5,0, 1,0,0,0), CT.vec(0,0,0, 0, 0,0,0))
			sim:updateSkin('wall', g_wallSkin)
		end
		local name=RE.generateUniqueName()
		g_balls[name]=RE.createVRMLskin(g_balls.skel, false)
		g_balls[name]:setScale(100,100,100)
		sim:registerDynamicCharacter(name, g_balls.skel, CT.vec(2,0.7,0.1, 1,0,0,0), CT.vec(-5,0,0, 0, 0,0,0))
	end
end

function frameMove(fElapsedTime)
end

RagdollSim=LUAclass ()

function RagdollSim:__init(loader, drawSkeleton, motdof, simulator, simulatorParam)
	if drawSkeleton==nil then drawSkeleton = true end
	self.simulator=simulator
	self.muscleSim = MS(loader, model.luamsclpath, model)
	self.skin=RE.createVRMLskin(loader, drawSkeleton)
	self.skin:setThickness(0.03)
	self.skin:scale(100,100,100)

	-- adjust initial positions

	self.motionDOF=motdof
	self.simulationParam=simulatorParam

	self.controlforce=vectorn(loader.dofInfo:numActualDOF())
	if self.motionDOF then
		for i=0, self.motionDOF:numFrames()-1 do
			self.motionDOF:row(i):set(1,self.motionDOF:row(i):get(1)+(model.initialHeight or 0) )
		end
	else
		motdofc=MotionDOFcontainer(loader.dofInfo)
		motdofc:resize(10)
		for i=0, 9 do
			motdofc.mot(i):setAllValue(0)
			motdofc.mot(i):set(3, 1) -- assuming quaternion (free root joint)
		end
		self.motionDOF=motdofc.mot
		for i=0, self.motionDOF:numFrames()-1 do
			self.motionDOF:row(i):set(1,self.motionDOF:row(i):get(1)+(model.initialHeight or 0) )
		end

	end
	if model.initialAngle then
		local delta= quater(model.initialAngle, vector3(1,0,0) )
		for i=0, self.motionDOF:numFrames()-1 do
			self.motionDOF:row(i):setQuater(3, self.motionDOF:row(i):toQuater(3)*delta)
		end
	end

	self.motionDOF_euler=self.motionDOF
	self.motionDOF=nil

	self.loader_euler=loader
	loader=nil
	-- important!!!
	-- convert loader, motionDOF, and its time-derivative to new formats.
	self.loader=self.loader_euler:copy()
	self.loader:changeAll3DOFjointsToSpherical()
	for i=1, self.loader:numBone()-1 do
		print(self.loader:VRMLbone(i):numHRPjoints())
	end
	self.loader:printHierarchy()

	self.motQ, self.motDQ=PDservo_spherical.convertMotionState(self.loader_euler, self.loader, self.motionDOF_euler, model.frame_rate)

	self.motDDQ=self.motDQ:derivative(model.frame_rate)
	self.simulator:registerCharacter(self.loader)
	local floor=mFloor or VRMLloader("../Resource/mesh/floor_y.wrl")
	self.simulator:registerCharacter(floor)
	registerContactPairAll(model, self.loader, floor, self.simulator)   
	--registerContactPairOnly(model, self.loader, collisionBones, floor, self.simulator)   

	self.simulator:init(simulatorParam.timestep, simulatorParam.integrator)

	self.simulator:setSimulatorParam("debugContact", simulatorParam.debugContactParam) 
	self.simulator:setSimulatorParam("contactForceVis", {0.001,0.001,0.001})
	self.simulator:setSimulatorParam("penaltyDepthMax", {0.0005})
	k_p=model.k_p_PD	-- Nm/rad
	k_d=model.k_d_PD --  Nms/rad. worked in range [0, 1]
	self.pdservo=PDservo_spherical(self.loader.dofInfo)
	self.pdservo:initPDservo(model.start, self.motQ:rows(), 
	self.motQ, self.motDQ, simulator, 0)

	if self.motQ then
		model.start=math.min(model.start, self.motQ:rows()-1)
		initialState=vectorn()
		initialState:assign(self.motQ:row(model.start))

		print("initialState=",initialState)
		self.simulator:setSphericalState(0, 
		self.motQ:row(model.start),
		self.motDQ:row(model.start) )
		self.simulator:initSimulation()
	else
		local wldst=self.simulator:getWorldState(0)
		wldst:localFrame(self.loader:getBoneByTreeIndex(1)).rotation:identity()
		wldst:localFrame(self.loader:getBoneByTreeIndex(1)).translation:radd(vector3(0, model.initialHeight, 0))
		wldst:forwardKinematics()
		self.simulator:setWorldState(0)
	end
	--	debug.debug()
	self.simulator.setPose(self.skin,self.simulator,0)

	self.skin:setMaterial("lightgrey_transparent")

	--self.simulator.setGVector(vector3(0,0,9.8))
	self.simulator:setGVector(vector3(0,9.8,0))
	self.simulator:initSimulation()
	local q=vectorn()
	local dq=vectorn()
	local startq=self.simulator:getSphericalState(0, q, dq)
	self.muscleSim:initSimulation(self.simulator, 0)

	self.floor=floor -- have to be a member to prevent garbage collection
end
function RagdollSim:__finalize()
	-- remove objects that are owned by C++
	if self.skin~=nill then
		RE.remove(self.skin)
		self.skin=nil
	end
	self.simulator=nil

end
function RagdollSim:frameMove(niter)
	--assert(math.floor(niter)==niter)
	--		debug.debug()
	temp=vectorn()
	self.controlforce:zero()
	self.simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, temp)

	for iter=1,niter do
		do
			local maxForce=9.8*80
			if not self.pdservo:generateTorque(self.simulator, maxForce) then
				self.pdservo:rewindTargetMotion(self.simulator)
				self.pdservo:generateTorque(self.simulator, maxForce)
			end

			local controlforce=self.pdservo.controlforce
			if noControl then
				controlforce:setAllValue(0.0)
			end
			self.simulator:setTau(0, self.pdservo.controlforce)

			local q=self.pdservo.theta_d
			local temp_fk=self.loader:fkSolver()
			temp_fk:setSphericalQ(q)

			local pp=self.muscleSim.osim.pp
			local simPos=vector3N()
			local targetPos=vector3N()
			pp:_calcPathPointPositions(self.simulator:getWorldState(0), simPos)
			pp:_calcPathPointPositions(temp_fk, targetPos)

			local lm_sim=VectorXd()
			local lm_target=VectorXd()
			pp:_calcTendonMuscleLengths(simPos, lm_sim)
			pp:_calcTendonMuscleLengths(targetPos, lm_target)

			local muscle_kp=-10
			local delta=muscle_kp*(lm_target:vecView()-lm_sim:vecView())

			delta:clamp(0,1)

			self.muscleSim.osim.u:vecView():assign(delta)


			self.muscleSim:applyMuscleForces()
			self.simulator:stepSimulation()

			self.controlforce:radd(controlforce)
		end
		self.simulator.setPose(self.skin, self.simulator, 0)				
	end
	self.muscleSim:drawMuscles()
	if g_balls then
		for k,v in pairs(g_balls) do
			if k~='skel' then
				self.simulator:updateSkin(k, v)
				print(k)
			end
		end
	end

	self.controlforce:rdiv(niter)
end

function registerContactPairAll(model, loader, floor, simulator)
   param=vectorn ()
   param:setValues(0.5,0.5, model.penaltyForceStiffness, model.penaltyForceDamp)
   for i=1,loader:numBone()-1 do

      local bone_i=loader:VRMLbone(i)
      simulator:registerCollisionCheckPair(loader:name(),bone_i.NameId, floor:name(), floor:bone(1):name(), param)
   end
end
function registerContactPairOnly(model, loader, bones, floor, simulator)
   param=vectorn ()
   param:setValues(0.5,0.5, model.penaltyForceStiffness, model.penaltyForceDamp)
   for i=1,loader:numBone()-1 do

      local bone_i=loader:VRMLbone(i)
	  if bones[bone_i:name()] then
		  simulator:registerCollisionCheckPair(loader:name(),bone_i.NameId, floor:name(), floor:bone(1):name(), param)
	  end
   end
end

function handleRendererEvent(ev, x,y)
	return 0
end
