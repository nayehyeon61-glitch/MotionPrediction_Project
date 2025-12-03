
require("config")

package.projectPath='../Samples/QP_controller/'
package.path=package.path..";../Samples/QP_controller/lua/?.lua" --;"..package.path
package.path=package.path..";../Samples/classification/lua/?.lua" --;"..package.path
require("IPC_based/common")
require("IPC_based/useCases")

if useCase.unoptimized~=nil then
   useCase=useCase.unoptimized
end

require("subRoutines/Optimizer")

require("subRoutines/motiongraph")
require("IPC_based/LocoGraph")
require("subRoutines/CompareChain")

scenario=useCase.scenario
model=scenarios.toModel(scenario)

function init_globals()
end

require("IPC_based/LocoSynthesis")

PDservoLatency=0
PredictionLatency=0
outputSuperSample=1

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


function CartPoleController:__init()


	self.desiredVel={}
	RE.renderer():fixedTimeStep(false)


	OnlineSynthesis2.__init(self)


	--   self.controlskin=RE.createConnectedVRMLskin(self.skel, self.motionDOF)

	self.skin1=RE.createVRMLskin(self.skel, false)

	self.skin1:setMaterial("lightgrey_transparent")
	self.skin2:setMaterial("lightgrey_transparent")

	self.skin1:scale(100,100,100)
	--   self.skin1:setTranslation(0,100,0)

	RE.motionPanel():motionWin():changeCurrFrame(self.graph.initialSegment.first)

	self.fk=BoneForwardKinematics(self.skel)
	self.fk:init()
	self.compareChain=CompareChain(self.skel, self.fk, self.skel:fkSolver())


	self.controlforce=matrixn()

	do
		local pendOptimizationPath=useCase.pendOptimizationPath

		self.firstFrames=shallowCopyTable(pendOptimizationPath.firstFrames)
		self.sequence={}

		assert(table.getn(pendOptimizationPath.segments)==table.getn(pendOptimizationPath.firstFrames)-1)
		for i=1, table.getn(pendOptimizationPath.segments) do
			local prt=0
			--if pendOptimizationPath.pendRotRefTime then prt=pendOptimizationPath.pendRotRefTime[i] end
			self.sequence[i]={
				first=self.firstFrames[i], last=self.firstFrames[i+1], name=pendOptimizationPath.segments[i],-- pendRotRefTime=prt
			}
			self.desiredVel[self.sequence[i].name]=vector3(0,0,0)
		end

		self.firstFrame=self.firstFrames[1]
		self.lastFrame=self.firstFrames[table.getn(self.firstFrames)]

		do  -- calc inertia and COM height
			self.inertia=vector3N(self.lastFrame-self.firstFrame+1)
			local lff=self.firstFrame
			local sim= Physics.DynamicsSimulator_gmbs() 
			local skel=self.graph.skel_withoutCOMjoint
			sim:registerCharacter(skel)
			sim:setGVector(vector3(0,9.8,0)) 
			sim:init(model.timestep, Physics.DynamicsSimulator.EULER) 
			for i= self.firstFrame, self.lastFrame do

				local inertia3=vector3()
				local inertia=vectorn()
				local pose=self.graph.motions[1].mot_withoutCOMjoint:row(i)
				sim:calcInertia(0, pose, inertia)
				skel:setPoseDOF(pose)
				self.inertia(i-lff).x=inertia(0)
				self.inertia(i-lff).y=skel:calcCOM().y
				self.inertia(i-lff).z=inertia(2)
			end
		end
		print("segmentation", self.firstFrames[1], self.lastFrame)
	end

	do
		require('subRoutines/ZMPcalculator')
		self.ZMPcalculator=ZMPcalculator2:new(self.skel_withoutCOMjoint, self.graph.motions[1].mot_withoutCOMjoint, self.graph.motions[1].dmot, self.graph.motions[1].discontinuity)
	end
	self:initCartPoleState(self.firstFrames[1])

	self.theta=vectorn()
	self.dtheta=vectorn()

	self.pendulum:_saveStates(self.theta, self.dtheta)

	self.refTimeAll=vectorn(array.back(self.firstFrames)-self.firstFrames[1]+1)
	for i=1, table.getn(self.firstFrames)-1 do
		local ff=self.firstFrames[i]-self.firstFrames[1]
		local fl=self.firstFrames[i+1]-self.firstFrames[1]
		self.refTimeAll:range(ff, fl+1):linspace(i, i+1, fl-ff+1)
	end
end


function CartPoleController:estimatePendPos(refFrame, currPose)

   local pose=self.graph.motions[1].mot(refFrame)
   
   local rootRef=MotionDOF.rootTransformation(pose)

   self.skel:setPoseDOF(pose)

   self.fk:setPoseDOF(currPose)
   tf=self.compareChain:compare()

   local zmp=self.ZMPcalculator.zmp(refFrame)
   local com=self.ZMPcalculator.com(refFrame)

   local ldir=com-zmp
   local zmpOri=quater()
   zmpOri:axisToAxis(vector3(0,1,0), ldir)
   
   local zmpNew=tf*zmp
   local zmpOriNewTimesDrotY=tf.rotation*zmpOri

  
   --      com=calcCOMpos(zmpNew, zmpOriNewTimesDrotY)
   --      dbg.drawLine(self.objectList, zmpNew*100+vector3(0,100,0), com*100+vector3(0,100,0), "ZMPCOM")

   
   local zmpOriNew=quater()
   local drotY=quater()
   zmpOriNewTimesDrotY:decomposeNoTwistTimesTwist (vector3(0,1,0), zmpOriNew, drotY)

   local calcCOMpos=self.calcCOMpos

   -- calc COM and ZMP and project to the GROUND.
   local comNew=calcCOMpos(self,zmpNew, zmpOriNew)

   -- TODO: offsetting along pendulum direction.
   --      comNew.y=comNew.y-zmpNew.y
   zmpNew.y=0

   return zmpNew, comNew
end

function CartPoleController:initCartPoleState(refFrame)

	if true then
		self:initPendState(refFrame,true,true)
		return
	end

   local zmpVel=vector3(0,0,0)
   local comVel=vector3(0,0,0)
   
   local currPose=self.graph.motions[1].mot:row(refFrame)

   self.outputGlobal.start_transf=MotionDOF.rootTransformation(currPose):encode2D()

   
   local zmpNew, comNew=self:estimatePendPos(refFrame, currPose)


   dbg.namedDraw('Sphere', zmpNew*100, 'zmpNew')
   dbg.namedDraw('Sphere', comNew*100, 'comNew')

   local useInitUsingZMPCOM=util.chooseFirstNonNil(useCase.ipco_param_use_ZMP, true)

   if useInitUsingZMPCOM==true then
      local prevPose=self.graph.motions[1].mot:row(refFrame-1)
      local prevZMP, prevCOM=self:estimatePendPos(refFrame-1, prevPose)

      if self.initBaseX then
	 prevZMP.x=prevZMP.x+self.initBaseX 
	 prevZMP.z=prevZMP.z+self.initBaseZ
	 zmpNew.x=zmpNew.x+self.initBaseX 
	 zmpNew.z=zmpNew.z+self.initBaseZ
      end

      zmpVel:difference(prevZMP, zmpNew)
      zmpVel:scale(self.outputFrameRate)
      comVel:difference(prevCOM, comNew)
      comVel:scale(self.outputFrameRate)
   else
      zmpNew=comNew:copy()
      zmpNew.y=0
   end
       -- dbg.linecolor="solidblue"
       -- dbg.drawLine(self.objectList, prevZMP*100, prevCOM*100, "prev")
       dbg.linecolor="solidgreen"
       dbg.drawLine(self.objectList, zmpNew*100, comNew*100, "next")

   --     print(self.outputFrameRate)
   self.pendulum:setState(zmpNew, comNew, zmpVel, comVel)
   self.pendulum:setDesiredVelocity(vector3(0,0,0))
   self.pendulum:setOrientation2(MotionDOF.rootTransformation(currPose).rotation:rotationY())
end

function CartPoleController:restoreInitialState()
   self.pendulum:_restoreStates(self.theta, self.dtheta, 0)
   self.numFrames=0

   if self.initBaseX then
      self:initCartPoleState(self.firstFrames[1])
   end
end


function CartPoleController:changeControlParameter(title, param)
	local tokens=string.tokenize(title,',')
	if tokens[1]=='keyframe' and tokens[2]=='0' and tokens[3]=='pendDesiredVel' then
		local name=tokens[4]..','..tokens[5]
		if self.desiredVel[name]==nil then
			--assert(false)
			print('warning! ignoring '..name)
			self.desiredVel[name]=vector3(0,0,0)
		end
		self.desiredVel[name][tokens[6]]=param
	else
		--print(title.." ignored")
	end
end
function CartPoleController:getControlParameterFromGraph(title)
	local tokens=string.tokenize(title,',')
	if tokens[1]=='keyframe' and tokens[2]=='0' and tokens[3]=='pendDesiredVel' then
		local name=tokens[4]..','..tokens[5]
		assert(self.desiredVel[name]~=nil)
		return self.desiredVel[name][tokens[6]]
	else
		--print(title.." ignored")
		return 0
	end
end


function CartPoleController:generateCartPoleTrajectory(bRender)
	if bRender then
		util.writeFile("pendStateO.txt", " ")
	end
	-- 1. calculate pendstate at self.currFrame and self.currFrame-1
	self:restoreInitialState()

	local mot=self.graph.motions[1].mot
	local distance=0
	local lff=self.firstFrames[1]
	self.pendControlForce=vector3N(self.lastFrame-self.firstFrames[1]+1)
	for i=self.firstFrames[1], self.lastFrame do
		local iinterval=math.min(math.floor(self.refTimeAll(i-lff)), table.getn(self.firstFrames)-1)

		--update desired vel
		local desiredVelocity=self.desiredVel[self.sequence[iinterval].name]:copy()

		--local refRotTime=self.firstFrames[self.sequence[iinterval].pendRotRefTime+iinterval]
		local refRotTime=self.firstFrames[iinterval]
		--local refRotTime=0
		--print("pendRefTime:", i, refRotTime) 
		local rot_y=MotionDOF.rootTransformation(mot:row(refRotTime)).rotation:rotationY()

		if false then
			self:oneStep()
		else
			local linertia=rotate(self.inertia(i-lff), rot_y:inverse())
			local seg=self.sequence[iinterval]
			RE.output2("desiredVelocity", desiredVelocity)
			--print(desiredVelocity)
			desiredVelocity:rotate(rot_y)

			if not useCase.noTimeVaryingInertia then
				self.pendulum.pole:setLocalCOM(vector3(0, linertia.y,0))
				self.pendulum:setInertia(linertia)
			end
			self.pendulum:setOrientation2(rot_y)
			self.pendulum:setDesiredVelocity(desiredVelocity)

			self.pendulum:oneStep()
			if false then
				local dmmt=self.pendulum:calcDotMomentum()
				util.appendFile('pend1.txt', table.tostring2({i, iinterval, dmmt})..'\n')
			end
		end

		--self.pendControlForce(i-lff):assign(self.pendulum.controlForce)
		self.pendControlForce(i-lff):assign(vector3(10000,10000,10000)) --unused
		local tCOM=self.predictedCOM(i-lff)
		local cCOM=self.pendulum:calcCOMpos()
		distance=distance+tCOM:squaredDistance(cCOM)
		local optRenderFreq=240
		--local optRenderFreq=1
		if (bRender and math.mod(i,4)==0) or math.mod(i,optRenderFreq)==0 then
			self.pendulum:draw()
			self.skin1:setPoseDOF(mot:row(i))
			dbg.drawSphere(self.objectList, tCOM*100, "tCOM","red")
			dbg.drawSphere(self.objectList, cCOM*100, "cCOM")
			renderOneFrame()
		end
		if bRender then
			util.appendFile("pendStateO.txt", util.tostring(i,self.pendulum.theta, self.pendulum.dtheta).."\n")
			--util.appendFile("pendStateO.txt", util.tostring(tCOM, cCOM, self.pendulum:calcCartPos()).."\n")
			util.appendFile("pendStateO.txt", util.tostring(false, self.pendulum:calcCartPos(), desiredVelocity).."\n")
		end
	end
	if bRender then
		self.pendulum:drawFrames(0, self.objectList)
	end
	return distance
end

function CartPoleController:predictCOM()

   local graph=self.graph

   -- proceed first two frames seperately. This is to avoid immediate trigger.
 
   local predictedCOM=vector3N()
   predictedCOM:setSize(self.lastFrame-self.firstFrame+1)--self.graph.motions[1].mot:numFrames())

   for i=0, predictedCOM:size()-1 do
      self.skel:setPoseDOF(self.graph.motions[1].mot:row(self.firstFrame+i))
      predictedCOM(i):assign(self.skel:calcCOM())
   end

   self.objectList:registerObject("pelvis", "LineList", "solidgreen", predictedCOM:matView()*100, 0)
   self.predictedCOM=predictedCOM

end


function CartPoleController:oneStep(bRender)

	dbg.console()
--   self:updateKeyframes()
   local currFrame=self.numFrames


--   do
--      local info=self.trackingInfo
--
--      if info ~=nil then
--------	 print(currFrame, self.objectiveFunctionEnd, info.frac, info.currSeg.name)
--      end
--   end

   self:updateDesiredVel()
   self.pendulum:oneStep()

   self.controlforce:resize(currFrame+1, 3)
   --self.controlforce:row(currFrame):setVec3(0,self.pendulum.controlForce)
   self.controlforce:row(currFrame):setAllValue(1000000) -- unused
   if bRender==true then
      self.pendulum:draw()
      self.skin1:setPoseDOF(self.graph.motions[1].mot:row(currFrame))
   end

   self.outputGlobal.numFrames=currFrame+1
   
   if self.outputGlobal.numFrames>self.lastFrame-self.firstFrame then 
      self.objectiveFunctionEnd=true
   end

   self.numFrames=self.numFrames+1

   if self.predictedCOM==nil then 
      return 0 
   end
   
   local metric=0

   local tCOM=self.predictedCOM(currFrame)
   local cCOM=self.pendulum:calcCOMpos()

   if bRender==true then
      dbg.drawSphere(self.objectList, tCOM*100, "tCOM")
      dbg.drawSphere(self.objectList, cCOM*100, "cCOM")
   end

   if metric==0 then
      tCOM.y=0
      cCOM.y=0
      return tCOM:distance(cCOM)
   end

   local cZMP=self.pendulum:calcCartPos()

   -- metric==1 - distance along a line
   local line=Ogre.LineSegment(cZMP, cCOM)
   return line:pos(line:minDistTime(tCOM)):distance(tCOM)

   
end


--dbg.startDebug()

screenshot=false
captureUniqueId=0



CartPoleOptimizer=LUAclass(Optimizer)

-- actual parameters used for optimization
stepSize=5	-- GradientDescent only. very important to be set properly. need some intuition based on optimizelog.txt

--method=Optimizer.methods.GradientDecent
--method=Optimizer.methods.RandomizedSearch
--method=Optimizer.methods.GSLGradientDescent
--method=Optimizer.methods.GSLBFGS
--method=Optimizer.methods.NRconjugateGradient
--method=Optimizer.methods.CMAes
method=Optimizer.methods.ConjugateGradient
--method=Optimizer.methods.NRgradientDescent
--method=Optimizer.methods.GA

--method=Optimizer.methods.FullSearch
--method.numGrid:assign({3,3,3,3,3})
--method.gridSize:assign({0.1,0.1,5,5,5})

debugMode=false -- do not perform actual optimization..

-- Usage 1: to do optimization
-- 1. open this file
-- 2. set scenario
-- 3. set method above and opt_dimension below.

-- Usage 2: after optimization is finished, 
-- 1. open work/optimizelog.txt
-- 2. copy paste results to usecases.lua
-- 3. run this file
-- 4. export 

-- Usage 3: to see the optimization result clearly
--  lua short.lua ipcv

-- Usage 4: before starting motion synthesis (one-time operation)
--  lua short.lua ipco_postp

   opt_dimension={}


function CartPoleOptimizer:__init()
	--   self.objectList=Ogre.ObjectList() 
	local dofFN=useCase.mot_file or model.mot_file

	local graphFN=useCase.cartPoleFN or str.left(dofFN,-3).."zmpcom"

	self.backupCartPole=nil
	-- TODO: need to backup zmpcom.
	if util.isFileExist(graphFN) then
		self.backupCartPole=util.loadTable(graphFN)
	end

	self.synthesis = CartPoleController:new()


	self.synthesis:predictCOM()

	if table.count(useCase.pendControlParam)==0 then
		-- setup initial solution
		local firstFrames=self.synthesis.firstFrames
		local seq=self.synthesis.sequence
		local mot=self.synthesis.graph.motions[1].mot
		local desiredVel={}
		local desiredVelCount={}
		for i=1, table.getn(firstFrames)-1 do
			local ff=firstFrames[i]-firstFrames[1]
			local fl=firstFrames[i+1]-firstFrames[1]

			local comvel=vector3(0,0,0)
			for j=ff,fl do
				local rot_y=MotionDOF.rootTransformation(mot:row(j)).rotation:rotationY()
				comvel=comvel+rotate(self.synthesis.ZMPcalculator.comvel(j), rot_y:inverse())
			end
			comvel:scale(1/(fl-ff))
			desiredVel[seq[i].name]=(desiredVel[seq[i].name] or vector3(0,0,0))+comvel
			desiredVelCount[seq[i].name]=(desiredVelCount[seq[i].name] or 0)+1
		end
		for k,v in pairs(desiredVel) do
			desiredVel[k]:scale(1/desiredVelCount[k])
		end

		for k, v in pairs(desiredVel) do
			useCase.pendControlParam['keyframe,0,pendDesiredVel,'..k..',x']=v.x
			useCase.pendControlParam['keyframe,0,pendDesiredVel,'..k..',z']=v.z
		end
		printTable(useCase.pendControlParam)
	end
	-- 
	local startseg=1
	local endseg=500000000000
	--startseg=5	
	--endseg=5
	self.synthesis.lastFrame=useCases.set_opt_dimension(startseg, endseg,opt_dimension, 0.05, 0.02, "pendDesiredVel", "pendControlParam", useCase.pendOptExcludePattern)

	--for i=1,#opt_dimension do printTable(opt_dimension[i]) end
	Optimizer.__init(self, stepSize, opt_dimension, method)
	for k,v in pairs(useCase.pendControlParam) do
		if k~='sequentialControlParam' then
			self.synthesis:changeControlParameter(k, v)
		end
	end
	if useCase.funcUpdateConstraints then
		useCase.funcUpdateConstraints(self.synthesis)
	end
	self.synthesis:generateCartPoleTrajectory(true)
	--  export ZMP file.
	do

		local initFront=false
		local mot=self.synthesis.graph.motions[1].mot
		if self.backupCartPole==nil then
			self.backupCartPole={}
			self.backupCartPole.zmp=vector3N(mot:numFrames())
			self.backupCartPole.zmp:setAllValue(vector3(0,0,0))
			self.backupCartPole.com=vector3N(mot:numFrames())
			self.backupCartPole.com:setAllValue(vector3(0,0,0))
			self.backupCartPole.plannedComacc=vector3N(mot:numFrames())
			self.backupCartPole.plannedComacc:setAllValue(vector3(0,0,0))
			initFront=true
		end
		if self.backupCartPole.pendControlForce==nil then self.backupCartPole.pendControlForce=vector3N(mot:numFrames()) end

		local lff=self.synthesis.firstFrame
		local com=self.backupCartPole.com
		local zmp=self.backupCartPole.zmp
		local plannedComacc=self.backupCartPole.plannedComacc
		local pendControlForce=self.backupCartPole.pendControlForce


		local pend=self.synthesis.pendulum
		local numFrames=pend:numFrames()
		for i=0, numFrames-1 do
			if not useCase.noTimeVaryingInertia then
				pend.pole:setLocalCOM(vector3(0,self.synthesis.inertia(i).y ,0))
			end
			local zz,cc=pend:calcZMPCOM( i)
			com(i+lff):assign(cc)
			zmp(i+lff):assign(zz)
			pendControlForce(i+lff):assign(self.synthesis.pendControlForce(i))
		end

		self.synthesis.objectList:registerObject("com", "LineList", "solidblue", com:matView():range(
		lff, com:size(), 0, 3)*100,0)

		-- control force to the cart represent in global coordinate.
		-- This will later be converted into local coordinate in 
		-- ZMPgraphSegment:calcLocalPelvisAndFoot

		if initFront then
			for i=0, lff -1 do
				com(i):assign(com(lff))
				zmp(i):assign(zmp(lff))
				--cf:row(i):setAllValue(0)
			end
		end
		util.saveTable( {zmp=zmp, com=com, plannedComacc=plannedComacc, pendControlForce=pendControlForce}, graphFN)
	end


end




function CartPoleOptimizer:objectiveFunction(pos)
	local screenshotPrefix="../dump/optimize_"..captureUniqueId
	if screenshot==true then
		captureUniqueId=captureUniqueId+1
		RE.renderer():setScreenshotPrefix(screenshotPrefix)
		RE.renderer():screenshot(true)
		print("start")
	end

	local MSE
	do
		RE.output("pos", tostring(pos))
		local graph=self.synthesis.graph

		for i=0, pos:size()-1 do
			local title=opt_dimension[i+1].title

			self.synthesis:changeControlParameter(title, pos(i))
		end
		if useCase.funcUpdateConstraints then
			useCase.funcUpdateConstraints(self.synthesis)
		end

		MSE=self.synthesis:generateCartPoleTrajectory(false)
	end   
	MSE=MSE/(self.synthesis.lastFrame-self.synthesis.firstFrame)
	collectgarbage("collect")
	collectgarbage("collect")
	collectgarbage("collect")

	--coarseLog('mse', pos, MSE)
	if useCase.measureOptCost then
		print('measureOptCost')
		local optCost=useCase.measureOptCost(pos, self.synthesis)
		print("eval", MSE, optCost*0.02)
		return MSE+optCost*0.02
	end
	return MSE
end

RE.viewpoint():setFOVy(45.000002)
RE.viewpoint().vpos:set(-31.603369, 177.652362, 622.504089)
RE.viewpoint().vat:set(-39.439721, 177.466525, -14.588023)
RE.viewpoint():update()


-- following values are automatically connected to UI.
boolean_options={}
boolean_options.drawDisplacementMap=false
boolean_options.useOrientationCorrection=false
boolean_options.solveIK=true
boolean_options.drawPredictedCOM=false
boolean_options.drawPredictedZMP =false
boolean_options.drawPelvis=false
boolean_options.drawControlForce=false
boolean_options.drawMatching=false
float_options={}
float_options.impulseMagnitude={ val=100, min=10, max=1000}
float_options.impulseDuration={ val=20, min=1, max=100}

function CartPoleOptimizer:detectMemoryLeak()
   local pos=vectorn()
   pos:setSize(self.N_opt_dimension)
   
   
   for i=1, self.N_opt_dimension do
      local optvar=self.opt_dimension[i]
	  assert(optvar.curval)
      set1(pos, i, optvar.curval)
      optvar.prevval=optvar.curval
   end
   
   local eval=self:objectiveFunction(pos)
end

states={init=1, optimize=2, stop=3}
function ctor()

   
   mSynthesis	= CartPoleOptimizer()
   state=states.init
   mSynthesis.maxIteration=10000
--   mSynthesis:detectMemoryLeak()
   
   
   --this:create("Check_Button"	, "orientation correction", "orientation correction",0);
   --this:widget(0):checkButtonValue(true)
   this:updateLayout()
   
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
   if mSynthesis~=nil then
	   if state==states.init then
		   state=states.optimize
		   mSynthesis:optimize(true)
		   state=states.stop
	   elseif state==states.stop then
		   if rank~=nil then
			   if mSynthesis.server then
				   if outputFileName==nil then
					   mSynthesis.server:save("optimizeresult.lua")
				   else
					   mSynthesis.server:save(outputFileName)
				   end
			   end
		   else
			   if outputFileName==nil then
				   mSynthesis:save("optimizeresult.lua")
			   else
				   mSynthesis:save(outputFileName)
			   end
		   end
		   this("exit",0)
	   end
   end
end



