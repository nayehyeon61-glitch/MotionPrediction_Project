GenRefTrajectory=LUAclass(OnlineSynthesis2)

function GenRefTrajectory:init_globals()
   self.cpBackup=useCase.controlParam
   --useCase.controlParam=nil
   self.numFootBackup=useCase.numFootKeyFrames

   self.paramBackup={}
   self.paramBackup.PDservoLatency=PDservoLatency
   self.paramBackup.PredictionLatency=PredictionLatency
   PDservoLatency=0--2*outputSuperSample
   PredictionLatency=0
end

function GenRefTrajectory:restore_globals()

   PDservoLatency=self.paramBackup.PDservoLatency
   PredictionLatency=self.paramBackup.PredictionLatency

   self.pendulum:restoreStates(self.paramBackup.pendState)
end
 
function GenRefTrajectory:initStates(prevSeg, currSegment)


   self.outputLocal=LocoGraphOutputLocal:new(self.graph)
   self.outputGlobal=LocoGraphOutputGlobal:new(self.outputLocal, outputSuperSample)
   self.numFrames=0 -- with respect to outputFrameRate (=mocap frame rate * outputSuperSample)
   self.currSegment=currSegment
   self.outputLocal:append(currSegment)

   local pose, dpose, tf=self:calcInitialState()
   local skel=self.skel
   skel:setPoseDOF(pose)
   local pelvis=skel:VRMLbone(1):getFrame():copy()
   local ff=self.currSegment.first
   
   local pend=self.pendulum
   local ff=currSegment.first
   local com_pos=currSegment.locomot.ZMPcalculatorFullbody.com(ff)
   local com_vel=currSegment.locomot.ZMPcalculatorFullbody.comvel(ff)
   local zmp_pos=currSegment.locomot.ZMPcalculatorFullbody.zmp(ff)
   local zmp_vel=currSegment.locomot.ZMPcalculatorFullbody.comvel(ff)

   -- print(com_pos,com_vel, zmp_pos, zmp_vel)
   -- dbg.console()
   pend:setState(tf*zmp_pos, tf*com_pos, tf.rotation*zmp_vel, tf.rotation*com_vel)   

   local theta=vectorn()
   local dtheta=vectorn()
   pend:_saveStates(theta, dtheta)
   pend:_restoreStates(theta, dtheta,0)

   self.pelvisError:identity()
   self.prevSegment=prevSeg
   self.prevLocalFirst=0
end

function GenRefTrajectory:__init(refSyn)

   -- a modified version of OnlineSynthesis2.__init(self)
   do
      self:init_globals()
      self.skel=refSyn.skel
      self.skel_withoutCOMjoint=refSyn.skel_withoutCOMjoint
      self.floor=refSyn.floor
      self.desiredSpeedX=0			
      self.desiredSpeedZ=0
      self.desiredTurningSpeed=0

      self.trackingErrorRot=quater()
      self.trackingErrorRot:identity()
      self.pendControlForce=vector3(0,0,0)
      local skel=self.skel

      if useCase.mot_file~=nil then -- override
	 model.mot_file=useCase.mot_file
      end

      local leftFoot, rightFoot

      self.motionDOF=refSyn.motionDOF
      self.dmotionDOF=refSyn.dmotionDOF
      self.discontinuity=refSyn.discontinuity

      self.graph=refSyn.graph

      self.ik=refSyn.ik
	  self.ik_COM=refSyn.ik_COM

      self.outputLocal=LocoGraphOutputLocal:new(self.graph)
      self.outputGlobal=LocoGraphOutputGlobal:new(self.outputLocal, outputSuperSample)
      
	self.footStates={lfootStride=1, rfootStride=1}
	self.footStates.mapZMP={}
	do
		for i=1,3 do
			self.footStates.mapZMP['L'..tostring(i)]=DisplacementMaps(20)
			self.footStates.mapZMP['R'..tostring(i)]=DisplacementMaps(20)
			self.footStates.mapZMP['LH'..tostring(i)]=DisplacementMaps(20)
			self.footStates.mapZMP['RH'..tostring(i)]=DisplacementMaps(20)
		end
	end

      --   self.LQRlookahead=5*outputSuperSample
      self.skin2=RE.createVRMLskin(self.skel_withoutCOMjoint, false)
      self.skin2:scale(100,100,100)
      -- self.skinFloor=RE.createVRMLskin(self.floor, false)
      -- self.skinFloor:scale(100,100,100)

      self.outputFrameRate=model.frame_rate*outputSuperSample
      
      self.pendulum=refSyn.pendulum
	  self.verticalPend=refSyn.verticalPend
      
	  self.paramBackup.pendState=self.pendulum:getStates()

--      self.objectList=Ogre.ObjectList ()
      self.objectList=refSyn.objectList
      --   g_objectList=self.objectList

      self.numFrames=0 -- with respect to outputFrameRate (=mocap frame rate * outputSuperSample)
      self.currSegment=self.graph.initialSegment
      self.prevFrac=1
      
	  --self:produceSegment() -- the first segment becomes initialSegment.next
	  self.outputLocal:append(self.currSegment) -- the first segment becomes initialSegment
      self.samplePose=vectorn()
      self.samplePose2=vectorn()
      self.samplePoseBeforeIK=vectorn()
      self.samplePose_pdtarget=vectorn()

      self.pelvisError=quater()
      self.pelvisError:identity()
      self.prevSegment=self.graph.initialSegment
      self.prevLocalFirst=0

      
      self:initializeControlParameters()

	  self.desiredCOP=useCase.desiredPos:copy()
   end
   self.externalForce=vector3(0,0,0)
   self.impulse=0

   self.outputMotion=MotionDOFcontainer(self.skel_withoutCOMjoint.dofInfo)
end


function GenRefTrajectory:__finalize()
   self.outputMotion=nil
   self.simulator=nil
   self.skel=nil
   self.prevSegment=nil
   self.graph=nil
   self.outputLocal=nil
   self.outputGlobal=nil
   self.objectList=nil
end

function GenRefTrajectory:oneStep()
   
   OnlineSynthesis2.oneStep(self)
   
   local mrdMotion=self.outputMotion

   if mrdMotion then
      local currFrame=mrdMotion:numFrames()
      mrdMotion:resize(currFrame+1)
      mrdMotion:row(currFrame):assign(self.samplePose)

      if math.mod(self.numFrames, 50)==0 then
	 mrdMotion:exportMot("debug_plot.dof")
      end	 
   end

end
