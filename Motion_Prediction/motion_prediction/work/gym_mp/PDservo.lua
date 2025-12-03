-- PDServo class
--class 'PDservo'
PDservo=LUAclass()

function PDservo:setCoef(dofInfo,kp, kd, tgtVelScale, k_scale)
   kp:setSize(dofInfo:numDOF())
   kp:setAllValue(k_p)
   kd:setSize(dofInfo:numDOF())
   kd:setAllValue(k_d)
   tgtVelScale:setSize(dofInfo:numDOF())
   tgtVelScale:setAllValue(k_d)
   self.k_scale = {default={1,1,1}}
   k_scale = self.k_scale
   -- exclude root joint
   kp:range(0,7):setAllValue(0)
   kd:range(0,7):setAllValue(0)
   tgtVelScale:range(0,7):setAllValue(0)
  
   print("initPDservo:"..dofInfo:skeleton():bone(1):name())
   for i=2,dofInfo:skeleton():numBone()-1 do
      local bone=dofInfo:skeleton():bone(i)
      local vbone=bone:treeIndex()
      local nJoint=dofInfo:numDOF(vbone)
--      print("initPDservo:"..bone:name())
      for j=0, nJoint-1 do
	 
	 local dofIndex=dofInfo:DOFindex(vbone,j)
	 
	 kp:set(dofIndex, k_p*k_scale.default[1])
	 kd:set(dofIndex, k_d*k_scale.default[2])
	 tgtVelScale:set(dofIndex, k_scale.default[3])

	 if bone:voca()==MotionLoader.LEFTANKLE or bone:voca()==MotionLoader.RIGHTANKLE then
	    if k_scale.ankle~=nil then
	       kp:set(dofIndex, k_p*k_scale.ankle[1])
	       kd:set(dofIndex, k_d*k_scale.ankle[2])
	       tgtVelScale:set(dofIndex, k_scale.ankle[3])
	    end
	 elseif bone:voca()==MotionLoader.LEFTCOLLAR or bone:voca()==MotionLoader.RIGHTCOLLAR then
	    if k_scale.collar~=nil then
	       kp:set(dofIndex, k_p*k_scale.collar[1])
	       kd:set(dofIndex, k_d*k_scale.collar[2])
	       tgtVelScale:set(dofIndex, k_scale.collar[3])
	    end
	 elseif bone:voca()==MotionLoader.LEFTSHOULDER or bone:voca()==MotionLoader.RIGHTSHOULDER then
	    if k_scale.shoulder~=nil then
	       kp:set(dofIndex, k_p*k_scale.shoulder[1])
	       kd:set(dofIndex, k_d*k_scale.shoulder[2])
	       tgtVelScale:set(dofIndex, k_scale.shoulder[3])
	    end
	 elseif bone:voca()==MotionLoader.LEFTELBOW or bone:voca()==MotionLoader.RIGHTELBOW then
	    if k_scale.elbow~=nil then
	       kp:set(dofIndex, k_p*k_scale.elbow[1])
	       kd:set(dofIndex, k_d*k_scale.elbow[2])
	       tgtVelScale:set(dofIndex, k_scale.elbow[3])
	    end
	 elseif bone:voca()==MotionLoader.LEFTKNEE or bone:voca()==MotionLoader.RIGHTKNEE then
	    if k_scale.knee~=nil then
	       kp:set(dofIndex, k_p*k_scale.knee[1])
	       kd:set(dofIndex, k_d*k_scale.knee[2])
	       tgtVelScale:set(dofIndex, k_scale.knee[3])
	    end
	 elseif bone:voca()==MotionLoader.LEFTHIP or bone:voca()==MotionLoader.RIGHTHIP then
	    if k_scale.hip~=nil then
	       kp:set(dofIndex, k_p*k_scale.hip[1])
	       kd:set(dofIndex, k_d*k_scale.hip[2])
	       tgtVelScale:set(dofIndex, k_scale.hip[3])
	    end
	 elseif bone:voca()==MotionLoader.CHEST then
	    if k_scale.chest~=nil then
	       kp:set(dofIndex, k_p*k_scale.chest[1])
	       kd:set(dofIndex, k_d*k_scale.chest[2])
	       tgtVelScale:set(dofIndex, k_scale.chest[3])
	    end
	 elseif bone:voca()==MotionLoader.CHEST2 then
	    if k_scale.chest2~=nil then
	       kp:set(dofIndex, k_p*k_scale.chest2[1])
	       kd:set(dofIndex, k_d*k_scale.chest2[2])
	       tgtVelScale:set(dofIndex, k_scale.chest2[3])
	    end
	 elseif bone:voca()==MotionLoader.NECK then
	    if k_scale.neck~=nil then
	       kp:set(dofIndex, k_p*k_scale.neck[1])
	       kd:set(dofIndex, k_d*k_scale.neck[2])
	       tgtVelScale:set(dofIndex, k_scale.neck[3])
	    end
	 elseif bone:voca()==MotionLoader.HEAD then
	    if k_scale.head~=nil then
	       kp:set(dofIndex, k_p*k_scale.head[1])
	       kd:set(dofIndex, k_d*k_scale.head[2])
	       tgtVelScale:set(dofIndex, k_scale.head[3])
	    end
	 end
	 if str_include(bone:name(), "toes") then
	    local dofIndex=dofInfo:DOFindex(vbone,j)
	    if k_scale.toes~=nil then
	       kp:set(dofIndex, k_p*k_scale.toes[1])
	       kd:set(dofIndex, k_d*k_scale.toes[2])
	       tgtVelScale:set(dofIndex, k_scale.toes[3])
	    end

	 end
	--for i=1,dofInfo:skeleton():numBone()-1 do 
	--	local gain_scale=1.0
	--	local bone=dofInfo:skeleton():bone(i)
	--	local vbone=bone:treeIndex()
	--	local nJoint=dofInfo:numDOF(vbone)
	--	for j=0, nJoint-1 do	
	--		local dofIndex=dofInfo:DOFindex(vbone,j)
	--		if bone:name()=="pelvis" then
	--			kp:set(dofIndex, 1000.0*gain_scale)
	--			kd:set(dofIndex, 200.0*gain_scale)
	--		elseif bone:name()=="pelvis_lowerback" then
	--			kp:set(dofIndex, 75.0*gain_scale)
	--			kd:set(dofIndex, 17.0*gain_scale)
	--		elseif bone:name()=="lowerback_torso" then
	--			kp:set(dofIndex, 75.0*gain_scale)
	--			kd:set(dofIndex, 17.0*gain_scale)
	--		elseif bone:name()=="torso_head" then
	--			kp:set(dofIndex, 10.0*gain_scale)
	--			kd:set(dofIndex, 3.0*gain_scale)
	--		elseif bone:name()=="lShoulder" then
	--			kp:set(dofIndex, 15.0*gain_scale)
	--			kd:set(dofIndex, 5.0*gain_scale)
	--		elseif bone:name()=="lElbow" then
	--			kp:set(dofIndex, 5.0*gain_scale)
	--			kd:set(dofIndex, 1.0*gain_scale)
	--		elseif bone:name()=="rShoulder" then
	--			kp:set(dofIndex, 15.0*gain_scale)
	--			kd:set(dofIndex, 5.0*gain_scale)
	--		elseif bone:name()=="rElbow" then
	--			kp:set(dofIndex, 5.0*gain_scale)
	--			kd:set(dofIndex, 1.0*gain_scale)
	--		elseif bone:name()=="lHip" then
	--			kp:set(dofIndex, 300.0*gain_scale)
	--			kd:set(dofIndex, 35.0*gain_scale)
	--		elseif bone:name()=="lKnee" then
	--			kp:set(dofIndex, 300.0*gain_scale)
	--			kd:set(dofIndex, 35.0*gain_scale)
	--		elseif bone:name()=="lAnkle" then
	--			kp:set(dofIndex, 50.0*gain_scale)
	--			kd:set(dofIndex, 15.0*gain_scale)
	--		elseif bone:name()=="lToeJoint" then
	--			kp:set(dofIndex, 2.0*gain_scale)
	--			kd:set(dofIndex, 0.2*gain_scale)
	--		elseif bone:name()=="rHip" then
	--			kp:set(dofIndex, 300.0*gain_scale)
	--			kd:set(dofIndex, 35.0*gain_scale)
	--		elseif bone:name()=="rKnee" then
	--			kp:set(dofIndex, 300.0*gain_scale)
	--			kd:set(dofIndex, 35.0*gain_scale)
	--		elseif bone:name()=="rAnkle" then
	--			kp:set(dofIndex, 50.0*gain_scale)
	--			kd:set(dofIndex, 15.0*gain_scale)
	--		elseif bone:name()=="rToeJoint" then
	--			kp:set(dofIndex, 2.0*gain_scale)
	--			kd:set(dofIndex, 0.2*gain_scale)
	--		end
	--	end
	--end

	 if dofInfo:DOFtype(vbone, j)==MotionDOFinfo.SLIDE then
	    local dofIndex=dofInfo:DOFindex(vbone,j)
	    kp:set(dofIndex, model.k_p_slide)
	    kd:set(dofIndex, model.k_d_slide)
	    tgtVelScale:set(dofIndex, 0)
	 end
      end
   end
end

function PDservo:updateCoef()
   local dofInfo=self.dofInfo
   local k_scale_active=model.k_scale_active_pd

   self:setCoef(dofInfo,self.kp_active, self.kd_active, self.tgtVelScale_active, k_scale_active)

   local k_scale_passive=model.k_scale_passive_pd

   self:setCoef(dofInfo,self.kp_passive, self.kd_passive, self.tgtVelScale_passive, k_scale_passive)
end

function PDservo:__init(dofInfo)
	self.theta=vectorn()
	self.dtheta=vectorn()
	self.theta_d=vectorn() -- desired q
	self.dtheta_d=vectorn() -- desired dq
	self.controlforce=vectorn()
	self.kp=vectorn()
	self.kd=vectorn()
	self.tgtVelScale=vectorn()
	self.kp_active=vectorn()
	self.kd_active=vectorn()
	self.tgtVelScale_active=vectorn()
	self.kp_passive=vectorn()
	self.kd_passive=vectorn()
	self.tgtVelScale_passive=vectorn()
	self.mask_slide=vectorn()
	self.muscleActiveness=0.3
	self.kp_weight=1.0 -- use kp_active(1) or kp_passive(0)
	self.kd_weight=1.0 -- use kd_active(1) or kd_passive(0)
	self.mask_slide:setSize(dofInfo:numDOF())
	self.mask_slide:setAllValue(0)
	self.dofInfo=dofInfo
	self:updateCoef()
	print ("kp=",self.kp)
	print ("kd=",self.kd)
	local clampTorque=1000
	local clampForce=8000
	
	if model.clampTorque~=nil then
	   clampTorque=model.clampTorque
	end
	
	if model.clampForce~=nil then
	   clampForce=model.clampForce
	end
	
	clampTorque=800
	clampForce=8000
	
	self.clampMax=vectorn(dofInfo:numDOF())
	for i=2,dofInfo:skeleton():numBone()-1 do
	   local bone=dofInfo:skeleton():bone(i)
	   local vbone=bone:treeIndex()
	   local nJoint=dofInfo:numDOF(vbone)
	   for j=0, nJoint-1 do
	  local dofIndex=dofInfo:DOFindex(vbone,j)
	  if dofInfo:DOFtype(vbone, j)==MotionDOFinfo.SLIDE then
	     local dofIndex=dofInfo:DOFindex(vbone,j)
	     self.mask_slide:set(dofIndex, 1)
	     self.clampMax:set(dofIndex, clampForce)
	  else
	     self.clampMax:set(dofIndex, clampTorque)
	  end
	   end
	end   
	
	self.clampMin=self.clampMax*-1
	return o
end

function PDservo:initPDservo(startf, endf,motionDOF, dmotionDOF)

	self.startFrame=startf
	self.endFrame=endf
	self.currFrame=startf
	self.deltaTime=0
	self.motionDOF=motionDOF
	self.dmotionDOF=dmotionDOF
end

-- generate FBtorque
function PDservo:generateTorque(simulator, target_delta)

	self:_generateTorque(simulator, self.currFrame, target_delta)
	return true
end

function PDservo:stepSimul(simulator, drawDebugInformation, impulse)
	simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_TORQUE, self.controlforce)
	if drawDebugInformation then
		simulator:drawDebugInformation()
	end
	if impulse then
		simulator:addForceToBone(0, impulse.chest, vector3(0,0,0), impulse.lf);
	end
	simulator:stepSimulation()
end
--torque 계산하는거
--
function PDservo:_generateTorque(simulator, frame, target_delta)
	--theta, dtheta = 현재 시뮬레이션 되고 있는 포즈 정보
	simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, self.theta)
	simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, self.dtheta)
	
	self.motionDOF:sampleRow(frame, self.theta_d)
	self.theta_d:radd(target_delta)
	self.dmotionDOF:sampleRow(frame, self.dtheta_d)
	--self.dtheta_d=(self.theta_d-self.theta):copy()  --이건 문제 있어서 위에줄로 바꿈.
	self.dtheta_d:rmult(self.muscleActiveness) -- this corresponds to muscle activeness
	
	self.controlforce:setSize(self.motionDOF:numDOF())
	local delta=self.theta_d-self.theta
	--MainLib.VRMLloader.projectAngles(delta) -- [-pi, pi]
	self.kp:interpolate(self.kp_weight, self.kp_passive, self.kp_active)
	self.kd:interpolate(self.kd_weight, self.kd_passive, self.kd_active)
	self.tgtVelScale:interpolate(self.kd_weight, self.tgtVelScale_passive, self.tgtVelScale_active)
	
	self.controlforce:assign(self.kp*delta +
	self.kd*(self.dtheta_d*self.tgtVelScale-self.dtheta))
	self.controlforce:clamp(self.clampMin, self.clampMax)
end

function PDservo:rewindTargetMotion(simulator)
   self.deltaTime=-1*simulator:currentTime()
end
function PDservo:printDelta(initState)
	self.theta=initState:copy()		
	print(self.theta)
	print(self.theta:length())
	print('-------------------------------------')
end
function PDservo:resetPDset(dofInfo)
   self.theta=vectorn()
   self.dtheta=vectorn()
   self.theta_d=vectorn() -- desired q
   self.dtheta_d=vectorn() -- desired dq
   self.controlforce=vectorn()
   self.kp=vectorn()
   self.kd=vectorn()
   self.tgtVelScale=vectorn()
   self.kp_active=vectorn()
   self.kd_active=vectorn()
   self.tgtVelScale_active=vectorn()
   self.kp_passive=vectorn()
   self.kd_passive=vectorn()
   self.tgtVelScale_passive=vectorn()
   self.mask_slide=vectorn()
   self.muscleActiveness=0.3
   self.kp_weight=1.0 -- use kp_active(1) or kp_passive(0)
   self.kd_weight=1.0 -- use kd_active(1) or kd_passive(0)
   self.mask_slide:setSize(dofInfo:numDOF())
   self.mask_slide:setAllValue(0)
   self.dofInfo=dofInfo
   self:updateCoef()
	
   local clampTorque=1000
   local clampForce=8000

   self.clampMax=vectorn(dofInfo:numDOF())
   for i=2,dofInfo:skeleton():numBone()-1 do
      local bone=dofInfo:skeleton():bone(i)
      local vbone=bone:treeIndex()
      local nJoint=dofInfo:numDOF(vbone)
      for j=0, nJoint-1 do
	 local dofIndex=dofInfo:DOFindex(vbone,j)
	 if dofInfo:DOFtype(vbone, j)==MotionDOFinfo.SLIDE then
	    local dofIndex=dofInfo:DOFindex(vbone,j)
	    self.mask_slide:set(dofIndex, 1)
	    self.clampMax:set(dofIndex, clampForce)
	 else
	    self.clampMax:set(dofIndex, clampTorque)
	 end
      end
   end   

   self.clampMin=self.clampMax*-1
end

