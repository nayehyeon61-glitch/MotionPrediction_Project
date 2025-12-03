require("IPC_based/common")
require("subRoutines/motiongraph")
require("IPC_based/MotionGraph")
require("subRoutines/InterframeDifferenceRot")
--require("subRoutines/COPcontroller")
--list=require("functional/list")

LocoGraphMotion=LUAclass()
LocoGraph=LUAclass()
LocoGraphSegment=LUAclass()
LocoGraphOutputGlobal=LUAclass()
LocoGraphOutputLocal=LUAclass()

-- a motion graph is a table containing segments.
-- e.g. graph={ jump={ mot=mot1, dmot=dmot1}, walk1={ mot=mot2, dmot=dmot2}, ...}


function LocoGraphMotion:loadOptimizedTrajectory(skel, mot_file)
	-- load optimized trajectory
	local cartPoleFN=useCase.cartPoleFN or str.left(mot_file,-3).."zmpcom"
	print(cartPoleFN)
	
	if util.isFileExist(cartPoleFN) then
		self.ZMPcalculator=util.loadTable(cartPoleFN)
		self.ZMPcalculator.comvel=vector3N()
		self.ZMPcalculator.comacc=vector3N()
		self.ZMPcalculator.zmpvel=vector3N()
		self.ZMPcalculator.zmpacc=vector3N()
		local nn=self.ZMPcalculator.com:size()
		self.ZMPcalculator.comvel:resize(nn)
		self.ZMPcalculator.comacc:resize(nn)
		self.ZMPcalculator.zmpvel:resize(nn)
		self.ZMPcalculator.zmpacc:resize(nn)
		local comvel, comacc=calcFirstAndSecondDerivatives(self.ZMPcalculator.com:matView(), model.frame_rate)
		local zmpvel, zmpacc=calcFirstAndSecondDerivatives(self.ZMPcalculator.zmp:matView(), model.frame_rate)

		self.ZMPcalculator.comvel:matView():assign(comvel)
		self.ZMPcalculator.comacc:matView():assign(comacc)
		self.ZMPcalculator.zmpvel:matView():assign(zmpvel)
		self.ZMPcalculator.zmpacc:matView():assign(zmpacc)
		require('subRoutines/ZMPcalculator')
		self.ZMPcalculatorFullbody=ZMPcalculator2:new(skel, self.mot_withoutCOMjoint, self.dmot, self.discontinuity)
	else 
		require('subRoutines/ZMPcalculator')
		self.ZMPcalculator=ZMPcalculator2:new(skel, self.mot_withoutCOMjoint, self.dmot, self.discontinuity)
		self.ZMPcalculator.zmpvel=vector3N(self.ZMPcalculator.com:size())
		self.ZMPcalculator.zmpvel:setAllValue(vector3(0,0,0))
		self.ZMPcalculator.comacc=vector3N(self.ZMPcalculator.com:size())
		self.ZMPcalculator.comacc:setAllValue(vector3(0,0,0))
		print('run "driverExtractPendTrajectory.lua" first. If you are running "driverExtractPendTrajectory.lua" or "noopt" or "noopt_prep", then ignore this message and type "cont"')
		self.__bypass=true
		dbg.console()
		self.ZMPcalculatorFullbody=ZMPcalculator2:new(skel, self.mot_withoutCOMjoint, self.dmot, self.discontinuity)
	end
end
function LocoGraphMotion:__init(skel, mot_file)

	if mot_file==nil then return end
	do
		local motion=skel.mMotion

		local container=MotionDOFcontainer(skel.dofInfo, mot_file)
		if useCase.mot_file_upsample then
			container:upsample(useCase.mot_file_upsample )
			container.mot.dofInfo:setFrameRate(120)
			if not os.isFileExist(mot_file..'up4.dof') then
				container:exportMot(mot_file..'up4.dof')
			end
		end
		if container:numFrames()>3636 then
			container.discontinuity:set(3636,false)
		end

		if useCase.fixOrientation then
			local f=useCase.fixOrientation

			local function fixOri(first, last, name, axis)
				local index=skel:getTreeIndexByName(name)
				local start_rh=skel.dofInfo:startR(index)
				for i=first, last do 
					local pose=container.mot:row(i)
					skel:setPoseDOF(pose)
					local q=skel:VRMLbone(index):getLocalFrame().rotation
					q:assign(q*-1)
					local euler=vector3()
					q:getRotation(axis, euler)
					pose:setVec3(start_rh, euler)
				end
			end
			for i, j in ipairs(f) do
				fixOri(f.startFrame, f.endFrame-1, j[1], j[2])
			end
		end


		--container:exportMot(mot_file)
		local mot=container.mot

		skel.dofInfo:setFrameRate(model.frame_rate)   

		self.skel_withoutCOMjoint=skel
		MotionLoader.setVoca(self.skel_withoutCOMjoint, model.bones)

		skel=MainLib.VRMLloader(useCase.model_file_name or model.file_name)
		skel.dofInfo:setFrameRate(model.frame_rate)   
		assert(skel:name()==self.skel_withoutCOMjoint:name())

		local comtraj=vector3N(mot:rows())	
		local rotytraj=quaterN(mot:rows())

		local tiltedGround=model.tiltedGround
		if tiltedGround then
			for i=0, mot:numFrames()-1 do
				local tf=MotionDOF.rootTransformation(mot:row(i))
				MotionDOF.setRootTransformation(mot:row(i), tiltedGround*tf)
			end
		end
		if false then -- smooth original motion
			local rootOri=quaterN(mot:rows())
			local rootPos=vector3N(mot:rows())
			local mot_col=mot:row(0):size()
			local jointAngles=matrixn(mot:rows(), mot_col-7)
			for i=0, mot:rows()-1 do
				rootOri(i):assign(MotionDOF.rootTransformation(mot:row(i)).rotation)
				rootPos(i):assign(MotionDOF.rootTransformation(mot:row(i)).translation)
				jointAngles:row(i):assign(mot:row(i):range(7, mot_col))
			end

			local segFinder=SegmentFinder(container.discontinuity)
			for i=0, segFinder:numSegment()-1 do
				local s=segFinder:startFrame(i)
				local e=segFinder:endFrame(i)

				rootOri:range(s,e):smooth(5)
				rootPos:range(s,e):smooth(15)
				math.filter( jointAngles:range(s,e,0,mot_col-7),9)
			end

			for i=0, mot:rows()-1 do
				local tf=transf(rootOri(i), rootPos(i))
				MotionDOF.setRootTransformation(mot:row(i), tf)
				mot:row(i):range(7, mot_col):assign(jointAngles:row(i))
			end
		end

		for i=0,mot:rows()-1 do
			skel:setPoseDOF(mot:row(i))
			comtraj(i):assign(skel:calcCOM())
			rotytraj(i):assign(MotionDOF.rootTransformation(mot:row(i)).rotation:rotationY())
		end
		

		local initialHeight=useCase.initialHeight or model.initialHeight
		if initialHeight then
			for i=0, mot:numFrames()-1 do
				mot:row(i):set(1, mot:row(i)(1)+initialHeight)
			end
		end
		if useCase.customInitialHeight then
			local c=useCase.customInitialHeight
			for i=c.startFrame, c.endFrame-1 do
				mot:row(i):set(1, mot:row(i)(1)+c.deltaHeight)
			end
		end

		self.mot_withoutCOMjoint=mot
		-- insert joint after smoothing
		if not useCase.noCOMjoint then	
			if useCase.ignoreRotY then
				for i,interval in ipairs(useCase.ignoreRotY) do
					local s=interval[1]
					local e=interval[2]
					local subroty=rotytraj:range(s,e)
					subroty:transition(rotytraj(s-1),rotytraj(e), e-s)
					-- buggy.. workaround at the moment
					for k=s,e-1 do
						rotytraj(k):assign(subroty(k-s))
					end

					--print('hihi',s,e)
					-- for k=s,e do 
					-- 	print(math.deg(rotytraj(k):rotationY():rotationVector().y))
					-- end
				end
			end
			local segFinder=SegmentFinder(container.discontinuity)
			for i=0, segFinder:numSegment()-1 do
				local s=segFinder:startFrame(i)
				local e=segFinder:endFrame(i)

--##dos if g_debugOneStep then
		--##dos g_debugOneStep:pushBack({"comtraj",comtraj:range(s,e):copy(), rotytraj:range(s,e):copy()})
		--##dos end
				comtraj:range(s,e):smooth(5)
				rotytraj:range(s,e):smooth(15)
--##dos if g_debugOneStep then
		--##dos g_debugOneStep:pushBack({"comtraj",comtraj:range(s,e):copy(), rotytraj:range(s,e):copy()})
		--##dos end
				-- for j=s,e-1 do
				-- 	rotytraj(j):leftMult(quater(math.rad(30), vector3(0,1,0)))
				-- end
			end

			local nmot=MotionUtil.insertRootJoint(skel, comtraj:matView(0, comtraj:size()), rotytraj:matView(0, rotytraj:size()), 'COM', mot)
			mot=nmot
		end

		local dmot=calcDerivative(self.mot_withoutCOMjoint, container.discontinuity)

		self.mot=mot
		self.dmot=dmot
		
		self.leftFoot=container.leftFoot
		self.rightFoot=container.rightFoot
		self.discontinuity= container.discontinuity
		self.skel=skel
	end

	self.lmot=MotionDOF(self.mot)   
	self.lmot:convertToDeltaRep()			
	-- self.lacc=calcAcceleration(self.mot_withoutCOMjoint)
	
	if true then 
		self.lvel=self.dmot
		self.lacc=self.dmot:derivative(120, self.discontinuity)
	else
		self.lvel=self.mot_withoutCOMjoint:matView():derivative(120, self.discontinuity)
		local mot=self.mot_withoutCOMjoint
		for i=0, self.lvel:rows()-2 do
			local V=MotionDOF.rootTransformation(mot:row(i)):twist(
			MotionDOF.rootTransformation(mot:row(i+1)), 1/120)
			self.lvel:row(i):setVec3(0, V.v)
			self.lvel:row(i):setVec3(4, V.w)
		end
		self.lacc=self.lvel:derivative(120, self.discontinuity)
	end
	--math.gaussFilter(3,self.lvel)
	--math.gaussFilter(5,self.lacc)
	
	local offset=quater()
	local nf=self.mot:numFrames()
	self.rotY=quaterN(nf)

	-- calc rot_y		
	for i=0,nf-1 do 
		local tf=self.mot:rootTransformation(i)
		local rot_y=self.rotY(i)
		tf.rotation:decompose(rot_y, offset)
	end

	self:loadOptimizedTrajectory(self.skel_withoutCOMjoint, mot_file)

	self.COMtraj=calcCOMtrajectory(self.skel_withoutCOMjoint, self.mot_withoutCOMjoint, self.discontinuity)
end

function LocoGraphMotion:__finalize()
	if useCase.useFootEncodingMethod2 then
		local param=useCase.useFootEncodingMethod2
		if type(param.geom.L[1][1])~="string" then
			for k,v in pairs(param.geom) do
				-- heel
				param.geom[k][1][1]=param.geom[k][1][1]:name()
				-- toe
				param.geom[k][2][1]=param.geom[k][2][1]:name()
			end
		end
	end
	self.skel=nil
	self.mot=nil
	self.dmot=nil
	self.lmot=nil
	self.ZMPcalculator=nil
end

function setKeyframes(useCase, key, path, values)
	local cp_mod={}

	local numKey=useCase.keyframes[key].numKey
	assert(#path*numKey==#values)
	for iseg, seg in ipairs(path) do
		for j=1, numKey do
			local vidx=(iseg-1)*numKey+j
			if type(values[vidx])=='table' then
				for i=1, #values[vidx] do
					local k='keyframe,'..(j-1)..','..key..','..string.gsub(seg,'%.',',')..','..i
					array.pushBack(cp_mode, k,values[vidx][i])
				end
			end
		end
	end

	print(util.printString(cp_mod))
	dbg.console()
	useCase.controlParam=useCases.mergeCP(useCase.controlParam, cp_mod)
end
function LocoGraph:__finalize()
   self.skel=nil
   self.id=nil
   self.motions:foreach(LocoGraphMotion.__finalize)
   self.motions=nil
end

function LocoGraph:__init(skel, mot_file)
   self.id=InterframeDifference()
   -- id_rotY: temporary variable for rotY prediction
   --self.id_rotY=InterframeDifference()
   --self.id_rotY=InterframeDifferenceRot:new() 
   self.id_rotY=InterframeDifferenceRotPDservo:new() 
   self.groups=vector()
   self.motions=array:new()
   self.motions:pushBack(LocoGraphMotion:new(skel, mot_file))
   self.skel=self.motions[1].skel
   self.skel_withoutCOMjoint=self.motions[1].skel_withoutCOMjoint

end
   
function LocoGraph.createPendulum(outputFrameRate)
	require("control/IPC3d_approx")
	require("control/IPC3d_pivoted")
	if false then
		return CartPoleBallCpp(package.projectPath.."Resource/cart_pole_ball.wrl",0, 9.8,1.0/outputFrameRate, 30000)
	else      
		--return CartPoleBall("../Resource/scripts/ui/RigidBodyWin/cart_pole_ball.wrl",0, 9.8,1.0/self.outputFrameRate, 30000)
		--return IPC3d_cpp("../Resource/scripts/ui/RigidBodyWin/cart_pole_ball.wrl",0, 9.8,1.0/self.outputFrameRate, 30000)

		--if useCase.usePositionControl then
		--	return IPC3d_pivoted("../Resource/scripts/ui/RigidBodyWin/cart_pole_ball.wrl",0, 9.8,1.0/outputFrameRate, useCase.pendulumK or 30000)
		--else
		--	return IPC3d("../Resource/scripts/ui/RigidBodyWin/cart_pole_ball.wrl",0, 9.8,1.0/outputFrameRate, useCase.pendulumK or 30000)
		--end
		return IPC3d_multimodal(package.projectPath.."Resource/cart_pole_ball.wrl",0, 9.8,1.0/outputFrameRate, useCase.pendulumK or 30000)
	end
end
function LocoGraph:setSegment(grpName, name,first, last, nextLast, prevFirst )
	self:addSegment(grpName, name, LocoGraphSegment:new(self, grpName, name, first, last, nextLast, prevFirst ))
end

function LocoGraph.projectFoot(heelpos, toepos)
	if useCase.useToePos then
		return toepos:copy()
	end
	local gfootPos=toepos*0.5+heelpos*0.5

	local anchor, other, support
	if toepos.y<heelpos.y then
		support='t'
		anchor=toepos
		other=heelpos
	else
		support='h'
		anchor=heelpos
		other=toepos
	end

	local q=quater()
	local projectedfootpos=gfootPos:copy()
	projectedfootpos.y=anchor.y
	q:axisToAxis(other-anchor, projectedfootpos-anchor)

	local t=transf()
	t:identity()
	t:leftMultTranslation(anchor*-1)
	t:leftMultRotation(q)
	t:leftMultTranslation(anchor)
	return t*gfootPos
end
function LocoGraph:addSegment(grpName, name, seg)
	if self[grpName]==nil then
		self[grpName]={}
		self.groups:pushBack(grpName)
	end

	self[grpName][name]=seg
	return seg
end

function LocoGraphSegment:sampleKey(delta, srckey)
   local key=self[srckey]
   assert(key)
   local totalLen=key:size()-1
   if totalLen<0 then
	   print('warning! key '..srckey..' missing')
	   return vector3(0,0,0)
   end
   assert(totalLen>=0)
   local keyIdx=totalLen*delta
   --[[if useCase.keyframes[srckey].keytime then
	   local kc=useCase.keyframes[srckey].keytimeCurve
	   assert(kc)
	   keyIdx=kc:sample(delta)
   end]]--

   if totalLen==0 then 
	   return key(0):copy()
   end
   return key:sample(keyIdx)
end

function LocoGraphSegment:updateInterpolatedParams()
	local interpolationInfo=self.interpolated
	local seg=self
	local seg1=interpolationInfo[1]
	local seg2=interpolationInfo[2]
	local startWeight=interpolationInfo[3]
	local endWeight=interpolationInfo[4]

	local function blend(classobj, ...)
		for i, name in ipairs({...}) do
			assert(seg1[name])
			assert(seg2[name])
			seg[name]=classobj['blend'](startWeight, endWeight, seg1[name], seg2[name])
		end
	end
	blend(vector3N, 'lcom', 'lcomvel', 'lcomacc', 'ldotangMomentum','ldotlinMomentum', 'lpenddotangMomentum','lpenddotlinMomentum','langMomentum','linertia', 'llinMomentum', 'lpendangMomentum','lpendlinMomentum', 'lheadvel','lhead', 'ZMPlocal', 'COMlocal', 'zmpToPelvisT', 'comToCOM', 'footL', 'footR')
	blend(matrixn,'lpendDState')
	if useCase.keyframes then
		for k,v in pairs(useCase.keyframes) do
			if k=='subspace' then
			else
				if not v.numKeyFrom then
					if type(v.default)=="number" then
						blend(vectorn, k)
					elseif type(v.default)=="table" then
						blend(matrixn, k)
					else
						blend(vector3N, k)
					end
				end
			end
		end
	end

	blend(quaterN, 'offsetPendQ', 'zmpToPelvisR','zmpcomToFootL','zmpcomToFootR','deltaAngles','deltaRotY') 
	blend(vectorn, 'refL', 'refR','refP', 'COMspeed','stride', 'strideH', 'ZMPspeed', 'comacc_y')
	if useCase.keyframes.importanceLH then
		blend(vector3N, 'footLH', 'footRH')
		blend(vectorn, 'refLH', 'refRH')
		blend(quaterN,'zmpcomToFootLH','zmpcomToFootRH')
	end

	if useCase.calcDesiredFootPosFlightPhase then
		blend(vector3N, 'landingPosError')
	end
end
function LocoGraphSegment:interpolate( seg2, startWeight, endWeight, grpName, name )
	assert(self.updated)
	assert(seg2.updated)

	local seg1=self
	local seg=LocoGraphSegment:new(self.graph, grpName, name)
	local locomot=LocoGraphMotion:new(self.graph.skel)

	locomot.lmot=MotionDOF.blend(startWeight, endWeight, seg1:lmot(), seg2:lmot())
	-- locomot.lacc=calcAcceleration(locomot.lmot)
	locomot.lvel=matrixn.blend(startWeight, endWeight, seg1:lvel(), seg2:lvel())
	locomot.lacc=matrixn.blend(startWeight, endWeight, seg1:lacc(), seg2:lacc())
	math.gaussFilter(5,locomot.lacc)
	locomot.mot=locomot.lmot:copy()
	locomot.mot:convertFromDeltaRep(vector3(0,0,0))

	self.graph.motions:pushBack(locomot)
	seg.locomot=locomot

	seg.first=0
	seg.last=locomot.mot:length()
	seg.nextLast=seg.last+((seg2.nextLast-seg2.last)+(seg1.nextLast-seg1.last))*0.5
	seg.prevFirst=seg.first+((seg2.prevFirst-seg2.first)+(seg1.prevFirst-seg1.first))*0.5
	seg.name=name
	seg.grpName=grpName
	seg.interpolated={seg1, seg2, startWeight, endWeight}

	seg.lroot=seg.locomot.lmot:matView():range(seg.first, seg.last+1,0,7)
	seg.len=seg.last-seg.first
	--seg.desiredVelocity=vector3()
	local avgWeight=startWeight*0.5+endWeight*0.5
	--seg.desiredVelocity:interpolate(avgWeight, seg1.desiredVelocity, seg2.desiredVelocity)
	if avgWeight<=0.5 then
		for k,v in pairs(useCase.segProperties) do
			seg[k]=seg1[k]
		end
	else
		for k,v in pairs(useCase.segProperties) do
			seg[k]=seg2[k]
		end
	end
	seg:updateInterpolatedParams()
	return seg
end

function LocoGraphSegment:initModelSpecific(firstFrame, lastFrame)
      self.zmpToPelvisR=quaterN(lastFrame-firstFrame+1)
      self.zmpToPelvisT=vector3N(lastFrame-firstFrame+1)
	  self.pelvisToZMP_R=quaterN(lastFrame-firstFrame+1)
	  self.pelvisToZMP_T=vector3N(lastFrame-firstFrame+1)
end
function LocoGraphSegment:__init(graph,grpName, name, firstFrame, lastFrame, nextLastFrame, prevFirst )
   --dbg.startTrace()
   
   self.graph=graph
   self.name=name
   self.grpName=grpName
   if useCase.keyframes then
	   for k, v in pairs(useCase.keyframes) do
		   local numKey=v.numKey
		   if v.numKeyFrom and useCase.graphParam[self.grpName] then
			   numKey=useCases.getGraphParam(self.grpName, self.name, 'num_'..v.numKeyFrom)
			   numKey=numKey or 0
		   end
		   --print(self.grpName, self.name, k, 'has', numKey, 'keys')
		   if type(v.default)=="number" then
			   self[k]=vectorn(numKey)
			   self[k]:setAllValue(v.default)
		   elseif type(v.default)=="table" then
			   if v.default[1]=='fromFile' then
				   local cls=self.grpName.."_"..self.name 
				   self[k]=self.graph.keyframes[v.default[2]][cls]
				   if self[k]==nil then
					   self[k]=CT.zeros(numKey,1)
				   end
			   else
				   self[k]=matrixn(numKey, #v.default)
				   for i=1,numKey do
					   self[k]:row(i-1):setValues(unpack(v.default))
				   end
			   end
		   else
			   self[k]=vector3N(numKey)
			   self[k]:setAllValue(v.default)
		   end
	   end
   end

   if firstFrame==nil then return end
   
   self.locomot=self.graph.motions[1]

   self.first=firstFrame
   self.last=lastFrame

   -- following two variables are only for initializing members
   self.nextLast=nextLastFrame
   self.prevFirst=prevFirst
   
   --if graph=="motions" then
   do
      self.lroot=self.locomot.lmot:matView():range(firstFrame, lastFrame+1, 0, 7)
	  -- Global information should not be stored here.
--      self.rotY=self.locomot.rotY:range(firstFrame, lastFrame+1)
--      self.zmp=self.locomot.ZMPcalculator.zmp:range(firstFrame, lastFrame+1)
--      self.com=self.locomot.ZMPcalculator.com:range(firstFrame, lastFrame+1)
--      self.comvel=self.locomot.ZMPcalculator.comvel:range(firstFrame, lastFrame+1)
--
      self.lcom=vector3N(lastFrame-firstFrame+1)
      self.lcomvel=vector3N(lastFrame-firstFrame+1)
	  self.lcomacc=vector3N(lastFrame-firstFrame+1)
      self.lhead=vector3N(lastFrame-firstFrame+1)
      self.lheadvel=vector3N(lastFrame-firstFrame+1)
      self.langMomentum=vector3N(lastFrame-firstFrame+1)
      self.llinMomentum=vector3N(lastFrame-firstFrame+1)
      self.lpendangMomentum=vector3N(lastFrame-firstFrame+1)
      self.lpendlinMomentum=vector3N(lastFrame-firstFrame+1)
      self.lpendDState=matrixn(lastFrame-firstFrame+1,8) -- dx,dz,dq_x, dq_z,  ddx, ddz, ddq_x, ddq_z
      self.lpendlinMomentum=vector3N(lastFrame-firstFrame+1)
	  self.linertia=vector3N(lastFrame-firstFrame+1)
      self.ldotangMomentum=vector3N(lastFrame-firstFrame+1)
      self.ldotlinMomentum=vector3N(lastFrame-firstFrame+1)
      self.lpenddotangMomentum=vector3N(lastFrame-firstFrame+1)
      self.lpenddotlinMomentum=vector3N(lastFrame-firstFrame+1)

      self.ZMPlocal=vector3N(lastFrame-firstFrame+1)
      self.COMlocal=vector3N(lastFrame-firstFrame+1)
      self.offsetPendQ=quaterN(lastFrame-firstFrame+1)
      self.comToCOM=vector3N(lastFrame-firstFrame+1)
      self.deltaAngles=quaterN(lastFrame-firstFrame+1)
      self.footL=vector3N(lastFrame-firstFrame+1)
      self.footR=vector3N(lastFrame-firstFrame+1)
	  self.zmpcomToFootL=quaterN(lastFrame-firstFrame+1)
	  self.zmpcomToFootR=quaterN(lastFrame-firstFrame+1)
	  self.stride=vectorn(lastFrame-firstFrame+1)
	  self.strideH=vectorn(lastFrame-firstFrame+1)
      self.refL=vectorn(4)
      self.refR=vectorn(4)
      self.refP=vectorn(4)
      self.pelvisRef=vectorn(lastFrame-firstFrame+1)
	  if useCase.keyframes.importanceLH then
		  self.footLH=vector3N(lastFrame-firstFrame+1)
		  self.footRH=vector3N(lastFrame-firstFrame+1)
		  self.refLH=vectorn(4)
		  self.refRH=vectorn(4)
		  self.zmpcomToFootLH=quaterN(lastFrame-firstFrame+1)
		  self.zmpcomToFootRH=quaterN(lastFrame-firstFrame+1)
	  end
	  self.comacc_y=vectorn(lastFrame-firstFrame+1)

	  if useCase.calcDesiredFootPosFlightPhase then
		  self.landingPosError=vector3N(lastFrame-firstFrame+1)
	  end


	  --self.desiredVelocity=vector3(0,0,0e
	  --self.desiredVelocity.x= useCase.pendControlParam['keyframe,0,pendDesiredVel,'..grpName..','..name..',x'] or 0
	  --self.desiredVelocity.z= useCase.pendControlParam['keyframe,0,pendDesiredVel,'..grpName..','..name..',z'] or 0
      self.deltaRotY=quaterN(lastFrame-firstFrame+1) -- encode the difference between trajectory tangent and pelvis rot
      
      -- Frame no:  0123456789abcdefghijkl
      -- local fno:     0           c
      --                |    DLD    |
      -- Footstep:  RRRDDDLLLLLLLLLDDDRRRR
      --           
      
      -- segment DLD
      -- footR
      -- Ref1    :  -4
      -- Ref2    :  +4
      -- weightL :  00000000000 ~  0.5 ~ 1
      -- weightR :  0 ~ 0.5 ~ 111111111111
      
      self.COMspeed=vectorn(lastFrame-firstFrame+1)
      self.ZMPspeed=vectorn(lastFrame-firstFrame+1)
   end

   self:initModelSpecific(firstFrame, lastFrame)

   local skel=graph.skel 
   local locomot=self.locomot
   local mot=self.locomot.mot

   do -- default setup of footRef timing

      local len=lastFrame-firstFrame
      local ref=CT.vec(0, 0,0,1)

      self.refL=ref:copy()
      self.refR=ref:copy()
      self.refP=ref:copy()
	  if useCase.keyframes.importanceLH then
		  self.refLH=ref:copy()
		  self.refRH=ref:copy()
	  end
      self.len=len
   end

   if locomot.ZMPcalculator.comvel:size()~=0 then

	   if(locomot.ZMPcalculator.comvel:size()>lastFrame) then

		   for i=firstFrame, lastFrame do
			   local comVel=locomot.ZMPcalculator.comvel(i)
			   local rotY=locomot.rotY(i)

			   self.deltaAngles(i-firstFrame):assign(self.calcDeltaAngle(comVel, rotY))
			   self.deltaRotY(i-firstFrame):identity()

			   if locomot.ZMPcalculator.zmpvel then
				   local zmpspeed1=locomot.ZMPcalculator.zmpvel(i):length()
				   --	print(zmpspeed1,zmpspeed2,'speed')
				   self.ZMPspeed:set(i-firstFrame, zmpspeed1)
			   else
				   local zmpspeed2=(locomot.ZMPcalculator.zmp(i)
				   -locomot.ZMPcalculator.zmp(math.min(i+1,lastFrame))):length()*120
				   self.ZMPspeed:set(i-firstFrame, zmpspeed2)
			   end
			   self.COMspeed:set(i-firstFrame, comVel:length())
		   end
	   else
		   error(' incorrect ZMPcalculator information. Redo "lua short.lua ipco" after removing *.zmpcom!')
	   end
   else
	   dbg.console()
	   -- I guess unused code
	   locomot.ZMPcalculator.com:resize(mot:numFrames())
      locomot.ZMPcalculator.zmp:resize(mot:numFrames())
      for i=0, mot:numFrames()-1 do
	 local pelvis=MotionDOF.rootTransformation(mot:row(i)).translation
	 locomot.ZMPcalculator.com(i):assign(pelvis)
	 locomot.ZMPcalculator.zmp(i):assign(pelvis)
	 locomot.ZMPcalculator.zmp(i).y=0
      end
   end
end

function LocoGraphSegment.calcDeltaAngle(comVel, rotY)
   
   local trot=quater()
   local deltaAngle=quater()
   trot:setAxisRotation(vector3(0,1,0), vector3(0,0,1), comVel)

   deltaAngle:difference(trot, rotY)

   local speedMin=1 -- 1m/s

   deltaAngle:scale(sop.clampMap(comVel:length(), speedMin/2, speedMin, 0, 1))

   return deltaAngle
end


function LocoGraphSegment:calcFootShearTopPosition(refFrame)
	return self.locomot.ZMPcalculator.com:sampleRow(refFrame)
end

-- represent the foot position at curFrame with respect to a coordinate frame defined at refFrame.
-- this function assumes that skel:setPoseDOF(mot:row(curFrame)) has been called before itself being called.
function LocoGraphSegment:calcLocalPelvisAndFoot(curFrame)
   
	local graph=self.graph
	local locomot=self.locomot
	local mot=locomot.mot
	local firstFrame=self.first
	local lcurFrame=curFrame-firstFrame
	local tf=mot:rootTransformation(curFrame)
	local rot_y=locomot.rotY(curFrame)
	local zmp=locomot.ZMPcalculator.zmp(curFrame)
	local com=locomot.ZMPcalculator.com(curFrame)
	self.ZMPlocal(lcurFrame):assign(tf:toLocalPos(zmp))
	self.COMlocal(lcurFrame):assign(tf:toLocalPos(com))

	if useCase.calcDesiredFootPosFlightPhase then
		if locomot.ZMPcalculator.landingPosError then
			local landingPosError=locomot.ZMPcalculator.landingPosError:row(curFrame)
			self.landingPosError(lcurFrame):rotate(rot_y:inverse(), landingPosError)
		end
	end


	-- calc local foot positions

	-- foot encoding : see "foot decoding" for synthesis part. (locosynthesis.lua)
	local function encode(isL, ref, weight, foot, footpos, currFrame, firstFrame, lastFrame)

		local ret_stride=0
		local ret_zmp2
		-- calculate zmp coord
		local weight=sop.map(curFrame, firstFrame, lastFrame, weight(0), weight(1))
		--if useCase.smoothSwingFootTransition then
			--weight=math.smoothTransition(weight)
		--end
		local frame0=self:_timeToFrame(ref(0))
		local frame1=self:_timeToFrame(ref(1))
		if self.usePositionControl then
			--dbg.console()
			frame0=currFrame
			frame1=currFrame
		end
		local refFrame=frame0+weight*(frame1 -frame0)-- in [ref(0) , ref(1)]
		if false then 
			if currFrame==firstFrame or currFrame==lastFrame then -- debug output
				print(currFrame, firstFrame, lastFrame, ":", 
					  refFrame, self:_timeToFrame(ref(0)), self:_timeToFrame(ref(1)))
			end
		end
		local rot_y_ref=self.locomot.rotY:sampleRow(refFrame)

		local com_ref
		local zmp_ref
		local zmp_coord_ref
		local useShearedTransform =true
		do

			--if isL=='L' then print(frame0, frame1, refFrame, weight) end
			local zmp1= self.locomot.ZMPcalculator.zmp:sampleRow(frame0)
			local zmp2= self.locomot.ZMPcalculator.zmp:sampleRow(frame1)

			ret_stride=zmp1:distance(zmp2)
			ret_zmp2=zmp2
			-- local gamma=0.5
			-- weight=math.pow(weight, gamma)

			if true then -- use rot_y sampled at the frame for the foot
				local rot_y1=self.locomot.rotY:sampleRow(frame0)
				local rot_y2=self.locomot.rotY:sampleRow(frame1)
				rot_y_ref:safeSlerp(rot_y1, rot_y2, weight)
			end

			zmp_ref=vector3()
			zmp_ref:interpolate(weight, zmp1, zmp2)

			zmp_coord_ref=matrix4(rot_y_ref, zmp_ref)
			if useShearedTransform then
				local com1=self:calcFootShearTopPosition(frame0)
				local com2=self:calcFootShearTopPosition(frame1)

				tcom=vector3()
				tcom:interpolate(weight, com1, com2)
				com_ref=self:calcFootShearTopPosition(refFrame)

				if not useCase.noIntersectionPrevenction then
					zmp_ref=zmp_ref+com_ref-tcom
				end

				if false  and isL=="L" then -- debug draw
					dbg.namedDraw('Traj', self.locomot.ZMPcalculator.com:matView(firstFrame, lastFrame)*100,'comtraj')
					RE.output2("refFrame", ref(0), ref(1), 
					frame0, frame1,
					currFrame, refFrame, prevRefframe==nil or refFrame-prevRefframe)
					prevRefframe=refFrame
					local skel=self.graph.skel 
					local locomot=self.locomot
					if gSkin==nil then

						gSkin= RE.createVRMLskin(skel, false)
						gSkin:scale(100,100,100)
					end
					gSkin:setPoseDOF(locomot.mot:row(currFrame))
					dbg.draw('Sphere', zmp_coord_ref:getTranslation()*100, "zmp")
					if com_ref then
						dbg.namedDraw('Sphere', com_ref*100, "com_ref")
						if tcom then
							dbg.namedDraw('Sphere', tcom*100+vector3(0,15,0), "tcom")
						end
						dbg.namedDraw('Sphere', self.locomot.ZMPcalculator.com(currFrame)*100+vector3(0,7,0),"com")
					end
					noFrameMove=true
					RE.renderOneFrame(false)
					noFrameMove=nil
				end
				--	 zmp_coord_ref= CT.shearY(com_ref-zmp_ref)*zmp_coord_ref
				zmp_coord_ref= matrix4(quater(1,0,0,0), zmp_ref)*CT.shearY(com_ref-zmp_ref)*matrix4(rot_y_ref, vector3(0,0,0))
			end
		end	 

		-- encode foot position and orientation
		local method=useCase.useFootEncodingMethod2
		if method and method[1] then
			-- calc foot position:
			-- use different methods for swing foot and support foot
			local toepos, heelpos
			local lposinfo=method.geom[isL]
			heelpos=lposinfo[1][1]:getFrame():toGlobalPos(lposinfo[1][2])
			toepos=lposinfo[2][1]:getFrame():toGlobalPos(lposinfo[2][2])

			--local gfootPos=foot:getFrame():toGlobalPos(footpos)
			local gfootPos=LocoGraph.projectFoot(heelpos,toepos)

			self['foot'..isL](lcurFrame):assign(
			zmp_coord_ref:inverse()*gfootPos)

			-- footR=rotY*offsetQ
			---> offsetQ=rotY:inverse()*footR
			--self['zmpcomToFoot'..isL](lcurFrame):assign(rot_y:inverse()*lposinfo[1][1]:getFrame().rotation)
			--self['zmpcomToFoot'..isL](lcurFrame):assign(self.graph.skel:bone(1):getFrame().rotation:inverse()*lposinfo[1][1]:getFrame().rotation)
			local shearRot=quater()
			--local comCurr= self:calcFootShearTopPosition(weight)
			local comCurr=com_ref
			shearRot:axisToAxis(vector3(0,1,0), comCurr-zmp_ref)
			local footOriRefCoord=shearRot*rot_y_ref
			--footOriRefCoord*delta=currFootOri
			self['zmpcomToFoot'..isL](lcurFrame):assign(footOriRefCoord:inverse()*lposinfo[1][1]:getFrame().rotation)

			--      util.outputToFile("ddd.txt", "a"..tostring(curFrame)..":roty:"..tostring(rot_y_ref)..
			--		  ":zmp12:"..tostring(zmp1)..tostring(zmp2)..":w:"..tostring(weight).." "..tostring(firstFrame+self.refL(0)).." "..tostring(firstFrame+self.refL(1)))
			--calc foot orientation : zmpcomToFootL	
			if false and currFrame<700 and isL=="L" then -- debug draw
				local skel=self.graph.skel 
				local locomot=self.locomot
				if gSkin==nil then
					gSkin= RE.createVRMLskin(skel, false)
					gSkin:scale(100,100,100)
				end
				gSkin:setPoseDOF(locomot.mot:row(currFrame))
				--dbg.draw('Sphere',gfootPos*100, "foot")
				dbg.draw('Sphere',comCurr*100, "COM",'green', 10)
				dbg.draw('Sphere',zmp_ref*100, "zmp_ref")
				noFrameMove=true
				RE.renderOneFrame(false)
				noFrameMove=nil
			end
		else
			local gfootPos=foot:getFrame():toGlobalPos(footpos)

			self['foot'..isL](lcurFrame):assign(
			zmp_coord_ref:inverse()*gfootPos)

			--      util.outputToFile("ddd.txt", "a"..tostring(curFrame)..":roty:"..tostring(rot_y_ref)..
			--		  ":zmp12:"..tostring(zmp1)..tostring(zmp2)..":w:"..tostring(weight).." "..tostring(firstFrame+self.refL(0)).." "..tostring(firstFrame+self.refL(1)))
		end
		return ret_stride, ret_zmp2
	end

	assert(curFrame<=self.last)
	do
		local lfootStride,lpos=encode('L', self.refL:range(0,2), self.refL:range(2,4), graph.lfoot, graph.lfootpos, curFrame, self.first, self.last)
		local rfootStride,rpos=encode('R', self.refR:range(0,2), self.refR:range(2,4), graph.rfoot, graph.rfootpos, curFrame, self.first, self.last)
		local maxStride=math.max(lfootStride, rfootStride)
		local stride=0
		if maxStride>0 then
			stride=lpos:distance(rpos)
		end
		self['stride']:set(lcurFrame, stride)
	end
	if useCase.keyframes.importanceLH then
		lfootStride,lpos=encode('LH', self.refLH:range(0,2), self.refLH:range(2,4), graph.lhand, graph.lhandpos, curFrame, self.first, self.last)
		rfootStride,rpos=encode('RH', self.refRH:range(0,2), self.refRH:range(2,4), graph.rhand, graph.rhandpos, curFrame, self.first, self.last)
		local maxStride=math.max(lfootStride, rfootStride)
		local stride=0
		if maxStride>0 then
			stride=lpos:distance(rpos)
		end
		self['strideH']:set(lcurFrame, stride)
	end

	if useCase.keyframes.importanceLH then
		encode('LH', self.refLH:range(0,2), self.refLH:range(2,4), graph.lhand, graph.lhandpos, curFrame, self.first, self.last)
		encode('RH', self.refRH:range(0,2), self.refRH:range(2,4), graph.rhand, graph.rhandpos, curFrame, self.first, self.last)
	end
end

function LocoGraph:setRefTiming()

	--[[
	local function setFootRef(seg, isL, ref1, ref2, offset)
	--      assert(ref2==ref2_)

	local ref=CT.vec({ref1+offset,ref2+offset-1})

	local len=seg.len


	if isL then
	seg.refL=ref
	else
	seg.refR=ref
	end
	end

	do
	local STP=self.STP
	local StR=self.StR
	local run=self.run

	seq={STP.st, StR.SR, StR.RL, StR.LR, run.RL, run.LR}

	local c=0.5
	--      local c=0.7
	--                   1   2   3    4   5   6   7
	--                  St   S   R    L   R   L   R   L
	local mapL1=   {     1+c,2+c,2+c,4+c,4+c,6+c,6+c,8+c}
	local mapL2=   {	   2+c,2+c,4+c,4+c,6+c,6+c,8+c,8+c}
	local mapR1=   {	   1+c,2+c,3+c,3+c,5+c,5+c,7+c,7+c}
	local mapR2=   {	   2+c,3+c,3+c,5+c,5+c,7+c,7+c,9+c}
	--      local offset=  {   0.3,0.3,0.3,0.4,0.5,0.5,0.5,0.5}


	--      local offset=  {   co,co,co,co,co,co,co,co}

	for i=1,table.getn(seq) do
	setFootRef(seq[i], true, mapL1[i], mapL2[i], -i)
	setFootRef(seq[i], false, mapR1[i], mapR2[i], -i)
	end
	end
	]]--
end

function LocoGraphSegment:calcModelSpecificInfo(curFrame)
	local graph=self.graph
	local locomot=self.locomot
	local mot=locomot.mot
	local firstFrame=self.first
	local lcurFrame=curFrame-firstFrame
	local tf=mot:rootTransformation(curFrame)
	do 
		-- calc ZMP root orientation
		local weight=sop.map(curFrame, self.first, self.last,0,1)
		local segIndex=self.segInfo[2]
		--local frame0=self:_timeToFrame(segIndex+self.refP(0))
		--local frame1=self:_timeToFrame(segIndex+self.refP(1))
		--local refFrame=frame0+weight*(frame1-frame0)
		local refFrame=curFrame
	
		--print('curFrame, refFrame', curFrame, refFrame)
		local rot_y= self.locomot.rotY:sampleRow(refFrame)
		local com= self.locomot.ZMPcalculator.com:sampleRow(refFrame)
		local zmp= self.locomot.ZMPcalculator.zmp:sampleRow(refFrame)

		local ldir=com-zmp
		local pend_q=self.offsetPendQ(lcurFrame)
		ldir:rotate(rot_y:inverse())

		pend_q:axisToAxis(vector3(0,1,0), ldir)

		local zmp_coord=transf(rot_y*pend_q, zmp)
		local zmpToPelvis=zmp_coord:toLocal(tf) -- == zmp_coord:inverse()*tf

		self.zmpToPelvisR(lcurFrame):assign(zmpToPelvis.rotation)

		if useZMPtoPelvis_PendCOM then
			local pendCOM=zmp_coord:toGlobalPos(graph.pendLocalCOM)
			self.zmpToPelvisT(lcurFrame):assign(rotate(tf.translation-pendCOM, rot_y:inverse()))
		else
			self.zmpToPelvisT(lcurFrame):assign(zmpToPelvis.translation)
		end

		-- pelvis*zmpToPelvis=zmp_coord
		--> zmpToPelvis=pelvis:inverse()*zmp_coord
	end
end
function LocoGraphSegment:calculatePendControlParamDependentMembers()
	local firstFrame=self.first
	local lastFrame=self.last
	local name=self.name
	local grpName=self.grpName
	local graph=self.graph
	local locomot=self.locomot
	local skel=graph.skel 
	local mot=locomot.mot

	local com=self.locomot.ZMPcalculator.com:range(firstFrame, lastFrame+1)
	local comvel=self.locomot.ZMPcalculator.comvel:range(firstFrame, lastFrame+1)
	local comacc=self.locomot.ZMPcalculator.comacc:range(firstFrame, lastFrame+1)

	for i=firstFrame, lastFrame do
		local refTime=sop.map(i, firstFrame, lastFrame, 0,1)
		local pelvis=MotionDOF.rootTransformation(mot:row(i)) -- actually not pelvis, but the reference forward facing joint
		pelvis.rotation:assign(pelvis.rotation:rotationY())

		--local pendSim=graph.pendulumSimulator
		if true then
			local pend=graph.pendulum
			local zmp=locomot.ZMPcalculator.zmp(i):copy()
			local com=locomot.ZMPcalculator.com(i):copy()
			local zmp_vel=locomot.ZMPcalculator.zmpvel(i):copy()
			local com_vel=locomot.ZMPcalculator.comvel(i):copy()

			pend:changeMode(self.usePositionControl)

			if not useCase.noTimeVaryingInertia then
				local linertia=self.linertia(i-firstFrame)
				pend.pole:setLocalCOM(vector3(0, linertia.y,0))
				pend:setInertia(linertia)
			end
			pend:setState(zmp,com, zmp_vel, com_vel)
			local rot_y=MotionDOF.rootTransformation(mot:row(firstFrame)).rotation:rotationY()
			pend:setOrientation2(rot_y)
			--##dos if false and grpName=="straddle" then if i==5718 then g_debugPEND:pushBack({"01",self.usePositionControl, i, zmp, com, zmp_vel, com_vel}) g_debugPEND:pushBack({"02", pend:getStates()}) end end
			if self.usePositionControl then
				local desiredLeaning=self:sampleKey(refTime,'desiredLeaning')
				desiredLeaning:set(0,0,0) -- 여기서는 최적화 되기 전의 값을 써야함!!
				pend:setDesiredOrientation(desiredLeaning)
				if grpName=="straddle" then
					if i==5718 then 
						g_debugPEND:pushBack({"61",desiredLeaning})
						g_debugPEND:pushBack({"62", 
						pend.ipc_pivoted.pendX.xd,
						pend.ipc_pivoted.pendZ.xd,
					})
					end
				end
				if false and firstFrame==20 then
					print('lpendDState:1:',desiredLeaning)
					--print("lpendDState:1:", self.linertia(i-firstFrame).y, i,firstFrame)
					--print("lpendDstate:11:", self.usePositionControl, useCase.noTimeVaryingInertia) 
					--print('lpendDState:1:',zmp, com, com_vel, zmp_vel, pend.theta, pend.dtheta,desiredLeaning, useCase.name, useCase.grpName)
					--print('lpendDState:1:', self.desiredLeaning)
					--printTable(pend:getStates(), true, 400)
					--printTable(table.grep(useCase.controlParam, 'desiredLeaning'))
					--this('exit!',1)
					--dbg.console()
				end
			else
				local desiredVelocity=self:sampleKey(refTime,'pendDesiredVel')
				if desiredVelocity:length()==0 then
					print('Error! It seems that unoptimized pendulum parameter is used somehow', name, grpName)
					print('if you want to ignore this error, type')
					print('ignoreDesiredVel=true')
					print('This is not recommended. Running createInitialPendControlParam.lua ')
					print('without non-zero default penddesiredvel will remove this error message')
					print('cont')
					if not ignoreDesiredVel then
						dbg.console()
					end
				end
				--print(desiredVelocity, self.pendDesiredVel)
				desiredVelocity:rotate(rot_y)
				pend:setDesiredVelocity(desiredVelocity)
				--##dos if grpName=="straddle" then if i==5718 then g_debugPEND:pushBack({"31",desiredVelocity}) g_debugPEND:pushBack({"32", pend:getStates()}) end end
			end
			pend:oneStep()
			--##dos if grpName=="straddle" then if i==5718 then g_debugPEND:pushBack({"11", i, zmp, com, zmp_vel, com_vel}) g_debugPEND:pushBack({"12", pend:getStates()}) end end

			local pose_pend=pend.theta
			local vel_pend=pend.dtheta:copy()
			vel_pend:set(0, pend.dtheta(2))
			vel_pend:set(1, pend.dtheta(0))
			vel_pend:set(2, 0)

			--pendSim:setLinkData(0,Physics.DynamicsSimulator.JOINT_VALUE, pose_pend)
			--pendSim:setLinkData(0,Physics.DynamicsSimulator.JOINT_VELOCITY, vel_pend)
			--pendSim:initSimulation()

			local skel_pend=graph.pendulum.loader
			local pole=skel_pend:VRMLbone(2)
			if false then
				-- verify
				skel_pend:setPoseDOF(pose_pend)
				local p=pole:getFrame():toGlobalPos(pole:localCOM())
				local v=pendSim:getWorldVelocity(0, pole, pole:localCOM())
				local a=pendSim:getWorldAngVel(0, pole)
				print('angvel',a, vel_pend)
				print('pos', p, com)
				local p=pole:getFrame():toGlobalPos(vector3(0,0,0))
				print('pos2', p, zmp)
				print('vel', v, com_vel)
				local v=pendSim:getWorldVelocity(0, pole, vector3(0,0,0))
				print('zmpvel', v, zmp_vel)
				dbg.console()
				local mmm=pendSim:calcMomentumCOM(0)
			end
			local mmt,lmmt=pend:calcMomentum()
			self.lpendangMomentum(i-firstFrame):assign(pelvis:toLocalDir(mmt))
			self.lpendlinMomentum(i-firstFrame):assign(pelvis:toLocalDir(lmmt))
			local dmmt=pend:calcDotMomentum()
			local dlmmt=pend:calcDotLinMomentum()
			self.lpenddotangMomentum(i-firstFrame):assign(pelvis:toLocalDir(dmmt))
			self.lpenddotlinMomentum(i-firstFrame):assign(pelvis:toLocalDir(dlmmt))
			do -- calc pendDState
				local pose_pend=pend.theta:copy()
				local vel_pend=pend.dtheta:copy()
				local zmp=locomot.ZMPcalculator.zmp(i+1):copy()
				local com=locomot.ZMPcalculator.com(i+1):copy()
				local zmp_vel=locomot.ZMPcalculator.zmpvel(i+1):copy()
				local com_vel=locomot.ZMPcalculator.comvel(i+1):copy()
				pend:setState(zmp,com, zmp_vel, com_vel)
				local pose_pend2=pend.theta:copy()
				local vel_pend2=pend.dtheta:copy()
				--print(pose_pend,pose_pend2)
				local dx=vector3(pose_pend2(1)-pose_pend(1),0,pose_pend2(0)-pose_pend(0))
				local dq=pose_pend2:toQuater(2):rotationVector()-pose_pend:toQuater(2):rotationVector()
				local ddx=vel_pend2:toVector3(0)-vel_pend:toVector3(0)
				local ddq=vel_pend2:toVector3(3)-vel_pend:toVector3(3)
				dx:rotate(rot_y:inverse())
				dq:rotate(rot_y:inverse())
				ddx:rotate(rot_y:inverse())
				ddq:rotate(rot_y:inverse())
				self.lpendDState:row(i-firstFrame):setValues(dx.x, dx.z, dq.x, dq.z, ddx.x, ddx.z, ddq.x, ddq.z)
				pend:initState()
				--##dos if grpName=="straddle" and i==5718 then g_debugPEND:pushBack({dx, dq, ddx, ddq}) end
			end
			--##dos if true and grpName=='straddle' then if i==5718 then g_debugPEND:pushBack({i, iinterval, dmmt}) g_debugPEND:pushBack({i, iinterval, pelvis:toLocalDir(dmmt)}) end end
		else
			self.lpendangMomentum(i-firstFrame):assign(vector3(0,0,0))
			self.lpendlinMomentum(i-firstFrame):assign(vector3(0,0,0))
			self.lpenddotangMomentum(i-firstFrame):assign(vector3(0,0,0))
			self.lpenddotlinMomentum(i-firstFrame):assign(vector3(0,0,0))
		end

	end
end

function LocoGraphSegment:calculateMembers()
	local firstFrame=self.first
	local lastFrame=self.last
	local name=self.name
	local grpName=self.grpName
	local graph=self.graph
	local locomot=self.locomot
	local skel=graph.skel 
	local mot=locomot.mot

	local com=self.locomot.ZMPcalculator.com:range(firstFrame, lastFrame+1)
	local comvel=self.locomot.ZMPcalculator.comvel:range(firstFrame, lastFrame+1)
	local comacc=self.locomot.ZMPcalculator.comacc:range(firstFrame, lastFrame+1)

	for i=firstFrame, lastFrame do

		skel:setPoseDOF(mot:row(i)) 
		self:calcLocalPelvisAndFoot(i)
		self:calcModelSpecificInfo(i)
		self.comToCOM(i-firstFrame):assign(rotate(locomot.COMtraj(i)-com(i-firstFrame),locomot.rotY(i):inverse()))
		-- if i==300 then
		-- 	print(locomot.COMtraj(i).y, com(i-firstFrame).y)
		-- 	dbg.console()
		-- end

		local pelvis=skel:VRMLbone(1):getFrame():copy() -- actually not pelvis, but the reference forward facing joint
		pelvis.rotation:assign(pelvis.rotation:rotationY())

		self.lcom(i-firstFrame):assign(pelvis:toLocalPos(com(i-firstFrame)))
		self.lcomvel(i-firstFrame):assign(pelvis:toLocalDir(comvel(i-firstFrame)))
		self.lcomacc(i-firstFrame):assign(pelvis:toLocalDir(comacc(i-firstFrame)))
		self.comacc_y:set(i-firstFrame,comacc(i-firstFrame).y)

		if true then
			local head=skel:VRMLbone(skel:getTreeIndexByVoca(MotionLoader.HEAD))
			local prevHeadPos=head:getFrame():toGlobalPos(head:localCOM())
			skel:setPoseDOF(mot:row(i+1))
			local headPos=head:getFrame():toGlobalPos(head:localCOM())
			self.lhead(i-firstFrame):assign(pelvis:toLocalPos(headPos))

			local headVel=(headPos-prevHeadPos)*120
			self.lheadvel(i-firstFrame):assign(pelvis:toLocalDir(headVel))

			assert(graph.simulator)
			local pose=locomot.mot_withoutCOMjoint:row(i)
			graph.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, pose) 
			--graph.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY,MotionDOF.convertDPoseToDState(pose,locomot.dmot:row(i),locomot.mot_withoutCOMjoint.dofInfo:numSphericalJoint()) ) 
			graph.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY,locomot.lvel:row(i))
			graph.simulator:initSimulation()
			local mmm=graph.simulator:calcMomentumCOM(0)
			self.langMomentum(i-firstFrame):assign(pelvis:toLocalDir(mmm:M()))
			self.llinMomentum(i-firstFrame):assign(pelvis:toLocalDir(mmm:F()))

			do -- calc inertia
				local inertia3=vector3()
				local inertia=vectorn()

				graph.simulator:calcInertia(0, pose, inertia)
				inertia3.x=inertia(0)
				inertia3.y=inertia(1)
				inertia3.z=inertia(2)
				inertia3=pelvis:toLocalDir(inertia3)
				inertia3.y=locomot.ZMPcalculatorFullbody.com(i).y
				self.linertia(i-firstFrame):assign(inertia3)
			end

			--if name=='RL' and grpName=='walk3' then 
			--	print(pelvis.translation.y, headPos.y, self.lhead(i-firstFrame).y)
				--dbg.console()
			--end
			do 
				local pose=locomot.mot_withoutCOMjoint:row(i+1)
				graph.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, pose) 
				--graph.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY,MotionDOF.convertDPoseToDState(pose,locomot.dmot:row(i),locomot.mot_withoutCOMjoint.dofInfo:numSphericalJoint()) ) 
				graph.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY,locomot.lvel:row(i+1))
				graph.simulator:initSimulation()
				local mmm2=graph.simulator:calcMomentumCOM(0)
				self.ldotangMomentum(i-firstFrame):assign((pelvis:toLocalDir(mmm2:M())-self.langMomentum(i-firstFrame))*120)
				self.ldotlinMomentum(i-firstFrame):assign((pelvis:toLocalDir(mmm2:F())-self.llinMomentum(i-firstFrame))*120)
			end
			if false then -- verify setLinkData
				local skel=graph.skel_withoutCOMjoint
				local head=skel:VRMLbone(skel:getTreeIndexByVoca(MotionLoader.HEAD))
				--print(head, head:localCOM())
				--print('pos',ii,prevHeadPos, graph.simulator:getWorldPosition(0,head,head:localCOM()))
				local v2=graph.simulator:getWorldVelocity(0,head,head:localCOM())
				--print('velerr',(headVel-v2):length(),headVel,v2 )
			end
		end
	end
	--   math.plotVec(self.footL:matView():column(1):copy())
	self.updated=true

end

function LocoGraphSegment:COMvel()
   return locomot.ZMPcalculator.COMvel:range_c(self.first, self.last)
end

function LocoGraphSegment:COMacc()
   return locomot.ZMPcalculator.COMacc:range_c(self.first, self.last)
end

function LocoGraphSegment:length()
   return self.last-self.first
end

function LocoGraphSegment:mot()
   return self.locomot.mot:range_c(self.first, self.last)
end
function LocoGraphSegment:lmot()
   return self.locomot.lmot:range_c(self.first, self.last)
end
function LocoGraphSegment:lacc()
   return self.locomot.lacc:range_c(self.first, self.last)
end
function LocoGraphSegment:lvel()
   return self.locomot.lvel:range_c(self.first, self.last)
end

function LocoGraphSegment:dmot()
   return self.locomot.lvel:range_c(self.first, self.last)
end
function LocoGraphSegment:pose(frac)-- frac in [0,1] domain.
   local out=vectorn()
   self.locomot.mot:samplePose(sop.map(frac, 0,1, self.first,self.last), out)
   return out
end

function LocoGraphSegment:conLFoot()
   return self.locomot.leftFoot:range(self.first, self.last+1)
end

function LocoGraphSegment:conRFoot()
   return self.locomot.rightFoot:range(self.first, self.last+1)
end

function LocoGraphSegment:__eq(other)
   return self.first==other.first and self.last==other.last
end


LocoGraph.beginGroup=MotionGraph.beginGroup
LocoGraph.connect=MotionGraph.connect

function LocoGraph.getNumSeg(firstFrames)
	return useCases.getNumSeg(firstFrames)
end

function LocoGraph:getControlParameterFromUseCase(title)
	local function retrieveKey(title)
		local tokens=string.tokenize(title,',')
		local idx=tonumber(tokens[2])
		local nameid=tokens[3]
		--if useCase.keyframes[nameid].isContinuous then
		--	idx=idx+1
		--end
		local grp=tokens[4]
		local segNames=string.tokenize(tokens[5],'/')
		local group=self[grp]
		if group==nil then return 1e10 end -- group or seg hasn't created (probably because it is unnecessary).
		local seg=group[segNames[1]]
		if seg==nil then return 1e10 end

		local v=useCases[grp].controlParam[title]
		if v==nil then
			print('warning! retrieve key '..title)
			v=0
		end
		assert(v)
		return v
	end
	return self:_getControlParameter(title, retrieveKey)
end

function LocoGraph:getControlParameterFromGraph(title)
	local function retrieveKey(title)
		local tokens=string.tokenize(title,',')
		local idx=tonumber(tokens[2])
		local nameid=tokens[3]
		--if useCase.keyframes[nameid].isContinuous then
		--	idx=idx+1
		--end
		local grp=tokens[4]
		local segNames=string.tokenize(tokens[5],'/')
		local group=self[grp]
		if group==nil then return 1e10 end -- group or seg hasn't created (probably because it is unnecessary).
		local seg=group[segNames[1]]
		if seg==nil then return 1e10 end

		if nameid=='subspace' then
			local v=useCases[grp].controlParam[title]
			assert(v)
			return v
		end
		local v=seg[nameid]
		if v:size()<=idx then
			--print('getControlParamFromGraph:ignoring '..title)
			return 1e10
		end
		return v(idx)[tokens[6]]
	end
	return self:_getControlParameter(title, retrieveKey)
end

function LocoGraph:_getControlParameter(title, retrieveKey)
	local curval
	-- set initial solution (od.curval) from current keyframes 
	if string.sub(title,1,12)=="keyframeall," then
		curval=0
	elseif string.sub(title, 1, 8)=="keyframe" then
		curval=retrieveKey(title)
		assert(curval)
	elseif string.sub(title, 1,3)=="map" then
		local tbl=useCase.mapControlParam(self, title, 1)
		local val=0
		for it, iv in ipairs(tbl) do
			val=val+retrieveKey(iv[1])*iv[2]
		end
		curval=val/table.getn(tbl)
	elseif string.sub(title,1,9)=="useCases," then
		local tokens=string.tokenize(string.sub(title,10),',')
		local useCase=useCases[tokens[1]]
		if #tokens==3 then
			curval=useCase[tokens[2]][tokens[3]]
		else
			curval=useCase[tokens[2]]
		end
	elseif string.sub(title,1,8)=="useCase," then
		print(title)
		assert(false)
	elseif string.sub(title,1,6)=="model," then
		local tokens=string.tokenize(title,',')
		if #tokens==3 then
			curval=model[tokens[2]][tokens[3]]
		else
			curval=model[tokens[2]]
		end
	else
		assert(false)
		curval=useCase.controlParam[title]
	end
	return curval
end
function LocoGraph:changeControlParameterInUseCase(title, param1)
	local graph=self
	do
		if string.sub(title,1,9)=="keyframe," then
			local tokens=string.tokenize(title,',')
			local idx=tonumber(tokens[2])
			local name=tokens[3]
			assert(useCase.keyframes)
			assert(name)
			assert(useCase.keyframes[name])
			--if useCase.keyframes[name].isContinuous then
			--		idx=idx+1
			--	end
			
			local grp=tokens[4]
			local segNames=string.tokenize(tokens[5],'/')

			if useCases[grp].controlParam[title]==nil then
			--	dbg.console()
			end
			useCases[grp].controlParam[title]=param1
		elseif string.sub(title,1,4)=="map," then
			useCase.controlParam[title]=param1
		else
			--assert(false)
		end
	end
end

-- change graph
function LocoGraph:changeControlParameter(title, param1, ignoreignore, pattern)
	-- seg.transitions[...] is deprecated. use seg.keyframes array instead. -- edge is no longer used. 
	local graph=self
	if type(param1)=="table" then
		return
	end
	if pattern then
		if not select(1,string.find(title, pattern)) then return end
		--print('cp: ', title)
	end
	do
		if string.sub(title,1,4)=="map," then
			local tbl=useCase.mapControlParam(graph, title, param1)
			for i, v in ipairs(tbl) do
				self:changeControlParameter(v[1], v[2],ignoreignore)
			end
		elseif string.sub(title,1,12)=="keyframeall," then
			local out=useCases.convertKeyframeall(title, useCase)
			for i,key in ipairs(out) do
				local v=self:getControlParameterFromGraph(key)
				self:changeControlParameter(key, v+param1)
			end
		elseif string.sub(title, 1,9)=='setBases,' then
			useCases.setBases(useCase, title, param1)
		elseif string.sub(title,1,8)=="useCase," then
			print(title)
			assert(false)
		elseif string.sub(title,1,9)=='useCases,' then
			local tokens=string.tokenize(string.sub(title,10),',')
			local useCase=useCases[tokens[1]]
			if #tokens==3 then
				useCase[tokens[2]][tokens[3]]=param1
			else
				useCase[tokens[2]]=param1
			end
		elseif string.sub(title,1,6)=="model," then
			local tokens=string.tokenize(title,',')
			if #tokens==3 then
				model[tokens[2]][tokens[3]]=param1
			else
				model[tokens[2]]=param1
			end
		elseif string.sub(title,1,9)=="keyframe," then
			local tokens=string.tokenize(title,',')
			local idx=tonumber(tokens[2])
			local name=tokens[3]
			assert(useCase.keyframes)
			assert(name)
			assert(useCase.keyframes[name])
			--if useCase.keyframes[name].isContinuous then
			--		idx=idx+1
			--	end
			
			local grp=tokens[4]
			local segNames=string.tokenize(tokens[5],'/')

			if name=='subspace' then
				-- does nothing here. subspace keys will be used at the __updateConstraints
				if useCases[grp].controlParam[title]==nil then
					--dbg.console()
				end
				useCases[grp].controlParam[title]=param1
				return	
			end

			--##testDesiredLeaning if name=='desiredLeaning' then if select(1, string.find(title, 'desiredLeaning,roundoff2,stand,x')) then print('desiredLeaning:', title, param1) end end

			for isegName, segName in ipairs(segNames) do

				if graph[grp]~=nil and graph[grp][segName]~=nil then
					if table.getn(tokens)==5 then
						if type(param1)=='userdata' then
							graph[grp][segName][name](idx):assign(param1)
						else
							graph[grp][segName][name]:set(idx,param1)
						end
					else
						if tonumber(tokens[6]) then
							local mat= graph[grp][segName][name]
							assert(idx<mat:rows())
							assert(tonumber(tokens[6])<mat:cols())
							mat:set(idx,tonumber(tokens[6]),param1)
						else
							local vec3n=graph[grp][segName][name]
							if(vec3n~=nil and idx<vec3n:size()) then
								vec3n(idx)[tokens[6]]=param1
							else
								print('warning! idx overflow, '..title..' ignored')
							end
							if false then -- memory change debugging code
								if(grp=='walk3' and segName=='r1' and tokens[6]=='x') then
									print(grp,segName,name, idx,tokens[6], param1)
									SEG=graph[grp][segName]
									function debugSEG(event, line)
										if SEG.pendDesiredVel(0).x <-1 then
											print('debugSeg called')
											debug.sethook()
											dbg.console()
										end
									end
									debug.sethook(debugSEG,"l")
								end
							end
						end
					end
				else

					if not ignoreignore then
						if not string.find(title,"ignore") then
					 	--	print(title.." ignored")
							--dbg.console()
						end
					end
				end
			end

			-- 아래는 전부 deprecated.  
		elseif string.sub(title,1,6)=="velMod" then
			local grp="run"
			local axis=string.sub(title,7,7)
			graph[grp].LR["velMod"..axis]=param1
			graph[grp].RL["velMod"..axis]=param1*-1	 

		elseif string.sub(title,1,4)=="foot" then
			local tgt
			if string.sub(title,5,5)=="L" then
				tgt=footLcorrection
			else
				tgt=footRcorrection
			end

			tgt[string.sub(title,6,6)]=param1

		elseif string.sub(title,1,6)=="torque" then
			-- torqueSwingSRUN_comvel.x -- seg
			-- torqueSwingERUN_comvel.x -- edge

			local grp=string.sub(title, 13,15)
			local domain=string.sub(title, 17)
			local tgts

			local sss=string.sub(title,7,11)


			if sss=="Pelvs" then
				local kk=string.sub(title,12,12)
				if kk=="S" then
					tgts={"trqP_RL", "trqP_LR"}
				elseif kk=="E" then
					tgts={"t1qP_RL_LR", "t2qP_RL_LR", "t1qP_LR_RL", "t2qP_LR_RL"} -- continuous assumption
				end
			else

				if string.sub(title,12,12)=="S" then --segment key
					if sss=="Swing" then
						tgts={"trqL_RL", "trqR_LR"}
					else
						tgts={"trqR_RL", "trqL_LR"}
					end
				else
					assert(string.sub(title,12,12)=="E") -- edge key
					if sss=="Swing" then
						tgts={"t1qL_RL_LR", "t2qL_RL_LR"} -- continuous assumption
					else 
						tgts={"t1qR_RL_LR", "t2qR_RL_LR"} -- continuous assumption
					end
				end
			end
			for t,tgt in ipairs(tgts) do

				local target1=string.sub(tgt,1,5)..grp..string.sub(tgt,5)

				self:changeControlParameter("regResult_"..domain.."_"..target1, param1)
			end

		elseif title=="torsoPgain" then
			model.k_scale_active.default[1]=param1
		elseif title=="torsoDgain" then
			model.k_scale_active.default[2]=param1
		else
			print("Warning! "..title.. " ignored")
		end
	end
end
function LocoGraph:changeControlParameters(tbl, ignoreignore, pattern)
	
   if tbl==nil then return end
   for k,v in pairs(tbl) do
	   if k~='sequentialControlParam' then
		   self:changeControlParameter(k, v,ignoreignore,pattern)
	   end
   end
   if tbl.sequentialControlParam then
	   for i,cp_mod in ipairs(tbl.sequentialControlParam ) do
		   self:changeControlParameters(cp_mod, ignoreignore,pattern)
	   end
   end
end
function LocoGraph:changeControlParametersInUseCase(tbl)
   if tbl==nil then return end
   for k,v in pairs(tbl) do
	   if k~='sequentialControlParam' then
		   self:changeControlParameterInUseCase(k, v)
	   end
   end
end

function LocoGraph:preprocessPendulumMomentum()
	--##dos g_debugPEND=array:new()
	local grpNames=useCase.grpNames or { useCase.grpName}
	--for i,grpName in ipairs(self.groups.tf) do
	for igrp,grpName in ipairs(grpNames) do
		for k,seg in pairs(self[grpName]) do
			if not seg.interpolated then
				seg:calculatePendControlParamDependentMembers()
			end
		end
	end
	--##dos util.saveTable(g_debugPEND, "pend.tbl")
	--this('exit!',1)
	for igrp,grpName in ipairs(grpNames) do
		for k,seg in pairs(self[grpName]) do
			if seg.interpolated then
				seg:updateInterpolatedParams()
			end
		end
	end
end

function LocoGraph:createHoppingGraph() -- actually general enough to create any type of graph
	local mot=self.mot
	local dmot=self.dmot
	local useCaseOrig=useCase

	assert(useCase.grpNames~=nil or useCase.grpName~=nil)
	local grpNames=useCase.grpNames or { useCase.grpName}

	self.rectifyGraph(useCase)

	for igrp,grpName in ipairs(grpNames) do
		useCase=useCase.grpNameToUseCase[grpName]
		self.rectifyGraph(useCase)
		self:changeControlParameters(useCase.controlParam,true,'useCases,') 

		if useCase.useFootEncodingMethod2 then
			local param=useCase.useFootEncodingMethod2
			if type(param.geom.L[1][1])=="string" then
				for k,v in pairs(param.geom) do
					-- heel
					param.geom[k][1][3]=self.skel_withoutCOMjoint:getBoneByName(param.geom[k][1][1])
					-- toe
					param.geom[k][2][3]=self.skel_withoutCOMjoint:getBoneByName(param.geom[k][2][1])
					-- heel
					param.geom[k][1][1]=self.skel:getBoneByName(param.geom[k][1][1])
					-- toe
					param.geom[k][2][1]=self.skel:getBoneByName(param.geom[k][2][1])
				end
			end
			function param:calcProjPos(fk, isL)
				local method=self
				local lposinfo=method.geom[isL]
				heelpos=fk:globalFrame(lposinfo[1][3]):toGlobalPos(lposinfo[1][2])
				toepos=fk:globalFrame(lposinfo[2][3]):toGlobalPos(lposinfo[2][2])
				local currProjPos=LocoGraph.projectFoot(heelpos,toepos)
				return currProjPos
			end
		end

		self.lfootpos=useCase.lfootpos 
		self.rfootpos=useCase.rfootpos

		self.lhandpos=useCase.lhandpos 
		self.rhandpos=useCase.rhandpos 

		self.keyframes={}
		if useCase.keyframes then
			for k, v in pairs(useCase.keyframes) do
				if type(v.default)=="table" then
					if type(v.default[2])=='string' then
						self.keyframes[v.default[2]]=util.loadTable(v.default[2])
					end
				end
			end
		end

		--local segmentations=useCase.segmentations
		local segmentations={useCase.segmentations[grpName]}
		if segmentations[1]==nil then
			segmentations=useCase.segmentations
		end


		self.simulator=Physics.DynamicsSimulator_gmbs() 
		--self.simulator=Physics.DynamicsSimulator_AIST_penalty() 
		self.simulator:registerCharacter(self.skel_withoutCOMjoint)
		self.simulator:setGVector(vector3(0,9.8,0)) 
		self.simulator:init(model.timestep, Physics.DynamicsSimulator.EULER) 


		assert(self.pendulum )
		if false then
			self.pendulumSimulator=Physics.DynamicsSimulator_gmbs_penalty()
			self.pendulumSimulator:registerCharacter(self.pendulum.loader)
			self.pendulumSimulator:setGVector(vector3(0,9.8,0)) 
			self.pendulumSimulator:init(model.timestep, Physics.DynamicsSimulator.EULER) 
		end

		for segmentationName, segmentation in pairs(segmentations) do
			print(segmentationName)

			local firstFrames=segmentation.firstFrames

			local function setSeg(grpName, name, ii, refL, refR, refP, refLH, refRH)
				--print(grpName.."."..name..": "..firstFrames[ii]..'->'..firstFrames[ii+1])
				self:setSegment(grpName, name, firstFrames[ii], firstFrames[ii+1], firstFrames[ii+2], firstFrames[ii-1])
				self[grpName][name].segInfo={firstFrames, ii}
				for k,v in pairs(useCase.segProperties) do
					self[grpName][name][k]=segmentation[k][ii-1]
					--print(grpName,name,k,segmentation[k][ii-1])
				end
				local function setFootRef(seg, isL, ref)
					local ref=CT.vec(unpack(ref))
					if (ref:size()~=4) then
						ref:resize(4)
						ref:set(2,0) -- weight 1
						ref:set(3,1) -- weight 2
					end
					local len=seg.len
					seg['ref'..isL]=ref
				end
				setFootRef(self[grpName][name], 'L', refL)
				setFootRef(self[grpName][name], 'R', refR)
				setFootRef(self[grpName][name], 'P', refP)
				if useCase.keyframes.importanceLH then
					setFootRef(self[grpName][name], 'LH', refLH)
					setFootRef(self[grpName][name], 'RH', refRH)
				end
				self[grpName][name]:calculateMembers()
			end

			local nseg= self.getNumSeg(firstFrames)
			for i=1, nseg do
				local segname= segmentation.names[i]
				local grpName=segmentation.grpName
				local offset=segmentation.footRefL.offset or 0
				local refL=table.__add(segmentation.footRefL[i], {offset, offset,0,0})
				offset=segmentation.footRefR.offset or 0
				local refR=table.__add(segmentation.footRefR[i], {offset, offset,0,0})
				offset=segmentation.pelvisRef.offset or 0
				local refP=table.__add(segmentation.pelvisRef[i], {offset, offset,0,0})

				local refLH, refRH
				if useCase.keyframes.importanceLH then
					local offset=segmentation.footRefLH.offset or 0
					refLH=table.__add(segmentation.footRefLH[i], {offset, offset,0,0})
					offset=segmentation.footRefRH.offset or 0
					refRH=table.__add(segmentation.footRefRH[i], {offset, offset,0,0})
				end
				print('setSeg', i, grpName, segname)
				setSeg(grpName, segname, i+1, refL, refR, refP, refLH, refRH)
				--if segmentation.numFootModKeys then
				--	-- deprecated. do not use.

				--	local numKey=segmentation.numFootModKeys [i]
				--	local limbs={'L','R','LH','RH'}
				--	for il,l in ipairs(limbs) do
				--		local seg= self[grpName][segname]
				--		local keyarray=seg['foot'..l..'mod']
				--		if keyarray then keyarray:resize(numKey) end
				--	end
				--end
			end
			print('setSeg done')
		end
		--self.simulator=nil -- causes seg fault on mac. TODO: fix this
	end
		
	useCase=useCaseOrig
	local graph=useCase.graph or useCase:createGraph()

	-- set pendcontrolparam
	for igrp,grpName in ipairs(grpNames) do
		useCase=useCase.grpNameToUseCase[grpName]
		self:changeControlParameters(useCase.pendControlParam,true) -- set pendcontrolparam before interpolating segments
--		self:changeControlParameters(table.grep(useCase.controlParam, ',desiredLeaning,'),true) 
		self:changeControlParameters(useCase.controlParam, true)
	end
	for igrp,grpName in ipairs(grpNames) do
		if useCase.funcUpdateConstraints then
			useCase.funcUpdateConstraints(self)
		end
	end

	useCase=useCaseOrig
	self:preprocessPendulumMomentum()
	--[[
	--for i,grpName in ipairs(self.groups.tf) do
	for igrp,grpName in ipairs(grpNames) do
		for k,seg in pairs(self[grpName]) do
			seg:calculatePendControlParamDependentMembers()
		end
	end
	]]--
	useCase=useCaseOrig

	assert(graph)
	for i, cmd in ipairs(graph) do
		--printTable(cmd)
		if cmd[1]=="addInterpolatedSegment" then
			local grps=self
			local seg0=grps[cmd.seg0[1]][cmd.seg0[2]]
			local seg1=grps[cmd.seg1[1]][cmd.seg1[2]]
			local segn=seg0:interpolate(seg1, cmd.startWeight, cmd.endWeight, cmd.grpName, cmd.name)
			self:addSegment(cmd.grpName, cmd.name,segn)
		elseif cmd[1]=="beginGroup" then
			self:beginGroup(cmd[2], cmd[3], cmd[4] or cmd[3])
		elseif cmd[1]=="connect" then
			if table.getn(cmd)==3 then 
				self:connect(cmd[2], cmd[3])
			else
				assert(table.getn(cmd)==5)
				self:beginGroup(cmd[2], cmd[4])
				self:connect(cmd[3], cmd[5])
			end
		elseif cmd[1]=="connectMulti" then
			self:beginGroup(cmd[2], cmd[2])
			for i=3, table.getn(cmd)-1 do
				self:connect(cmd[i], cmd[i+1])
			end
		elseif cmd[1]=="initialSegment" then
			self.initialSegment=self[cmd[2]][cmd[3]]
		else assert(false)
		end
	end

	assert(self.initialSegment)
	assert(self.initialSegment.next)

	self.types={}
	self.types.NONE=0
	self.types.WALK=1
	self.types.RUN=2
	self.types.STAND=3


	self.transitionFunction=function (graph, desiredType, currSegment)
		currSegment=currSegment.next
		return desiredType, currSegment
	end
	
	for igrp,grpName in ipairs(grpNames) do
		useCase=useCase.grpNameToUseCase[grpName]
		self:changeControlParameters(useCase.controlParam, true)
		if useCase.funcUpdateConstraints then
			useCase.funcUpdateConstraints(self)
		end
	end
	useCase=useCaseOrig
end


function LocoGraph:setBones()
   MotionLoader.setVoca(self.skel, model.bones)
end


function LocoGraph.rectifyGraph(useCase)
	useCases.rectifyGraph(useCase)
end

LocoGraph.createIKsolver=MotionGraph.createIKsolver
LocoGraph.createIKsolver2=MotionGraph.createIKsolver2
LocoGraph.createIKsolver_COM=MotionGraph.createIKsolver_COM
LocoGraph._changeVoca=MotionGraph._changeVoca

function LocoGraph:logKeyframes()
	for i,grpName in ipairs(self.groups.tf) do
		for k,v in pairs(self[grpName]) do
			if useCase.keyframes then
				for kk, vv in pairs(useCase.keyframes) do
					if coarseLog then
						coarseLog(util.tostring(kk,v[kk]))
					else
						print (util.tostring(kk,v[kk]))
					end
				end
			end
		end
	end
end
-- input: global model, scenario
function LocoGraph:createGraph()

   self:setBones()
   self.lfoot=self.skel:getBoneByVoca(MotionLoader.LEFTANKLE)
   self.rfoot=self.skel:getBoneByVoca(MotionLoader.RIGHTANKLE)
   self.lknee=self.skel:getBoneByVoca(MotionLoader.LEFTKNEE)
   self.rknee=self.skel:getBoneByVoca(MotionLoader.RIGHTKNEE)
   if self.skel:getTreeIndexByVoca(MotionLoader.LEFTWRIST)~=-1 then
	   self.lhand=self.skel:getBoneByVoca(MotionLoader.LEFTWRIST)
	   self.rhand=self.skel:getBoneByVoca(MotionLoader.RIGHTWRIST)
	   self.lelbow=self.skel:getBoneByVoca(MotionLoader.LEFTELBOW)
	   self.relbow=self.skel:getBoneByVoca(MotionLoader.RIGHTELBOW)
	   assert(self.lelbow)
   end

   self.bone2={}
   self.bone2.lfoot=self.skel_withoutCOMjoint:getBoneByVoca(MotionLoader.LEFTANKLE)
   self.bone2.rfoot=self.skel_withoutCOMjoint:getBoneByVoca(MotionLoader.RIGHTANKLE)
   self.bone2.lknee=self.skel_withoutCOMjoint:getBoneByVoca(MotionLoader.LEFTKNEE)
   self.bone2.rknee=self.skel_withoutCOMjoint:getBoneByVoca(MotionLoader.RIGHTKNEE)
   if self.skel:getTreeIndexByVoca(MotionLoader.LEFTWRIST)~=-1 then
	   self.bone2.lhand=self.skel_withoutCOMjoint:getBoneByVoca(MotionLoader.LEFTWRIST)
	   self.bone2.rhand=self.skel_withoutCOMjoint:getBoneByVoca(MotionLoader.RIGHTWRIST)
	   self.bone2.lelbow=self.skel_withoutCOMjoint:getBoneByVoca(MotionLoader.LEFTELBOW)
	   self.bone2.relbow=self.skel_withoutCOMjoint:getBoneByVoca(MotionLoader.RIGHTELBOW)
	   assert(self.bone2.lelbow)
   end
   if self.skel:getTreeIndexByVoca(MotionLoader.HEAD)~=-1 then
	   self.bone2.head=self.skel_withoutCOMjoint:VRMLbone(self.skel_withoutCOMjoint:getTreeIndexByVoca(MotionLoader.HEAD))
   end

--   if model==model_files.justin_runf3_cart then
----      self:createRunningGraph3()
--      self:createSkippingGraph()
--   elseif model==model_files.justin_run_cart then
--      self:createRunningGraph2()
   --else
   if useCase.graph or useCase.createGraph then
	   print('graph')
      self:createHoppingGraph()
   else
      assert(false)
   end

   

   
end

function LocoGraphOutputGlobal:__init(outputLocal)
   self.outputLocal=outputLocal


   self.synRoot=matrixn()
   self.footLglobal=vector3N()
   self.footRglobal=vector3N()


   if useCase.keyframes.importanceLH then
	   self.footLHglobal=vector3N()
	   self.footRHglobal=vector3N()
   end
   self.refTime=vectorn() 

   if false then -- reserve mem space 
	   local reserveAmt=1800
	   self.synRoot:setSize(reserveAmt,7)
	   self.synRoot:resize(0,7)
	   self.footLglobal:reserve(reserveAmt)
	   self.footRglobal:reserve(reserveAmt)
	   if useCase.keyframes.importanceLH then
		   self.footLHglobal:reserve(reserveAmt)
		   self.footRHglobal:reserve(reserveAmt)
	   end
   end

   -- above variables are fixed in range [0, self.numFrames).
   self.numFrames=0
end

function LocoGraphOutputGlobal:frameRate()
   local outputL=self.outputLocal
   return model.frame_rate
end

-- reference time (for accessing LocoGraphOutputLocal)
function LocoGraphOutputGlobal:getRefTime(time)
   if self.numFrames==0 then
      return 1
   end
   if time==nil then
      return self.refTime(self.numFrames-1)
   else
      if time<0 then
	 return 1
      end
      return self.refTime(time)
   end
end


function LocoGraphOutputGlobal:calcTimeScale(dvz, leaning, pendSpeed, cartSpeed,frameL)


   local outputL=self.outputLocal

   local frameRateL=model.frame_rate
   --   local frameRateL=outputL.synLMot.dofInfo:frameRate()

   local comSpeedOrig=outputL:sampleV3(frameL, 'lcomvel'):length()


   RE.output("leaning", leaning.."")
   --
   --		    desired curve      .
   --		    		 		     .		 .	     .
   --		    		 	   .
   --		    		    .
   --		    		.
   --		    	     .
   --		       	   .
   --		         .
   --		       .
   --		 1   .----------------------------------------------------------
   --
   --

   --		return ((zmpSpeedOrig+2.0)/(cartSpeed+2.0))*math.sqrt(1-leaning)
   --		return ((comSpeedOrig+2.0)/(pendSpeed+2.0))*math.sqrt(math.max(1-leaning,0.2*0.2))-- prevent too fast playback (0.2*totalDuration)
--   return ((comSpeedOrig+3.0)/(pendSpeed+3.0))/(1+1*math.sin(leaning))--math.sqrt(math.max(1-leaning,0.2*0.2))
   local damp=2.0
--   return 1
   return ((comSpeedOrig+damp)/(comSpeedOrig+dvz+damp))/(1+0*leaning*leaning)--math.sqrt(math.max(1-leaning,0.2*0.2))
   --return ((zmpSpeedOrig+2.0)/(cartSpeed+2.0))*((comSpeedOrig+2.0)/(pendSpeed+2.0))*math.sqrt(1-leaning)

   --		--[[local icurSeg=outputL:findSegment(frameL)
   --		local curSeg=outputL.seg(icurSeg)
   --
   --		print("iseg", icurSeg, frameL)
   --
   --		print(curSeg.name)
   --
   --		local len=curSeg:length()
   --		local newStride=curSeg.stride+dvz*len/frameRate
   --
   --		if newStride>curSeg.stride then
   --			newStride=curSeg.stride*0.5+newStride*0.5
   --		else
   --			newStride=(curSeg.stride*2+newStride)/3
   --		end
   --
   --		local newLen=curSeg.stride/newStride*len
   --
   --		newLen=newLen*sop.map(leaning, 0, 0.5, 1, 0.25)
   --
   --		if newLen>len then
   --			newLen=(newLen+len)/2
   --			newLen=math.min(len*1.1, newLen)	-- prevent too long segment.
   --		end
   --
   --		return newLen/len]]--
end

-- input: outputLocal, refTime, synRoot(self.numFrames-1)
-- output: synRoot.
function LocoGraphOutputGlobal:predictGlobal(dTurningSpeed, dvx, dvz, maxPredict,  syn)

	if dTurningSpeed==nil then
		dTurningSpeed=0
	end

	--   local start_transf=vector3(0,0,0)
	local start_transf
	local fixedGlobal=math.max(self.numFrames-1,0)

	local lowerBoundGlobal=fixedGlobal+PDservoLatency+2
	local upperBoundLocal=0 -- initialize with a small number
	local fixedLocal=self:getRefTime()
	local outputL=self.outputLocal

	-- g_fixedLocal=fixedLocal
	local endLocal=outputL:numSegments()
	local frameRate=self:frameRate()

	local id=syn.graph.id_rotY

	local col=id:getNumCol()
	--print("1")
	if self.numFrames==0 then
		start_transf=self.start_transf
		--      assert(start_transf~=nil)
		if start_transf==nil then
			start_transf=vector3(0,0,0)
		end
	else
		start_transf=id:extract2Dconfig(self.synRoot,fixedGlobal)
	end

	RE.output("startTransf", tostring(start_transf))
	--   print("2")

	local time=fixedGlobal
	local refTime=fixedLocal

	upperBoundLocal=math.floor(fixedLocal)+(useCase.upperBoundLocal or 6 )

	if scenario>=scenarios.STAND1 and scenario<=scenarios.STAND4 then
		upperBoundLocal=math.floor(fixedLocal)+2
	end


	local timewarped=matrixn ();

	--   print("3")
	self.refTime:resize(time+1)

	-- local stopCount=0
	-- if lockLateTouchDown then
	--    stopCount=20
	-- end

	local lroot=vectorn(7)
	local function genRoot(time, refTime)

		while true do

			while refTime>outputL:numSegments()-1 do -- we always need one more segment in the list.
				syn:produceSegment()
			end

			if (time>lowerBoundGlobal and refTime>upperBoundLocal ) then --or timewarped:rows()>=maxPredict then
				-- print("genRoot", time)
				break
			end

			timewarped:resize(timewarped:rows()+1, col)

			
			outputL:sampleVec(refTime,"lroot",lroot)
			id.convertFromLocalPose(lroot, timewarped:row(timewarped:rows()-1))

			self.refTime:resize(time+1)
			self.refTime:set(time, refTime)

			time=time+1
			--	 local invTimeScale=1/outputL.segInfo(math.floor(refTime)):length()
			local invTimeScale=1/outputL:length(math.floor(refTime)) --.segInfo(math.floor(refTime)):length()

			local newRefTime=refTime+invTimeScale

			-- if stopCount>0 then
			--    newRefTime=refTime
			--    stopCount=stopCount-1
			-- end

			local prevSegIndex=math.floor(refTime)
			local newSegIndex=math.floor(newRefTime)
			if newSegIndex~= prevSegIndex then

				--	    dbg.count(1)
				outputL.segInfo(newSegIndex).invRefTime=sop.map(newSegIndex, refTime, newRefTime, time-1, time)
			end
			refTime=newRefTime
		end
		return time, refTime
	end

	time,refTime=genRoot(time, refTime)

	upperBoundGlobal=time+PredictionLatency

	time,refTime=genRoot(time, refTime)


	id:initFromDeltaRep(start_transf, timewarped, frameRate)

	gTimer3:start()
	gTimer3:stopMsg("predictGlobal::trajectory edit")
	local numPredictedFrames=self.refTime:size()
	self.synRoot:resize(numPredictedFrames,col)

	--   print("5")
	--   debug.debug()

	if numPredictedFrames>self.synRoot:rows() then
		debug.debug()
	end

	if fixedGlobal>=numPredictedFrames-1 then
		debug.debug()
	end

	if numPredictedFrames-fixedGlobal~=id:numFrames() then
		debug.debug()
	end

	-- dbg.count(141)
	id:retarget(self.synRoot,fixedGlobal, numPredictedFrames, frameRate,dTurningSpeed,dvx,dvz)

end

-- function LocoGraphOutputGlobal:predictZMP(i)
--    local tf=MotionDOF.rootTransformation(self.synRoot:row(i))
--    local ri=self.refTime(i)
--    return tf:toGlobalPos(self.outputLocal.ZMPlocal:sampleRow(ri))
-- end



-- function LocoGraphOutputGlobal:predictCOM(i)
--    local tf=MotionDOF.rootTransformation(self.synRoot:row(i))
--    local ri=self.refTime(i)
--    return tf:toGlobalPos(self.outputLocal.COMlocal:sampleRow(ri))
-- end

function LocoGraphOutputGlobal:predictFootCon(i)
   return false
end

function LocoGraphOutputGlobal:invRefTime(f)

   -- search
   
   local segIndex=math.floor(f)
   local startSearch=self.outputLocal.segInfo(segIndex).invRefTime
--   assert(startSearch~=-1)
   startSearch=math.max(startSearch,0)
--   assert(self.refTime(math.floor(startSearch))<=segIndex and 
   --    self.refTime(math.ceil(startSearch))>=segIndex)
   
   local i=math.floor(startSearch)
   while i<self.refTime:size()-2 do
      if self.refTime(i)<=f and self.refTime(i+1)>=f then
	 return sop.map(f, self.refTime(i), self.refTime(i+1), i, i+1)
      end
      i=i+1
   end
   assert(false)
   return self.refTime:size()-1
   -- local outputL=self.outputLocal
   -- local irt1=
   -- local irt2=outputL.segInfo(segIndex+1).invRefTime
   
   -- return sop.map(f, segIndex, segIndex+1, irt1, irt2)
end



LocoSegInfo=LUAclass()

function LocoSegInfo:__init(segment)
   self.seg=segment
   self.len=segment.len
   self.timeScale=1
self.invRefTime=-1
end

function LocoSegInfo:length()
   return self.len*self.timeScale
end

function LocoSegInfo:nextLength()
	dbg.console()
end


function LocoGraphOutputLocal:length(segIndex)
   return (self.segInfo(segIndex):length())
end

function LocoGraphOutputLocal:__init(graph)

   -- for actual motion generation.
   self.graph=graph

   self.segInfo=vector()

   -- shortcuts
--   math.mrdplotMat(self.lmot:matView():range(0, self.lmot:matView():rows(),7,10)..self.lacc:range(0, self.lacc:rows(), 7,10)) -- rhip

   self.dofInfo=self.graph.skel.dofInfo

   -- temporaries
   self.tempv1=vectorn()
   self.tempv2=vectorn()

   self:append(graph.initialSegment) -- fill in unused index 0
 self.segInfo(0).invRefTime=0

end
-- param1 : LocoGraphSegment (lmot,m ZMPlocal, COMlocal,zmpToPelvisR,zmpToPelvisT, offsetPendQ, footL, footR, footRef)
function LocoGraphOutputLocal:append(segment)
   self.segInfo:push_back(LocoSegInfo(segment))
end

function LocoGraphOutputLocal:numSegments()
   return self.segInfo:size()
end


function LocoGraphOutputLocal:samplePose(refTime, pose)
   local segIndex=math.floor(refTime)
   local seg=self.segInfo(segIndex)

   local delta=refTime-segIndex
   
   seg.seg.locomot.lmot:matView():sampleRow(seg.seg.first+seg.len*delta, pose)
end

function LocoGraphOutputLocal:sampleVel(refTime, pose)
   local segIndex=math.floor(refTime)
   local seg=self.segInfo(segIndex)

   local delta=refTime-segIndex
   
   seg.seg.locomot.lvel:sampleRow(seg.seg.first+seg.len*delta, pose)
end
function LocoGraphOutputLocal:sampleAcc(refTime, pose)
   local segIndex=math.floor(refTime)
   local seg=self.segInfo(segIndex)

   local delta=refTime-segIndex
   
   seg.seg.locomot.lacc:sampleRow(seg.seg.first+seg.len*delta, pose)
end


function LocoGraphOutputLocal:sampleVec(refTime, srckey, out)
   local segIndex=math.floor(refTime)
   local seg=self.segInfo(segIndex)

   local delta=refTime-segIndex

   seg.seg[srckey]:sampleRow(seg.len*delta, out)
end

function LocoGraphOutputLocal:sampleV3(refTime, srckey)
   local out=vector3()
   local segIndex=math.floor(refTime)
   local seg=self.segInfo(segIndex)

   local delta=refTime-segIndex

   return seg.seg[srckey]:sampleRow(seg.len*delta)
end

function LocoGraphOutputLocal:sampleKey(refTime, srckey)
   local out=vector3()
   local segIndex=math.floor(refTime)
   local seg=self.segInfo(segIndex)

   local delta=refTime-segIndex

   return seg.seg:sampleKey(delta, srckey)
end

function LocoGraphOutputLocal:sampleQ(refTime, srckey)
   local out=quater()
   local segIndex=math.floor(refTime)
   local seg=self.segInfo(segIndex)

   local delta=refTime-segIndex

   return seg.seg[srckey]:sampleRow(seg.len*delta)
end

function LocoGraphOutputLocal:sampleVal(refTime, srckey)

   local segIndex=math.floor(refTime)
   local seg=self.segInfo(segIndex)

   local delta=refTime-segIndex

   return seg.seg[srckey]:sample(seg.len*delta)
end

function LocoGraphOutputLocal:sampleFootRef(refTime, srckey)

	-- deprecated: use getFootRef instead
   local segIndex=math.floor(refTime)
   local seg=self.segInfo(segIndex)

   local delta=refTime-segIndex

   return seg.seg[srckey]:sample(delta)
end
function LocoGraphOutputLocal:getFootRef(iseg, isL)
   local seg=self.segInfo(iseg)
   return seg.seg['ref'..isL]
end

-- refTime can be in [-1,2]. used only in analysis step.
function LocoGraphSegment:_timeToFrame(refTime)
	local segInfo=self.segInfo
	local firstFrames=segInfo[1]
	local segIdx=segInfo[2]
	local idx=math.max(segIdx+math.floor(refTime),1)
	return sop.map(segIdx+refTime, idx, idx+1, firstFrames[math.min(idx,#firstFrames)], firstFrames[math.min(idx+1,#firstFrames)])
end

function LocoGraphSegment:_timeToFrameOld(refTime)
   if refTime<0 then
      return sop.map(refTime, -1, 0, self.prevFirst, self.first)
   elseif refTime<=1 then
      return sop.map(refTime, 0, 1, self.first, self.last)
   end
   return sop.map(refTime, 1, 2, self.last, self.nextLast)
end
-- refTime can be in [-1,2]. used only in analysis step.
function LocoGraphSegment:_sample(refTime, src)
	return src:sampleRow(self:_timeToFrame(refTime))
end
