require('subRoutines/QPservo')

package.path=package.path..";../Samples/ysscripts/lib/?.lua" --;"..package.path
require("simulfunc")

QPservo2=LUAclass(QPservo)

function QPservo2:sampleTargetPoses( frame)
	if frame>=self.motionDOF:rows() then frame=self.motionDOF:rows()-1
	end

--	frame=0
	RE.output2("frame", frame)

	-- desired (target) pose
	self.motionDOF:samplePose(frame, self.theta_d)
	self.dmotionDOF:sampleRow(frame, self.dtheta_d)
	self.ddmotionDOF:sampleRow(frame, self.ddtheta_d)

	--print(self.theta_d(10)) -- knee dof

--self.theta_d:range(7,self.theta_d:size()):setAllValue(0)
--self.dtheta_d:setAllValue(0)
--self.ddtheta_d:setAllValue(0)
end

function QPservo2:__init(dofInfo, timestep,integrator, simulator, model)
	self.simulator=simulator

	---- external force
	--self.impulse = {}
	--self.impulse.chest = simulator:skeleton(0):VRMLbone(simulator:skeleton(0):getBoneByVoca(MotionLoader.LEFTANKLE):treeIndex())
	--self.impulse.lf = vector3(100,0,0)

	-- load useCase
	self.actuationType = useCase.actuationType
	self.invfricCoef=useCase.invFricCoef or 0.5
	self.invAccConeCoef=useCase.invAccConeCoef or 0.1
	self.excludeRoot=useCase.excludeRoot
	self.cmargin = useCase.contactMargin or 0.005

	if self.actuationType == ActuationType.tau then
		self.min = -useCase.tauMax
		self.max = useCase.tauMax
	elseif self.actuationType == ActuationType.ft then
		self.min = -useCase.ftMax
		self.max = useCase.ftMax
	elseif self.actuationType == ActuationType.a
		  or self.actuationType == ActuationType.u then
		self.min = 0
		self.max = 1

		if useCase.initMsclForceScale then
			for i_muscle=1,mOsim:getNumMuscles() do
				mOsim.f_m_o[i_muscle] = useCase.initMsclForceScale*mOsim.f_m_o[i_muscle]
			end
		end
	end
		
	-- settings
	self.useMLCPmatFromAIST=false
	self.excludeNormalBasis=false
	self.giOffset=0

	self.dtinv=1/timestep

	self.numActualDOF = dofInfo:numActualDOF()

	-- num setting
	self.numDof = dofInfo:numActualDOF()
	if self.actuationType == ActuationType.tau then
		self.numActuator = self.numDof
	elseif self.actuationType == ActuationType.ft
		or self.actuationType == ActuationType.a
		or self.actuationType == ActuationType.u then
		self.numActuator = mOsim:getNumMuscles()
	end

	-- load obj weights 
	self.ddqObjWeight = useCase.ddqObjWeight/self.numDof
	self.tauObjWeight = useCase.tauObjWeight
	self.ftObjWeight = useCase.ftObjWeight
	self.aObjWeight = useCase.aObjWeight/self.numActuator
	self.lambdaObjWeight = useCase.lambdaObjWeight

	-- defalt
	self.state={previousFlightPhase=false, flightPhase=false, supportPhaseElapsed=100, flightPhaseElapsed=0, prevContact={}, contact={}, contactDuration={}, contactClass={}}
	self.numCLinks=simulator:getNumAllLinkPairs()
	local index=intvectorn()
	simulator:getContactLinkBoneIndex(0,index)

	if tablelength(model.bones)>0 then
		--_checkpoints:pushBack(deepCopyTable({'clinks', self.numCLinks, index}))
		self.bi={}
		local bi=self.bi
		do 
			bi.L=simulator:skeleton(0):getTreeIndexByVoca(MotionLoader.LEFTANKLE)
			bi.R=simulator:skeleton(0):getTreeIndexByVoca(MotionLoader.RIGHTANKLE)
			bi.LH=simulator:skeleton(0):getTreeIndexByVoca(MotionLoader.LEFTWRIST)
			bi.RH=simulator:skeleton(0):getTreeIndexByVoca(MotionLoader.RIGHTWRIST)
			assert(bi.L~=-1)
		end

		do
			-- set footLoop jacobian tools
			local footLoop={}
			local skel=simulator:skeleton(0)
			footLoop.skel= MainLib.VRMLloaderView(skel,skel:getBoneByVoca(MotionLoader.LEFTANKLE), vector3(0,0,0))
			MotionLoader.setVoca(footLoop.skel, model.bones)
			footLoop.sim= Physics.DynamicsSimulator_gmbs(true)
			footLoop.sim:registerCharacter(footLoop.skel)
			footLoop.sim:init(timestep, Physics.DynamicsSimulator.EULER)
			footLoop.Rindex=footLoop.skel:getTreeIndexByVoca(MotionLoader.RIGHTANKLE)
			self.footLoop=footLoop
		end

		for i=1,self.numCLinks do
			self.state.prevContact[i]=false
			self.state.contact[i]=false
			self.state.contactDuration[i]=0

			for k,isL in ipairs({'L','R','LH','RH'}) do
				if index(i-1)==bi[isL] or index(i-1)==bi[isL]+1 then
					self.state.contactClass[i]=isL
				end
			end
			if self.state.contactClass[i]==nil then
				self.state.contactClass[i]='O'
			end
		end
	end
	
	self.state.aggContactForce=vector3(0,0,0)

	--_checkpoints:pushBack(deepCopyTable({'state', state}))
	self.theta=vectorn()
	self.dtheta=vectorn()
	-- HD servo
	self.theta_d=vectorn() -- desired q
	self.dtheta_d=vectorn() -- desired dq
	self.ddtheta_d=vectorn() -- desired ddq

	-- PD servo
	--self.theta_d_pd=vectorn()

	self.desiredacceleration=vectorn()
	self.controlforce=vectorn()
	self.kp=vectorn()
	self.kd=vectorn()
	self.kp_id=vectorn()
	self.kd_id=vectorn()

	self.genActuationForce=vectorn(self.numDof+1)
	self.genContactForce=vectorn(self.numDof+1)

	self.tgtVelScale=vectorn()
	self.mask_slide=vectorn()
	

	-- lleg+rleg+upperbody=all
	self.mask_lleg=vectorn() -- excluding sliding joints
	self.mask_rleg=vectorn() -- excluding sliding joints
	self.mask_upperbody=vectorn()
	self.scale_lleg=1
	self.scale_rleg=1
	self.scale_upperbody=1
	
	self.muscleActiveness=0.3
	self.kp_weight=1.0 -- use kp_active(1) or kp_passive(0)
	self.kd_weight=1.0 -- use kd_active(1) or kd_passive(0)
	self.mask_slide:setSize(dofInfo:numDOF())
	self.mask_slide:setAllValue(0)
	self.mask_lleg:setSize(dofInfo:numDOF())
	self.mask_rleg:setSize(dofInfo:numDOF())
	self.mask_upperbody:setSize(dofInfo:numDOF())
	self.mask_lleg:setAllValue(0)
	self.mask_rleg:setAllValue(0)
	self.mask_upperbody:setAllValue(1)

	self.dofInfo=dofInfo
	self:updateCoef()
	print ("kp=",self.kp)
	print ("kd=",self.kd)

	if tablelength(model.bones)>0 then
		local skel=dofInfo:skeleton()

		local lhip=skel:getBoneByVoca(MotionLoader.LEFTHIP)
		local rhip=skel:getBoneByVoca(MotionLoader.RIGHTHIP)

		self.lkneeDOF=dofInfo:DOFindex(skel:getBoneByVoca(MotionLoader.LEFTKNEE):treeIndex(),0)
		self.rkneeDOF=dofInfo:DOFindex(skel:getBoneByVoca(MotionLoader.RIGHTKNEE):treeIndex(),0)
		local function setClampMax(clampForce, clampTorque)
			local clampMax=vectorn(dofInfo:numDOF())
			clampMax:setAllValue(0)
			for i=2,skel:numBone()-1 do
				local bone=skel:bone(i)
				local vbone=bone:treeIndex()
				local nJoint=dofInfo:numDOF(vbone)
				for j=0, nJoint-1 do
					local dofIndex=dofInfo:DOFindex(vbone,j)
					if dofInfo:DOFtype(vbone, j)==MotionDOFinfo.SLIDE then
						local dofIndex=dofInfo:DOFindex(vbone,j)
						self.mask_slide:set(dofIndex, 1)
						clampMax:set(dofIndex, clampForce)
					else
						clampMax:set(dofIndex, clampTorque)

						if bone:isDescendent(lhip) then
							self.mask_lleg:set(dofIndex,1)
							self.mask_upperbody:set(dofIndex,0)
						elseif bone:isDescendent(rhip) then
							self.mask_rleg:set(dofIndex,1)
							self.mask_upperbody:set(dofIndex,0)
						end


					end
				end
			end   
			return clampMax
		end

		local clampTorque=model.clampTorqueID or 400
		local clampForce=model.clampForceID or 4000

		self.clampMaxID=setClampMax(clampForce, clampTorque)

		clampTorque=model.clampTorque or 800
		clampForce=model.clampForce or 8000

		self.clampMax=setClampMax(clampForce, clampTorque)
		
		self.clampMin=self.clampMax*-1
		self.clampMinID=self.clampMaxID*-1
	end

	self.workspace={}
	local w=self.workspace

	w.M=matrixn()
	w.b=vectorn(self.numDof)

	-- contact related matrices
	w.Jc=matrixn()
	w.JcT_Vcb=matrixn()
	w.Vcb=matrixn()

	w.dJc=matrixn()
	w.dVcb=matrixn()

	w.VcbT_Jc = matrixn()
	w.VcbT_dJc = matrixn()

	-- for acceleration cone
	w.Vcb2=matrixn()
	w.dVcb2=matrixn()
	w.Vcb2T_Jc = matrixn()
	w.Vcb2T_dJc = matrixn()
	w.dVcb2T_Jc = matrixn()

	-- etc
	w.Mlcp=matrixn()
	w.Mlcp_bias=vectorn()

	w.CE=matrixn()
	w.ce0=vectorn()

	w.CI=matrixn()
	w.ci0=vectorn()

	w.x=vectorn()
	w.x_lcp=vectorn()
	w.CElcp=matrixn()
	w.ce0lcp=vectorn()
	w.CIinvM=matrixn()

	-- muscle related matrices
	w.Ja = matrixn()
	w.JaT_Vaf = matrixn()
	w.Vaf = matrixn()


	w.clearCI=function(self, max_numCon, totalDIM) -- max_numCon은 정확할 필요 없음. 대충 적당히 넉넉히 주면 메모리 재할당이 줄듯.
		self.CI:setSize(max_numCon, totalDIM)
		self.CI:resize(0,totalDIM)
		self.ci0:setSize(max_numCon)
		self.ci0:resize(0)
	end
	w.addCI=function(self, CI, startIndex)
		local nrow=self.CI:rows()
		self.CI:resize(nrow+CI:rows(), self.CI:cols())
		self.CI:sub(nrow, nrow+CI:rows(),startIndex, startIndex+CI:cols()):assign(CI)
	end
	w.addCI0=function(self, ci0)
		local nrow=self.ci0:size()
		self.ci0:resize(nrow+ci0:size())
		self.ci0:range(nrow, nrow+ci0:size()):assign(ci0)
		assert(self.ci0:size()==self.CI:rows())
	end

	return o
end

function QPservo2:QPsolve(sim, state, dstate, spprtImportance, importance)
	--self:__QPsolve1(sim, state, dstate, spprtImportance, importance)
	self:__QPsolve2(sim, state, dstate, spprtImportance, importance)
end

function QPservo2:__QPsolve2(sim, state, dstate, spprtImportance, importance)
	--if self.actuationType~=ActuationType.tau then
		--mOsim:invalidatePPIndicesBasedInfo()
		--mOsim:invalidatePPPoszBasedInfo()

		--if self.actuationType == ActuationType.a
			--or self.actuationType == ActuationType.u then
			--mOsim:integrateMuscleDynamics(sim:getTimestep())
		--end
	--end
	
	local simulator=self.simulator

	--(ys) number of contact bodies
	local link_pair_count=simulator:getNumContactLinkPairs()

	local numDof = self.numDof
	local numActuator = self.numActuator

	--(ys) workspace contains various matrices
	local w=self.workspace
	simulator:calcMassMatrix3(0, w.M, w.b)

	--(ys) used in other functions
	--self.controlforce:setSize(numDof+1)
	--self.controlforce:setAllValue(0)
	w.link_pair_count=link_pair_count

	--(ys) transfer contact state to prev state and initialize current contact state
	local state=self.state
	for i=1, self.numCLinks do state.prevContact[i]=state.contact[i] state.contact[i]=false end
	w.CI_additional=nil

	--------------------------------------------------------
	-- computation!
	--------------------------------------------------------
	
	local dimSolution, numLambda
	if link_pair_count > 0 then
		-- calc contact basis
		if w.bases==nil then w.bases=Physics.Vec_ContactBasis() end
		if w.accbases==nil then w.accbases=Physics.Vec_ContactBasis() end
		simulator:getContactBases(w.bases,self.invfricCoef)
		simulator:getContactBases(w.accbases,self.invAccConeCoef) -- basis vectors for acceleration cone can be different from those of friction cones

		-- update contact state
		for i=0, w.bases:size()-1 do
			state.contact[w.bases(i).ilinkpair+1]=true
		end
		-- state.contact are
		-- Left heel, Left toe, Right heel, Right toe, Left hand, Right hand, respectively
		-- w.bases(i).ilinkpair has those values for each w.bases(i)
		-- print(w.bases(1).normal:dotProduct(vector3(1,1,1)))

		-- calc contact related matrices
		simulator:calcContactJacobianAll(w.Jc, w.dJc, w.Vcb, w.dVcb, link_pair_count,self.invfricCoef)
		--calcContactJacobianAll(w.bases, w.Jc, w.dJc, w.Vcb, w.dVcb, link_pair_count,self.invfricCoef)
		--calcContactJacobianAll_new(simulator, w.bases, numDof, link_pair_count, w.Jc, w.dJc, w.Vcb, w.dVcb)
		w.JcT_Vcb:multAtB(w.Jc, w.Vcb)
		
		-- setting acceleration cone
		simulator:calcContactBasisAll(w.Vcb2, w.dVcb2, link_pair_count,self.invAccConeCoef)
		if self.excludeNormalBasis then
			w.Vcb:assign(w.Vcb:sub(0,0,self.giOffset,0):copy())
			w.dVcb:assign(w.dVcb:sub(0,0,self.giOffset,0):copy())
			w.Vcb2:assign(w.Vcb2:sub(0,0,self.giOffset,0):copy())
			w.dVcb2:assign(w.dVcb2:sub(0,0,self.giOffset,0):copy())
		end

		-- num setting
		numLambda = w.Vcb:cols()
		dimSolution = numDof + numActuator + numLambda
	else
		dimSolution = numDof + numActuator
	end

	---------------------------------------------------------------
	-- objective
	do
		local qp=HessianQuadratic(dimSolution)
		self.qp=qp

		do
			-- minimize desired acc error
			-- basically, excludeRoot & excludeRootFlight are assumed to be true
			--if link_pair_count > 0 then	
				--for i=0,3 do -- root orientation
					--qp:addD(1*self.weight(i+4),i,0)
				--end
				--for i=3,6 do -- root position
					--qp:addD(1*self.weight(i-3),i,0)
				--end
			--else
				--for i=0,3 do -- root orientation
					--qp:addD(1*self.weight(i+4),i,self.desiredacceleration(i+4))
				--end
				--for i=3,6 do -- root position
					--qp:addD(1*self.weight(i-3),i,self.desiredacceleration(i-3))
				--end
			--end
		end
		
		-- minimize desired acc error
		-- basically, excludeRoot & excludeRootFlight are assumed to be true
		for i=0,3 do -- root orientation
			qp:addD(.00001*self.weight(i+4),i,0)
		end
		for i=3,6 do -- root position
			qp:addD(.00001*self.weight(i-3),i,0)
		end

		for i=6,numDof-1 do -- remaining dof
			qp:addD(self.ddqObjWeight*self.weight(i+1),i,self.desiredacceleration(i+1))
		end

		-- minimize joint torque / tendon force / activation
		local actObjWeight
		if self.actuationType == ActuationType.tau then
			actObjWeight = self.tauObjWeight
		elseif self.actuationType == ActuationType.ft then
			actObjWeight = self.ftObjWeight
		elseif self.actuationType == ActuationType.a
			or self.actuationType == ActuationType.u then
			actObjWeight = self.aObjWeight
		end
		for i=0,numActuator-1 do
			qp:addD(actObjWeight, i+numDof,0)
		end

		if link_pair_count > 0 then
			-- minimize contact force
			local lw=useCase.lambdaObjWeight
			for i=0, w.bases:size()-1 do
				local b=w.bases(i)
				local gi=b.globalIndex

				if not self.excludeNormalBasis then
					--qp:addD(lw,gi+numActualDOF*2,0)
					qp:addD(lw,gi+numDof+numActuator,0)
					--print(gi+numDof+numActuator)
				end
				for j=0, b.frictionNormal:size()-1 do
					gi=b.globalFrictionIndex+j-self.giOffset
					--qp:addD(lw,gi+numActualDOF*2,0)
					qp:addD(lw,gi+numDof+numActuator,0)
					--print(gi+numDof+numActuator)
				end
			end
		end
	end
	--printmatn(self.qp.H)
	--print(self.qp.R)

	---------------------------------------------------------------
	-- equality constraints
	do
		---------------------------------------------------------------
		-- constraints
		-- Ax + b =0
		-- Cx + d >= 0
		-- A : w.CE
		-- b : w.ce0
		-- C : w.CI
		-- d : w.ci
		if self.actuationType == ActuationType.tau then
			-- w.M*ddq - tau = w.b
			--> (w.M -I)(ddq;tau) = w.b
			w.CE:setSize(numDof+6, dimSolution)
			w.CE:sub(0,numDof,0,numDof):assign(w.M)
			local minusI=w.CE:sub(0,numDof,numDof,numDof+numActuator)
			minusI:identity()
			minusI:rmult(-1)

			-- constrain tau[0:6]=0
			w.CE:sub(numDof, numDof+6):setAllValue(0)	
			w.CE:sub(numDof, numDof+6, numDof, numDof+6):identity()  --(ys) set root torque=0

			w.ce0:setSize(numDof+6)
			w.ce0:range(0,numDof):assign(w.b)
			w.ce0:range(numDof,numDof+6):setAllValue(0)

		elseif self.actuationType == ActuationType.ft
			or self.actuationType == ActuationType.a
			or self.actuationType == ActuationType.u then

			calcMuscleRelatedMatrices(simulator, mOsim, numDof, w.Ja, w.Vaf)
			--##dos
			if g_debugOneStep then --##dos
				g_debugOneStep:pushBack({"muscleRelated",w.M:copy(), w.b:copy(), w.Ja:copy(), w.Vaf:copy()}) --##dos
			end
			
			w.CE:setSize(numDof, dimSolution)
			w.CE:sub(0,numDof,0,numDof):assign(w.M)

			w.ce0:setSize(numDof)

			w.JaT_Vaf:multAtB(w.Ja, w.Vaf)
			local C = mOsim:getC()
			local JaT_Vaf_C = w.JaT_Vaf * C

			if self.actuationType == ActuationType.ft then
				-- w.M*ddq - Ja.T*Vaf*C*ft = w.b
				-- -> (w.M -Ja.T*Vaf*C)(ddq;ft) = w.b
				w.CE:sub(0, numDof, numDof, numDof+numActuator):assign(-JaT_Vaf_C)
				w.ce0:range(0, numDof):assign(w.b)

				---- rank check!!
				--printmatn(JaT_Vaf_C)
				--local rM = gmbs.RMatrix()
				--rM:assign(JaT_Vaf_C)
				--print(rM:Rank())

			elseif self.actuationType == ActuationType.a
				or self.actuationType == ActuationType.u then
				-- w.M*ddq - J.T*Vaf*C*P*A*a - J.T*Vaf*C*P*p = w.b
				-- w.M*ddq - J.T*Vaf*C*P*A*a = w.b + J.T*Vaf*C*P*p 
				-- -> (w.M -J.T*Vaf*C*P*A)(ddq;a) = w.b + J.T*Vaf*C*P*p 
				local P = mOsim:getP()
				local A = mOsim:getA()
				local p = mOsim:getp()

				--##dos 
				if g_debugOneStep then --##dos
					g_debugOneStep:pushBack({"muscleRelated2",deepCopyTable(mOsim.l_m), deepCopyTable(mOsim.l_m_opt), deepCopyTable(mOsim.dl_m), deepCopyTable(mOsim.a_f), deepCopyTable(mOsim.f_m_o), deepCopyTable(mOsim.f_m_len), deepCopyTable(mOsim.v_m_max), deepCopyTable(mOsim.gamma), JaT_Vaf_C:copy(), P:copy(), A:copy(), p:copy()}) --##dos
				end

				-- temp
				--local P = CT.eye(numActuator)
				--local A = A + 1500*CT.eye(numActuator)
				--local A = 2.*A

				local JaT_Vaf_C_P_A = JaT_Vaf_C * P * A

				---- rank check!!
				--local checkM = P
				----printmatn(checkM)
				--local rM = gmbs.RMatrix()
				--rM:assign(checkM)
				--print(rM:Rank())

				w.CE:sub(0, numDof, numDof, numDof+numActuator):assign(-JaT_Vaf_C_P_A)

				local pM = matrixn(p:size(),1)
				pM:column(0):assign(p)
				local JaT_Vaf_C_P_p = (JaT_Vaf_C * P * pM):column(0)
				w.ce0:range(0, numDof):assign(w.b + JaT_Vaf_C_P_p)
				
				--printtblh(mOsim.l_m)
				--printtblh(mOsim.dl_m)
				--print(p)
				--printmatn(pM)
			end
		end

		if link_pair_count > 0 then
			-- w.M*ddq - tau = w.b
			-- (w.M -I)(ddq;tau) = w.b
			-->
			-- w.M*ddq - tau - JcT_Vcb*lambda = w.b
			-- (w.M -I -JcT_Vcb)(ddq;tau;lambda) = w.b

			-- w.M*ddq - Ja.T*Vaf*C*ft = w.b
			-- (w.M -Ja.T*Vaf*C)(ddq;ft) = w.b
			-->
			-- w.M*ddq - Ja.T*Vaf*C*ft - JcT_Vcb*lambda = w.b
			-- (w.M -Ja.T*Vaf*C -JcT_Vcb)(ddq;ft;lambda) = w.b

			-- w.M*ddq - J.T*Vaf*C*P*A*a - J.T*Vaf*C*P*p = w.b
			-- w.M*ddq - J.T*Vaf*C*P*A*a = w.b + J.T*Vaf*C*P*p 
			-- (w.M -J.T*Vaf*C*P*A)(ddq;a) = w.b + J.T*Vaf*C*P*p 
			-->
			-- w.M*ddq - J.T*Vaf*C*P*A*a - J.T*Vaf*C*P*p - JcT_Vcb*lambda = w.b
			-- w.M*ddq - J.T*Vaf*C*P*A*a - JcT_Vcb*lambda = w.b + J.T*Vaf*C*P*p 
			-- (w.M -J.T*Vaf*C*P*A -JcT_Vcb)(ddq;a;lambda) = w.b + J.T*Vaf*C*P*p 

			local minusJcT_Vcb=w.CE:sub(0,numDof,numDof+numActuator, dimSolution)
			minusJcT_Vcb:assign(w.JcT_Vcb)
			minusJcT_Vcb:rmult(-1)
		end
	end
	--printmatn(w.CE)
	--print(w.ce0)

	---------------------------------------------------------------
	-- inequality constraints
	do 
		-- use [de Lasa et al, SIGGRAPH2010]
		-- a_c = Vcb2.T*Jc*ddq + Vcb2.T*dJc*dq + dVcb2.T*Jc*dq >= 0

		-- contact
		-- |Vcb2.T*Jc 0 0||ddq      |   |Vcb2.T*dJc*dq + dVcb2.T*Jc*dq|    |0|
		-- |0         0 I||actuation| + | 0                           | >= |0|
		-- |0         I 0||lambda   |   |-min                         |    |0|
		-- |0        -I 0|              | max                         |    |0|

		-- no contact
		-- |0         I||ddq      | + |-min| >= |0|
		-- |0        -I||actuation|   | max|    |0|
		-- (a <= x <= b)
		
		if link_pair_count > 0 then
			w.VcbT_Jc:transpose(w.JcT_Vcb)
			w.Vcb2T_Jc:multAtB(w.Vcb2, w.Jc)
			w.Vcb2T_dJc:multAtB(w.Vcb2, w.dJc)
			w.dVcb2T_Jc:multAtB(w.dVcb2, w.Jc)

			------------------------------------------------
			-- determine agglambda introduced or not
			local cdim = numLambda	
			
			local clampAggLambda=true
			--local clampAggLambda=false
			local maxAggLambda=4500
			local maxDepth=-1
			local avgDpth=0
			local cmargin=self.cmargin

			-- calc max Depth and avgDpth
			local argMax=-1
			for i=0, w.bases:size()-1 do
				local b=w.bases(i)
				avgDpth=avgDpth+b.depth
				if b.depth>maxDepth then
					maxDepth=b.depth
					argMax=i
				end
			end
			avgDpth=avgDpth/w.bases:size()

			local criteria=2*avgDpth
			maxAggLambda=sop.map(criteria, 0, cmargin, 0, useCase.maxAggLambda or 500)
			if criteria>cmargin then clampAggLambda=false end			

			local numConAggLambda=0
			local agg_weight={}
			if clampAggLambda then 
				local weight=agg_weight
				numConAggLambda=1
				local axes={'y'}
				for i,a in ipairs(axes) do
					weight[a]=vectorn(cdim)
					weight[a]:zero()
				end
				for i=0, w.bases:size()-1 do
					local b=w.bases(i)
					local gi=b.globalIndex
					if not self.excludeNormalBasis then
						for k,a in ipairs(axes) do weight[a]:set(gi, weight[a](gi)+b.normal[a]) end
					end
					for j=0, b.frictionNormal:size()-1 do
						gi=b.globalFrictionIndex+j-self.giOffset
						for k,a in ipairs(axes) do weight[a]:set(gi, weight[a](gi)+b.frictionNormal(j)[a]) end
					end
				end
			end
			--------------------------------------------------

			-- calc dq
			local dq=w.dq
			if dq==nil then w.dq=vectorn(self.dtheta:size()-1) dq=w.dq end
			dq:range(0,3):assign(self.dtheta:range(4,7))
			dq:range(3,6):assign(self.dtheta:range(0,3))
			dq:range(6,dq:size()):assign(self.dtheta:range(7,self.dtheta:size()))

			-- set matrices
			local numConAggLambda=0
			w:clearCI(w.VcbT_Jc:rows()+numLambda+numActuator*2+numConAggLambda, dimSolution)

			w:addCI(w.Vcb2T_Jc,0)	
			w:addCI0((w.Vcb2T_dJc*dq:column()):column(0)+
					(w.dVcb2T_Jc*dq:column()):column(0), 0)

			w:addCI(CT.eye(numLambda), numDof+numActuator)
			w:addCI0(CT.zeros(numLambda))

			if numConAggLambda>0 then
				w:addCI((agg_weight.y*-1):row(), numDof+numActuator)
				w:addCI0(CT.ones(1)*maxAggLambda)
			end
		else
			w:clearCI(numActuator*2, dimSolution)
		end

		-- actuation limit
		if self.actuationType == ActuationType.tau then
			w:addCI(CT.eye(numActuator-6), numDof+6)
			w:addCI0(-self.min*CT.ones(numActuator-6))
			w:addCI(-CT.eye(numActuator-6), numDof+6)
			w:addCI0(self.max*CT.ones(numActuator-6))
		else
			w:addCI(CT.eye(numActuator), numDof)
			w:addCI0(-self.min*CT.ones(numActuator))
			w:addCI(-CT.eye(numActuator), numDof)
			w:addCI0(self.max*CT.ones(numActuator))
		end

		-- contact
		-- |Vcb2.T*Jc 0 0||ddq      |   |Vcb2.T*dJc*dq + dVcb2.T*Jc*dq|    |0|
		-- |0         0 I||actuation| + | 0                           | >= |0|
		-- |0         I 0||lambda   |   |-min                         |    |0|
		-- |0        -I 0|              | max                         |    |0|

		-- no contact
		-- |0         I||ddq      | + |-min| >= |0|
		-- |0        -I||actuation|   | max|    |0|
		-- (a <= x <= b)

		--if false then
		if link_pair_count > 0 then
			if (not useCase.useSoftContactModel) or useCase.useSoftContactModel_old then
				-- cmargin dependent
				local cmargin=self.cmargin
				local velMarginStrength=useCase.velMarginStrength or 1.0
				local maxPenetratingVel=useCase.maxPenetratingVel or 0
				local dtinv=self.dtinv
				-- add velocity-dependent margin
				for i=0, w.accbases:size()-1 do
					local b=w.accbases(i)
					local dp=b.normal:dotProduct(b.relvel)

					local cmargin_dep_max_penetrating_vel
					if b.depth>cmargin then
						--cmargin_dep_max_penetrating_vel =maxPenetratingVel 
						cmargin_dep_max_penetrating_vel =maxPenetratingVel +sop.clampMap(b.depth, cmargin, cmargin*1.5, cmargin*dtinv, 0)
					else
						--cmargin_dep_max_penetrating_vel =maxPenetratingVel + sop.clampMap(b.depth ,0, cmargin, cmargin*dtinv, 0) 
						cmargin_dep_max_penetrating_vel =maxPenetratingVel + cmargin*dtinv 
					end
					local relvel=b.relvel:copy()+cmargin_dep_max_penetrating_vel*b.normal
					local projectionOfRelvel=b.normal:dotProduct(relvel)*dtinv

					local gi=b.globalIndex
					if not self.excludeNormalBasis then
						w.ci0:set(gi, w.ci0(gi)+projectionOfRelvel)
					end
					for j=0, b.frictionNormal:size()-1 do
						--print(b.normal, b.frictionNormal(j))
						--projectionOfRelvel=(b.frictionNormal(j):dotProduct(relvel)+maxPenetratingVel)*dtinv*velMarginStrength -- allows slight foot slipping
						projectionOfRelvel=b.frictionNormal(j):dotProduct(relvel)*dtinv*velMarginStrength -- do not allow foot slipping

						local gi=b.globalFrictionIndex+j-self.giOffset
						w.ci0:set(gi, w.ci0(gi)+projectionOfRelvel)
					end
				end
			end
		end
	end
	--w.CI:setSize(0,0)
	--w.ci0:setSize(0)


	--if true then
	if false then
		-- clamp knee angle
		-- acceleration bounds
		-- acc > minAcc
		local CI_cols=w.CI:cols()
		--if w.CI:rows()==0 then
			----CI_cols=numActualDOF*2
			--CI_cols=numDof*2
		--end
		local angleBound=math.rad(-2)
		local minAcc=0
		local maxAcc=100
		--RE.output2('kneeDOF', self.theta(self.lkneeDOF), self.theta(self.rkneeDOF), self.dtheta(self.lkneeDOF), self.dtheta(self.rkneeDOF))

		local knees={self.lkneeDOF, self.rkneeDOF}
		for iknee,idof in ipairs(knees) do
			if self.theta(idof)<angleBound then
				local idq=self.dofInfo:DOFtoDQ(idof)
				w.CI:resize(w.CI:rows()+1, CI_cols)
				local ci0=w.ci0
				ci0:resize(ci0:size()+1)
				local lastRow=w.CI:row(w.CI:rows()-1)
				-- acc- minAcc>0
				lastRow:setAllValue(0)
				lastRow:set(idq,1)
				-- 0.5a*t^2=angleBound-self.theta(idof) 인 a찾자. 
				--minAcc=2*(angleBound-self.theta(idof))/(0.1*0.1)
				local vel=self.dtheta(idof)
				if vel<0 then
					--ci0:set(iknee-1, math.min(minAcc-vel*self.dtinv*0.5, maxAcc)*-1)
					ci0:set(ci0:size()-1, (minAcc)*-1)
				else
					ci0:set(ci0:size()-1, minAcc*-1)
				end
			end
		end

		--local www=self.weight2:range(7,numActualDOF+1)
		--w.ci0:range(start, start+numConTau):assign(maxTorque*www)
		---- tau+maxTorque>0
		--w.CI:sub(start+numConTau, start+numConTau*2, startc+6, startc+numActualDOF):identity()
		----w.ci0:range(start+numConTau, start+numConTau*2):setAllValue(maxTorque)
		--w.ci0:range(start+numConTau, start+numConTau*2):assign(maxTorque*www)
	end

	local state=self.state
	local perClassContactMargin=useCase.perClassContactMargin==1 
	if not perClassContactMargin then -- dynamic adjustment of collision margin
		local cmargin=self.cmargin
		for i=1, self.numCLinks do
			if state.prevContact[i] and not state.contact[i] then
				simulator:setCollisionMargin(i-1, 0) 
				--RE.output2('contactmargin'..i,'0')
				state.contactDuration[i]=0
			elseif not state.prevContact[i] and state.contact[i] then
				simulator:setCollisionMargin(i-1, cmargin) -- allow some penetration
				--RE.output2('contactmargin'..i,'1')
				--simulator:setCollisionMargin(i-1, 0.013) -- allow some penetration
				state.contactDuration[i]=0
			elseif state.contact[i] then
				state.contactDuration[i]=state.contactDuration[i]+1
			end
		end
	end
	RE.output2("contactDuration", table.tostring(state.contactDuration))
end

function QPservo2:stepSimul(sim, impulse)
	local simulator=self.simulator

--------------------------------------------
--prepare solving QP
	local qp=self.qp
	local w=self.workspace
	local numActualDOF=self.numActualDOF

	--##dos
	if g_debugOneStep then --##dos
		g_debugOneStep:pushBack({"qpR",qp.H:copy(), qp.R:copy()}) --##dos
	end

	if impulse then

		local J=matrixn()
		simulator:calcJacobian(0,impulse.chest:treeIndex(),J)
		local tf=simulator:getWorldState(0):globalFrame(impulse.chest)
		-- refer to DynamicsSimulator_gmbs.cpp:calcContactjacobianAll
		local dAd=matrixn(6,6)
		dAd:setSize(6,6)
		dAd:setAllValue(0)
		dAd:diag():setAllValue(1)
		dAd:sub(0,3,3,6):assign(CT.skew(tf:toGlobalPos(impulse.chest:localCOM())*-1))
		local lf=impulse.lf
		local mlf=CT.mat(6,1,0,0,0,lf.x, lf.y, lf.z)
		print(lf)
		local tau=matrixn()
		tau:multAtB(J,dAd*liegroup.dAd(tf:inverse())*mlf)

		w.ce0:range(0,numActualDOF):rsub(tau:column(0))
	end

	if false then
		--if true then
		print('qp.H')
		printmatn(qp.H)
		print()
		print('qp.R')
		print(qp.R)
		print()
		print('w.CE')
		printmatn(w.CE)
		print()
		--print('inv(w.CE)')
		--printmatn(CT.inverse(w.CE))
		--print()
		print('w.ce0')
		print(w.ce0)
		print()
		print('w.CI')
		printmatn(w.CI)
		print()
		print('w.ci0')
		print(w.ci0)
		print()
	end
--------------------------------------------

--------------------------------------------
--solve QP
	Eigen.solveQuadprog(qp, w.CE, w.ce0, w.CI, w.ci0, w.x)
	--Eigen.solveQuadprog(qp,w.CE, w.ce0, w.CI, w.ci0, w.x, true)
	
			--##dos
	if g_debugOneStep then --##dos
	g_debugOneStep:pushBack({"qpcontrolForce", w.x:copy(), w.CE:copy(), w.ce0:copy(), w.CI:copy(), w.ci0:copy()}) end 

	local ddq=w.x:range(0, self.numDof)
	local actu = w.x:range(self.numDof,self.numDof+self.numActuator)
	local lambda = w.x:range(self.numDof+self.numActuator,w.x:size())

	if true then
		--print('ddq      ', ddq)
		--print('actuation', actu)
		--print('lambda   ', lambda)
		--print('lambda size', lambda:size())

		--local cnt=0
		--local actu_excess = {}
		--for i=0,self.numActuator-1 do
			--local actu_elem = actu(i)
			--if actu_elem < self.min or actu_elem > self.max then
				--cnt = cnt+1
				--table.insert(actu_excess, actu_elem)
			--end
		--end
		--print('actuator range excess '..cnt..'/'..self.numActuator)
		--printtblh(actu_excess)
	end
--------------------------------------------

--------------------------------------------
--set actuation to mOsim
	if self.actuationType==ActuationType.a or self.actuationType==ActuationType.u then
		local actu_tbl = vecn2tbl(actu)
		for i=1,#actu_tbl do
			if actu_tbl[i] < self.min then
				actu_tbl[i] = self.min
			elseif actu_tbl[i] > self.max then
				actu_tbl[i] = self.max
			end
		end
		mOsim:setExcitations(actu_tbl)
		if self.actuationType==ActuationType.a then
			mOsim:setActivations(actu_tbl)
		end
	elseif self.actuationType == ActuationType.ft then
		local aM = matrixn(actu:size(),1)
		aM:column(0):assign(actu)
		local actingforce_render = (mOsim:getC()*aM):column(0)
		mOsim:setActingForces_render(actingforce_render)
	end
--------------------------------------------

--------------------------------------------
--step one step
	if true then
		--integrate 1/120s once
		
		--if false then
		if true then
			--ddq integration
			simulator:stepKinematic(ddq, vectorn(), true)
		else	
			-- simulation
			
			-- set actuation force
			local gen3dofAF
			if self.actuationType == ActuationType.tau then
				gen3dofAF = actu
			elseif self.actuationType == ActuationType.ft then
				local C = mOsim:getC()
				local JaT_Vaf_C = w.JaT_Vaf * C
				local ftM = matrixn(actu:size(),1)
				ftM:column(0):assign(actu)
				gen3dofAF = (JaT_Vaf_C*ftM):column(0)
			elseif self.actuationType == ActuationType.a then
				local C = mOsim:getC()
				local JaT_Vaf_C = w.JaT_Vaf * C
				local ftM = matrixn(mOsim:getNumMuscles(),1)
				ftM:column(0):assign(tbl2vecn(mOsim:getTendonForces()))
				--printtblh(mOsim:getFiberLengths())
				--print'tendon force into the simulation'
				--printtblh(mOsim:getTendonForces())
				gen3dofAF = (JaT_Vaf_C*ftM):column(0)
			end
			self.genActuationForce:range(7,self.genActuationForce:size()):assign(gen3dofAF:range(6,gen3dofAF:size()))

			-- set contact force
			local link_pair_count=simulator:getNumContactLinkPairs()
			if link_pair_count > 0 then
				local lambdaM = matrixn(lambda:size(),1)
				lambdaM:column(0):assign(lambda)
				local gen3dofCF = (w.JcT_Vcb*lambdaM):column(0)

				local root1 = gen3dofCF:range(0,3):toVector3()
				local root2 = gen3dofCF:range(3,6):toVector3()

				local rootBone = mOsim.mLoader:getTreeIndexByName(mOsim:getJointNames()[1])
				local rootR = self.simulator:getWorldState(0):globalFrame(rootBone).rotation
				root1:rotate(rootR)
				root2:rotate(rootR)
				--root1:rotate(rootR:inverse())
				--root2:rotate(rootR:inverse())

				self.genContactForce:range(0,3):assign(root2)
				self.genContactForce:range(4,7):assign(root1)
				self.genContactForce:range(7,self.genContactForce:size()):assign(gen3dofCF:range(6,gen3dofCF:size()))
			else
				self.genContactForce:setAllValue(0)
			end

			simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_TORQUE, self.genActuationForce+self.genContactForce)
			simulator:stepSimulation()
		end

		--update muscles
		if self.actuationType~=ActuationType.tau then
			mOsim:invalidatePPIndicesBasedInfo()
			mOsim:invalidatePPPoszBasedInfo()

			if self.actuationType == ActuationType.a
				or self.actuationType == ActuationType.u then
				mOsim:integrateMuscleDynamics(sim:getTimestep())
			end
		end
	else
		--integrate 1/120s by small timestep
		
		local muscleIntegTimes = round(simulator:getTimestep() / mOsim.muscleTimeStep)
		for i=1,muscleIntegTimes do
			simulator:stepKinematicMuscle_integrate(ddq, mOsim.muscleTimeStep)

			if self.actuationType~=ActuationType.tau then
				mOsim:invalidatePPIndicesBasedInfo()
				mOsim:invalidatePPPoszBasedInfo()

				if self.actuationType == ActuationType.a
					or self.actuationType == ActuationType.u then
					mOsim:integrateMuscleDynamics(mOsim.muscleTimeStep)
				end
			end
		end
		simulator:stepKinematicMuscle_updateother()
	end
--------------------------------------------

--------------------------------------------
--draw contact force
if not finalRender then
	if lambda:size()>0 then 
		local mat=vector3N(w.bases:size()*2) -- draw contact forces
		for i=0, w.bases:size()-1 do
			local b=w.bases(i)
			local cf=vector3(0,0,0)
			if not self.excludeNormalBasis then
				cf:radd(lambda(b.globalIndex)*b.normal)
			end
			for j=0, b.frictionNormal:size()-1 do
				local gi=b.globalFrictionIndex+j-self.giOffset
				cf:radd(lambda(gi)*b.frictionNormal(j))
			end
			mat(i*2):assign(w.bases(i).globalpos)
			mat(i*2+1):assign(w.bases(i).globalpos+cf*0.005)
			dbg.draw('Traj', mat:matView()*100, "contacts")
		end
	end
end
--------------------------------------------
		
	----##dos
	--if g_debugOneStep then --##dos
	--g_debugOneStep:pushBack({"before muscle sim", deepCopyTable(mOsim.a), deepCopyTable(mOsim.l_m), deepCopyTable(mOsim.u), deepCopyTable(mOsim.f_t), deepCopyTable(mOsim.dl_m)}) --##dos
	--end

	--if self.actuationType~=ActuationType.tau then
		--mOsim:invalidatePPIndicesBasedInfo()
		--mOsim:invalidatePPPoszBasedInfo()

		--if self.actuationType == ActuationType.a
			--or self.actuationType == ActuationType.u then
			--mOsim:integrateMuscleDynamics(sim:getTimestep())
		--end
	--end

			----##dos
	--if g_debugOneStep then --##dos
	--g_debugOneStep:pushBack({"after muscle sim", deepCopyTable(mOsim.a), deepCopyTable(mOsim.l_m), deepCopyTable(mOsim.u), deepCopyTable(mOsim.f_t), deepCopyTable(mOsim.dl_m)}) --##dos
	--end
end

