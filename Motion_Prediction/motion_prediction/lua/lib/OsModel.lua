require('Lee2013MuscleDyn')
require("utilfunc")

OsModel = LUAclass()

-----------------------------------------------------
-- constructor
-----------------------------------------------------
function OsModel:__init(wrlpath, luamsclpath, model, option)
	if not option then
		option={
			createSkin=true,
		}
	end
	local luamsclpath = luamsclpath or ''
	local model = model or nil
	
	-- load .wrl
	if type(wrlpath)=='string' then
		self.mLoader=MainLib.VRMLloader(wrlpath)
	else
		self.mLoader=wrlpath
	end

	if option.createSkin then
		local drawSkeleton = false
		self.mSkin=RE.createVRMLskin(self.mLoader, drawSkeleton)
		self.mSkin:scale(100,100,100)
	end

	-- load .luamscl
	local muscles = {}
	if type(luamsclpath)=='table' then
		muscles=luamsclpath
	elseif luamsclpath~='' then
		muscles = dofile(luamsclpath)
	end
	self.muscles = muscles
	self:initMuscleInfo(muscles)

	-- weaken test
	--if useCase.grpName=='gait1956_gait_repeat' and useCase.weakenMuscles~=nil then
	if useCase and useCase.weakenMuscles~=nil then
		self:weakenMuscles(useCase.weakenMuscles, useCase.weakRatio)
	end

	-- store dof index
	if model~=nil and model.bones and tablelength(model.bones)>0 then
		local skel = self.mLoader.dofInfo:skeleton()
		MotionLoader.setVoca(skel, model.bones)
		self.lkneeDOF = self.mLoader.dofInfo:DOFindex(skel:getBoneByVoca(MotionLoader.LEFTKNEE):treeIndex(),0)
		self.rkneeDOF = self.mLoader.dofInfo:DOFindex(skel:getBoneByVoca(MotionLoader.RIGHTKNEE):treeIndex(),0)
		self.lhipflexDOF = self.mLoader.dofInfo:DOFindex(skel:getBoneByVoca(MotionLoader.RIGHTHIP):treeIndex(),0)
		self.rhipflexDOF = self.mLoader.dofInfo:DOFindex(skel:getBoneByVoca(MotionLoader.RIGHTHIP):treeIndex(),0)
	end

	-- muscle dynamics simulation
	--self.muscleTimeStep = 1./600.
	self.muscleTimeStep = 1./840.
	--self.muscleTimeStep = 1./1200.
	--self.muscleTimeStep = 1./1800.
	--self.muscleTimeStep = 1./3600.
	--self.muscleTimeStep = 1./8400.
	--self.muscleTimeStep = 1./12000.

	-- records render info
	self.records = {}

	-- for rendering
	self.actingforce_render = vectorn()

	-- muscle save
	self.first_f_m_o = deepCopyTable(self.f_m_o)

	--self.totalEnergy = 0
end

function OsModel:initMuscleInfo(muscles)
	print('#dof', self.mLoader.dofInfo:numActualDOF())
	print('#muscle', #muscles)

	-- muscle properties
	local msclProp = {}
	for i_muscle, muscle in ipairs(muscles) do
		muscle.index = i_muscle
		for key, value in pairs(muscle) do
			if i_muscle==1 then
				msclProp[key] = {}
			end
			if msclProp[key]~=nil then
				msclProp[key][i_muscle] = value
			end
		end
	end
	self.tau_act = msclProp.activation_time_constant
	self.tau_deact = msclProp.deactivation_time_constant
	self.l_m_opt = msclProp.optimal_fiber_length
	self.pa_opt = msclProp.pennation_angle_at_optimal
	self.l_t_sl = msclProp.tendon_slack_length
	self.eps_t_o = msclProp.FmaxTendonStrain or CT.ones(#muscles)*0.033
	self.f_m_o = msclProp.max_isometric_force
	self.eps_m_o = msclProp.FmaxMuscleStrain or CT.ones(#muscles)*0.6
	self.k_pe = msclProp.KshapePassive or CT.ones(#muscles)*4.0
	self.gamma = msclProp.KshapeActive or CT.ones(#muscles)*0.5
	self.a_f = msclProp.Af or CT.ones(#muscles)*0.3
	self.f_m_len = msclProp.Flen or CT.ones(#muscles)*1.8
	self.v_m_max = msclProp.max_contraction_velocity or CT.ones(#muscles)*10
	self.mmass = msclProp.mass
	self.msclname = msclProp.name
	--self.damping = msclProp.damping
	self.damping = {}
	for i_muscle, muscle in ipairs(muscles) do
		self.damping[i_muscle] = 0.05
	end

	-- pathpoints
	self.pathpoints = {}
	for i_muscle, muscle in ipairs(muscles) do
		for j, pp in ipairs(muscle.GeometryPath.PathPointSet) do
			pp.muscleindex = i_muscle
			pp.treeIndex=self.mLoader:getTreeIndexByName(pp.joint)
			pp.location_wrl=tbl2vec3(pp.location_wrl)
			if(pp.treeIndex<1) then
				local fixedJointInfo=mLoaderFixedJoints[pp.joint]
				pp.treeIndex=self.mLoader:getTreeIndexByName(fixedJointInfo[1])
				assert(pp.treeIndex>=1)
				pp.location_wrl=pp.location_wrl+fixedJointInfo[2]
			end
			table.insert(self.pathpoints, pp)
		end
	end
	
	-- pathpoints states
	self.pathpointposz = {}
	for i=1,#self.pathpoints do
		self.pathpointposz[i] = vector3(0)
	end

	-- muscle internal states (simulated)
	self.a = {}
	self.l_m = {}
	for i=1,#muscles do
		self.a[i] = 0.
		self.l_m[i] = 0.
	end

	-- muscle related states (not simulated)
	self.u = {}	-- excitation. actually not simulate state buf for convinience
	self.f_t = {}	-- tendon force. actually not simulate state buf for convinience
	self.dl_m = {}
	for i=1,#muscles do
		self.u[i] = 0.
		self.f_t[i] = 0.
		self.dl_m[i] = 0.
	end

	-- update info
	self.eppis = {}
	self.vppis = {}
	self.afppis = {}
	self.afmsclis = {}
	self.aflposz = {}
	self.afforces = {}
	self.afpoints = {}
	self.afdirecs = {}

	-- matrices
	self.C = matrixn()
	self.P = matrixn(self:getNumMuscles(), self:getNumMuscles())
	self.A = matrixn(self:getNumMuscles(), self:getNumMuscles())
	self.p = vectorn(self:getNumMuscles())
	self.P:setAllValue(0)
	self.A:setAllValue(0)

	-- update flag
	self.validPathPointPosz= false
	self.validTendonForces = false
	self.validPPIndices = false
	self.validActingForceStaticInfo = false
	self.validActingForceDynamicInfo = false
	self.validC = false
	self.validP = false
	self.validA = false
	self.validp = false
end

--function OsModel:initMetabolicEnergyConsumption()
	--self.totalEnergy = 0
--end

--function OsModel:getMetabolicEnergyConsumption()
	--return self.totalEnergy
--end

--function OsModel:integrateMetabolicEnergyRate(duration)
	--self.totalEnergy = self.totalEnergy + duration*self:getMetabolicEnergyRate()
--end

function OsModel:getMetabolicEnergyRate()
	local aM = matrixn(self:getNumMuscles(),1)
	aM:column(0):assign(tbl2vecn(self.a))

	local p = self:getp()
	local pM = matrixn(p:size(),1)
	pM:column(0):assign(p)

	local f_ce = vecn2tbl((self:getP() * self:getA() * aM):column(0))
	local f_p = vecn2tbl((self:getP() * pM):column(0))
	local f_mtu = t_add_t(f_ce, f_p)

	return getMetabolicEnergyRate(75, self.mmass, self.u, self.a, self.l_m, self.l_m_opt, self.dl_m, f_mtu, f_ce)
end

function OsModel:getActingForces()
	local P=self:getP()
	return (P*self:getA()*(tbl2vecn(self.a):column())+P*(self:getp():column())):column(0):copy()
end

function OsModel:removeMusclesExcept(exceptMuscles)
	----muscleSelection = {'psoas_r'}		-- hip flexor
	----muscleSelection = {'glut_max2_r'}	-- hip extensor
	----muscleSelection = {'vas_int_r'}	-- knee flexor
	--muscleSelection = {'bifemsh_r'}	-- knee extensor

	muscleIndexes = {}
	for i, exceptName in ipairs(exceptMuscles) do
		index = -1
		for i_muscle, muscleName in ipairs(self.msclname) do
			if muscleName==exceptName then
				index = i_muscle
				break
			end
		end
		muscleIndexes[i] = index 
	end
	newmuscles = {}
	for i=1,#muscleIndexes do
		newmuscles[i] = self.muscles[muscleIndexes[i]]
	end
	self.muscles = newmuscles

	self:initMuscleInfo(newmuscles)
end

function OsModel:removeMuscles(removeMuscles)
end

function OsModel:weakenMuscles(muscleNames, weakRatio)
	for i, weakenName in ipairs(muscleNames) do
		for i_muscle, muscleName in ipairs(self.msclname) do
			if muscleName==weakenName then
				self.f_m_o[i_muscle] = self.f_m_o[i_muscle]*weakRatio
				break
			end
		end
	end
end

function OsModel:setBoneForwardKinematics(bfk)
	self.bfk = bfk 

	-- initial equilibrium
	self:setIsometricFiberLengths()
end

function OsModel:writeMuscleProperty(filepath)
	util.saveTable(self.f_m_o, filepath)
end

function OsModel:readMuscleProperty(filepath)
	self.f_m_o = util.loadTable(filepath)
end

function OsModel:strengthenMuscles(method, param, option)
	if method=='best_activation' then
		local total_a = param
		local ratio = {}
		for i=1,#total_a do
			self.f_m_o[i] = self.f_m_o[i] + self.first_f_m_o[i]*(total_a[i]/10000.)
		end
	end
end

-----------------------------------------------------
-- update functions
-----------------------------------------------------
function OsModel:invalidatePPIndicesBasedInfo()
	self.validPPIndices = false
	self.validActingForceStaticInfo = false
	self.validC = false
end

function OsModel:invalidatePPPoszBasedInfo()
	self.validPathPointPosz = false
	self.validTendonForces = false
	self.validActingForceDynamicInfo = false
	self.validP = false
	self.validA = false
	self.validp = false
end

function OsModel:updatePathPointPosz()
	local is_pp = {}	
	for i_muscle=1,self:getNumMuscles() do
		extend_array(is_pp, self:getEnabledPPIndices(i_muscle))
	end

	for temp, i_pp in ipairs(is_pp) do
		local pathPoint = self.pathpoints[i_pp]
		local j = self.mLoader:bone(pathPoint.treeIndex)
		local lp = pathPoint.location_wrl
		--local gp = j:getFrame():toGlobalPos(tbl2vec3(lp))
		local gp = self.bfk:globalFrame(j):toGlobalPos(lp)
		self.pathpointposz[i_pp] = gp
	end

	self.validPathPointPosz= true
end

function OsModel:updateTendonForces()
	self.f_t = self:__getTendonForces()
	self.validTendonForces = true
end

function OsModel:updatePPIndices()
	self.eppis = {}
	self.vppis = {}

	local posedof = vectorn()
	self.bfk:getPoseDOFfromGlobal(posedof)

	for i_muscle=1,self:getNumMuscles() do

		-- update enabled pp indices
		self.eppis[i_muscle] = {}
		for i_pp, pp in ipairs(self.pathpoints) do
			if pp.muscleindex==i_muscle then
				if pp.type=='ConditionalPathPoint' then
					local dofIndex = -1
					if pp.coordinate=='knee_angle_r' then
						dofIndex = self.rkneeDOF
					elseif pp.coordinate=='knee_angle_l' then
						dofIndex = self.lkneeDOF
					elseif pp.coordinate=='hip_flexion_r' then
						dofIndex = self.rhipflexDOF
					elseif pp.coordinate=='hip_flexion_l' then
						dofIndex = self.lhipflexDOF
					elseif pp.coordinate=='joint1_coord_0' then
						dofIndex = 7
					end
					if dofIndex~=-1 then
						--print(pp.name, pp.range[1], posedof(dofIndex), pp.range[2])
						if posedof(dofIndex)>pp.range[1] and posedof(dofIndex)<pp.range[2] then
							table.insert(self.eppis[i_muscle], i_pp)
						end
					end
				else
					table.insert(self.eppis[i_muscle], i_pp)
				end
			end
		end

		-- update valid force pp indices
		self.vppis[i_muscle] = {}
		local ais = self.eppis[i_muscle]
		local fais = self.vppis[i_muscle]
		for i=2,#ais do
			if self.pathpoints[ais[i-1]].body~=self.pathpoints[ais[i]].body then
				if fais[#fais]~=ais[i-1] then
					table.insert(fais, ais[i-1])
				end
				table.insert(fais, ais[i])
			end
		end
	end

	self.validPPIndices = true
end

function OsModel:updateActingForceStaticInfo()
	self.afppis = {}
	self.aflposz = {}
	self.afmsclis = {}

	for i_joint=1,self:getNumJoints() do
		-- update acting force pp indices
		local actForcePPIndices = {}

		for i_muscle=1,self:getNumMuscles() do
			local is_pp = self:__getActingForceCandPPIndices(i_muscle)


			for i, i_pp in ipairs(is_pp) do
				if self.pathpoints[i_pp].treeIndex==i_joint then
					if i==1 then
						table.insert(actForcePPIndices, i_pp)
					elseif i==#is_pp then
						table.insert(actForcePPIndices, i_pp)
					else
						table.insert(actForcePPIndices, i_pp)
						table.insert(actForcePPIndices, i_pp)
					end
				end
			end
		end
		self.afppis[i_joint] = actForcePPIndices

		-- acting force local posz
		local localposz = {}
		for temp, i_pp in ipairs(actForcePPIndices) do
			table.insert(localposz, self.pathpoints[i_pp].location_wrl)
		end
		self.aflposz[i_joint] = localposz

		-- acting force muscle indices
		local actForceMsclIndices = {}
		for temp, i_pp in ipairs(actForcePPIndices) do
			table.insert(actForceMsclIndices, self.pathpoints[i_pp].muscleindex)
		end
		self.afmsclis[i_joint] = actForceMsclIndices
	end

	self.validActingForceStaticInfo = true
end

function OsModel:updateActingForceDynamicInfo()
	self.afforces = {}
	self.afpoints = {}
	self.afdirecs = {}

	for i_joint=1,self:getNumJoints() do
		-- acting force forces
		local actForceMsclIndices = self:getActingForceMuscleIndices(i_joint)
		local forceScalars = {}
		for temp, i_muscle in ipairs(actForceMsclIndices) do
			table.insert(forceScalars, self:getTendonForce(i_muscle))
		end
		self.afforces[i_joint] = forceScalars
		
		-- acting force points
		local actForcePPIndices = self:getActingForcePPIndices(i_joint)
		self.afpoints[i_joint] = self:__getPathPointPosz(actForcePPIndices)
	
		-- acting force directions
		local forceDirections = {}
		local dp, dn

		for i_muscle=1,self:getNumMuscles() do
			local is_pp = self:__getActingForceCandPPIndices(i_muscle)

			for i, i_pp in ipairs(is_pp) do
				if self.pathpoints[i_pp].treeIndex==i_joint then
					if i==1 then
						dn = self:__getPathPointPos(is_pp[i+1])-self:__getPathPointPos(i_pp); dn:normalize()
						table.insert(forceDirections, dn)
					elseif i==#is_pp then
						dp = self:__getPathPointPos(is_pp[i-1])-self:__getPathPointPos(i_pp); dp:normalize()
						table.insert(forceDirections, dp)
					else
						dn = self:__getPathPointPos(is_pp[i+1])-self:__getPathPointPos(i_pp); dn:normalize()
						table.insert(forceDirections, dn)
						dp = self:__getPathPointPos(is_pp[i-1])-self:__getPathPointPos(i_pp); dp:normalize()
						table.insert(forceDirections, dp)
					end
				end
			end
		end
		self.afdirecs[i_joint] = forceDirections
	end

	self.validActingForceDynamicInfo = true
end

function OsModel:updateC()
	self.C:setSize(self:getNumActingForces(),self:getNumMuscles())
	self.C:setAllValue(0)

	local start_row = 0
	for i_joint=1,self:getNumJoints() do
		local actForceMsclIndices = self:getActingForceMuscleIndices(i_joint)
		local C_i = matrixn(#actForceMsclIndices, self:getNumMuscles())
		C_i:setAllValue(0)

		for row=1, #actForceMsclIndices do
			local r = row-1
			local c = actForceMsclIndices[row]-1
			C_i:set(r, c, 1)
		end

		self.C:range(start_row,start_row+#actForceMsclIndices,0,self:getNumMuscles()):assign(C_i)
		start_row = start_row + #actForceMsclIndices
	end
	self.validC = true
end

function OsModel:updateP()
	local cos_pa = computeCosPennation(self.l_m, self.l_m_opt, self.pa_opt)
	for i_muscle=1,self:getNumMuscles() do
		local i = i_muscle - 1
		self.P:set(i, i, cos_pa[i_muscle])
	end
	self.validP = true
end
function OsModel:computeCosPennation()
	local cos_pa = computeCosPennation(self.l_m, self.l_m_opt, self.pa_opt)
	return cos_pa
end


function OsModel:updateA()
	for i_muscle=1,self:getNumMuscles() do
		local i = i_muscle - 1

		-- set larger activation only when solving qp.
		-- because l_m and l_m_prev is constant so problem can be infeasible
		local norm_l_m_prev = self.l_m[i_muscle]/self.l_m_opt[i_muscle]
		if norm_l_m_prev < .1 then
			norm_l_m_prev = .1
		end
		local norm_dl_m_prev = self.dl_m[i_muscle]/self.l_m_opt[i_muscle]
		if norm_dl_m_prev < -.9 then
			norm_dl_m_prev = -.9
		end

		local gal = computeNormActiveFiberForceByLength_scalar(norm_l_m_prev, self.gamma[i_muscle])
		local gv = computeNormActiveFiberForceByVelocity_scalar(norm_dl_m_prev, self.a_f[i_muscle], self.f_m_len[i_muscle], self.v_m_max[i_muscle])
		self.A:set(i, i, self.f_m_o[i_muscle]*gal*gv)
		--##dos
	end
	self.validA = true
end

function OsModel:updatep()
	for i_muscle=1,self:getNumMuscles() do
		local i = i_muscle -1 
		self.p:set(i,
			self.f_m_o[i_muscle]*computeNormPassiveFiberForceByLength_scalar(self.l_m[i_muscle]/self.l_m_opt[i_muscle], self.eps_m_o[i_muscle], self.k_pe[i_muscle])
			+ self.damping[i_muscle]*self.dl_m[i_muscle])
	end
	self.validp = true
end

-----------------------------------------------------
-- get / set properties
-----------------------------------------------------
function OsModel:getMuscleNames()
	return self.msclname
end

function OsModel:getJointNames()
	local names = {}
	for i_joint=1,self:getNumJoints() do
		table.insert(names, self.mLoader:getBoneByTreeIndex(i_joint):name())
	end
	return names
end

function OsModel:getFiberLengths()
	return self.l_m
end

function OsModel:getActivations()
	return self.a
end

function OsModel:setActivations(a)
	self.a = a
end

function OsModel:setExcitations(u)
	self.u = u
end

function OsModel:getNumMuscles()
	return #self.msclname
end

function OsModel:getNumJoints()
	return self.mLoader:numBone()-1
end

function OsModel:getC()
	if not self.validC then self:updateC() end
	return self.C
end

function OsModel:getP()
	if not self.validP then self:updateP() end
	return self.P
end

function OsModel:getA()
	if not self.validA then self:updateA() end
	return self.A
end

function OsModel:getp()
	if not self.validp then self:updatep() end
	return self.p
end

-----------------------------------------------------
-- muscle dynamics
-----------------------------------------------------
function OsModel:setIsometricFiberLengths()
	local a = self.a
	local l_mt = self:getTendonMuscleLengths()

	self.l_m = getIsometricFiberLength(a, l_mt, self.l_m_opt, self.pa_opt, self.l_t_sl, self.eps_t_o, self.eps_m_o, self.k_pe, self.gamma, self.a_f, self.f_m_len, self.damping,  self.v_m_max);

	--print'-----------getIsometricFiberLength'
	--print'fiber length'
	--printtblh(self.l_m)
	--print'tendon force'
	--printtblh(self:getTendonForces())
	--print'-----------getIsometricFiberLength'
end

function OsModel:getActivationDerivs(u, a)
	return computeActivationDeriv(u, a, self.tau_act, self.tau_deact)
end

function OsModel:getFiberLengthDerivs(a, l_m)
	local l_mt = self:getTendonMuscleLengths()
	return getFiberLengthDeriv(a, l_m, l_mt, self.l_m_opt, self.pa_opt, self.l_t_sl, self.eps_t_o, self.eps_m_o, 
						self.k_pe, self.gamma, self.a_f, self.f_m_len, self.damping, self.v_m_max, 'modified_damping')
end

function OsModel:getStateDerivs(u, a, l_m)
	return self:getActivationDerivs(u, a), self:getFiberLengthDerivs(a, l_m)
end

function OsModel:integrateMuscleDynamics(duration)
	local function muscledyn(t, state, u)
		local a,l_m,da,dl_m;
		a, l_m = split_array(state)
		da, dl_m = self:getStateDerivs(u, a, l_m)

		for i=1,#dl_m do
			if dl_m[i] < -self.l_m_opt[i]*self.v_m_max[i] then
				dl_m[i] = -self.l_m_opt[i]*self.v_m_max[i]
			elseif dl_m[i] > self.l_m_opt[i]*self.v_m_max[i] then
				dl_m[i] = self.l_m_opt[i]*self.v_m_max[i]
			end
		end

		self.dl_m = dl_m

		--io.write('\t\t');print'muscledyn'
		--io.write('\t\t');printtblh(l_m)
		--io.write('\t\t');printtblh(dl_m)

		return merge_array(da, dl_m)
	end

	local t = 0.
	local y = merge_array(self.a, self.l_m)
	--while t < duration do
	for i=1,round(duration/self.muscleTimeStep) do
		t, y = rk4_step(muscledyn, t, y, self.muscleTimeStep, self.u)

		local a, l_m = split_array(y)

		--io.write('\t');print'rk4_step'
		--io.write('\t');printtblh(a)
		--io.write('\t');printtblh(l_m)
		--io.write('\t');printtblh(self.dl_m)
	end
	self.a, self.l_m = split_array(y)

	--print'integrate'
	--printtblh(self.l_m)
end

function OsModel:getTendonForce(i_muscle)
	if not self.validTendonForces then self:updateTendonForces() end
	return self.f_t[i_muscle]
end

function OsModel:getTendonForces()
	if not self.validTendonForces then self:updateTendonForces() end
	return self.f_t
end

function OsModel:__getTendonForces()
	local l_m = self:getFiberLengths()
	local l_mt = self:getTendonMuscleLengths()

	local cos_pa = computeCosPennation(l_m, self.l_m_opt, self.pa_opt)

	--local l_t = l_mt - l_m * cos_pa
	--local eps_t = (l_t - self.l_t_sl) / self.l_t_sl
	local l_t = t_sub_t(l_mt, t_mul_t(l_m, cos_pa))
	local eps_t = t_div_t(t_sub_t(l_t, self.l_t_sl), self.l_t_sl)
	local f_t_norm = computeNormTendonForce(eps_t, self.eps_t_o)
	--local f_t = f_t_norm * self.f_m_o;
	local f_t = t_mul_t(f_t_norm, self.f_m_o)

	return f_t
end

function OsModel:getTendonMuscleLengths()
	local lengths = {}
	for i=1,self:getNumMuscles() do
		local ppPosz = self:getPathPointPositionsGlobal(i)
		lengths[i] = 0.
		for j=1,#ppPosz-1 do
			lengths[i] = lengths[i] + (ppPosz[j]-ppPosz[j+1]):length()
		end
	end
	return lengths
end

-----------------------------------------------------
-- pathpoint functions
-----------------------------------------------------

function OsModel:getEnabledPPIndices(i_muscle)
	if not self.validPPIndices then self:updatePPIndices() end
	return self.eppis[i_muscle]
end

function OsModel:getValidForcePPIndices(i_muscle)
	if not self.validPPIndices then self:updatePPIndices() end
	return self.vppis[i_muscle]
end

function OsModel:__getPathPointPos(i_pp)
	if not self.validPathPointPosz then self:updatePathPointPosz() end
	return self.pathpointposz[i_pp]
end

function OsModel:__getPathPointPosz(is_pp)
	if not self.validPathPointPosz then self:updatePathPointPosz() end

	local ps = {}
	for temp, i_pp in ipairs(is_pp) do
		table.insert(ps, self.pathpointposz[i_pp])
	end
	return ps
end

function OsModel:getPathPointPositionsGlobal(i_muscle)
	return self:__getPathPointPosz(self:getEnabledPPIndices(i_muscle))
end

function OsModel:getValidForcePathPointPositionsGlobal(i_muscle)
	return self:__getPathPointPosz(self:getValidForcePPIndices(i_muscle))
end

-----------------------------------------------------
-- acting force functions
-----------------------------------------------------

function OsModel:__getActingForceCandPPIndices(i_muscle)
	return self:getValidForcePPIndices(i_muscle)
	--return self:getEnabledPPIndices(i_muscle)
end

function OsModel:getNumActingForces()
	local count = 0
	for i_joint=1,self:getNumJoints() do
		count = count + #self:getActingForcePPIndices(i_joint)
	end
	return count
end

function OsModel:getActingForcePPIndices(i_joint)
	if not self.validActingForceStaticInfo then self:updateActingForceStaticInfo() end
	return self.afppis[i_joint]
end

function OsModel:getActingForceLocalPosz(i_joint)
	if not self.validActingForceStaticInfo then self:updateActingForceStaticInfo() end
	return self.aflposz[i_joint]
end

function OsModel:getActingForceMuscleIndices(i_joint)
	if not self.validActingForceStaticInfo then self:updateActingForceStaticInfo() end
	return self.afmsclis[i_joint]
end

-- changed over time
function OsModel:getActingForceDirections(i_joint)
	if not self.validActingForceDynamicInfo then self:updateActingForceDynamicInfo() end
	return self.afdirecs[i_joint]
end

function OsModel:getActingForcePoints(i_joint, actForcePPIndices)
	if not self.validActingForceDynamicInfo then self:updateActingForceDynamicInfo() end
	return self.afpoints[i_joint]
end

function OsModel:getActingForceScalars(i_joint, actForceMsclIndices)
	if not self.validActingForceDynamicInfo then self:updateActingForceDynamicInfo() end
	return self.afforces[i_joint]
end

--------------------------------------------
-- rendering
--------------------------------------------
function OsModel:recordBfkPose(iframe)
	if self.records[iframe]==nil then
		self.records[iframe] = {}
	end
	if self.records[iframe]["bfkPoseDOF"]~=nil then
		return
	end
	local state = vectorn()
	self.bfk:getPoseDOFfromGlobal(state)
	self.records[iframe]["bfkPoseDOF"] = state
end

function OsModel:recordRegisterObject(iframe, ...)
	if self.records[iframe]==nil then
		self.records[iframe] = {}
	end
	if self.records[iframe]["registerObject"]==nil then
		self.records[iframe]["registerObject"] = {}
	end
	self.records[iframe]["registerObject"][#self.records[iframe]["registerObject"]+1] = {...}
end

function OsModel:drawRecordedScene(iframe, objectList)
	if self.records[iframe]~=nil then
		local bfkpose = self.records[iframe]["bfkPoseDOF"]
		--self.bfk:setPoseDOF(bfkpose)
		self.mSkin:setPoseDOF(bfkpose)

		local regobjs = self.records[iframe]["registerObject"]
		if regobjs~=nil then
			for i=1,#regobjs do
				local regobj = regobjs[i]
				objectList:registerObject(regobj[1],regobj[2],regobj[3],regobj[4],regobj[5])
			end
		end
	end
end

function OsModel:drawJoints(iframe, objectList)
	is_joint = {}; for i_joint=1,self:getNumJoints() do table.insert(is_joint, i_joint) end
	local lines_j = matrixn()
	for temp, i_joint in ipairs(is_joint) do
		local gp = self.bfk:globalFrame(i_joint):toGlobalPos(vector3(0,0,0))
		lines_j:pushBack(vec32vecn(gp)*100)
		lines_j:pushBack(vec32vecn(gp+vector3(0,.1,0))*100)
		lines_j:pushBack(vec32vecn(gp)*100)
		lines_j:pushBack(vec32vecn(gp+vector3(.1,0,0))*100)
	end
	objectList:registerObject("joints", "BillboardLineList", "solidlightgreen", lines_j, 1.)
	self:recordRegisterObject(iframe, "joints", "BillboardLineList", "solidlightgreen", lines_j, 1.)
	self:recordBfkPose(iframe)
end

function OsModel:drawMuscles(iframe, objectList, lineWidth)
	lineWidth = lineWidth or 1
	local zero = {0,0,1}
	local one = {1,0,0}

	local is_muscle = {}; for i_muscle=1,self:getNumMuscles() do table.insert(is_muscle, i_muscle) end
	--local is_muscle = {6}

	local lines =vector3N()
	for temp, i_muscle in ipairs(is_muscle) do
		local ppPosz = self:getPathPointPositionsGlobal(i_muscle)
		local color = lerptbl(zero, one, self.a[i_muscle])
		for j=1,#ppPosz-1 do
			lines:pushBack((ppPosz[j])*100)
			lines:pushBack((ppPosz[j+1])*100)
			lines:pushBack((vector3(color[1], color[2], color[3])))
		end
	end

	if self:getNumMuscles()>0 then
		objectList:registerObject("muscles", "ColorBillboardLineList", "use_vertex_color", lines:matView(), lineWidth)

		--self:recordRegisterObject(iframe, "muscles", "ColorBillboardLineList", "use_vertex_color", lines:matView(), lineWidth)
	end
	--self:recordBfkPose(iframe)
end


function OsModel:drawMuscleWithForces(iframe, objectList)
	self:drawMuscles(iframe, objectList, .2)

	local is_muscle = {}; for i_muscle=1,self:getNumMuscles() do table.insert(is_muscle, i_muscle) end
	--local is_muscle = {6}

	is_joint = {}; for i_joint=1,self:getNumJoints() do table.insert(is_joint, i_joint) end
	--is_joint = {1,2,3}
	--is_joint = {2}

	-- all enabled pathpoints
	local lines_pp = matrixn()
	for temp, i_muscle in ipairs(is_muscle) do
		local ppPosz = self:getPathPointPositionsGlobal(i_muscle)
		for i_pp=1,#ppPosz do
			lines_pp:pushBack(vec32vecn(ppPosz[i_pp])*100)
			lines_pp:pushBack(vec32vecn(ppPosz[i_pp]+vector3(0,.01,0))*100)
		end
	end

	-- valid force pathpoints
	local lines_fpp = matrixn()
	for temp, i_muscle in ipairs(is_muscle) do
		local ppPosz = self:getValidForcePathPointPositionsGlobal(i_muscle)
		for i_pp=1,#ppPosz do
			lines_fpp:pushBack(vec32vecn(ppPosz[i_pp])*100)
			lines_fpp:pushBack(vec32vecn(ppPosz[i_pp]+vector3(0,.01,0))*100)
		end
	end

	local forceScale = .001
	--local forceScale = .0

	-- muscle force vectors by activation
	local lines_mf = matrixn()
	for temp, i_joint in ipairs(is_joint) do
		local forces = self:getActingForceScalars(i_joint)
		local points = self:getActingForcePoints(i_joint)
		local directions = self:getActingForceDirections(i_joint)
		for j=1,#forces do
			lines_mf:pushBack(vec32vecn(points[j])*100)
			lines_mf:pushBack(vec32vecn(points[j]+forces[j]*directions[j]*forceScale)*100)
		end
	end

	-- acting force vectors for rendering
	local lines_afr = matrixn()
	if self.actingforce_render:size() > 0 then
		local cnt = 0
		for temp, i_joint in ipairs(is_joint) do
			local points = self:getActingForcePoints(i_joint)
			local directions = self:getActingForceDirections(i_joint)
			for j=1,#points do

				lines_afr:pushBack(vec32vecn(points[j])*100)
				lines_afr:pushBack(vec32vecn(points[j]+self.actingforce_render(cnt)*directions[j]*forceScale)*100)
				if self.actingforce_render(cnt)>0 then
					lines_afr:pushBack(vec32vecn(vector3(1,0,0)))
				else
					lines_afr:pushBack(vec32vecn(vector3(0,0,0)))
				end

				cnt = cnt+1
			end
		end
	end

	-- register object
	if self:getNumMuscles()>0 then
		objectList:registerObject("pathPoints", "BillboardLineList", "solidblue", lines_pp, .5)
		objectList:registerObject("forceActingPathPoints", "BillboardLineList", "solidgreen", lines_fpp, .7)
		objectList:registerObject("muscleForces", "BillboardLineList", "solidgreen", lines_mf, 1.)
		if self.actingforce_render:size() > 0 then
			objectList:registerObject("actingforce_render", "ColorBillboardLineList", "solidred", lines_afr, 1.)
		end

		--[[
		self:recordRegisterObject(iframe, "pathPoints", "BillboardLineList", "solidblue", lines_pp, .5)
		self:recordRegisterObject(iframe, "forceActingPathPoints", "BillboardLineList", "solidgreen", lines_fpp, .7)
		self:recordRegisterObject(iframe, "muscleForces", "BillboardLineList", "solidgreen", lines_mf, 1.)
		if self.actingforce_render:size() > 0 then
			self:recordRegisterObject(iframe, "actingforce_render", "ColorBillboardLineList", "solidred", lines_afr, 1.)
		end
		]]
	end
	--self:recordBfkPose(iframe)
end

function OsModel:setActingForces_render(actingforce_render)
	self.actingforce_render = actingforce_render
end

--------------------------------------------
-- load
--------------------------------------------
function OsModel:loadDOF(dofpath)
	self.mMotionDOFcontainer=MotionDOFcontainer(self.mLoader.dofInfo, dofpath)

	mMotionDOFcontainer = self.mMotionDOFcontainer
	mLoader = self.mLoader

	_G.chosenMotFile=chosenFile
	--self.mSkin:applyMotionDOF(self.mMotionDOFcontainer.mot)
	--RE.motionPanel():motionWin():detachSkin(self.mSkin)
	--RE.motionPanel():motionWin():addSkin(self.mSkin)
	useNearestSamplingWhenExportingBVH=true
end

function OsModel:loadDOF_repeat(dofpath)
	self.mMotionDOFcontainer=MotionDOFcontainer(self.mLoader.dofInfo, dofpath)

	local origlen = self.mMotionDOFcontainer.mot:rows()
	local sixsteplen
	if string.find(dofpath, 'ipfrun')~=nil then
		sixsteplen = origlen
	else
		sixsteplen = math.floor((origlen/7)*6)
	end

	local motc=self.mMotionDOFcontainer
	local startConfig=motc.mot:convertToDeltaRep()
	motc:resize(sixsteplen*40*4)
	
	for i=0,40-1 do
		for j=0, sixsteplen-1 do
			motc.mot:row(j+sixsteplen*i):assign(motc.mot:row(j))
		end
	end
	motc.mot:reconstructData(startConfig)

	--self.mMotionDOFcontainer:resize(sixsteplen*2)
	--self.mMotionDOFcontainer.mot:range(assign(dof)

	mMotionDOFcontainer = self.mMotionDOFcontainer
	mLoader = self.mLoader

	_G.chosenMotFile=chosenFile
	--self.mSkin:applyMotionDOF(self.mMotionDOFcontainer.mot)
	--RE.motionPanel():motionWin():detachSkin(self.mSkin)
	--RE.motionPanel():motionWin():addSkin(self.mSkin)
	useNearestSamplingWhenExportingBVH=true
end

function OsModel:loadBVH(bvhpath, heightOffset)
	heightOffset=heightOffset or 0.
	self.mMotionDOFcontainer=MotionDOFcontainer(self.mLoader.dofInfo)

	mMotionDOFcontainer = self.mMotionDOFcontainer
	mLoader = self.mLoader

	local dof=_importBVHys(bvhpath, heightOffset, 1.0)
	self.mMotionDOFcontainer:resize(dof:rows())
	self.mMotionDOFcontainer.mot:assign(dof)

	---- fix except root joint
	--self.mMotionDOFcontainer.mot:matView():sub(0,0,6,0):setAllValue(0)

	_G.chosenMotFile=chosenFile
	self.mSkin:applyMotionDOF(self.mMotionDOFcontainer.mot)
	RE.motionPanel():motionWin():detachSkin(self.mSkin)
	RE.motionPanel():motionWin():addSkin(self.mSkin)
	useNearestSamplingWhenExportingBVH=true
end

