
require("utilfunc")
local modified_damping=0;
local OsimModel = LUAclass()

-----------------------------------------------------
-- constructor
-----------------------------------------------------
function OsimModel:__init(wrlpath, luamsclpath, model, option)
	--debug mode (drawing current muscle model pose, made by taesoo 240216)
	self.debugMode=false
	local luamsclpath = luamsclpath or ''
	local model = model or nil
	
	-- load .wrl
	if type(wrlpath)=='string' then
		self.mLoader=MainLib.VRMLloader(wrlpath)
	else
		self.mLoader=wrlpath
	end
	if self.debugMode then
		self.mSkin=RE.createVRMLskin(self.mLoader, false)
		self.mSkin:setTranslation(100,0,0)
		self.mSkin:setScale(100)
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
		self:weakenMuscles(model.weakenMuscles, model.weakRatio)
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

	--do return end

	for i_pp, pp in ipairs(self.pathpoints) do
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
			pp.conditionalDOFindex=dofIndex
		else
			assert(pp.range==nil)
		end
	end

	self:prepareCPP()

	-- muscle dynamics simulation
	--self.muscleTimeStep = 1./600.
	self.muscleTimeStep = 1./840.
	--self.muscleTimeStep = 1./1200.
	--self.muscleTimeStep = 1./1800.
	--self.muscleTimeStep = 1./3600.
	--self.muscleTimeStep = 1./8400.
	--self.muscleTimeStep = 1./12000.



	-- muscle save
	self.first_f_m_o = deepCopyTable(self.f_m_o)
	self.f_m_o=tbl2vecd(self.f_m_o)
	if model.mscl_strength_scale then
		self.f_m_o:assign(self.f_m_o:vecView()* model.mscl_strength_scale )
	end

	--self.totalEnergy = 0
end
function OsimModel:setAction(action)
	self.u=vecn2vecd(action)
	self.a=vecn2vecd(action)
end
function OsimModel:get_max_isometric_force()
	return self.f_m_o:vecView()
end

function OsimModel:init_A_U()
	local osim=self
	osim.a = {} -- activation. 
	osim.u = {}	-- excitation. actually not simulate state buf for convinience
	for i=1, osim:getNumMuscles() do
		osim.a[i]=0.0
		osim.u[i]=0.0
	end

	osim.a=tbl2vecd(osim.a)
	osim.u=tbl2vecd(osim.u)
end
function OsimModel:setU(imuscle, u)
	self.u:vecView():set(imuscle-1, u)
end

function OsimModel:prepareCPP()
	-- for c++
	self.a=tbl2vecd(self.a)
	self.l_m=tbl2vecd(self.l_m)
	self.dl_m=tbl2vecd(self.dl_m)
	self.l_m_opt=tbl2vecd(self.l_m_opt)
	self.pa_opt=tbl2vecd(self.pa_opt)
	self.l_t_sl=tbl2vecd(self.l_t_sl)
	self.eps_t_o=tbl2vecd(self.eps_t_o)
	self.eps_m_o=tbl2vecd(self.eps_m_o)
	self.k_pe=tbl2vecd(self.k_pe)
	self.gamma=tbl2vecd(self.gamma)
	self.a_f=tbl2vecd(self.a_f)
	self.f_m_len=tbl2vecd(self.f_m_len)
	self.damping=tbl2vecd(self.damping)
	self.v_m_max=tbl2vecd(self.v_m_max)
	self.tau_act=tbl2vecd(self.tau_act)
	self.tau_deact=tbl2vecd(self.tau_deact)
	self.mmass=tbl2vecd(self.mmass)
	self.pp=path_points(self:getNumMuscles(), #self.pathpoints)
	for i_pp,pp in ipairs(self.pathpoints) do
		if pp.range then
			self.pp:setPathPoint(i_pp-1, pp.treeIndex, pp.muscleindex-1, pp.conditionalDOFindex or -1, pp.range[1], pp.range[2], pp.location_wrl)
		else
			self.pp:setPathPoint(i_pp-1, pp.treeIndex, pp.muscleindex-1, -2, 0,0, pp.location_wrl)
		end
	end
	-- for c++ finished
end
function OsimModel:initMuscleInfo(muscles)
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
	self.tau_act = msclProp.activation_time_constant or CT.ones(#muscles)*0.01
	self.tau_deact = msclProp.deactivation_time_constant or CT.ones(#muscles)*0.04
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

	if self.mmass ==nil then
		-- error-fallback
		-- 없는 정보는 다른 파일에서 가져오기.
		local fullbodyInfo=loadfile('../Resource/motion/opensim/FullBody2_lee.luamscl')()
		fullbodyMuscles={}
		for i,v in ipairs(fullbodyInfo) do
			fullbodyMuscles[v.name]=v
		end

		self.mmass={}
		local failed=false
		for i,v in ipairs(self.msclname) do
			local msclinfo=fullbodyMuscles[v]
			if msclinfo then
				assert(msclinfo.mass)
				self.mmass[i]=msclinfo.mass
			else
				failed=true
			end
		end
		if failed then
			self.mmass=CT.ones(#muscles)
		end
	end
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
				if pp.treeIndex<1 then
					local fixedJointInfo2=mLoaderFixedJoints[fixedJointInfo[1]]
					pp.treeIndex=self.mLoader:getTreeIndexByName(fixedJointInfo2[1])
					pp.location_wrl=pp.location_wrl+fixedJointInfo[2]+fixedJointInfo2[2]
				else
					pp.location_wrl=pp.location_wrl+fixedJointInfo[2]
				end
				assert(pp.treeIndex>=1)
			end
			table.insert(self.pathpoints, pp)
		end
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


	self.P = vectorn(self:getNumMuscles()) -- diagonal matrix
	self.A = vectorn(self:getNumMuscles()) -- diagonal matrix
	self.p = vectorn(self:getNumMuscles())
	self.P:setAllValue(0)
	self.A:setAllValue(0)

	-- update flag
	self.validPPIndices = false
	self.validPAp = false
end

--function OsimModel:initMetabolicEnergyConsumption()
	--self.totalEnergy = 0
--end

--function OsimModel:getMetabolicEnergyConsumption()
	--return self.totalEnergy
--end

--function OsimModel:integrateMetabolicEnergyRate(duration)
	--self.totalEnergy = self.totalEnergy + duration*self:getMetabolicEnergyRate()
--end

function OsimModel:getMetabolicEnergyRate()

	local P, A, p=self:getPAp()
	
	local f_ce = P * A* self.a
	local f_p = P*p
	local f_mtu = f_ce+f_p

	return Eigen.getMetabolicEnergyRate(75, self.mmass, self.u, self.a, self.l_m, self.l_m_opt, self.dl_m, f_mtu, f_ce)
end

function OsimModel:getActingForces()
	local P, A, p=self:getPAp()
	local forces=P*A*self.a +P*p

	--[[
	print('lm', self.l_m)
	print('dlm', self.dl_m)
	print('P', P)
	print('A', A)
	print('a', self.a)
	print('p', p)
	]]--

	return forces
	-- w.M*ddq - J.T*Vaf*C*P*A*a - J.T*Vaf*C*P*p = w.b
	-- w.M*ddq - J.T*Vaf*C*P*A*a = w.b + J.T*Vaf*C*P*p 
	-- -> (w.M -J.T*Vaf*C*P*A)(ddq;a) = w.b + J.T*Vaf*C*P*p 
end

function OsimModel:removeMusclesExcept(exceptMuscles)
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

function OsimModel:removeMuscles(removeMuscles)
end

function OsimModel:weakenMuscles(muscleNames, weakRatio)
	for i, weakenName in ipairs(muscleNames) do
		for i_muscle, muscleName in ipairs(self.msclname) do
			if muscleName==weakenName then
				self.f_m_o[i_muscle] = self.f_m_o[i_muscle]*weakRatio
				break
			end
		end
	end
end

function OsimModel:setBoneForwardKinematics(bfk)
	self.bfk = bfk 
	if not self.validPPIndices then self:updatePPIndices() end

	-- initial equilibrium
	self:setIsometricFiberLengths()
end

function OsimModel:writeMuscleProperty(filepath)
	util.saveTable(self.f_m_o, filepath)
end

function OsimModel:readMuscleProperty(filepath)
	self.f_m_o = util.loadTable(filepath)
end

function OsimModel:strengthenMuscles(method, param, option)
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
function OsimModel:invalidatePPIndicesBasedInfo()
	self.validPPIndices = false
end

function OsimModel:invalidatePPPoszBasedInfo()
	self.validPAp = false
end


function OsimModel:updateTendonForces()
	self.f_t = self:__getTendonForces()
	self.validTendonForces = true
end

function OsimModel:updatePPIndices()
	local posedof = vectorn()
	self.bfk:getPoseDOFfromGlobal(posedof)

	self.pp:updatePPindices(self.bfk, posedof)
	if false then
		for i=0, self:getNumMuscles()-1 do
			print(self.pp:get_eppis(i))
		end
		local pose=vectorn()
		self.bfk:getPoseDOFfromGlobal(pose)
		print('pose', pose)
	end

	self.validPPIndices = true
end



function OsimModel:updatePAp()
	local cos_pa = Eigen.computeCosPennation(self.l_m, self.l_m_opt, self.pa_opt)
	self.P=cos_pa
	local norm_l_m_prev = self.l_m/self.l_m_opt
	local norm_dl_m_prev = self.dl_m/self.l_m_opt

	local gal_gv = Eigen.computeNormActiveFiberForce(norm_l_m_prev, norm_dl_m_prev, self.md)
	self.A=self.f_m_o*gal_gv
	self.p=Eigen.computePassiveFiberForce(self.f_m_o, norm_l_m_prev, self.dl_m, self.md)

	self.validPAp = true
end
function OsimModel:computeCosPennation()
	local cos_pa = Eigen.computeCosPennation(self.l_m, self.l_m_opt, self.pa_opt)
	return cos_pa
end


-----------------------------------------------------
-- get / set properties
-----------------------------------------------------
function OsimModel:getMuscleNames()
	return self.msclname
end

function OsimModel:getJointNames()
	local names = {}
	for i_joint=1,self:getNumJoints() do
		table.insert(names, self.mLoader:getBoneByTreeIndex(i_joint):name())
	end
	return names
end

function OsimModel:getFiberLengths()
	return self.l_m
end

function OsimModel:getActivations()
	return self.a
end

function OsimModel:setActivations(a)
	self.a = a
end

function OsimModel:setExcitations(u)
	self.u = u
end

function OsimModel:getNumMuscles()
	return #self.msclname
end

function OsimModel:getNumJoints()
	return self.mLoader:numBone()-1
end


function OsimModel:getPAp()
	if not self.validPAp then self:updatePAp() end
	return self.P, self.A, self.p
end

-----------------------------------------------------
-- muscle dynamics
-----------------------------------------------------
function OsimModel:setIsometricFiberLengths()
	local a = self.a
	local l_mt = self:getTendonMuscleLengths()

	self.l_m = Eigen.getIsometricFiberLength(a, l_mt, self.l_m_opt, self.pa_opt, self.l_t_sl, self.eps_t_o, self.eps_m_o, self.k_pe, self.gamma, self.a_f, self.f_m_len, self.damping,  self.v_m_max);

	--print'-----------getIsometricFiberLength'
	--print'fiber length'
	--printtblh(self.l_m)
	--print'tendon force'
	--printtblh(self:getTendonForces())
	--print'-----------getIsometricFiberLength'
end



function OsimModel:integrateMuscleDynamics(duration)

	if not self.validPPIndices then self:updatePPIndices() end

	local l_mt=self:getTendonMuscleLengths()
	local md=muscle_data(l_mt,  self.l_m_opt, self. pa_opt, self. l_t_sl, self. eps_t_o, self. eps_m_o, self. k_pe, self. gamma, self. a_f, self. f_m_len, self. damping, self. v_m_max, modified_damping, self. tau_act, self. tau_deact);

	if type(self.u)=='table' then
		self.u=tbl2vecd(self.u)
	end
	Eigen.integrateMuscleDynamics(duration, self.muscleTimeStep, self.a,
	self.l_m, self.dl_m, self.u, md)

	self.md=md
	--print'integrate'
	--printtblh(self.l_m)
end

function OsimModel:getTendonForce(i_muscle)
	if not self.validTendonForces then self:updateTendonForces() end
	return self.f_t:vecView()(i_muscle-1)
end

function OsimModel:getTendonForces()
	if not self.validTendonForces then self:updateTendonForces() end
	return self.f_t
end

function OsimModel:__getTendonForces()
	local l_m = self:getFiberLengths()
	local l_mt = self:getTendonMuscleLengths()

	local cos_pa = Eigen.computeCosPennation(l_m, self.l_m_opt, self.pa_opt)

	local l_t = l_mt- l_m* cos_pa
	local eps_t = (l_t- self.l_t_sl)/ self.l_t_sl
	local f_t_norm = Eigen.computeNormTendonForce(eps_t, self.eps_t_o)
	local f_t = f_t_norm* self.f_m_o

	return f_t
end

function OsimModel:getTendonMuscleLengths()
	assert(self.validPPIndices )
	return self.pp:get_lm_t()
end

-----------------------------------------------------
-- pathpoint functions
-----------------------------------------------------

function OsimModel:getEnabledPPIndices(i_muscle)
	if not self.validPPIndices then self:updatePPIndices() end
	return self.pp:get_eppis(i_muscle-1)
end

function OsimModel:getValidForcePPIndices(i_muscle)
	if not self.validPPIndices then self:updatePPIndices() end
	return self.pp:get_vppis(i_muscle-1)
end

function OsimModel:__getPathPointPos(i_pp)
	if not self.validPPIndices then self:updatePPIndices() end
	return self.pp:getPathPointPos(i_pp-1)
end

function OsimModel:__getPathPointPosz(is_pp)
	if not self.validPPIndices then self:updatePPIndices() end

	local ps = {}
	for i=0, is_pp:size()-1 do
		local i_pp=is_pp(i)+1
		table.insert(ps, self.pp:getPathPointPos(i_pp-1))
	end
	return ps
end

function OsimModel:getPathPointPositionsGlobal(i_muscle)
	return self:__getPathPointPosz(self:getEnabledPPIndices(i_muscle))
end

function OsimModel:getValidForcePathPointPositionsGlobal(i_muscle)
	return self:__getPathPointPosz(self:getValidForcePPIndices(i_muscle))
end

--------------------------------------------
-- rendering
--------------------------------------------


function OsimModel:drawJoints(iframe, objectList)
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
end

function OsimModel:drawMuscles(iframe, objectList, lineWidth)

	if self.debugMode then
		self.mSkin:setSamePose(self.bfk)
	end
	lineWidth = lineWidth or 1
	local zero = {0,0,1}
	local one = {1,0,0}

	local is_muscle = {}; for i_muscle=1,self:getNumMuscles() do table.insert(is_muscle, i_muscle) end
	--local is_muscle = {6}

	local lines =vector3N()
	for temp, i_muscle in ipairs(is_muscle) do
		local ppPosz = self:getPathPointPositionsGlobal(i_muscle)
		local color = lerptbl(zero, one, self.a:vecView()(i_muscle-1))
		for j=1,#ppPosz-1 do
			lines:pushBack((ppPosz[j])*100)
			lines:pushBack((ppPosz[j+1])*100)
			lines:pushBack((vector3(color[1], color[2], color[3])))
		end
	end

	if self:getNumMuscles()>0 then
		--objectList:registerObject("muscles", "ColorBillboardLineList", "use_vertex_color", lines:matView(), lineWidth)
		dbg.drawBillboard(lines:matView(), "muscles", "use_vertex_color", lineWidth, "ColorBillboardLineList")

	end
end



function OsimModel:drawMuscleWithForces(iframe, objectList)
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

	do return end
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

--------------------------------------------
-- load
--------------------------------------------
function OsimModel:loadDOF(dofpath)
	self.mMotionDOFcontainer=MotionDOFcontainer(self.mLoader.dofInfo, dofpath)

	mMotionDOFcontainer = self.mMotionDOFcontainer
	mLoader = self.mLoader

	_G.chosenMotFile=chosenFile
	--self.mSkin:applyMotionDOF(self.mMotionDOFcontainer.mot)
	--RE.motionPanel():motionWin():detachSkin(self.mSkin)
	--RE.motionPanel():motionWin():addSkin(self.mSkin)
	useNearestSamplingWhenExportingBVH=true
end

function OsimModel:loadDOF_repeat(dofpath)
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

function OsimModel:loadBVH(bvhpath, heightOffset)
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

function OsimModel:applyMuscleForces(ms)
	local osim=self
	local numJoint = osim:getNumJoints()
	local numMuscle = osim:getNumMuscles()

	--local forces_t=osim:getTendonForces()
	local forces=osim:getActingForces()

	self.pp:applyMuscleForces(ms.ichara, forces, ms.sim)
end

return OsimModel
