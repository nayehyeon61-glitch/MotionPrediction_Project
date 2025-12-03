--OsModel=require("subRoutines/OsimModel")
OsModel=require("subRoutines/OsimModel_cpp") -- much faster
--require("OsModel") -- no  longer maintained
local ms=LUAclass()


function ms:__init(loader, luamsclepath,model )

	self.loader=loader
	self.loader:setVoca(model.bones)
	self.osim=OsModel(loader, luamsclepath, model, {})
	self.objectList=Ogre.ObjectList()
end

-- call after initSimulation
function ms:initSimulation(sim, ichara)
	self.sim=sim
	self.ichara=ichara
	self.worldState=sim:getWorldState(ichara)
	local osim=self.osim

	osim:init_A_U()

	osim:invalidatePPIndicesBasedInfo()
	osim:invalidatePPPoszBasedInfo()
	-- input: a
	-- output: l_mt (tendon muscle lengths), l_m
	osim:setBoneForwardKinematics(self.worldState)

end

function ms:addWorldForceToBone(i_joint, point, force)

	if self.sim.addWorldForceAtPos then

		self.sim:addWorldForceAtPos(self.ichara, self.loader:bone(i_joint), point, force);
		return 
	end


	local ichara=self.ichara
	local state=self.worldState
	local invT=state:globalFrame(i_joint):inverse()
	self.sim:addForceToBone(ichara, self.loader:bone(i_joint), invT*point, invT.rotation*force)
end

function ms:setActivation(imuscle, u)
	local osim=self.osim
	 -- desired activation. it takes time to actually update osim.a
	osim:setU(imuscle, u)
end
function ms:findMuscle(name)
	local index = -1
	local osim=self.osim
	for i_muscle, muscleName in ipairs(osim.msclname) do
		if muscleName==name then
			index = i_muscle
			break
		end
	end
	return index
end

-- before stepSimulation
function ms:applyMuscleForces()
	local osim=self.osim
	osim:invalidatePPIndicesBasedInfo()
	osim:invalidatePPPoszBasedInfo()

	local timer=util.PerfTimer2()--1, niter.."simulation")
	timer:start()
	-- input: osim.u
	-- output. osim.a, osim.l_m
	osim:integrateMuscleDynamics(self.sim:getTimestep())

	--RE.output2('muscle_integ', timer:stop())
	timer:start()


	local osim=self.osim
	osim:applyMuscleForces(self)
	--RE.output2('muscle_apply', timer:stop())
end

function ms:getMetabolicEnergyRate()
	return self.osim:getMetabolicEnergyRate()
end
function ms:drawMuscles()
	local lineWidth=1
	self.osim:drawMuscles(nil, self.objectList, lineWidth)
	--self.osim:drawMuscleWithForces(nil, self.objectList, lineWidth)
end



return ms


