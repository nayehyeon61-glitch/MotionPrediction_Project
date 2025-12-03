require("IPC_based/LocoSynthesis")

OnlineSynthesisMuscle=LUAclass(OnlineSynthesis2)

function OnlineSynthesisMuscle:getStates()
	local storeInfo = OnlineSynthesis2.getStates(self)

	storeInfo.pathpointposz    = mOsim.pathpointposz

	storeInfo.a    = mOsim.a
	storeInfo.l_m  = mOsim.l_m
	storeInfo.u    = mOsim.u
	storeInfo.f_t  = mOsim.f_t
	storeInfo.dl_m = mOsim.dl_m

	storeInfo.eppis    = mOsim.eppis   
	storeInfo.vppis    = mOsim.vppis   
	storeInfo.afppis   = mOsim.afppis  
	storeInfo.afmsclis = mOsim.afmsclis
	storeInfo.aflposz  = mOsim.aflposz 
	storeInfo.afforces = mOsim.afforces
	storeInfo.afpoints = mOsim.afpoints
	storeInfo.afdirecs = mOsim.afdirecs

	storeInfo.C = mOsim.C
	storeInfo.P = mOsim.P
	storeInfo.A = mOsim.A
	storeInfo.p = mOsim.p

	storeInfo.validPathPointPosz          = mOsim.validPathPointPosz         
	storeInfo.validTendonForces           = mOsim.validTendonForces          
	storeInfo.validPPIndices              = mOsim.validPPIndices             
	storeInfo.validActingForceStaticInfo  = mOsim.validActingForceStaticInfo 
	storeInfo.validActingForceDynamicInfo = mOsim.validActingForceDynamicInfo
	storeInfo.validC                      = mOsim.validC                     
	storeInfo.validP                      = mOsim.validP                     
	storeInfo.validA                      = mOsim.validA                     
	storeInfo.validp                      = mOsim.validp                     

	--storeInfo.totalEnergy = mOsim.totalEnergy 

	return storeInfo
end

function OnlineSynthesisMuscle:_restoreStates(storeInfo)
	OnlineSynthesis2._restoreStates(self, storeInfo)

	mOsim.pathpointposz    = storeInfo.pathpointposz

	mOsim.a    = storeInfo.a
	mOsim.l_m  = storeInfo.l_m
	mOsim.u    = storeInfo.u
	mOsim.f_t  = storeInfo.f_t
	mOsim.dl_m = storeInfo.dl_m

	mOsim.eppis    = storeInfo.eppis   
	mOsim.vppis    = storeInfo.vppis   
	mOsim.afppis   = storeInfo.afppis  
	mOsim.afmsclis = storeInfo.afmsclis
	mOsim.aflposz  = storeInfo.aflposz 
	mOsim.afforces = storeInfo.afforces
	mOsim.afpoints = storeInfo.afpoints
	mOsim.afdirecs = storeInfo.afdirecs

	mOsim.C = storeInfo.C
	mOsim.P = storeInfo.P
	mOsim.A = storeInfo.A
	mOsim.p = storeInfo.p

	mOsim.validPathPointPosz          = storeInfo.validPathPointPosz         
	mOsim.validTendonForces           = storeInfo.validTendonForces          
	mOsim.validPPIndices              = storeInfo.validPPIndices             
	mOsim.validActingForceStaticInfo  = storeInfo.validActingForceStaticInfo 
	mOsim.validActingForceDynamicInfo = storeInfo.validActingForceDynamicInfo
	mOsim.validC                      = storeInfo.validC                     
	mOsim.validP                      = storeInfo.validP                     
	mOsim.validA                      = storeInfo.validA                     
	mOsim.validp                      = storeInfo.validp                     

	--mOsim.totalEnergy = storeInfo.totalEnergy 
end
