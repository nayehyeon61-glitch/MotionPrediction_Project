require("config")

package.projectPath='../'
package.path=package.path..";../Samples/QP_controller/lua/?.lua" --;"..package.path
package.path=package.path..";../Samples/classification/lua/?.lua" --;"..package.path

require("IPC_based/common")

package.path=package.path..";../Samples/ysscripts/samples/?.lua" --;"..package.path
--require("IPC_based/useCases")
require("useCasesMuscle")

require("subRoutines/motiongraph")

require("IPC_based/LocoGraph")

-- 최적화 루틴 사용법
-- trackingOptimizer.lua최적화 후 cartpoleanalysis2.lua로 타겟포즈 익스포트한다.
-- 그 후 useCase.lua적당히 고친 후 zmpcontrollerfullbody2.lua에서 불러온다.
--require("IPC_based/LocoSimulation")
-- following values are automatically connected to UI.

package.path=package.path..";../Samples/ysscripts/lib/?.lua" --;"..package.path
require('QPservo2')
require("utilfunc")
require("OsModel")
require("LocoSimMuscle")


finalRender=true
cleanVisualize=true
function ctor()
	this:create('Button', 'attach camera', 'attach camera')
	this:create("Button"	, "impulse", "impulse",0)
	this:updateLayout()

	RE.renderer():fixedTimeStep(true)
	mEventReceiver=EVR()

	model = scenarios.toModel(useCase.scenario)
	mOsim = OsModel(model.wrlpath, model.luamsclpath, model)
	mLoader = mOsim.mLoader

	print('original f_m_o')
	printtblh(mOsim.f_m_o)
	if useCase.f_m_o ~= nil then
		mOsim.f_m_o = useCase.f_m_o
		print('adapted f_m_o')
		printtblh(mOsim.f_m_o)
	end

	do
		--temp
		local container=MotionDOFcontainer(mLoader.dofInfo, model.mot_file)
		mMotionDOF=container.mot
		mMotionDOF:resize(1200)

		-- fill mot dof
		mOsim.mSkin:applyMotionDOF(mMotionDOF)
		RE.motionPanel():motionWin():detachSkin(mOsim.mSkin)
		RE.motionPanel():motionWin():addSkin(mOsim.mSkin)
	end
	mObjectList=Ogre.ObjectList()
		
	mSynthesis	= OnlineLocoSynthesis:new()

	boolean_options={}
	boolean_options.attachCamera=useCase.attachCamera or false
	boolean_options.drawDisplacementMap=false
	boolean_options.useOrientationCorrection=false
	boolean_options.solveIK=true
	boolean_options.drawPredictedCOM=false
	boolean_options.drawPredictedZMP =true
	boolean_options.drawPelvis=false
	boolean_options.drawControlForce=true
	boolean_options.drawMatching=false

	-- camera
	RE.viewpoint().vpos:set(30, 100, 300)
	--RE.viewpoint().vpos:set(300, 100, 50)
	RE.viewpoint().vat:set(0,80,0)
	RE.viewpoint():update()
	
	mCameraInfo={}
	local curPos=vector3(0,0,0)
	mCameraInfo.vpos=RE.viewpoint().vpos-curPos
	mCameraInfo.vat=RE.viewpoint().vat-curPos
	recCurPos = {}

	titlebar:create()
end

function dtor()
   dbg.finalize()
   if mSynthesis then
	   mSynthesis:__finalize()
	   mSynthesis=nil
   end
   titlebar:destroy()
end

function onCallback(w, userData)

	if w:id()=='attach camera' then

		mEventReceiver:attachCamera()
   elseif w:id()=="impulse" then
	float_options={}
	float_options.impulseMagnitude={ val=200, min=10, max=1000}
	float_options.impulseDuration={ val=0.2, min=1, max=100}
	float_options.desiredAngleX={ val=0, min=-1, max=1}
	float_options.desiredAngleZ={ val=0, min=-1, max=1}
	float_options.timescaleL={ val=0, min=-0.5, max=0.5}
      mSynthesis.impulse=float_options.impulseDuration.val*model.simulationFrameRate
      mSynthesis.impulseDir=vector3(1,0,0)*float_options.impulseMagnitude.val
      mSynthesis.impulseGizmo=mSynthesis.objectList:registerEntity("arrow2", "arrow2.mesh")
      mSynthesis.impulseGizmo:setScale(2,2,2)
	end
end

function frameMove(fElapsedTime)
	--mSynthesis:oneStep()
	--mSynthesis:oneStepSimul()
	--if error_feedback_method~=EFM.NONE then	 
		--mSynthesis:prepareNextStep()      
	--end
end

if EventReceiver then
	EVR=LUAclass(EventReceiver)
	function EVR:__init(graph)
		self.currFrame=0
		self.cameraInfo={}
	end
end

g_kcal = 0
g_prev_iframe = 0
g_int_met = 0
function EVR:attachCamera()
	local s=0
	local e=g_prev_iframe
	self.trajectory=matrixn(e,3)
	self.trajectoryOri=matrixn(e,4)

	for f=s,e-1 do
		--self.trajectory:row(f):setVec3(0, MotionDOF.rootTransformation(mMotionDOF:row(f)).translation)
		--self.trajectory:row(f):set(1,0)
		--self.trajectoryOri:row(f):setQuater(0, MotionDOF.rootTransformation(mMotionDOF:row(f)).rotation:rotationY())
		self.trajectory:row(f):setVec3(0, MotionDOF.rootTransformation(mOsim.records[f+1]["bfkPoseDOF"]).translation)
		self.trajectory:row(f):set(1,0)
		self.trajectoryOri:row(f):setQuater(0, MotionDOF.rootTransformation(mOsim.records[f+1]["bfkPoseDOF"]).rotation:rotationY())
	end
	print("filtering",s,e)
	--math.filter(self.trajectory:range(s,e,0, 3), 63)
	--math.filter(self.trajectoryOri:range(s,e,0, 4), 63)
	math.filter(self.trajectory:range(s,e,0, 3), 63)
	math.filter(self.trajectoryOri:range(s,e,0, 4), 121)
	local curPos=self.trajectory:row(self.currFrame):toVector3(0)*100
	--self.cameraInfo.vpos=RE.viewpoint().vpos-curPos
	--self.cameraInfo.vat=RE.viewpoint().vat-curPos
	--self.cameraInfo.dist=RE.viewpoint().vpos:distance(curPos)
	self.cameraInfo.vpos=mCameraInfo.vpos-curPos
	self.cameraInfo.vat=mCameraInfo.vat-curPos
	self.cameraInfo.dist=RE.viewpoint().vpos:distance(curPos)
	self.cameraInfo.refRot=self.trajectoryOri:row(self.currFrame):toQuater(0):rotationY()
end

g_rec_energy = {}
function EVR:onFrameChanged(win, iframe)
	if iframe > g_prev_iframe then
		for f=1,iframe-g_prev_iframe do
			local ifr = g_prev_iframe + f
			print()
			print('simulate frame ', ifr)

			-- simulation
			if dbg_testmw then -- search for dbg_testmw in locoSimMuscle.lua 
				-- debug testmw
				local g_nf=mSynthesis.numFrames
				if preciseComparison and g_nf>=preciseComparison[1] and g_nf<preciseComparison[2] then
					g_debugOneStep=array:new()
					g_debugOneStepSaveFlag=true
				else
					g_debugOneStep=nil
				end
				mSynthesis:oneStep()
				if g_debugOneStep then
					util.saveTable(g_debugOneStep, "debugstates_comp/debugStates_oneStep1-"..g_nf..".tbl" )
					g_debugOneStepSaveFlag=false
				end
				mSynthesis:saveStates("debugstates_comp/debugStates1__"..g_nf..".tbl")
				if preciseComparisonSimul and g_nf>=preciseComparisonSimul[1] and g_nf<preciseComparisonSimul[2] then
					g_debugOneStep=array:new()
					g_debugOneStepFlag =true -- so that only the result of the first simulation frame is stored.
				else
					g_debugOneStep=nil
				end
				if g_nf==0 then
					saveDebugInfo(mSynthesis.simulator, "debugstates_comp/debugInfo_oneStep1.tbl")
				end
				mSynthesis:oneStepSimul()
				if g_nf==0 then
					saveDebugInfo(mSynthesis.simulator, "debugstates_comp/debugInfo_oneStepSimul1.tbl")
				end
				if g_debugOneStep then
					util.saveTable(g_debugOneStep, "debugstates_comp/debugStates_oneStepSimul1_"..g_nf..".tbl" )
				end
				mSynthesis:saveStates("debugstates_comp/debugStates1_"..g_nf..".tbl")
			else
				mSynthesis:oneStep()
				mSynthesis:oneStepSimul()
			end
			if error_feedback_method~=EFM.NONE then	 
				mSynthesis:prepareNextStep()      
			end

			------------------
			--energy print
			--1 energyRate = 1 J/s = (1/4.184) cal/s
			local energyRate = mOsim:getMetabolicEnergyRate()


			local kcalRate = energyRate*(1/4.184)*.001
			local met = (kcalRate*3600)/75
			g_kcal = g_kcal + kcalRate*(1/120.)
			g_int_met = g_int_met + met

			----print('energy rate', energyRate)
			----print('kcal rate', kcalRate)
			--print('kcal', g_kcal)
			----print('met', met)
			--print('avg met', g_int_met/ifr)

			local j = mOsim.mLoader:getBoneByName('ground_pelvis')
			local gp = mOsim.bfk:globalFrame(j):toGlobalPos(vector3(0,0,0))
			gp:setY(0)
			--print('pelvis pos', gp)
			--print('move dist', gp:length())
			--print('int_met/movedist', g_int_met/gp:length())
			--print('kcal/movedist', g_kcal/gp:length())
			--print('move speed (m/s)', gp:length()/(ifr/120.))
			--print('move speed (km/h)', (gp:length()/(ifr/120.))*(1/1000.)*3600)

			g_rec_energy[ifr] = {}
			g_rec_energy[ifr].avgmet = g_int_met/ifr
			g_rec_energy[ifr].kcal = g_kcal
			g_rec_energy[ifr].kcaldist = g_kcal/gp:length()
			g_rec_energy[ifr].mps = gp:length()/(ifr/120.)
			g_rec_energy[ifr].kmph = (gp:length()/(ifr/120.))*(1/1000.)*3600
			------------------

			mOsim:drawMuscles(ifr, mObjectList)
			--mOsim:drawMuscles2(ifr, mObjectList)
			--mOsim:drawJoints(iframe, mObjectList)
			--mOsim:drawMuscleWithForces(ifr, mObjectList)
			
			if boolean_options.attachCamera then
			   local curPos= mSynthesis.pendulum:calcCOMpos()*100
			   curPos.y=0
			   recCurPos[ifr] = curPos
			   RE.viewpoint().vpos:assign(mCameraInfo.vpos+curPos)
			   RE.viewpoint().vat:assign(mCameraInfo.vat+curPos)
			   RE.viewpoint():update()     
			end
		end	
		g_prev_iframe = iframe
	else
		if boolean_options.attachCamera then
			local curPos = recCurPos[iframe]
			if curPos~=nil then
				RE.viewpoint().vpos:assign(mCameraInfo.vpos+curPos)
				RE.viewpoint().vat:assign(mCameraInfo.vat+curPos)
				RE.viewpoint():update()     
			end
		end
		if self.trajectory then
			local ifr = iframe
			if ifr<self.trajectory:rows() then
				local curPos=self.trajectory:row(ifr):toVector3(0)*100
				local currRot=self.trajectoryOri:row(ifr):toQuater(0):rotationY()

				local tf=transf()
				tf:identity()
				tf:leftMultTranslation(curPos*-1)
				local qd=quater()
				qd:difference(self.cameraInfo.refRot, currRot)
				tf:leftMultRotation(qd)
				tf:leftMultTranslation(curPos)

				RE.viewpoint().vpos:assign(tf*(self.cameraInfo.vpos+curPos))
				RE.viewpoint().vat:assign(tf*(self.cameraInfo.vat+curPos))
				RE.viewpoint():update()     
			end
		end
		
		mOsim:drawRecordedScene(iframe, mObjectList)
	end

	if g_rec_energy[iframe]~=nil then
		local avgmet = g_rec_energy[iframe].avgmet
		local cal = g_rec_energy[iframe].kcal*1000.
		local calpm = g_rec_energy[iframe].kcaldist*1000.
		local mps = g_rec_energy[iframe].mps
		local kmph = g_rec_energy[iframe].kmph

		titlebar:setCaption(string.format("avg.met    %.1f\nenergy     %.1f cal\nenergy/dist %.1f cal/m\nspeed      %.1f km/h",
							avgmet, cal, calpm, kmph))
	end
end
