require("config")

package.projectPath='../Samples/QP_controller/'
package.path=package.path..";../Samples/QP_controller/lua/?.lua" --;"..package.path
package.path=package.path..";../Samples/classification/lua/?.lua" --;"..package.path

g_config={ path='IPC_based'}

useGraph="Loco"
--useGraphLin=true

require("IPC_based/common")
require("IPC_based/useCases")
-- testing parameters
 preciseComparison={0,5}
 preciseComparisonSimul={0,5}
-- preciseComparison={-1,-1}
-- preciseComparisonSimul={-1,-1}
function setParam()
	g_stageParam={}

	useCase.updateStageParam(useCase, 1, g_stageParam, 0.99, 1e-10)
	useCase.updateStageParam(useCase, 2, g_stageParam, 0.99, 1e-10)
	--useCase.updateStageParam(useCase, 3, g_stageParam, 0.99, 1e-10)
	--
	g_stageParam[1].startSeg=1
	g_stageParam[2].startSeg=1
	--g_stageParam[1].endSeg=2
	--g_stageParam[2].endSeg=2
end
lastStage=2
n_repetition=1
test_restoreStates=false -- test_restoreStates cannot pass unless full 3x3 matrix is stored instead of a quaternion for all spherical joint states.

require("subRoutines/Optimizer")
require("IPC_based/MovingWindowOptimizerSubroutines4")

screenshot=false
captureUniqueId=0








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
boolean_options.drawControlForce=true
boolean_options.drawMatching=true
float_options={}
float_options.impulseMagnitude={ val=100, min=10, max=1000}
float_options.impulseDuration={ val=20, min=1, max=100}


function Tracker:accumulateResult(tbl) -- not needed here.
end

require('IPC_based/testmw_save')

fineDebug=true
if fineDebug then
	function Tracker:__prepareNextStage()

		saveDebugInfo(self.synthesis.simulator, "debugStatesPN_restoreState"..g_i..".tbl")
		-- proceed first two frames seperately. This is to avoid immediate trigger.

		local errorOccurred=false

		local function OneStep()
			local nf=self.synthesis.numFrames

			local g_nf=nf+g_startG

			if preciseComparison and g_nf>=preciseComparison[1] and g_nf<preciseComparison[2] then
				g_debugOneStep=array:new()
			else
				g_debugOneStep=nil
			end

			self.synthesis:oneStep()

			if g_debugOneStep then
				util.saveTable(g_debugOneStep, "debugStatesPN_oneStep"..g_i..g_nf..".tbl" )
				g_debugOneStep=nil
			end
			if self.synthesis.errorOccurred==true then
				fineLog( "error0")
				return false
			end

			self.synthesis:saveStates("debugStatesPN"..g_i.."__"..g_nf..".tbl")

			if preciseComparisonSimul and g_nf>=preciseComparisonSimul[1] and g_nf<preciseComparisonSimul[2] then
				g_debugOneStep=array:new()
				g_debugOneStepFlag =true -- so that only the result of the first simulation frame is stored.
			else
				g_debugOneStep=nil
			end

			if g_nf==0 then
				saveDebugInfo(self.synthesis.simulator, "debugInfoPN_oneStep"..g_i..".tbl")
			end

			self.synthesis:oneStepSimul()

			if g_nf==0 then
				saveDebugInfo(self.synthesis.simulator, "debugInfoPN_oneStepSimul"..g_i..".tbl")
			end

			if g_debugOneStep then
				util.saveTable(g_debugOneStep, "debugStatesPN_oneStepSimul"..g_i..g_nf..".tbl" )
			end

			self.synthesis:saveStates("debugStatesPN"..g_i.."_"..g_nf..".tbl")
			if self.synthesis.errorOccurred==true then
				fineLog( "error1")
				return false
			end

			if error_feedback_method==EFM.NONE then
				fineLog( "error2")
				return false
			end

			if error_feedback_method~=EFM.NONE then	 
				self.synthesis:prepareNextStep()      
			end
			self.synthesis:saveStates("debugStatesPN"..g_i.."___"..g_nf..".tbl")


			if self.synthesis.errorOccurred==true then
				fineLog( "error3")
				return false
			end
			return true
		end

		do
			self.synthesis.objectiveFunctionEnd=false

			local i=0
			local prevFrac=-1
			while self.synthesis.objectiveFunctionEnd==false do

				if not OneStep() then 
					errorOccurred=true
					break
				end


				i=i+1

		local event=self:updateCurrSeg(i, self.synthesis, self.nextStartSeg)
		if event then
			if event=="normal ending" then
				local stt=self:incr({self.stage, self.subStage})
				assert(stt[1]~=self.stage)
				self:saveStates(stt)
				--		  RE.output("prepareNextStage", "done"..seq.curSeg)
			end
			break
		end

				if math.mod(i,renderFreq)==0 then
					--	    if true then
					self.synthesis.simulator:drawDebugInformation()
					renderOneFrame()
				end

			end
		end
		assert(errorOccurred==false)
		collectgarbage("collect")
		collectgarbage("collect")
		collectgarbage("collect")

	end
	function Tracker:objectiveFunction(pos, renderFreq)

		local objectiveFunctionTypes={POSE=1, SEGMENT=2, POSE2=3}
		local objectiveFunctionType=objectiveFunctionTypes.SEGMENT
		local markerFile
		do
			local fn, path=os.processFileName(string.sub(model.file_name ,1, -5))
			markerFile=path.."/"..fn.."_sd/optComparision_marker.lua"
		end

		mrd_info.outputMotion=MotionDOFcontainer(self.synthesis.skel_withoutCOMjoint.dofInfo)
		self.compareChain=CompareChain2D(self.synthesis.skel_withoutCOMjoint,
		--self.compareChain=CompareChain2Dori(self.synthesis.skel_withoutCOMjoint,
		self.synthesis.simulator:getWorldState(0),
		self.synthesis.skel_withoutCOMjoint:fkSolver(),markerFile)

		local origDMot=MotionDOF(self.synthesis.skel_withoutCOMjoint.dofInfo) -- original motion that is temporally aligned to the synthesized motion
		local origMot=MotionDOF(self.synthesis.skel_withoutCOMjoint.dofInfo) -- original motion that is temporally aligned to the synthesized motion


		local MSE=0
		local MAX_E=0
		local nMSE=0
		local kenergy=0
		local errorOccurred=false
		local bCont=false
		local skin=self.skin
		do

			RE.output("pos", tostring(pos))
			local graph=self.synthesis.graph

			local seq=self.sequence
			seq.curSeg=self.startSeg

		--print("objectiveFunction")
			fineLog("objectiveFunction", self.stage, self.subStage, self.startSeg, self.endSeg, pos)

			local bRestoreStateSkipped=false

			if g_bRestoreStates then
				self:restoreStates(self.stage, self.subStage)
				self.synthesis:saveStates("debugStates_arbc1"..g_i..".tbl")

				--self.synthesis:changeControlParameters(useCase.controlParam)

				for i=0, pos:size()-1 do
					local title=opt_dimension[i+1].title
			--coarseLog('chcp', title, pos(i))
			-- Temporarily change useCase.controlParam.
			-- so that the changed values are used in updateConstraints
					self.synthesis.graph:changeControlParameterInUseCase(title, pos(i))
				end
			end

			self.synthesis:saveStates("debugStates_arbc2"..g_i..".tbl")
			self.synthesis:changeControlParameters(useCase.controlParam)
			useCases.unmapControlParam(useCase)

			self:updateConstraints()

			saveDebugInfo(self.synthesis.simulator, "debugInfo_restoreState"..g_i..".tbl")
			self.synthesis:saveStates("debugStates_ar"..g_i..".tbl")

			-- proceed first two frames seperately. This is to avoid immediate trigger.

			local function OneStep()

				local nf=self.synthesis.numFrames

				g_nf=nf+g_startG

				if preciseComparison and g_nf>=preciseComparison[1] and g_nf<preciseComparison[2] then
					g_debugOneStep=array:new()
					g_debugOneStepSaveFlag=true
				else
					g_debugOneStep=nil
				end
				self.synthesis:oneStep()

				if g_debugOneStep then
					util.saveTable(g_debugOneStep, "debugStates_oneStep"..g_i.."-"..g_nf..".tbl" )
					g_debugOneStepSaveFlag=false
				end
				--this('exit!',0)

				assert(self.synthesis.numFrames==nf+1)
				if self.synthesis.errorOccurred==true then
					fineLog( "error0")
					return false
				end

				self.synthesis:saveStates("debugStates"..g_i.."__"..g_nf..".tbl")


				if preciseComparisonSimul and g_nf>=preciseComparisonSimul[1] and g_nf<preciseComparisonSimul[2] then
					g_debugOneStep=array:new()
					g_debugOneStepFlag =true -- so that only the result of the first simulation frame is stored.
				else
					g_debugOneStep=nil
				end

				if g_nf==0 then
					saveDebugInfo(self.synthesis.simulator, "debugInfo_oneStep"..g_i..".tbl")
				end

				self.synthesis:oneStepSimul()
				kenergy=kenergy+self.synthesis.simulator:calcKineticEnergy()

				if g_nf==0 then
					saveDebugInfo(self.synthesis.simulator, "debugInfo_oneStepSimul"..g_i..".tbl")
				end

				if g_debugOneStep then
					util.saveTable(g_debugOneStep, "debugStates_oneStepSimul"..g_i.."_"..g_nf..".tbl" )
				end


				self.synthesis:saveStates("debugStates"..g_i.."_"..g_nf..".tbl")
				if self.synthesis.errorOccurred==true then fineLog( "error1") return false end

				if error_feedback_method==EFM.NONE then fineLog( "error2") return false end

				if error_feedback_method~=EFM.NONE then	 
					self.synthesis:prepareNextStep()      
				end

				self.synthesis:saveStates("debugStates"..g_i.."___"..g_nf..".tbl")

				if self.synthesis.errorOccurred==true then fineLog( "error3") return false end
				return true
			end

			do
				self.synthesis.objectiveFunctionEnd=false

				local i=0
				local prevFrac=-1
				while self.synthesis.objectiveFunctionEnd==false do

					local pcall_ok, errMsg=pcall(OneStep)

					if g_debugOneStep and g_debugOneStepSaveFlag then
						util.saveTable(g_debugOneStep, "debugStates_oneStep"..g_i..g_nf..".tbl" )
						g_debugOneStep=nil
						g_debugOneStepSaveFlag=nil
					end

					if pcall_ok then 
						if errMsg~=true then
							errorOccurred=true
							break
						end
					else
					fineLog("objFn:pcall_error", errMsg)
						errorOccurred=true
						break
					end


					i=i+1

					local event=self:updateCurrSeg(i, self.synthesis, self.endSeg)

					if event then
						if event=="error" then errorOccurred=true end
						break
					end

					do

						if objectiveFunctionType==objectiveFunctionTypes.POSE then
							local info2
							do -- calc info without PDservoLatency 
								currFrame=self.synthesis.numFrames-1
								local seg, refTime, f, l, frac1, weight=self.synthesis:calcWeights(math.max(currFrame,0))
								info2={currSeg=seg, frac=frac1}
							end

							local MSEp=self:comparePose(skin,info2)
							if boolean_options.drawMatching then
							    self.compareChain:drawMatching(self.objectList)
							end
							RE.output("MSE", tostring(MSEp))
							MSE=MSE+MSEp
							MAX_E=math.max(MAX_E, MSEp)
							nMSE=nMSE+1
						else

							local skel=self.synthesis.skel_withoutCOMjoint
							local nf=origDMot:numFrames()+1
							origDMot:resize(nf)
							local dpose=origDMot:row(nf-1)
							if useGraph=="Loco" then

								local deltaSeq=self.synthesis.deltaSeq
								if deltaSeq==nil then deltaSeq=0 end

							if self.refTrajectory then
								local refFrame=(self.synthesis.outputGlobal.refTime(self.synthesis.numFrames-1)+deltaSeq-1)*self.refTrajectory.nsample
								RE.output2("refFrame", refFrame)

								self.refTrajectory.mot:sampleRow(refFrame, dpose)
							end
							else
								self.synthesis.outputLocal.synLMot:sampleRow(self.synthesis.outputGlobal.refTime(self.synthesis.numFrames-1), dpose)
							end
							origMot:resize(nf)
							origMot:row(nf-1):assign(origDMot:row(nf-1))

							local startTransf=vector3(0,0,0)
							if nf>2 then
								startTransf=origMot:rootTransformation(nf-2):encode2D()
								origDMot:range(nf-2,nf):reconstructData(startTransf, origMot:range(nf-2, nf))
							else
								origDMot:range(nf-1,nf):reconstructData(startTransf, origMot:range(nf-1, nf))
							end		     


							local pose=origMot:row(nf-1)

							--local pose=info.currSeg.refSeg:pose(info.frac)

							g_cdbg={}
							local cdbg=g_cdbg


							skin:setPoseDOF(pose)
							skel:setPoseDOF(pose)


							array.pushBack(cdbg, {"pose", pose:copy()})
							local info=self.synthesis.trackingInfo -- targetPose foregoes currentPose by PDservoLatency
							if info.frac<prevFrac  then
								if bCont then
									bCont=false
								else
									--local compareScore,nframes=self.compareChain:compareQueue(self.objectList, seq[seq.curSeg-2]) --per stride
									local compareScore,nframes=self.compareChain:compareQueue(self.objectList, seq[seq.curSeg-1], self.objectCost) --per segment
									MSE=MSE+compareScore
									nMSE=nMSE+nframes
									print('objFcnDbg', g_nf,MSE,nMSE, compareScore, nframes)
									array.pushBack(cdbg, {"objFcnDbg", MSE, nMSE, compareScore, nframes})
									RE.output("compareScore", tostring(compareScore))
									bCont=false
								end
							end
							self.compareChain:addQueue(mrd_info.outputMotion, self.synthesis)

							array.pushBack(cdbg, {"pose", info.frac, pose:copy(), mrd_info.outputMotion:row(mrd_info.outputMotion:numFrames()-1):copy()})
							util.saveTable(cdbg, "debugStates_compareChain"..g_i.."_"..g_nf..".tbl" )
							g_cdbg=nil
							prevFrac=info.frac
						end
					end
					if math.mod(i,renderFreq)==0 then
						self.synthesis.simulator:drawDebugInformation()
						renderOneFrame()
					end

					if debugMode and i==20 then break end
				end


				if objectiveFunctionType~=objectiveFunctionTypes.POSE and not errorOccurred then
					--local sc,nframes=self.compareChain:compareQueue(self.objectList, seq[seq.curSeg-1], self.objectCost, true) -- per stride
					local sc,nframes=self.compareChain:compareQueue(self.objectList, seq[seq.curSeg-1], self.objectCost, true) -- per segment
					print('objFcnDbg', MSE,nMSE, sc, nframes)
					MSE=MSE+sc
					nMSE=nMSE+nframes
					RE.output("compareScore", tostring(sc))
				end


			end
			collectgarbage("collect")
			collectgarbage("collect")
			collectgarbage("collect")

		end
		if errorOccurred then

			if bRestoreStateSkipped then
				self:restoreStates(self.stage, self.subStage) -- so that skipping restorestate at next step should not have problem
			end

			local seq=self.sequence
		local objFn=100000+(self.endSeg-(seq.curSeg+self.synthesis.trackingInfo.frac))*10000+MSE/nMSE
		if nMSE==0 then
			objFn=100000+(self.endSeg-(seq.curSeg+self.synthesis.trackingInfo.frac))*100000+100000
		end
		fineLog("objFn"..tostring(objFn))
		return objFn
		end

	fineLog( MSE/nMSE)--+kenergy*1e-11)

		self.compareChain=nil

	local boundEnergy=0


	--print(MSE/nMSE,kenergy*1e-11)
	--print(MSE/nMSE,boundEnergy)
	return MSE/nMSE+boundEnergy--+kenergy*1e-11
	end


end

function ctor()

	os.execute('lua clean all')

	g_debugOneStep=array:new()
	gTracker=Tracker:new()
	util.saveTable(g_debugOneStep, "debugStates_init.tbl" )
	g_debugOneStep=nil
	saveDebugInfo(gTracker.synthesis.simulator, "debugInfo_nocomp1.tbl")
	--if true then this('exit',1) return end


--	if useCase.genStageParamInitial then
--		useCase.stageParamInitial=useCase:genStageParamInitial()
--	end
	setParam()
	gTracker.stageParam= g_stageParam
	gTracker.numStage=table.getn(gTracker.stageParam)

	g_startG=0
	g_i=0
	gTracker:setStage(1,0)

	local obj_r={}
	for i=1,1+n_repetition do

		if n_repetition>0 and i==1 and test_restoreStates then
			g_bRestoreStates=false
			local func=Tracker[gTracker.stageParam[i].setStageFunc]
			local param=gTracker.stageParam[i].param
			func(gTracker, 1, 0, param)
		else
			g_bRestoreStates = true
		end

		local pos=vectorn()
		local N_opt_dimension=table.getn(opt_dimension)
		pos:resize(N_opt_dimension)

		for i=1, N_opt_dimension do
			local optvar=opt_dimension[i]
			set1(pos, i, optvar.curval)
		end

		g_i=i

		obj_r[i]=gTracker:objectiveFunction(pos, 4)
		if gTracker.synthesis.numFrames~=gTracker.synthesis.outputGlobal.numFrames then
			-- fall down case. just to avoid assertion failure.
			print("fall down")
			local nnn=math.min(gTracker.synthesis.numFrames, gTracker.synthesis.outputGlobal.numFrames)
			gTracker.synthesis.numFrames=nnn
			gTracker.synthesis.outputGlobal.numFrames=nnn
		end
		gTracker.synthesis:saveStates("debugStates"..i..".tbl")
	end
	for i=1,1+n_repetition do
		print(obj_r[i])
	end

	--this("exit!",{})
	tbl1=util.loadTable('debugStates1.tbl')

	if n_repetition>0 then
		tbl2=util.loadTable('debugStates2.tbl')
		util.compareTable(tbl1, tbl2)
	end
	for i_stage=2,lastStage do
		gTracker:prepareNextStage()

		local pn=gTracker.synthesis.numFrames-1

		-- gTracker.synthesis:saveStates("debugStates_pn"..pn..".tbl")
		-- tbl1=util.loadTable("debugStates1".."___"..pn..".tbl")
		-- tbl2=util.loadTable("debugStates_pn"..pn..".tbl")
		-- util.compareTable(tbl1, tbl2)

		gTracker:setStage(i_stage,0)
		g_startG=tbl1.startG
		for i=1,1+n_repetition do
			local pos=vectorn()
			local N_opt_dimension=table.getn(opt_dimension)
			pos:resize(N_opt_dimension)

			for i=1, N_opt_dimension do
				local optvar=opt_dimension[i]
				set1(pos, i, optvar.curval)
			end

			g_i=i+2*(i_stage-1)

			obj_r[i]=gTracker:objectiveFunction(pos, 4)

			gTracker.synthesis:saveStates("debugStates"..g_i..".tbl")
		end
		for i=1,1+n_repetition do
			print(obj_r[i])
		end
	end

	tbl1=util.loadTable('debugStates1.tbl')
	tbl3=util.loadTable('debugStates3.tbl')
	if n_repetition>0 then
		tbl4=util.loadTable('debugStates4.tbl')
		util.compareTable(tbl3, tbl4)
	end



	tbl1=util.loadTable('debugStates1.tbl')
	util.compareTable(tbl1, tbl3)

	this:updateLayout()
	
	this("exit",{})
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
	--	debug.debug()

end

