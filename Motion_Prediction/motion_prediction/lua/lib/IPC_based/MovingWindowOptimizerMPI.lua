require("config")
useGraph="Loco"

package.projectPath='../Samples/QP_controller/'
package.path=package.path..";../Samples/QP_controller/lua/?.lua"
package.path=package.path..";../Samples/classification/lua/?.lua"


require("IPC_based/common")
require("IPC_based/useCases")
require("subRoutines/OptimizerMPI")
--require("RigidBodyWin/subRoutines/MovingWindowOptimizerSubroutines")
--require("RigidBodyWin/subRoutines/MovingWindowOptimizerSubroutines2")
--require("RigidBodyWin/subRoutines/MovingWindowOptimizerSubroutines3")
require("IPC_based/MovingWindowOptimizerSubroutines4") -- this is the one
--require("RigidBodyWin/subRoutines/MovingWindowOptimizerSubroutines5")

screenshot=false
captureUniqueId=0

--class 'TrackingOptimizer'(OptimizerMPI)
TrackingOptimizer=LUAclass(OptimizerMPI)

-- actual parameters used for optimization
stepSize=5	-- very important to be set properly. need some intuition based on optimizelog.txt

--method=Optimizer.methods.GradientDecent2
--method=Optimizer.methods.NRgradientDescent
--method=Optimizer.methods.NRconjugateGradient
--method=Optimizer.methods.ConjugateGradient
--method=Optimizer.methods.Test
method=Optimizer.methods.CMAes
--method=Optimizer.methods.RandomizedSearch
--method=Optimizer.methods.GSLGradientDescent
--method=Optimizer.methods.GSLBFGS
--method=Optimizer.methods.GA
--method=Optimizer.methods.FullSearch
--method.numGrid:assign({3,3,3,3,3})
--method.gridSize:assign({0.1,0.1,5,5,5})



function TrackingOptimizer:__init()
	--   self.objectList=Ogre.ObjectList()
	OptimizerMPI.__init(self, stepSize, opt_dimension, method)
end

function TrackingOptimizer:objectiveFunction(pos)


	local screenshotPrefix="../dump/optimize_"..captureUniqueId
	if screenshot==true then
		captureUniqueId=captureUniqueId+1

		RE.renderer():setScreenshotPrefix(screenshotPrefix)
		RE.renderer():screenshot(true)
		--print("start")
	end

	local tracker=gTracker
	local renderFreq=12

	if screenshot==false then
		renderFreq=4
	end

	obj=tracker:objectiveFunction(pos, renderFreq)

	if screenshot==true then
		RE.renderer():screenshot(false)
		FastCapture_convert(screenshotPrefix..".dat")
		os.remove(screenshotPrefix..".dat")
	end

	return obj
end









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
boolean_options.drawMatching=false
float_options={}
float_options.impulseMagnitude={ val=100, min=10, max=1000}
float_options.impulseDuration={ val=20, min=1, max=100}


states={init=1, optimize=2, stop=3}
function ctor()
	numCores=MPI.size()
	rank=MPI.rank()

	startStage=1
	startSubStage=0

	useCaseInitial=useCase
	gTracker=Tracker:new()

	if startState~=0 or startSubStage~=0 then
		gTracker:setStage(startStage, startSubStage)
	end

	trackOpt	= TrackingOptimizer()
	trackOpt.maxIteration=200


	trackOpt:sync({"constructor"})


	--   mSynthesis:detectMemoryLeak()
	state=states.init


	--this:create("Check_Button"	, "orientation correction", "orientation correction",0);
	--this:widget(0):checkButtonValue(true)
	this:updateLayout()

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

	if noFrameMove then return end
	--	debug.debug()
	if state==states.init then


		if trackOpt.server then
			state=states.optimize

			util.outputToFile(Optimizer.outFile,os.date().." optimize")
			trackOpt:optimize()

			local fn
			if rank==nil then
				fn="optresult"..gTracker.stage.."_0.lua"
			else
				assert(rank==0)
				fn="optresult"..gTracker.stage.."_rank"..rank..".lua"
			end

			if useCaseInitial.updateStageParam then
				useCases.saveOptResult(useCaseInitial, trackOpt.server
				, gTracker.stageParam[gTracker.stage].afterOpt, fn)
				useCaseInitial:updateStageParam(gTracker.stage+1, gTracker.stageParam, 
				trackOpt.server.best_eval, trackOpt.server.best_eval_unscaled)
			else
				useCases.saveOptResult(useCaseInitial, trackOpt.server
				, gTracker.stageParam[gTracker.stage].afterOpt, fn)
			end
			util.outputToFile(Optimizer.outFile,os.date().." prepareNextStage"..gTracker.stage.."_"..gTracker.subStage)
			do
				local i=gTracker.stage
				cp_mod=nil
				assert(rank==0)
				dofile("optresult"..i.."_rank0.lua")
				assert(cp_mod~=nil)
				local baseStage=gTracker.stageParam[i+1].baseStage
				local baseStartSeg
				if baseStage==0 then
					baseStartSeg=1
				else
					baseStartSeg= gTracker.stageParam[baseStage].startSeg
				end
				local nextStartSeg= gTracker.stageParam[i+1].startSeg
				trackOpt:sync({"prepareNextStage", cp_mod, baseStage, baseStartSeg, nextStartSeg}) -- ask all procs to call prepareNextStage
			end
			gTracker:prepareNextStage()

			local tbl=gTracker:incr({gTracker.stage, gTracker.subStage})
			local stageParam=gTracker.stageParam[tbl[1]]
			util.outputToFile(Optimizer.outFile,os.date().." setStage"..tbl[1].." startSeg:"..stageParam.startSeg.." endSeg:"..stageParam.endSeg.. ' '..(stageParam.info or ':'))


			if tbl[1]>#gTracker.stageParam then
				state=states.finish
				trackOpt:sync({"finish"})
				return
			else
				trackOpt:sync({"setStage",tbl[1], gTracker.stageParam[tbl[1]]})
			end

			gTracker:setStage(tbl[1], tbl[2])

			state=states.stop
		else
			trackOpt:loop()
		end
	elseif state==states.stop then
		state=states.init
	elseif state==states.finish then
		this("exit", 1)
	end
end

function TrackingOptimizer:loop()
	if self.client then
		local work=MPI.recvT(0)

		fineLog(jobId, work[1])

		if work[1]=="doThis" then

			local jobId=work[2]
			local pos=CT.vec(unpack(work[3]))


			fineLog(jobId, work[2], pos)

			local obj=self.client:objectiveFunction(pos)


			MPI.sendT({"result", jobId, obj},0)

		elseif work[1]=="sync" then

			tbl= work[2]
			if tbl[1]=="setStage" then



				--[[if tbl[2]==gTracker.numStage or 
					tbl[3]==gTracker.numSubStage then
					MPI.sendT({"sync_received"},0)
					this("exit", 1)
					return
				end
				]]--

				gTracker:_setStage(tbl[2], tbl[3])

				MPI.sendT({"sync_received"},0)
			elseif tbl[1]=="finish" then
				MPI.sendT({"sync_received"},0)
				this("exit", 1)
				return

			elseif tbl[1]=="prepareNextStage" then
				MPI.sendT({"sync_received"},0)
				local outputString=table.tostring(tbl[2])
				local fn="optresult"..gTracker.stage.."_rank"..rank..".lua"
				local fout,msg=io.open(fn, "w")
				if fout==nil then
					util.outputToFile(Optimizer.outFile, msg)
				end
				fout:write(' cp_mod='..outputString..'\n')
				fout:close()

				local baseStage=tbl[3]
				local baseStartSeg=tbl[4]
				local nextStartSeg=tbl[5]
				gTracker:_prepareNextStage(baseStage, baseStartSeg, nextStartSeg)

			elseif tbl[1]=='constructor' then
				MPI.sendT({"sync_received"},0)
			else
				MPI.sendT({"sync_received"},0)
			end
		else
			MPI.sendT(work,0)
		end
	end
end
