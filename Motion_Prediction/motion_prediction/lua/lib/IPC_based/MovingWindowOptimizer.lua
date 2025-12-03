
require("config")
useGraph="Loco"

package.projectPath='../Samples/QP_controller/'
package.path=package.path..";../Samples/QP_controller/lua/?.lua"
package.path=package.path..";../Samples/classification/lua/?.lua"


require("IPC_based/common")
require("IPC_based/useCases")
require("subRoutines/Optimizer")
--require("RigidBodyWin/subRoutines/MovingWindowOptimizerSubroutines")
--require("RigidBodyWin/subRoutines/MovingWindowOptimizerSubroutines2")
--require("RigidBodyWin/subRoutines/MovingWindowOptimizerSubroutines3")
require("IPC_based/MovingWindowOptimizerSubroutines4") -- this is the one
--require("RigidBodyWin/subRoutines/MovingWindowOptimizerSubroutines5")

screenshot=false
captureUniqueId=0



--class 'TrackingOptimizer'(Optimizer)
TrackingOptimizer=LUAclass(Optimizer)

-- actual parameters used for optimization
stepSize=5	-- very important to be set properly. need some intuition based on optimizelog.txt

--method=Optimizer.methods.GradientDecent2
--method=Optimizer.methods.ConjugateGradient
--method=Optimizer.methods.NRgradientDescent
--method=Optimizer.methods.NRconjugateGradient
--method=Optimizer.methods.CMAes
--method.lambda=2
method=Optimizer.methods.Test
method.testCount=4 -- 1<=testCount<=3 NOTE!!! when testCount==1 trackOpt:save2 won't be called
--method=Optimizer.methods.RandomizedSearch
--method=Optimizer.methods.GSLGradientDescent
--method=Optimizer.methods.GSLBFGS
--method=Optimizer.methods.GA
--method=Optimizer.methods.FullSearch
--method.numGrid:assign({3,3,3,3,3})
--method.gridSize:assign({0.1,0.1,5,5,5})



function TrackingOptimizer:__init()
	--   self.objectList=Ogre.ObjectList()
	Optimizer.__init(self, stepSize, opt_dimension, method)
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


states={init=1, optimize=2, stop=3, finish=4}
function ctor()

	startStage=1
	startSubStage=0

	useCaseInitial=useCase
	gTracker=Tracker:new()

	util.outputToFile(Optimizer.outFile,os.date().." ----------------------------------")
	util.outputToFile(Optimizer.outFile,os.date().." -------- started           -------")
	util.outputToFile(Optimizer.outFile,os.date().." ----------------------------------")
	fineLog(" -----------------started--------------------- ")

	do
		local stageParam=gTracker.stageParam[startStage]
		util.outputToFile(Optimizer.outFile,os.date().." setStage"..startStage.." startSeg:"..stageParam.startSeg.." endSeg:"..stageParam.endSeg)
		gTracker:setStage(startStage, startSubStage)
	end

	trackOpt	= TrackingOptimizer()
	trackOpt.maxIteration=200

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

		do
			state=states.optimize
			
			util.outputToFile(Optimizer.outFile,os.date().." optimize")
			trackOpt:optimize()
			if false then
				this('exit',1)
				return
			end
			local fn
			assert(rank==nil)
			if rank==nil then
				fn="optresult"..gTracker.stage.."_0.lua"
			end
			if useCaseInitial.updateStageParam then
				useCases.saveOptResult(useCaseInitial, trackOpt
				, gTracker.stageParam[gTracker.stage].afterOpt, fn)
				useCaseInitial:updateStageParam(gTracker.stage+1, gTracker.stageParam, 
				trackOpt.best_eval, trackOpt.best_eval_unscaled)
			else
				--if not ( method==Optimizer.methods.Test and method.testCount==1) then
					trackOpt:save2(fn)
				--end
			end
			util.outputToFile(Optimizer.outFile,os.date().." prepareNextStage"..gTracker.stage.."_"..gTracker.subStage)
			gTracker:prepareNextStage() -- accumulateResult and saveStates

			local tbl=gTracker:incr({gTracker.stage, gTracker.subStage})
			local stageParam=gTracker.stageParam[tbl[1]]
			util.outputToFile(Optimizer.outFile,os.date().." setStage"..tbl[1].." startSeg:"..stageParam.startSeg.." endSeg:"..stageParam.endSeg.. ' '..(stageParam.info or ':'))

			if tbl[1]>#gTracker.stageParam then
				state=states.finish
				return
			end

			gTracker:setStage(tbl[1], tbl[2])

			state=states.stop
		end
	elseif state==states.stop then
		state=states.init
	elseif state==states.finish then
		this("exit", 1)
	end
end

