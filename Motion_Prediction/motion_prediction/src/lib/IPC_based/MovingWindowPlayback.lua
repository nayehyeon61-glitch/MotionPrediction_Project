
require("config")
useGraph="Loco"
cleanVisualize=true

require("RigidBodyWin/common")
require("RigidBodyWin/IPC_based/useCases")
require("RigidBodyWin/subRoutines/Optimizer")
--require("RigidBodyWin/subRoutines/MovingWindowOptimizerSubroutines2")
--require("RigidBodyWin/subRoutines/MovingWindowOptimizerSubroutines3")
require("RigidBodyWin/IPC_based/MovingWindowOptimizerSubroutines4")



screenshot=false
captureUniqueId=0




RE.viewpoint():setFOVy(45.000002)
RE.viewpoint().vpos:assign({-31.603369, 177.652362, 622.504089})
RE.viewpoint().vat:assign({-39.439721, 177.466525, -14.588023})
RE.viewpoint():update()


-- following values are automatically connected to UI.
boolean_options={}
boolean_options.attachCamera=true
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


function ctor()

   
   gTracker=Tracker:new()
   mrd_info.outputMotion=MotionDOFcontainer(gTracker.synthesis.skel.dofInfo)

   gTracker.skin=nil
   
   collectgarbage("collect")
   collectgarbage("collect")
   collectgarbage("collect")


   this:updateLayout()
   
   mCameraInfo={}
   local curPos=vector3(0,0,0)
   mCameraInfo.vpos=RE.viewpoint().vpos-curPos
   mCameraInfo.vat=RE.viewpoint().vat-curPos

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
   if noFrameMove then return end
   if gTracker then


	   if stage then
	   
		   gTracker:playback(stage, 0, true, false)

		   gTracker=nil
	   else
		   local numStage, numSubStage
		   if gTracker.calcNumOptimizedStages then
			   numStage, numSubStage=gTracker:calcNumOptimizedStages()
		   else
			   numStage, numSubStage=gTracker.numStage, gTracker.numSubStage
		   end	    


		   -- override automatic settings (commandline parameters)
		   if g_numStage then numStage=g_numStage end -- when there exists external input (lua short.lua mwp g_numStage=1;g_numSubStage=1)
		   if g_numSubStage then numSubStage=g_numSubStage end

		   if g_accumulate==nil then g_accumulate=false end

		   if g_setStage then gTracker:setStage(1,0) end

		   for i=1, numStage-1 do

			   --	 RE.output("playBack", tostring(i))
			   print("playback", i)
			   gTracker:playback(i, numSubStage-1, false, g_accumulate )
		   end
		   gTracker:playback(numStage, numSubStage-1, true, g_accumulate )

		   gTracker=nil
	   end
   end
end

