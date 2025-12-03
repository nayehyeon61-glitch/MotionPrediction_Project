
require("config")
require("module")

package.projectPath='../'
package.path=package.path..";../Samples/QP_controller/lua/?.lua" --;"..package.path
package.path=package.path..";../Samples/classification/lua/?.lua" --;"..package.path

require("IPC_based/common")

package.path=package.path..";../Samples/ysscripts/samples/?.lua" --;"..package.path
--require("IPC_based/useCases")
require("useCasesMuscle")

require("IPC_based/LocoGraph")
require("IPC_based/cartPoleBall")
require("subRoutines/CompareChain")

package.path=package.path..";../Samples/ysscripts/libs/?.lua" --;"..package.path
--require("IPC_based/LocoSynthesis")
require("LocoSynthMuscle")

-- dbg.startTrace3()

function init_globals()
   scenario=useCase.scenario

   if OnlineSynthesis and OnlineSynthesis.init_globals then
      OnlineSynthesis.init_globals()
   end
   model=scenarios.toModel(scenario)
   outputSuperSample=1
   renderDownSample=4

   if useCase.init_globals then
	   useCase.init_globals()
   elseif scenario==scenarios.JUMP then
   elseif scenario==scenarios.STAND then
   elseif scenario>=scenarios.STAND1 and scenario<=scenarios.STAND4 then
   else
   end


   useCartPolClass=true
   useID_C1=false
   renderSuperSample=false
   --PDservoLatency=3*outputSuperSample
   PDservoLatency=0--2*outputSuperSample
   IDservoLatency=0
   PredictionLatency=0

end

-- following values are automatically connected to UI.
boolean_options={} 
boolean_options.drawPDtarget =false
boolean_options.attachCamera=true
boolean_options.drawDisplacementMap=false
boolean_options.useOrientationCorrection=false
boolean_options.solveIK=true
boolean_options.drawPredictedCOM=false
boolean_options.drawPredictedZMP =true
boolean_options.drawPoseBeforeIK=false
boolean_options.drawPelvis=false
boolean_options.drawControlForce=false
boolean_options['show character']=true

float_options={}
float_options.impulseMagnitude={ val=100, min=10, max=1000}
float_options.impulseDuration={ val=5, min=1, max=100}
float_options.errorFeedbackAmt={ val=1, min=0, max=1}
float_options.amt_sideway_vel={ val=useCase.amt_sideway_vel, min=0, max=5}


gTimer=util.PerfTimer2()--util.PerfTimer(1,"timer1")
gTimer2=util.PerfTimer2()--util.PerfTimer(1,"timer2")
gTimer3=util.PerfTimer2()--util.PerfTimer(1,"timer3")

function util.PerfTimer:stopMsg(str)
   -- self:stop()
   -- print(str)
end

function util.PerfTimer2:stopMsg(str)
   -- RE.output(str, tostring(self:stop()))
end

--OnlineLocoSynthesis=LUAclass(OnlineSynthesis2)
OnlineLocoSynthesis=LUAclass(OnlineSynthesisMuscle)

function OnlineLocoSynthesis:__init()
   OnlineSynthesisMuscle.__init(self)

	mrd_info={ filename="debug_plot.mrd", min_frame=0, export_freq=200, outputMotion=true, outputContactForce=true }
	mrd_info.outputContactForce={matrixn(), {vector3(0,0,0), vector3(0,0,0)}}
   self.externalForce=vector3(0,0,0)
   self.impulse=0

   self.outputMotion=MotionDOFcontainer(self.skel_withoutCOMjoint.dofInfo)

--
--   for i,grpName in ipairs(self.graph.groups.tf) do
--      for k,v in self.graph[grpName] do
--	 v:plot()
--      end
--   end
end


function OnlineLocoSynthesis:__finalize()
   self.outputMotion=nil
   self.simulator=nil
   OnlineSynthesisMuscle.__finalize(self)
end

function OnlineLocoSynthesis:oneStep()
   
--[[
   if self.impulse>0 then 
      
      self.pendulum:addExtForce(vector3(float_options.impulseMagnitude.val,0,0))
      self.impulse=self.impulse-1
   end
   
   self.pendulum:addExtForce(self.externalForce)
]]--
   OnlineSynthesisMuscle.oneStep(self)
   
   -- visualize contact state
   
   local refLfoot,refRfoot=self.outputGlobal:predictFootCon(self.numFrames-1)

   RE.output("outputState", 
	     tostring(refLfoot)..","..
	     tostring(refRfoot))
   
   local con=vector3N()
   if refLfoot then
      con:pushBack(self.outputGlobal.footLglobal(self.numFrames-1))
   end

   if refRfoot then
      con:pushBack(self.outputGlobal.footRglobal(self.numFrames-1))
   end

   if con:size()~=0 then
      self.objectList:registerObject("footContact", "QuadList", "redCircle",
				     con:matView()*100,20)
   end
   

   local mrdMotion=self.outputMotion

   if mrdMotion then
      local currFrame=mrdMotion:numFrames()
      mrdMotion:resize(currFrame+1)
	  if boolean_options.drawPDtarget then
		  mrdMotion:row(currFrame):assign(self.samplePose_pdtarget)
	  elseif boolean_options.drawPoseBeforeIK then
		  mrdMotion:row(currFrame):assign(self.samplePoseBeforeIK)
	  else
		  mrdMotion:row(currFrame):assign(self.samplePose)
	  end

      if math.mod(self.numFrames, 50)==0 then
	 mrdMotion:exportMot("debug_plot.dof")
      end	 
   end

end

function ctor()
   --dbg.startTrace()
   
   mSynthesis	= OnlineLocoSynthesis:new()
   this:create("Choice", "type")
   this:widget(0):menuSize(4)
   this:widget(0):menuItem(0, "choose desired type")
   this:widget(0):menuItem(1, "walk", "w")
   this:widget(0):menuItem(2, "run", "r")
   this:widget(0):menuItem(3, "stand", "s")
   
   this:create("Value_Slider"	, "set desired speed X", "set desired speed X",1);
   this:widget(0):sliderRange(-5, 5);
   this:widget(0):sliderValue(0);
   
   this:create("Value_Slider"	, "set desired speed Z", "set desired speed Z",1);
   this:widget(0):sliderRange(-0.78, 2);
   this:widget(0):sliderValue(0);

   this:create("Value_Slider"	, "set timeScale", "set timeScale",1);
   this:widget(0):sliderRange(0.1,10);
   this:widget(0):sliderValue(1);
   
   this:create("Value_Slider"	, "set desired turning speed", "set desired turning speed",1);
   this:widget(0):sliderRange(-1.4, 1.4);
   this:widget(0):sliderValue(0);
   
   this:create("Value_Slider"	, "set externalForce X", "set externalForce X",1);
   this:widget(0):sliderRange(0, 100);
   this:widget(0):sliderValue(0);
   
   this:create("Value_Slider"	, "set externalForce Z", "set externalForce Z",1);
   this:widget(0):sliderRange(0, 100);
   this:widget(0):sliderValue(0);
   
   
   for k, v in pairs(float_options) do
	   print(k,v.min, v.max, v.val)
      this:create("Value_Slider"	, k, k,1)
      this:widget(0):sliderRange(v.min, v.max)
      this:widget(0):sliderValue(v.val or 0)
   end
   
   this:create("Button"	, "impulse", "impulse",0)
   
   this:create("Button", "center viewpoint", "center viewpoint",0)
   
   local k='attachCamera'
   this:create("Check_Button"	, k, k,0)
   this:widget(0):checkButtonValue(boolean_options.attachCamera)
   for k, v in pairs(boolean_options) do
	   if k~='attachCamera' then
		   this:create("Check_Button"	, k, k,0)
		   this:widget(0):checkButtonValue(v)
	   end
   end
   
   
   --this:create("Check_Button"	, "orientation correction", "orientation correction",0);
   --this:widget(0):checkButtonValue(true)
   this:updateLayout()		

   mCameraInfo={}
   local curPos=vector3(0,0,0)
   mCameraInfo.vpos=RE.viewpoint().vpos+vector3(300,0,0)
   mCameraInfo.vat=RE.viewpoint().vat-vector3(0,-100,0)

--   dbg.startTrace()


      if mSynthesis and mSynthesis.skin2 then
	 mSynthesis.skin2:setVisible(boolean_options['show character'])
      end

end

function dtor()
   dbg.finalize()
end

function onCallback(w, userData)
   if w:id()=="type" then
      print(w:menuValue())
      --dbg.startTrace()
      mSynthesis.desiredType=w:menuValue()
   elseif w:id()=="set desired speed X" then
      mSynthesis.desiredSpeedX=w:sliderValue()
   elseif w:id()=="set desired speed Z" then
      mSynthesis.desiredSpeedZ=w:sliderValue()
   elseif w:id()=="set timeScale" then
      mSynthesis.desiredTimeScale=w:sliderValue()
   elseif w:id()=="set desired turning speed" then
      mSynthesis.desiredTurningSpeed=w:sliderValue()
   elseif w:id()=="set externalForce X" then
      mSynthesis.externalForce.x=w:sliderValue()
   elseif w:id()=="set externalForce Z" then
      mSynthesis.externalForce.z=w:sliderValue()
   elseif w:id()=="impulse" then
      mSynthesis.impulse=float_options.impulseDuration.val
   elseif w:id()=="center viewpoint"then
      local center=mSynthesis.pendulum:calcCOMpos()
      RE.viewpoint().vat:assign(center*100)
      RE.viewpoint():update()
      
   elseif w:id()=="show character" then
      if mSynthesis and mSynthesis.skin2 then
	 mSynthesis.skin2:setVisible(w:checkButtonValue())
      end
   else
      for k, v in pairs(float_options) do
	 if w:id()==k then
	    float_options[k].val=w:sliderValue()
		if k=='amt_sideway_vel' then
			useCase.amt_sideway_vel=w:sliderValue()
		end
	    break
	 end
      end
      for k, v in pairs(boolean_options) do
	 if w:id()==k then
	    boolean_options[k]=w:checkButtonValue()
	    break
	 end
      end
   end
end

function frameMove(fElapsedTime)
   --	debug.debug()
   if mSynthesis~=nil and noFrameMove==nil  then

      for rdd=1, renderDownSample do
	 for i=1, outputSuperSample do
	    mSynthesis:oneStep()
		mSynthesis:debugOut()
		if boolean_options.drawPDtarget then
			mSynthesis.skin2:setPoseDOF(mSynthesis.samplePose_pdtarget)
		end
	    if boolean_options.attachCamera then
	       local curPos= mSynthesis.pendulum:calcCOMpos()*100
	       curPos.y=0
		   --RE.viewpoint().vpos:assign(mCameraInfo.vpos+curPos)
	       RE.viewpoint().vat:assign(mCameraInfo.vat+curPos)
	       RE.viewpoint():update()     
	    end

	    if renderSuperSample and i~=outputSuperSample then
	       noFrameMove=true
	       RE.renderOneFrame(false)	
	       noFrameMove=nil
	    end
	 end
      end
   end
end


