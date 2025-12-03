require("config")
require("common")

package.projectPath='../'
package.path=package.path..";../Samples/QP_controller/lua/?.lua" --;"..package.path
package.path=package.path..";../Samples/classification/lua/?.lua" --;"..package.path

package.path=package.path..";../Samples/ysscripts/samples/?.lua" --;"..package.path
--require("IPC_based/useCases")
require("useCasesMuscle")

require("IPC_based/LocoGraph")
--require("IPC_based/useCases")

scenario=useCase.scenario
model=scenarios.toModel(scenario)
model.motionFrameRate=model.frame_rate
loadGraph=true
OnlineSynthesis=LUAclass()

--dbg.setFunctionHook(RE, 'createVRMLskin')

function OnlineSynthesis:__finalize()
	self.skel=nil
	self.graph:__finalize()
	self.graph=nil
end

function OnlineSynthesis:setFootState(f, l, state)
	local function set(f,l, lf, rf)
		for i=f,l do
			self.graph.leftFoot:set(i,lf)
			self.graph.rightFoot:set(i,rf)
		end
	end

	if state=='L' then
		set(f,l,true,false)
	elseif state=='R' then
		set(f,l,false,true)
	elseif state=='D' or state=='S' then
		set(f,l,true,true)
	else
		assert(state=='F')
		set(f,l,false,false)
	end
end

function OnlineSynthesis:setFootStateSeg(seg, m)
	local f=seg.first
	local l=seg.last
	local name=seg.name
	self:setFootState(f, m, string.sub(name,1,1))
	self:setFootState(m,l,string.sub(name,2,2))
end

function OnlineSynthesis:__init()
	self.skel=MainLib.VRMLloader(model.file_name)
	self.skel:printHierarchy()
	self.skel.dofInfo:setFrameRate(model.frame_rate)

	local skel=self.skel

	if useCase.mot_file~=nil then -- override
		model.mot_file=useCase.mot_file
	end


	if loadGraph then
		self.skel_withoutCOMjoint=self.skel
		self.graph=LocoGraph:new(self.skel,model.mot_file)
		self.skel=self.graph.skel
		skel=self.skel
		if useCase.mot_file~=nil then -- override
			model.mot_file=useCase.mot_file
		end


		self.outputFrameRate=model.frame_rate
		self.graph.pendulum=LocoGraph.createPendulum(self.outputFrameRate)
		self.graph:createGraph(graphFile)
		self.motionDOF=self.graph.motions[1].mot
		self.graph.ZMPcalculator=self.graph.motions[1].ZMPcalculator
		self.graph.rotY=self.graph.motions[1].rotY
		assert(self.motionDOF, self.graph.ZMPcalculator, self.graph.rotY)

	else
		self.motionDOF,self.dmotionDOF,leftFoot,rightFoot
		=scenarios.loadAnimation(skel, file)

		print("Create graph")
		self.graph={}

		do	 -- calc rot_y		
			local offset=quater()
			local nf=self.motionDOF:numFrames()
			self.graph.rotY=quaterN(nf)

			for i=0,nf-1 do 
				local tf=self.motionDOF:rootTransformation(i)
				local rot_y=self.graph.rotY(i)
				tf.rotation:decompose(rot_y, offset)
			end
		end

		self.graph.ZMPcalculator={ zmp=vector3N(), com=vector3N()}
		self.graph.__finalize=function(...) end

		-- TODO: if optimized cartpole trajectories exist, load them.
		-- please refer to CartPoleController2.lua and CartPoleOptimizer.lua for details.
		local cartPoleFN=str.left(model.mot_file,-3).."zmpcom"
		if util.isFileExist(cartPoleFN) then
			self.graph.ZMPcalculator=util.loadTable(cartPoleFN)
		end

		self.graph.leftFoot=leftFoot
		self.graph.rightFoot=rightFoot      
	end
	self.graph.pendulum:removeSkin()
	self.skin2=RE.createVRMLskin(skel, false)
	self.skin2:setThickness(0.03)
	self.skin2:scale(100,100,100)
	self.skin2:setMaterial("lightgrey_transparent")

	self.skin2:applyMotionDOF(self.motionDOF)
	RE.motionPanel():motionWin():addSkin(self.skin2)

	self.skelPendulum=MainLib.VRMLloader(package.projectPath.."Resource/cart_pole_ball.wrl")
	self.skinPendulum=RE.createVRMLskin(self.skelPendulum, false)
	self.skinPendulum:scale(100,100,100)
	self.skinPendulum:setMaterial("green")
	self.viewer=CartPoleView(self.graph)
end

function ctor()
	mSynthesis=OnlineSynthesis:new()   

	this:create("Button", "initializeFootState", "initializeFootState")

	this:create("Button", "adjustFootTransition", "adjustFootTransition")
	this:widget(0):buttonShortcut("FL_ALT+S")

	this:create("Button", "rotate motion", "rotate motion",0,3,0)
	this:create("Button", "rotate light", "rotate light",0,3,0)
	this:create("Button", "export", "export")
	this:create("Button", "plot delta", "plot delta")
	this:create("Button", "foot cleanup", "foot cleanup")
	this:create("Button", "import constraints","import constraints")
	this:create("Button", "plot COM", "plot COM")
	this:create("Button", "playback", "playback")
	this:create("Button", "plot poses", "plot poses")
	this:create("Button", "set delta 20","set delta 20",0,2,3)
	this:create("Button", "set delta 80","80",2,3,0)
	this:create("Button", "setS","setS",0,1,3)
	this:create("Button", "setE","setE",1,2,3)
	this:create("Button", "draw poses", "draw poses",2,3,3)
	this:create("Check_Button", "draw orientation", "draw orientation")
	this:widget(0):checkButtonValue(false)
	this:create("Check_Button", "draw MMIPM", "draw MMIPM")
	this:widget(0):checkButtonValue(true)
	this:create("Button", "draw scripted poses", "draw scripted poses")

	this:updateLayout()
end


CartPoleView=LUAclass (EventReceiver)

function CartPoleView:redrawPanel()
	self.panel:clear(0,100000)

	-- if loadGraph then
	-- 	local seg=self.seg
	-- 	for i=1,seg:size() do
	-- 		self.panel:drawBoxColormap(seg[i][2],seg[i][3],i)
	-- 	end
	-- end

	if self.graph.leftFoot then
		for i=0, self.graph.leftFoot:size()-1 do
			local conL,conR
			if self.graph.leftFoot(i) then conL=1 else conL=0 end
			if self.graph.rightFoot(i) then conR=1 else conR=0 end
			self.panelL:drawBoxColormap(i,i+1, conL)
			self.panelR:drawBoxColormap(i,i+1, conR)
		end
	end
	--   RE.motionPanel():scrollPanel():addPanel(self.graph.leftFoot:bit(), CPixelRGB8(255,255,255))
	RE.motionPanel():scrollPanel():redraw()

end

function CartPoleView:__init(graph)
	--EventReceiver.__init(self)
	self.objectList=Ogre.ObjectList ()

	self.graph=graph

	self.panel=SelectPanel()
	self.panelL=SelectPanel()
	self.panelR=SelectPanel()

	if loadGraph then
		local seg=array:new()
		for i,grpName in ipairs(self.graph.groups.tf) do
			for k,v in pairs(self.graph[grpName]) do
				seg:pushBack({v.name, v.first, v.last, v})
			end
		end

		self.panel:init(RE.motionPanel(), "segments", 1, seg:size())
	else
		self.panel:init(RE.motionPanel(), "segments", 1, 2)
	end
	self.panelL:init(RE.motionPanel(), "conL", 1, 2)
	self.panelR:init(RE.motionPanel(), "conR", 1, 2)


	self.seg=seg

	self:redrawPanel()
end

function CartPoleView:onFrameChanged(win, iframe)

	print(iframe)

	function getState(v1, v2)
		local theta=vectorn(6)
		theta:set(0, v1.z)
		theta:set(1, v1.x)

		local q=quater()
		q:axisToAxis(vector3(0,1,0), v2-v1)
		theta:setQuater(2,q)
		return theta
	end

	mSynthesis.skinPendulum:setPoseDOF(getState(self.graph.ZMPcalculator.zmp(iframe), self.graph.ZMPcalculator.com(iframe)))
	local zmpEntity=self.objectList:registerEntity("ZMP", "sphere1010.mesh")
	zmpEntity:setScale(5,5,5)
	zmpEntity:setPosition(self.graph.ZMPcalculator.zmp(iframe)*100)

	--RE.output("com", tostring(self.graph.ZMPcalculator.com(iframe)))
	--RE.output("ZMP", tostring(self.graph.ZMPcalculator.zmp(iframe)))
--	RE.output("com", tostring(self.graph.ZMPcalculator.com(iframe)))
--	RE.output("ZMP", tostring(self.graph.ZMPcalculator.zmp(iframe)))
	local comEntity=self.objectList:registerEntity("COM", "sphere1010.mesh")
	comEntity:setScale(5,5,5)
	comEntity:setPosition(self.graph.ZMPcalculator.com(iframe)*100)

	dbg.drawLine(self.objectList, self.graph.ZMPcalculator.com(iframe)*100,

	self.graph.ZMPcalculator.zmp(iframe)*100, "zmp_com")
	--		self.pendulum:inverseDynamics(mZMPcalculator.com(iframe),mZMPcalculator.COMvel(iframe), mZMPcalculator.COMacc(iframe))

	--		local zmpEntity2=self.objectList:registerEntity("ZMP", "sphere1010.mesh")
	--		zmpEntity2:setScale(10,10,10)
	--		zmpEntity2:setPosition(self.pendulum.pend.translation*100)

	if this:findWidget('draw orientation'):checkButtonValue() then
		local synRoot=mSynthesis.motionDOF
		local out=matrixn(0,3)
		local tempVec2=vectorn(3)
		local fixed=iframe
		local freq=4
		for i=math.floor((fixed+freq)/freq)*freq, math.min(fixed+240, synRoot:rows()-1), freq do
			local r=synRoot:row(i)
			local s=vector3(r(0),0.03,r(2))
			local rot_y_ref=MotionDOF.rootTransformation(synRoot:row(i)).rotation:rotationY()
			local v=rotate( vector3(0.1,0,0),rot_y_ref)
			tempVec2:setVec3(0,s-v)
			out:pushBack(tempVec2);
			tempVec2:setVec3(0,s+v)
			out:pushBack(tempVec2);
		end
		self.objectList:registerObject("oriPredict", "LineList", "solidred", out*100 , 0)
	end
	self.currFrame=iframe
end


function dtor()	
	mSynthesis:__finalize()
	mSynthesis=nil
end

function onCallback(w, userData)

	if w:id()=="rotate motion" then
		local mot=mSynthesis.motionDOF
		local tf=transf()
		tf:identity()
		tf:leftMultRotation(quater(math.rad(90), vector3(0,1,0)))

		local zmpc=mSynthesis.graph.ZMPcalculator
		for i=0, mot:numFrames()-1 do
			MotionDOF.setRootTransformation(mot:row(i), tf*MotionDOF.rootTransformation(mot:row(i)))
			zmpc.zmp(i):assign(tf*zmpc.zmp(i))
			zmpc.com(i):assign(tf*zmpc.com(i))
		end
		mSynthesis.skin2:applyMotionDOF(mSynthesis.motionDOF)
	elseif w:id()=='rotate light' then
		local osm=RE.ogreSceneManager()
		if osm:hasSceneNode("LightNode") then
			local lightnode=osm:getSceneNode("LightNode")
			lightnode:rotate(quater(math.rad(30), vector3(0,1,0)))
		end
	elseif w:id()=='draw scripted poses' then
		local frames={267, 290, 311, 325}
		local ERROR=10
		local zmp_offset_z={-0.11-0.03, 0.2-0.02-0.02, -0.20+0.02, ERROR}
		local self=mSynthesis.viewer
		local drawMMIPM=this:findWidget("draw MMIPM"):checkButtonValue()
		RE.viewpoint():setFOVy(45.000000)
		RE.viewpoint().vpos:set(258.862345, 100.571576, 136.665168)
		RE.viewpoint().vat:set(-5.977640, 86.210595, 132.920554)
		RE.viewpoint():update()
		for iii, iframe in ipairs(frames) do
			print(iframe)

			function getState(v1, v2)
				local theta=vectorn(6)
				theta:set(0, v1.z)
				theta:set(1, v1.x)

				local q=quater()
				q:axisToAxis(vector3(0,1,0), v2-v1)
				theta:setQuater(2,q)
				return theta
			end
			local stI="_"..tostring(iframe)


			if true then -- draw pose
				local mot=mSynthesis.motionDOF
				local skel=mSynthesis.skel
				local skin2=RE.createVRMLskin(skel, false)
				skin2:setThickness(0.03)
				skin2:scale(100,100,100)
				skin2:setMaterial("lightgrey_transparent")
				skin2:setPoseDOF(mot:row(iframe))
			end

			local zmpc=self.graph.ZMPcalculator
			if not drawMMIPM then
				require('RigidBodyWin/subRoutines/ZMPcalculator')
				zmpc=ZMPcalculator2:new(mSynthesis.skel, self.graph.motions[1].mot_withoutCOMjoint,  self.graph.motions[1].dmot, self.graph.motions[1].discontinuity)
				zmpc.zmp(iframe):radd(vector3(0,0,zmp_offset_z[iii]))
			end
			
			if drawMMIPM or zmp_offset_z[iii]~=ERROR then
				if true then --drawPendulum
					local skinPendulum =RE.createVRMLskin(mSynthesis.skelPendulum, false)
					skinPendulum:scale(100,100,100)
					skinPendulum:setMaterial("green")
					skinPendulum:setPoseDOF(getState(zmpc.zmp(iframe), zmpc.com(iframe)))
				end

				local zmpEntity=self.objectList:registerEntity("ZMP"..stI, "sphere1010.mesh")
				zmpEntity:setScale(5,5,5)
				zmpEntity:setPosition(zmpc.zmp(iframe)*100)

				local comEntity=self.objectList:registerEntity("COM"..stI, "sphere1010.mesh")
				comEntity:setScale(5,5,5)
				comEntity:setPosition(zmpc.com(iframe)*100)

				dbg.drawLine(self.objectList, zmpc.com(iframe)*100, zmpc.zmp(iframe)*100, "zmp_com"..stI)
			end
		end
	elseif w:id()=="setS" then
		mPlotPoses=mPlotPoses or {}
		mPlotPoses.startFrame= mSynthesis.viewer.currFrame
	elseif w:id()=="setE" then
		mPlotPoses=mPlotPoses or {}
		mPlotPoses.endFrame= mSynthesis.viewer.currFrame
	elseif w:id()=="set delta 20" then
		mPlotPoses=mPlotPoses or {}
		mPlotPoses.delta= 20
	elseif w:id()=="set delta 80" then
		mPlotPoses=mPlotPoses or {}
		mPlotPoses.delta= 80

	elseif w:id()=="draw poses" then

		os.createDir('../dump/ipcv')
		RE.renderer():setScreenshotPrefix('../dump/ipcv')
		RE.renderer():screenshot(true)
		if mPlotPoses.startFrame and mPlotPoses.endFrame then
			for iframe=mPlotPoses.startFrame, mPlotPoses.endFrame,10 do
				RE.motionPanel():motionWin():changeCurrFrame(iframe)
				mSynthesis.viewer:onFrameChanged(win, iframe)
				RE.renderOneFrame(false)
			end
		end
		RE.renderer():screenshot(false)
	elseif w:id()=="plot poses" then
		--local delta=50
		--local startF=60
		--local endF=500
		
		-- popa
		--local delta=(1598-1347)/2
		--local startF=1347
		--local endF=1714

		-- justin_run
		--delta=20
		--startF=220
		--endF=400
		local startF=mPlotPoses.startFrame
		local endF=mPlotPoses.endFrame
		local delta=mPlotPoses.delta or 50

		local self=mSynthesis.viewer
		for iframe=startF, endF, delta do
			print(iframe)

			function getState(v1, v2)
				local theta=vectorn(6)
				theta:set(0, v1.z)
				theta:set(1, v1.x)

				local q=quater()
				q:axisToAxis(vector3(0,1,0), v2-v1)
				theta:setQuater(2,q)
				return theta
			end
			local stI="_"..tostring(iframe)

			if true then --drawPendulum
				local skinPendulum =RE.createVRMLskin(mSynthesis.skelPendulum, false)
				skinPendulum:scale(100,100,100)
				skinPendulum:setMaterial("green")
				skinPendulum:setPoseDOF(getState(mSynthesis.graph.ZMPcalculator.zmp(iframe), mSynthesis.graph.ZMPcalculator.com(iframe)))
			end

			if true then -- draw pose
				local mot=mSynthesis.motionDOF
				local skel=mSynthesis.skel
				local skin2=RE.createVRMLskin(skel, false)
				skin2:setThickness(0.03)
				skin2:scale(100,100,100)
				skin2:setMaterial("lightgrey_transparent")
				skin2:setPoseDOF(mot:row(iframe))
			end

			local zmpEntity=self.objectList:registerEntity("ZMP"..stI, "sphere1010.mesh")
			zmpEntity:setScale(5,5,5)
			zmpEntity:setPosition(self.graph.ZMPcalculator.zmp(iframe)*100)

			local comEntity=self.objectList:registerEntity("COM"..stI, "sphere1010.mesh")
			comEntity:setScale(5,5,5)
			comEntity:setPosition(self.graph.ZMPcalculator.com(iframe)*100)

			dbg.drawLine(self.objectList, self.graph.ZMPcalculator.com(iframe)*100,

			self.graph.ZMPcalculator.zmp(iframe)*100, "zmp_com"..stI)
		end
	elseif w:id()=="foot cleanup" then
		local mot=mSynthesis.motionDOF
		local dmot=mSynthesis.dmotionDOF
		local skel=mSynthesis.skel
		local floor=MainLib.VRMLloader("../Resource/mesh/floor_y.wrl")

		usePenaltyMethod=true
		local simulator=createSimulator(simulators.gmbs)
		simulator:registerCharacter(skel)
		simulator:registerCharacter(floor)
		if model==model_files.hyunwoo_real_cart or model==model_files.hyunwoo_full_cart then
			registerContactPair(model, skel, floor, simulator)
		else
			registerContactPairJump(model, skel, floor, simulator)
		end 

		simulator:init(0.0001, DynamicsSimulator.EULER)
		MotionLoader.setVoca(skel, model.bones)

		function createIKsolver(isLeftFoot, footpos)
			local ik={}
			ik.effectors=MotionUtil.Effectors()
			ik.effectors:resize(1)

			local lfoot=skel:getBoneByVoca(MotionLoader.LEFTANKLE)
			local lknee=skel:getBoneByVoca(MotionLoader.LEFTKNEE)
			local rfoot=skel:getBoneByVoca(MotionLoader.RIGHTANKLE)
			local rknee=skel:getBoneByVoca(MotionLoader.RIGHTKNEE)

			if isLeftFoot then
				ik.effectors(0):init(lfoot,footpos)
			else
				ik.effectors(0):init(rfoot,footpos)
			end

			ik.solver=LimbIKsolver(skel.dofInfo,ik.effectors, lknee, rknee)
			ik.conPos=vector3N(1)
			ik.conDelta=quaterN(1)

			--if iterativeIK~=nil then  
			ik.fk=BoneForwardKinematics(skel)
			ik.fk:init()
			ik.skel=skel
			--end

			-- rotY is used for defining foot global orientations.
			function ik:solve(rotY, roottf, foot, pose)
				assert(pose~=nil)
				self.conPos(0):assign(foot)
				self.solver:IKsolve(pose, rotY, roottf, self.conPos)
			end

			return ik
		end


		if false then
			for i=0,mot:numFrames()-1 do
				local pose=mot:row(i):copy()

				-- use simulator
				simulator:setLinkData(0, DynamicsSimulator.JOINT_VALUE, pose)
				simulator:setLinkData(0, DynamicsSimulator.JOINT_VELOCITY, dmot:row(i))
				simulator:initSimulation()

				local cqInfo=simulator:queryContactAll()
				local roottf=MotionDOF.rootTransformation(pose)
				for i=0, cqInfo:size()-1 do

					if cqInfo(i).chara==0 then
						local bone=cqInfo(i).bone
						local localpos=cqInfo(i).p:copy()

						skel:setPoseDOF(pose)
						local gp=skel:fkSolver():globalFrame(bone):toGlobalPos(localpos)

						local iksolver=createIKsolver(bone==skel:getBoneByVoca(MotionLoader.LEFTANKLE), localpos)


						local collisionMargin=0.01
						if gp.y<collisionMargin*2 then 
							gp.y=collisionMargin*2 
						end

						iksolver:solve(roottf.rotation:rotationY(), roottf, gp, pose)
					end
				end
			end
			mot:row(i):assign(pose)
		else
			local lfoot=skel:getBoneByVoca(MotionLoader.LEFTANKLE)
			for foot=1,2 do
				local bone
				if foot==1 then
					bone=skel:getBoneByVoca(MotionLoader.RIGHTANKLE)
				else
					bone=skel:getBoneByVoca(MotionLoader.LEFTANKLE)
				end
				local mesh=bone:getMesh()

				local iksolver=createIKsolver(bone==skel:getBoneByVoca(MotionLoader.LEFTANKLE), vector3(0,0,0))

				for i=0,mot:numFrames()-1 do
					local pose=mot:row(i):copy()

					skel:setPoseDOF(pose)
					local roottf=MotionDOF.rootTransformation(pose)
					local miny=100
					for v=0, mesh:numVertex()-1 do
						local lpos=mesh:getVertex(v)
						local gpos=skel:fkSolver():globalFrame(bone):toGlobalPos(lpos)
						if gpos.y<miny then miny=gpos.y end
					end
					local gpos=skel:fkSolver():globalFrame(bone):toGlobalPos(vector3(0,0,0))
					if miny<0 then 
						gpos.y=gpos.y-miny 
						iksolver:solve(roottf.rotation:rotationY(), roottf, gpos, pose)
						mot:row(i):assign(pose)
					end

				end
			end
		end

	elseif w:id()=="plot COM" then
		local comtraj=calcCOMtrajectory(mSynthesis.skel, mSynthesis.motionDOF)
		mObjectList=mObjectList or Ogre.ObjectList()
		mObjectList:registerObject("comtraj", "LineList", "solidred", comtraj:matView()*100,0)
	elseif w:id()=="playback" then
		mPlayback=true
	elseif w:id()=="plot delta" then
		local com=mSynthesis.graph.ZMPcalculator.com
		local ZMPc=ZMPcalculator:new(mSynthesis.skel, mSynthesis.motionDOF, mSynthesis.dmotionDOF)
		local actCom=ZMPc.com

		local comvel=vector3N()
		local actComVel=vector3N()
		comvel:derivative(com, model.frame_rate)
		actComVel:derivative(actCom, model.frame_rate)

		local deltaVel=vector3N(comvel:size())
		local deltaCOM=vector3N(com:size())

		for i=0,com:size()-1 do
			local invRot=mSynthesis.graph.rotY:row(i):inverse()
			deltaVel(i):assign(invRot*(comvel(i)-actComVel(i)))
			deltaCOM(i):assign(invRot*(com(i)-actCom(i)))
			comvel(i):assign(invRot*comvel(i))
			actComVel(i):assign(invRot*actComVel(i))
		end
		math.mrdplotVec(deltaCOM:x(), "deltacom.x", deltaCOM:z(), "deltacom.z", 
		comvel:x(), "comvel.x", comvel:z(), "comvel.z")
		--		      actComVel:x(), "comvel.x", actComVel:z(), "comvel.z")
	elseif w:id()=="import constraints" then
		local dofFN=Fltk.chooseFile("choose dof", "../Resource/scripts/ui/rigidbodywin/", "*.dof", true)
		if string.len(dofFN)~=0 then
			local container=MotionDOFcontainer(mSynthesis.skel.dofInfo, dofFN)

			for i=0,mSynthesis.graph.leftFoot:size()-1 do
				if i<container.conL:size() then
					mSynthesis.graph.leftFoot:set(i, container.conL(i))
					mSynthesis.graph.rightFoot:set(i, container.conR(i))
				end
			end

			mSynthesis.viewer:redrawPanel()
		end

	elseif w:id()=="export" then
		local dofFN=Fltk.chooseFile("choose dof", "../Resource/scripts/ui/rigidbodywin/", "*.dof", true)
		if string.len(dofFN)==0 then return end

		-- 1. export .dof
		do
			local skel=mSynthesis.skel
			local srcDOF=mSynthesis.motionDOF
			local container=MotionDOFcontainer(skel.dofInfo)
			container:resize(srcDOF:numFrames())
			container.mot:assign(srcDOF)

			container.conL:assign(mSynthesis.graph.leftFoot)
			container.conR:assign(mSynthesis.graph.rightFoot)
			RE.motionPanel():scrollPanel():addPanel(container.conL:bit(), CPixelRGB8(255,255,255))
			RE.motionPanel():scrollPanel():addPanel(container.conR:bit(), CPixelRGB8(255,255,255))

			container:exportMot(dofFN)
		end
	elseif w:id()=="initializeFootState" then

		for i=1, mSynthesis.viewer.seg:size() do
			local seg=mSynthesis.viewer.seg[i]
			local name=seg[1]
			local f=seg[2]
			local l=seg[3]
			local m=math.round((f+l)/2)
			if string.len(name)==2 then
				mSynthesis:setFootStateSeg(seg[4], m)
			end
		end
		mSynthesis.viewer:redrawPanel()
	elseif w:id()=="adjustFootTransition" then
		local view=mSynthesis.viewer
		if view.currFrame==nil then return end
		for i=1, view.seg:size() do
			local seg=view.seg[i][4]
			if view.currFrame>=seg.first and view.currFrame<=seg.last then
				mSynthesis:setFootStateSeg(seg, view.currFrame)
				break
			end
		end
		view:redrawPanel()
	end


end

function frameMove(fElapsedTime)

	if mPlayback then
		local currFrame=mSynthesis.viewer.currFrame+2

		if currFrame<mSynthesis.motionDOF:numFrames() then
			RE.motionPanel():motionWin():changeCurrFrame(currFrame)
			mSynthesis.viewer:onFrameChanged(win, currFrame)
		else
			mPlayback=false
		end
	end
end


