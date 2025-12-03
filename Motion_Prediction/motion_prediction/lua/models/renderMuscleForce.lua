
require("config")
require("module")
require("IPC_based/common")

package.projectPath='../Samples/classification/'
package.path=package.path..";../../taesooLib/Samples/classification/lua/?.lua" --;"..package.path

package.path=package.path..";../Samples/ysscripts/samples/?.lua" --;"..package.path
--require("IPC_based/useCases")
require("useCasesMuscle")

package.path=package.path..";../Samples/ysscripts/lib/?.lua" --;"..package.path
require("utilfunc")
require("OsModel")


debugVis=true 
--function loadBVH(chosenFile)
	--local initialHeight=0
	--mMotionDOFcontainer=MotionDOFcontainer(mLoader.dofInfo)
	--local dof=_importBVH(chosenFile, initialHeight, 1.0)
	--mMotionDOFcontainer:resize(dof:rows())
	--mMotionDOFcontainer.mot:assign(dof)

	---- fix except root joint
	--mMotionDOFcontainer.mot:matView():sub(0,0,6,0):setAllValue(0)

	--_G.chosenMotFile=chosenFile
	--mSkin:applyMotionDOF(mMotionDOFcontainer.mot)
	--RE.motionPanel():motionWin():detachSkin(mSkin)
	--RE.motionPanel():motionWin():addSkin(mSkin)
	--useNearestSamplingWhenExportingBVH=true
--end

function ctor()
	RE.renderer():fixedTimeStep(true)   

	mEventReceiver=EVR()

	this:create("Button", "draw COM traj", "draw COM traj")
	this:create("Button", "attach camera", "attach camera")
	this:widget(0):buttonShortcut("FL_ALT+c")

	this:create("Check_Button", "draw skeleton", "draw skeleton")
	this:widget(0):checkButtonValue(0)

	this:updateLayout()
	this:redraw()

	mObjectList=Ogre.ObjectList()

	gMode = 'zoomout'
	--gMode = 'zoomin'
	----gMode = 'zoomin2'

	local initialHeight
	do -- set viewpoint
		RE.viewpoint():setFOVy(40)
		local vpos, vat
		if gMode=='zoomout' then
			initialHeight=.0
			vpos=vector3(.05*100,1.*100,-2.5*100)
			vat=vector3(.05*100,.85*100,0)
		elseif gMode=='zoomin' then
			initialHeight=.5
			--vpos=vector3(.1*100,.64*100,-.7*100)
			--vat=vector3(.1*100,.64*100,0)
			vpos=vector3(-1.03*100,.48*100,-.35*100)
			vat=vector3(-1.03*100,.48*100,0)
		elseif gMode=='zoomin2' then
			initialHeight=.5
			vpos=vector3(.32*100,.58*100,-.2*100)
			vat=vector3(.32*100,.58*100,0)
		end

		RE.viewpoint().vpos:assign(vpos)
		RE.viewpoint().vat:assign(vat)
		RE.viewpoint():update()
	end

	--model = model_files.gait1992_tong_repeat
	model = model_files.gait1992_gait_repeat
	
	mOsim = OsModel(model.wrlpath, model.luamsclpath, model)
	mOsim:removeMusclesExcept({'sar_r'})

	mOsim.mSkin:setMaterial("lightgrey")

	local motpath = model.mot_file

	--mOsim = OsModel(wrlpath, luamsclpath)
	mOsim:setBoneForwardKinematics(mOsim.mLoader:fkSolver())

	if string.find(motpath, '.dof') then
		mOsim:loadDOF(motpath)
	elseif string.find(motpath, '.bvh') then
		mOsim:loadBVH(motpath,heightOffset)
	end

	for i=0, mOsim.mMotionDOFcontainer.mot:numFrames()-1 do
		mOsim.mMotionDOFcontainer.mot:row(i):set(1,mOsim.mMotionDOFcontainer.mot:row(i):get(1)+(initialHeight or model.initialHeight) )
	end

	--mOsim.mMotionDOFcontainer.mot:assign(mOsim.mMotionDOFcontainer.mot:range(100,mOsim.mMotionDOFcontainer.mot:numFrames()))
	if gMode=='zoomin' or gMode=='zoomin2' then
		local tf=transf(quater(1.2, vector3(0,0,1)), vector3(0,0,0))
		local mot = mOsim.mMotionDOFcontainer.mot
		MotionDOF.setRootTransformation(mot:row(0), tf*MotionDOF.rootTransformation(mot:row(0)))
	end

	mOsim:setIsometricFiberLengths()
 
	RE.motionPanel():motionWin():addSkin(mOsim.mSkin)
end


function onCallback(w, userData)
	if w:id()=="draw COM traj" then
		do -- test
			--local comtraj=calcCOMtrajectory(mLoader, mMotionDOFcontainer.mot, mMotionDOFcontainer.discontinuity)
			local comtraj=calcCOMtrajectory(mOsim.mLoader, mMotionDOFcontainer.mot, mMotionDOFcontainer.discontinuity)
			mObjectList:registerObject("comtraj", "LineList", "solidred", comtraj:matView()*100,0)
		end
	elseif w:id()=="attach camera" then
		mEventReceiver:attachCamera()
	end
end


function dtor()
	--RE.motionPanel():motionWin():detachSkin(mSkin)
	RE.motionPanel():motionWin():detachSkin(mOsim.mSkin)
	mOsim=nil
	--mLoader=nil
	mObjectList=nil
	--mSkin=nil
	mMotionDOFcontainer=nil
	collectgarbage()
	dbg.finalize()
	detachSkins()
end
if EventReceiver then
	--class 'EVR'(EventReceiver)
	EVR=LUAclass(EventReceiver)
	function EVR:__init(graph)
		--EventReceiver.__init(self)
		self.currFrame=0
		self.cameraInfo={}
	end
end

function EVR:onFrameChanged(win, iframe)
	self.currFrame=iframe
	if self.trajectory then
		if self.currFrame<self.trajectory:rows() then
			local mMotionDOF=mMotionDOFcontainer.mot
			local curPos=self.trajectory:row(self.currFrame):toVector3(0)*100
			local pPos=
			MotionDOF.rootTransformation(mMotionDOF:row(self.currFrame)).translation
			local currRot=
			self.trajectoryOri:row(self.currFrame):toQuater(0):rotationY()

			dbg.draw('Line', pPos*100, pPos*100+rotate(vector3(0,0,100), currRot), 'prot')

			do
				RE.viewpoint().vpos:assign(self.cameraInfo.vpos+curPos)
				RE.viewpoint().vat:assign(self.cameraInfo.vat+curPos)
				RE.viewpoint():update()     
			end
		end
	end
	
	if mOsim.records[iframe]==nil then
		print(iframe, 'orig')
		mOsim.mLoader:setPoseDOF(mMotionDOFcontainer.mot:row(self.currFrame))

		mOsim:invalidatePPIndicesBasedInfo()
		mOsim:invalidatePPPoszBasedInfo()
		mOsim:integrateMuscleDynamics(1/30.)

		--print('frame', iframe)
		--local u = create_array(mOsim:getNumMuscles(), 1.)
		local u = create_array(mOsim:getNumMuscles(), math.sin((iframe/10)%math.pi))
		--mOsim:setExcitations(u)
		
		if gMode=='zoomout' then
			mOsim:drawMuscles(iframe, mObjectList)
		elseif gMode=='zoomin' or gMode=='zoomin2' then
			mOsim:drawMuscles(iframe, mObjectList,.01)
		end
		--mOsim:drawJoints(iframe, mObjectList)
		--mOsim:drawMuscleWithForces(iframe, mObjectList)
	else
		print(iframe, 'rec')
		mOsim:drawRecordedScene(iframe, mObjectList)
	end
end

function EVR:attachCamera()

	--if mLoader~=nil then
	if mOsim~=nil then

		local discont=mMotionDOFcontainer.discontinuity
		local mMotionDOF=mMotionDOFcontainer.mot

		self.trajectory=matrixn(mMotionDOFcontainer:numFrames(),3)

		self.trajectoryOri=matrixn(mMotionDOFcontainer:numFrames(),4)
		local segFinder=SegmentFinder(discont)

		for i=0, segFinder:numSegment()-1 do
			local s=segFinder:startFrame(i)
			local e=segFinder:endFrame(i)

			for f=s,e-1 do
				self.trajectory:row(f):setVec3(0, MotionDOF.rootTransformation(mMotionDOF:row(f)).translation)
				self.trajectory:row(f):set(1,0)
				self.trajectoryOri:row(f):setQuater(0, MotionDOF.rootTransformation(mMotionDOF:row(f)).rotation:rotationY())
			end
			print("filtering",s,e)
			math.filter(self.trajectory:range(s,e,0, 3), 63)
			math.filter(self.trajectoryOri:range(s,e,0, 4), 63)
		end

		local curPos=self.trajectory:row(self.currFrame):toVector3(0)*100
		self.cameraInfo.vpos=RE.viewpoint().vpos-curPos
		self.cameraInfo.vat=RE.viewpoint().vat-curPos
		self.cameraInfo.dist=RE.viewpoint().vpos:distance(curPos)
		self.cameraInfo.refRot=self.trajectoryOri:row(self.currFrame):toQuater(0):rotationY()
	end
end

function frameMove(fElapsedTime)
end

require('subRoutines/classifyLib')
