require("config")
package.projectPath='../Samples/classification/'
package.path=package.path..";../Samples/classification/lua/?.lua" --;"..package.path
require("common")
require("module")
fc=require('retargetting/module/footSkateCleanup')
require('subRoutines/Timeline')
require("RigidBodyWin/subRoutines/Constraints")
require("RigidBodyWin/subRoutines/CollisionChecker")
update_encode_size = 35
pose_size = 33
hand = false 
config = {
	skinScale = 100,
	rendering_step = 1/30,
	frameRate=30,
	scaling_factor=1.0,
	draw = true,
	trackerBones ={
		{name='Hips',  trackerTransf=transf(quater(1, 0,0,0), vector3(0,0.1,0.1))},
		{name='LeftKnee',  trackerTransf=transf(quater(1, 0,0,0), vector3(0,-0.35,0.07))},
		{name='RightKnee',   trackerTransf=transf(quater(1, 0,0,0), vector3(0,-0.35,0.07))},
		{name='LeftElbow',    trackerTransf=transf(quater(1, 0,0,0), vector3(0.3,0.05,0))},
		{name='RightElbow',   trackerTransf=transf(quater(1, 0,0,0), vector3(-0.3,0.05,0))},
		{name='Neck',    trackerTransf=transf(quater(1, 0,0,0), vector3(0,0.15,0.1))},
	},
	trackerPosScale=1.0,
	useFilter=3, -- kernel size (additional delay)
}
config.bonesNames = {
	hips = config.trackerBones[1],
	left_ankle = config.trackerBones[2],
	right_ankle = config.trackerBones[3],
	left_elbow = config.trackerBones[4],
	right_elbow = config.trackerBones[5],
	neck = config.trackerBones[6],
}
config.numTracker = #config.trackerBones
-- mMotionDOF =  33개의 모션 DOF값 으로 이루어짐 
gui = true
draw = false
useNoise= false

TEST_VERSION=true
FUTURE_SIZE=30
ATTACH_CAMERA=true
filter__=false
use_ik__=false
HAND_DRAWER=false
DRAW_HISTORY=true
EVAL=false
function ctor()
	mEventReceiver = EVR()
	if gui then 
		local osm =RE.ogreSceneManager()
		osm:setFogNone()
	end
	mLoader_woH=MainLib.VRMLloader ("../Resource/motion/prediction/hyunwoo_lowdof_T_boxfoot.wrl")
	mLoader_woH_pre=MainLib.VRMLloader ("../Resource/motion/prediction/hyunwoo_lowdof_T_boxfoot.wrl")
	mLoader_woH_future=MainLib.VRMLloader ("../Resource/motion/prediction/hyunwoo_lowdof_T_boxfoot.wrl")
	mLoader_woH_pre_10=MainLib.VRMLloader ("../Resource/motion/prediction/hyunwoo_lowdof_T_boxfoot.wrl")
	mLoader_woH_future_10=MainLib.VRMLloader ("../Resource/motion/prediction/hyunwoo_lowdof_T_boxfoot.wrl")
	

	mLoader=MainLib.VRMLloader ("../Resource/motion/prediction/hyunwoo_lowdof_T_boxfoot_parable.wrl")
	mLoader_pre=MainLib.VRMLloader ("../Resource/motion/prediction/hyunwoo_lowdof_T_boxfoot_parable.wrl")
	mLoader_pre_future=MainLib.VRMLloader ("../Resource/motion/prediction/hyunwoo_lowdof_T_boxfoot_parable.wrl")
	mLoader_pre_10=MainLib.VRMLloader ("../Resource/motion/prediction/hyunwoo_lowdof_T_boxfoot_parable.wrl")
	mLoader_future_10=MainLib.VRMLloader ("../Resource/motion/prediction/hyunwoo_lowdof_T_boxfoot_parable.wrl")
	
	DOFcontainer = MotionDOFcontainer(mLoader.dofInfo , "../Resource/motion/prediction/stitchMotion_ubi3.dof")
	--DOFcontainer = MotionDOFcontainer(mLoader.dofInfo , "../Resource/motion/prediction/hyunwoo_lowdof_T_boxfoot_parable_walk3_subject1.dof")
	mMotionDOF = DOFcontainer.mot
	--DOFcontainer.discontinuity:load(mMotionDOF:numFrames(),"../Resource/motion/prediction/stitchMotion_ubi3.dof.discont")

	mSkin = RE.createVRMLskin(mLoader , false)
	mSkin:scale(config.skinScale,config.skinScale,config.skinScale)
	mSkin:setMaterial('red_transparent')
	mSkin:setVisible(HAND_DRAWER)
	mSkin_future = RE.createVRMLskin(mLoader , false)
	mSkin_future:scale(config.skinScale,config.skinScale,config.skinScale)
	mSkin_future:setMaterial('blue_transparent')
	mSkin_future:setVisible(HAND_DRAWER)
	
	mSkin_pre_10 = RE.createVRMLskin(mLoader , false)
	mSkin_pre_10:scale(config.skinScale,config.skinScale,config.skinScale)
	mSkin_pre_10:setMaterial('black')
	mSkin_pre_10:setVisible(HAND_DRAWER)
	mSkin_future_10 = RE.createVRMLskin(mLoader , false)
	mSkin_future_10:scale(config.skinScale,config.skinScale,config.skinScale)
	mSkin_future_10:setMaterial('blue_transparent')
	mSkin_future_10:setVisible(HAND_DRAWER)

	mSkin_woH = RE.createVRMLskin(mLoader_woH , false)
	mSkin_woH:scale(config.skinScale,config.skinScale,config.skinScale)
	mSkin_woH:setMaterial('red_transparent')
	mSkin_woH:setVisible(not HAND_DRAWER)
	mSkin_woH_future = RE.createVRMLskin(mLoader_woH_future, false)
	mSkin_woH_future:scale(config.skinScale,config.skinScale,config.skinScale)
	mSkin_woH_future:setMaterial('blue_transparent')
	mSkin_woH_future:setVisible(not HAND_DRAWER)

	mSkin_woH_pre_10 = RE.createVRMLskin(mLoader_woH_future, false)
	mSkin_woH_pre_10:scale(config.skinScale,config.skinScale,config.skinScale)
	mSkin_woH_pre_10:setMaterial('black')
	mSkin_woH_pre_10:setVisible(not HAND_DRAWER)
	mSkin_woH_future_10 = RE.createVRMLskin(mLoader_woH_future, false)
	mSkin_woH_future_10:scale(config.skinScale,config.skinScale,config.skinScale)
	mSkin_woH_future_10:setMaterial('blue_transparent')
	mSkin_woH_future_10:setVisible(not HAND_DRAWER)

	if TEST_VERSION then
		MOTION_HISTORY={}
		MOTION_HISTORY_woH={}
		MOTION_HISTORY_POSE=matrixn(FUTURE_SIZE,89)
		MOTION_HISTORY_POSE_woH=matrixn(FUTURE_SIZE,33)
		for i=0, FUTURE_SIZE-1 do
			local skin_ = RE.createVRMLskin(mLoader , false)
			local skin_woH = RE.createVRMLskin(mLoader_woH , false)
			skin_:scale(config.skinScale,config.skinScale,config.skinScale)
			skin_woH:scale(config.skinScale,config.skinScale,config.skinScale)
			if i<FUTURE_SIZE-1 or not i==9 then
				skin_:setMaterial('lightgrey_verytransparent')
				skin_woH:setMaterial('lightgrey_verytransparent')
			else
				skin_:setMaterial('black')
				skin_woH:setMaterial('black')
			end
			skin_:setVisible(HAND_DRAWER)
			skin_woH:setVisible(not HAND_DRAWER)
			table.insert(MOTION_HISTORY,skin_)
			table.insert(MOTION_HISTORY_woH,skin_woH)
		end
	end
	mLoader:printHierarchy()
	this:create('Check_Button','use_filter','use_filter')
	this:create('Check_Button','use_ik','use_ik')
	this:create('Check_Button','draw_hand','draw_hand')
	this:widget(0):checkButtonValue(HAND_DRAWER) 
	this:create('Check_Button','draw_tracker','draw_tracker')
	this:widget(0):checkButtonValue(draw) -- 1 for imediate start
	this:create('Check_Button','draw_between','draw_between')
	this:widget(0):checkButtonValue(DRAW_HISTORY)
	this:create('Check_Button','pose error calculating','pose error calculating')
	this:widget(0):checkButtonValue(EVAL)
	this:create("Check_Button", "attach camera", "attach camera")
	this:widget(0):checkButtonValue(ATTACH_CAMERA) -- 1 for imediate start
	this:updateLayout()
	latent = vectorn(120)
	latent:setAllValue(0)
	iframe = 0
	

	local filterSize=3*2+1 -- has to be an odd number
	local initialPose=vectorn()
	mLoader:updateInitialBone()
	mLoader:getPoseDOF(initialPose)
	mFilter=OnlineFilter(mLoader, initialPose, filterSize, true)

	tracker_data = All_tracker_data()

	prev_pose = vectorn(33)
	prev_pose:setAllValue(0)
	encodeBuffer = matrixn(5,update_encode_size)
	encodeBuffer:setAllValue(0)
	mTimeline=Timeline('timeline', mMotionDOF:rows()-1, config.rendering_step)
	hand_input = matrixn(3,26)
	hand_input:setAllValue(0)
	RE.viewpoint().vpos:set(200, 400,400)
	RE.viewpoint().vat:set(200, 0, -300)
	RE.viewpoint():update()
	for i, bone in ipairs(config.trackerBones) do
		local bone2=mLoader:getBoneByName(bone.name)
		bone.idx=bone2:treeIndex()
		bone.startT=mLoader.dofInfo:startT(bone2:treeIndex())

		bone.idx1=mLoader_woH:getBoneByName(bone.name):treeIndex()
	end
	createIKsolver1()
	filter = -1
	use_ik = -1
end

function dtor()
end

function vector2transf(vec)
	assert(vec:size()==7,'vector2trasnf 함수에서 input data 문제')
	local transform = transf()
	transform.translation:assign(vec:toVector3(0))
	transform.rotation:assign(vec:toQuater(3))
	return transform
end

if EventReceiver then 
	EVR = LUAclass(EventReceiver)
	function EVR:__init(graph)
		self.currFrame = 0
		self.cameraInfo = {}
	end 
end	

function EVR:onFrameChanged(win, iframe)
	if TEST_VERSION then
		--maximum=0~29, current version : learned based on 30 frame
		local rendering_frame = FUTURE_SIZE-1
		-- iframe ==currFramer , 30 : tracker data, rendering_frame=future
		local td = getTrackerData(iframe+FUTURE_SIZE+rendering_frame,draw)
		local td1 = vectorn(42)
		local td_10 = getTrackerData(iframe+FUTURE_SIZE+9,draw)
		local td1_10 = vectorn(42)
		for i=0,5 do 
			td1:setVec3(7*i,td.pos(i))
			td1:setQuater(7*i+3,td.ori(i))
			td1_10:setVec3(7*i,td_10.pos(i))
			td1_10:setQuater(7*i+3,td_10.ori(i))
		end
		local pose = mMotionDOF:row(iframe+(FUTURE_SIZE)):copy()
		--+59
		local pose_future = mMotionDOF:row(iframe+(FUTURE_SIZE*2)-1):copy()
		--30+9
		local pose_future_10 = mMotionDOF:row(iframe+(FUTURE_SIZE)+9):copy()
		local pose_woH=vectorn(33)
		local pose_woH_future=vectorn(33)
		local pose_woH_future_10=vectorn(33)
	
		if HAND_DRAWER then
			mSkin:setPoseDOF(pose)
			mSkin_future:setPoseDOF(pose_future)
			mSkin_future_10:setPoseDOF(pose_future_10)
		else
			pose_woH:range(0,26):assign(pose:range(0,26):copy())
			pose_woH:range(26,30):assign(pose:range(54,58):copy())
			pose_woH:range(30,33):assign(pose:range(86,89):copy())
			pose_woH_future:range(0,26):assign(pose_future:range(0,26):copy())
			pose_woH_future:range(26,30):assign(pose_future:range(54,58):copy())
			pose_woH_future:range(30,33):assign(pose_future:range(86,89):copy())
			pose_woH_future_10:range(0,26):assign(pose_future_10:range(0,26):copy())
			pose_woH_future_10:range(26,30):assign(pose_future_10:range(54,58):copy())
			pose_woH_future_10:range(30,33):assign(pose_future_10:range(86,89):copy())
			mSkin_woH:setPoseDOF(pose_woH)
			mSkin_woH_future:setPoseDOF(pose_woH_future)
			mSkin_woH_future_10:setPoseDOF(pose_woH_future_10)
		end
		local output = matrixn(30,mMotionDOF:numDOF()-56-7)
		local input = matrixn(30,26)
		for i=0,29 do 
			input:row(i):assign(getOnlyPose(mMotionDOF:row(iframe+i)))
		end
		python.F("play_mp","test_MotionPrediction",input,output)
		--0~30
		--30~60
		for i=0,FUTURE_SIZE-1 do
			local pose_prediction=vectorn(89)
			local pose_prediction_woH=vectorn(33)
			local pose_prediction_10=vectorn(89)
			local pose_prediction_woH_10=vectorn(33)

			pose_prediction:range(7,26):assign(output:row(i):range(0,19))
			pose_prediction:range(54,58):assign(output:row(i):range(19,23))
			pose_prediction:setVec3(86,output:row(i):toVector3(23))
			pose_prediction:setVec3(0,mMotionDOF:row(iframe+i+FUTURE_SIZE):toVector3(0):copy())
			pose_prediction:setQuater(3,mMotionDOF:row(iframe+i+FUTURE_SIZE):toQuater(3):copy())
			pose_prediction_10:setVec3(0,mMotionDOF:row(iframe+9+FUTURE_SIZE):toVector3(0):copy())

			local hand_output = vectorn(56)
			python.F("play_mp","test_HandModel",hand_input,hand_output)
			pose_prediction:range(26,54):assign(hand_output:range(0,28))
			pose_prediction:range(58,86):assign(hand_output:range(28,56))
			pose_prediction:setVec3(26,vector3(0,0,0))
			pose_prediction:setVec3(58,vector3(0,0,0))
			if filter__ then 
				mFilter:setCurrPose(pose_prediction)
				pose_prediction = mFilter:getFiltered()
			end
			if use_ik__ then 
				if i==9 then
					pose_prediction = solveIK1(pose_prediction:copy(),td1_10)
				else
					pose_prediction = solveIK1(pose_prediction:copy(),td1)
				end
			end
			pushBack(hand_input,output:row(i))
			if HAND_DRAWER then	
				MOTION_HISTORY_POSE:row(i):range(0,89):assign(pose_prediction:copy())
				MOTION_HISTORY[i+1]:setPoseDOF(MOTION_HISTORY_POSE:row(i))
				mLoader_pre:setPoseDOF(MOTION_HISTORY_POSE:row(i))
				mLoader:setPoseDOF(mMotionDOF:row(iframe+FUTURE_SIZE+i))
			
				mLoader_pre_10:setPoseDOF(MOTION_HISTORY_POSE:row(9))
				mLoader_future_10:setPoseDOF(mMotionDOF:row(iframe+FUTURE_SIZE+9))	
				mSkin_pre_10:setPoseDOF(MOTION_HISTORY_POSE:row(9))
				mSkin_future_10:setPoseDOF(mMotionDOF:row(iframe+FUTURE_SIZE+9))	
				if EVAL then
					eval_prediction_error(i)
				end
			else
				pose_prediction_woH:range(0,26):assign(pose_prediction:range(0,26):copy())
				pose_prediction_woH:range(26,30):assign(pose_prediction:range(54,58):copy())
				pose_prediction_woH:range(30,33):assign(pose_prediction:range(86,89):copy())

				MOTION_HISTORY_POSE_woH:row(i):assign(pose_prediction_woH)
				MOTION_HISTORY_woH[i+1]:setPoseDOF(MOTION_HISTORY_POSE_woH:row(i))
				mLoader_woH_pre:setPoseDOF(MOTION_HISTORY_POSE_woH:row(i))
				mLoader_woH_pre_10:setPoseDOF(MOTION_HISTORY_POSE_woH:row(9))

				local ref_pose_woH=vectorn(33)
				ref_pose_woH:range(0,26):assign(pose_future:range(0,26):copy())
				ref_pose_woH:range(26,30):assign(pose_future:range(54,58):copy())
				ref_pose_woH:range(30,33):assign(pose_future:range(86,89):copy())
				local ref_pose_woH_10=vectorn(33)
				ref_pose_woH_10:range(0,26):assign(pose_future_10:range(0,26):copy())
				ref_pose_woH_10:range(26,30):assign(pose_future_10:range(54,58):copy())
				ref_pose_woH_10:range(30,33):assign(pose_future_10:range(86,89):copy())

				mLoader_woH:setPoseDOF(ref_pose_woH)
				mLoader_woH_future_10:setPoseDOF(ref_pose_woH_10)
				mSkin_woH_pre_10:setPoseDOF(MOTION_HISTORY_POSE_woH:row(9))
				mSkin_woH_future_10:setPoseDOF(ref_pose_woH_10)
				if EVAL then
					eval_prediction_error(i)
				end
			end
		if ATTACH_CAMERA and i==FUTURE_SIZE-1 then
			local COM = pose:toVector3(0)
			local gridposx = math.floor(COM.x/4)
			local gridposz = math.floor(COM.z/4) 
			local bgnode = RE.ogreSceneManager():getSceneNode("BackgroundNode")
			bgnode:setPosition(vector3(gridposx*400,0,gridposz*400))
    		RE.viewpoint().vat:set(pose:toVector3(0).x*100,pose:toVector3(0).y*100,pose:toVector3(0).z*100)
			RE.viewpoint().vpos:set(pose:toVector3(0).x*100+500 , 500 ,pose:toVector3(0).z*100+800 )
			RE.viewpoint():update()
		end
		end
	end
end

function All_tracker_data()
	local data = matrixn(mMotionDOF:numFrames()-1,42)
	for i=0,data:rows()-1 do
		local trackerData = getTrackerData(i,false)
		for j=0,5 do
			if i%500 == 0 and useNoise then 
				local r = random.randint(0,5)
				if r == j then 
					data:row(i):setVec3(7*j,vector3(0,0,0))
					data:row(i):setQuater(7*j+3,quater(1,0,0,0))
				else 
					data:row(i):setVec3(7*j,trackerData.pos(j))
					data:row(i):setQuater(3+7*j,trackerData.ori(j))
				end
			else
				data:row(i):setVec3(7*j,trackerData.pos(j))
				data:row(i):setQuater(3+7*j,trackerData.ori(j))
			end
		end
	end
	---------- change local coord ------------
	local local_data = matrixn(data:rows(),data:cols()+6*6)
	for i=0,data:rows()-2 do 
		local_data:row(i):assign(changeTrackerLocal(data:row(i):range(0,7),data:row(i),data:row(i+1)))
	end
	local final_data = matrixn(data:rows(),local_data:cols()+12)
	final_data:assign(changeTrackerRotation(local_data))
	return final_data 
end




function All_noise_tracker_data()
	local data = matrixn(mMotionDOF:numFrames()-1,54)
	for i=0,data:rows()-1 do
		local trackerData = getTrackerData(i,false)
		local rootTransf = transf(trackerData.ori(0),trackerData.pos(0)):project2D()
		for j=0,5 do
			local pos = rootTransf:inverse()*trackerData.pos(j):copy()
			pos = pos + vector3((math.random()-0.5)*4,(math.random()-0.5)*4,(math.random()-0.5)*4)
			data:row(i):setVec3(9*j,pos)
			local ori = rootTransf.rotation:inverse()*trackerData.ori(j)
			ori.x = ori.x + (math.random()-0.5)*0.08
			ori.y = ori.y + (math.random()-0.5)*0.08
			ori.z = ori.z + (math.random()-0.5)*0.08
			ori:normalize()
			data:row(i):setVec3(3+9*j,ori:getFrameAxis(1))
			data:row(i):setVec3(6+9*j,ori:getFrameAxis(2))
		end
	end
	return data
end

function All_no_noise_tracker_data()
	local data = matrixn(mMotionDOF:numFrames()-1,54)
	for i=0,data:rows()-1 do
		local trackerData = getTrackerData(i,false)
		local rootTransf = transf(trackerData.ori(0),trackerData.pos(0)):project2D()
		for j=0,5 do
			local pos = rootTransf:inverse()*trackerData.pos(j):copy()
			data:row(i):setVec3(9*j,pos)
			local ori = rootTransf.rotation:inverse()*trackerData.ori(j)
			data:row(i):setVec3(3+9*j,ori:getFrameAxis(1))
			data:row(i):setVec3(6+9*j,ori:getFrameAxis(2))
		end
	end
	return data
end


function All_data_frames()
	local data = matrixn(mMotionDOF:numFrames()-1,mMotionDOF:numDOF()-7-56)
	for i=0,data:rows()-1 do 
		data:row(i):assign(getOnlyPose(mMotionDOF:row(i)))
	end
	return data
end
-- 89 --> 26 dof 
function getOnlyPose(pose)
	local vec = vectorn(26)
	vec:range(0,19):assign(pose:range(7,26))
	vec:range(19,23):assign(pose:range(54,58))
	vec:range(23,26):assign(pose:toVector3(86))
	return vec
end

function poseencode(iframe)
	local vec = vectorn(26)
	vec:assign(mMotionDOF:row(iframe):range(7,33))
	return vec
end

function posedecode(vec,prev_pose)
	local output = vectorn(30)
	local quaternion = quater()
	quaternion:setFrameAxesYZ(vec:toVector3(0),vec:toVector3(3))
	quaternion = prev_pose:toTransf(0):project2D().rotation:rotationY()*quaternion
	output:setQuater(0,quaternion)
	output:range(4,30):assign(vec:range(6,32))
	return output
end

--discontinuity check하기--
function getDiscontinuity()
  	local  discont_frames = vectorn()
	for i=0,mMotionDOF:numFrames()-1 do 
		if DOFcontainer.discontinuity(i) == true then 
			discont_frames:pushBack(i)
		end
	end
	--discont_frames:pushBack(12)
	discont_frames:pushBack(mMotionDOF:rows())
	return discont_frames
end

function changePoseLocal(centerCoord,pose)
	local local_pose = vectorn(pose_size)
	local root_transf = transf()
	if lunaType(centerCoord) == 'transf' then 
		root_transf.translation:assign(centerCoord.translation)
		root_transf.rotation:assign(centerCoord.rotation)
	else --vectorn
		root_transf.translation:assign(centerCoord:toVector3(0))
		root_transf.rotation:assign(centerCoord:toQuater(3))
	end
	local transform = transf()
	transform.translation:assign(pose:toVector3(0))
	transform.rotation:assign(pose:toQuater(3))

	transform = root_transf:project2D():inverse()*transform
	local_pose:setVec3(0,transform.translation)
	local_pose:setQuater(3,transform.rotation)
	for i=7,32 do 
		local_pose:set(i,pose(i))
	end
	return local_pose
end


function changeTrackerLocal(centerCoord,t1,t2)
	local local_tracker_data = vectorn()
	local root_transf = transf()
	root_transf.translation:assign(centerCoord:toVector3(0))
	root_transf.rotation:assign(centerCoord:toQuater(3))
	for i=0,5 do 
		local other_joint = transf()
		other_joint.translation:assign(t2:toVector3(7*i))
		other_joint.rotation:assign(t2:toQuater(7*i+3))
		other_joint:assign(root_transf:inverse()*other_joint)
		local changed_data = vectorn(7)
		changed_data:setVec3(0,other_joint.translation)
		changed_data:setQuater(3,other_joint.rotation)
		for j=0,6 do
			local_tracker_data:pushBack(changed_data(j))
		end
		local tracker_linvel = vector3()
		local tracker_angvel = vector3()
		tracker_linvel:linearVelocity(t1:toVector3(7*i),t2:toVector3(7*i))
		tracker_linvel:assign(t1:toTransf(7*i):inverse()*tracker_linvel)
		tracker_angvel:angularVelocity(t1:toQuater(7*i+3),t2:toQuater(7*i+3))
		tracker_angvel:assign(t1:toTransf(7*i):inverse()*tracker_angvel)
		local_tracker_data:pushBack(tracker_linvel.x)
		local_tracker_data:pushBack(tracker_linvel.y)
		local_tracker_data:pushBack(tracker_linvel.z)
		local_tracker_data:pushBack(tracker_angvel.x)
		local_tracker_data:pushBack(tracker_angvel.y)
		local_tracker_data:pushBack(tracker_angvel.z)
	end
	return local_tracker_data
end

function pushBack(matrix,data)
	for i=0,matrix:rows()-2 do
		matrix:row(i):assign(matrix:row(i+1))
	end
	matrix:row(matrix:rows()-1):assign(data)
end

function six_to_nine(x,y)
	local x = x:Normalize()
	local z = vector3()
	local y = y:Normalize()
	z:cross(x,y)
	local rotmat = matrix4()
	rotmat:setColumn(0,x)
	rotmat:setColumn(1,y)
	rotmat:setColumn(2,z)
	local local_quaternion = quater()
	local_quaternion:setRotation(rotmat)
	return local_quaternion
end


function getTrackerData(iframe,draw)
	--root,leftAnkle,rightAnkle,leftElbow,righrElbow,neck
	mLoader:setPoseDOF(mMotionDOF:row(iframe))
	local pos = vector3N(6)
	local ori = quaterN(6)
	for ii, bone in ipairs(config.trackerBones) do
		local j= ii-1
		local transform = mLoader:getBoneByName(bone.name):getFrame()*bone.trackerTransf
		pos(j):assign(transform.translation)
		ori(j):assign(transform.rotation)
		if draw then 
			dbg.draw('Axes',transform,''..ii,100)
		end
	end
	return {pos=pos, ori=ori}
end


function poseEncodeUpdate(iframe)
    local vec = vectorn(update_encode_size)
    local pose1 = mMotionDOF:row(iframe)
    local pose2 = mMotionDOF:row(iframe+1)
	local rotY = pose1:toTransf(0):project2D().rotation:rotationY()
    local dv = rotY:inverse()*(pose2:toVector3(0) - pose1:toVector3(0))
	local rotmat = rotationMatrix(rotY:inverse()*pose2:toQuater(3))
	local Y = rotmat:getColumn(1)
	local Z = rotmat:getColumn(2)
	vec:set(0,dv.x)
	vec:set(1,dv.z)
	vec:set(2,pose2:toVector3(0).y)
	vec:setVec3(3,Y)
	vec:setVec3(6,Z)
	vec:range(9,35):assign(pose2:range(7,33))
	return vec
end

function poseDecodeUpdate(encodeData,prev_pose)
	local pose_output = vectorn(33)
	local prev_root_pos = prev_pose:toVector3(0)
	local prev_root_ori = prev_pose:toTransf(0):project2D().rotation
	local rotY = prev_root_ori:rotationY()
	local newRootPos = prev_root_pos + rotY*vector3(encodeData(0),0,encodeData(1))
	newRootPos.y = encodeData(2)
	local quaternion = quater()
	quaternion:setFrameAxesYZ(encodeData:toVector3(3),encodeData:toVector3(6))
	quaternion = rotY*quaternion
	pose_output:setVec3(0,newRootPos)
	pose_output:setQuater(3,quaternion)
	pose_output:range(7,33):assign(encodeData:range(9,35))
	return pose_output
end


function rotationMatrix(q)
	local mat=matrix4()
	mat:setRotation(q)
	return mat
end


function calc_distance(a,b)
	return math.sqrt((a.x-b.x)^2 +(a.z-b.z)^2 )
end

function normalized(a)
	local length = math.sqrt(a.x^2+a.y^2+a.z^2)
	return a/length
end 

function calc_dot(a,b)
	return (a.x*b.x)+(a.y*b.y)+(a.z*b.z)
end 

function onCallback(w, userData)
	if w:id() == 'use_filter'then 
		filter__=w:checkButtonValue()
		--filter=-1*filter
	elseif w:id()=='use_ik' then
		use_ik__=w:checkButtonValue()
		--use_ik = -1*use_ik		
	elseif w:id()=='draw_hand' then
		if w:checkButtonValue() then
			HAND_DRAWER = true
		else
			HAND_DRAWER = false
		end
		mSkin:setVisible(HAND_DRAWER)
		mSkin_future:setVisible(HAND_DRAWER)
		mSkin_pre_10:setVisible(HAND_DRAWER)
		mSkin_future_10:setVisible(HAND_DRAWER)

		mSkin_woH:setVisible(not HAND_DRAWER)
		mSkin_woH_future:setVisible(not HAND_DRAWER)
		mSkin_woH_pre_10:setVisible(not HAND_DRAWER)
		mSkin_woH_future_10:setVisible(not HAND_DRAWER)
		MOTION_HISTORY[FUTURE_SIZE]:setVisible(HAND_DRAWER)
		MOTION_HISTORY_woH[FUTURE_SIZE]:setVisible(not HAND_DRAWER)

		if DRAW_HISTORY then
			for i=1,table.count(MOTION_HISTORY) do
				MOTION_HISTORY[i]:setVisible(HAND_DRAWER)
				MOTION_HISTORY_woH[i]:setVisible(not HAND_DRAWER)
			end
		end
	elseif w:id()=='draw_tracker' then
		if w:checkButtonValue() then
			draw = true		
		else
			draw = false	
			dbg.eraseAllDrawn()
		end
	elseif w:id()=='draw_between' then
		for i=1,table.count(MOTION_HISTORY)-1 do
			if HAND_DRAWER then
				MOTION_HISTORY[i]:setVisible(w:checkButtonValue())
			else
				MOTION_HISTORY_woH[i]:setVisible(w:checkButtonValue())
			end
		end
		DRAW_HISTORY=w:checkButtonValue()
	elseif w:id()=='pose error calculating' then
		EVAL=w:checkButtonValue()
	elseif w:id()=="attach camera" then
		ATTACH_CAMERA=w:checkButtonValue()
	end 
end
function handleRendererEvent(ev, button, x, y)
end



function changeTrackerRotation(tracker_data)
	local changedTrackerData = matrixn(tracker_data:rows(),tracker_data:cols()+12)
	for i=0,changedTrackerData:rows()-1 do 
		for j=0,5 do 
			changedTrackerData:row(i):setVec3(15*j,tracker_data:row(i):toVector3(13*j))
			changedTrackerData:row(i):setVec3(3+15*j,tracker_data:row(i):toQuater(13*j+3):getFrameAxis(1))
			changedTrackerData:row(i):setVec3(6+15*j,tracker_data:row(i):toQuater(13*j+3):getFrameAxis(2))
			changedTrackerData:row(i):setVec3(9+15*j,tracker_data:row(i):toVector3(13*j+7))
			changedTrackerData:row(i):setVec3(12+15*j,tracker_data:row(i):toVector3(13*j+10))
		end
	end
	return changedTrackerData
end

function getCenterCoord(a,b)
	assert(lunaType(a)=='transf','a is not trnasf')
	assert(lunaType(b)=='transf','b is not transf')
	local output = transf()
	output:interpolate(0.5,a,b)
	return output 
end

function changeTrackerQuater(changed_data)
	-- local tracker_quater = matrixn(changed_data:rows(),42)
	if lunaType(changed_data) == 'vectorn' then 
		before_data = matrixn(1,54)
		before_data:row(0):assign(changed_data)
		tracker_quater = matrixn(1,42)
	else 
		before_data = changed_data
		tracker_quater = matrixn(changed_data:rows(),42)
	end
	changed_data = before_data

	for i=0,tracker_quater:rows()-1 do 
		for j=0,5 do 
			tracker_quater:row(i):setVec3(7*j,changed_data:row(i):toVector3(9*j))
			local quaternion = quater()
			quaternion:setFrameAxesYZ(changed_data:row(i):toVector3(3+9*j),changed_data:row(i):toVector3(6+9*j))
			tracker_quater:row(i):setQuater(3+7*j,quaternion)
		end
	end
	return tracker_quater
end


elapsedTime = 0
function frameMove(fElapsedTime)
end

Queue=LUAclass()
function Queue:__init(n)
	self.n=n
	self.data={}
	self.front=1
end
function Queue:pushBack(data)
	if #self.data==self.n then
		self.data[self.front]=data
		self.front=self.front+1
		if self.front>self.n then
			self.front=1
		end
	else
		table.insert(self.data, data)
	end
end
function Queue:back()
	local f=self.front
	if f==1 then
		return self.data[#self.data]
	else
		return self.data[f-1]
	end
end
-- i in [0, n-1]
function Queue:getElt(i)
	return self.data[math.fmod(i+self.front-1, self.n)+1]
end

function Queue:front()
	return self.data[self.front]
end

OnlineFilter=LUAclass()

function OnlineFilter:__init(loader, pose, filterSize, useMitchellFilter)
	self.filterSize=filterSize
	self.loader=loader
	self.queue=Queue(filterSize)
	if pose then
		self:setCurrPose(pose)
	end
	self.useMitchellFilter=useMitchellFilter or false
end

function OnlineFilter:setCurrPose(pose)
	self.queue:pushBack(pose:copy())
end

function OnlineFilter:getFiltered()
	if #self.queue.data==self.queue.n then

		local sum=vectorn(self.queue:back():size())
		sum:setAllValue(0)
		if true then
			-- use gaussian filter for joint angles
			local arrayV=matrixn(self.queue.n, self.queue:back():size()-7)
			for i=0, arrayV:size()-1 do
				arrayV:row(i):assign(self.queue:getElt(i):slice(7,0))
			end
			local v=math.filterSingle(arrayV, self.queue.n, self.useMitchellFilter)
			sum:slice(7,0):assign(v)
		else

			for i,v in ipairs(self.queue.data) do
				sum:radd(self.queue.data[i])
			end
			sum:rmult(1/self.queue.n)
		end
		if false then
			-- simple renormalization works only when filter size is small
			sum:setQuater(3, sum:toQuater(3):Normalize())
		else
			-- use gaussian filter for root pos and ori.
			local arrayQ=quaterN(self.queue.n)
			local arrayV=vector3N(self.queue.n)
			for i=0, arrayQ:size()-1 do
				arrayQ(i):assign(self.queue:getElt(i):toQuater(3))
				arrayV(i):assign(self.queue:getElt(i):toVector3(0))
			end
			--math.filterQuat(arrayQ:matView(), self.queue.n)
			--sum:setQuater(3, arrayQ(math.floor(self.queue.n/2)))
			local q=math.filterQuatSingle(arrayQ:matView(), self.queue.n, self.useMitchellFilter)
			local v=math.filterSingle(arrayV:matView(), self.queue.n, self.useMitchellFilter)
			sum:setVec3(0, v:toVector3())
			sum:setQuater(3, q:toQuater())
		end
		return sum
	else
		return self.queue:back()
	end
end

function createIKsolver1()
	mEffectors=MotionUtil.Effectors()
	mEffectors:resize(4)
	local kneeIndex=intvectorn(4)
	local axis=CT.vec(1,1,-1,1)
	for i=2,5 do
		local binfo=config.trackerBones[i]
		kneeIndex:set(i-2, binfo.idx)
		local eebone=mLoader:bone(binfo.idx+1)
		local localpos=vector3(0,0,0)
		mEffectors(i-2):init(eebone,localpos)
	end

	mLoader:updateInitialBone()
	mInitialPose=mLoader:getPoseDOF()
	mSolver=LimbIKsolver(mLoader.dofInfo,mEffectors, kneeIndex, axis)
end

function solveIK1(ref_pose,td)
	local poseInout=ref_pose:copy()

	local conpos=vector3N(4)
	local conori=quaterN(4)
	-- root*trackerOffset=trackerData
	mLoader:setPoseDOF(ref_pose)
	local newRootTF=td:toTransf(0)*config.trackerBones[1].trackerTransf:inverse()
	for i=0,3 do
		-- parentGlobal*selfOffset=selfG
		-- parentG*trackerOffset=trackerData
		-- selfG*selfOffset:inverse()*trackerOffset=trackerData
		--> selfG=trackerData*trackerOffset:inverse()*selfOffset
		local eeTF=td:toTransf(7*(i+1))*config.trackerBones[i+2].trackerTransf:inverse()*mEffectors(i).bone:getOffsetTransform()

		conpos(i):assign(eeTF.translation)
		--conori(i):assign(eeTF.rotation)
		if i == 0 then 
			conori(i):assign(eeTF.rotation*mLoader:getBoneByName("LeftAnkle"):getLocalFrame().rotation)
		elseif i==1 then
			conori(i):assign(eeTF.rotation*mLoader:getBoneByName("RightAnkle"):getLocalFrame().rotation)
		elseif i==2 then
			conori(i):assign(eeTF.rotation*mLoader:getBoneByName("LeftElbow"):getLocalFrame().rotation)
		elseif i==3 then
			conori(i):assign(eeTF.rotation*mLoader:getBoneByName("RightElbow"):getLocalFrame().rotation)
		end
	end
	mSolver:IKsolve3(poseInout, newRootTF, conpos, conori, CT.vec(1,1,1,1))

	return poseInout
end

function eval_prediction_error(i_frame)
	local num_bone=0;
	if HAND_DRAWER then
		num_bone=mLoader:numBone()-1
	else
		num_bone=mLoader_woH:numBone()-1
	end
    local mpjre = 0.0
    local mpjre_radian = 0.0
	local mpjpe = 0.0
	local root_transf
    for i=1, num_bone do
		if HAND_DRAWER then
			local tmpGTBoneOri = mLoader:getBoneByTreeIndex(i):getLocalFrame().rotation
			local tmpOutBoneOri = mLoader_pre:getBoneByTreeIndex(i):getLocalFrame().rotation
			local tmpGTBonePos = mLoader:getBoneByTreeIndex(i):getFrame().translation
			local tmpOutBonePos = mLoader_pre:getBoneByTreeIndex(i):getFrame().translation
			local diff = quater()
			diff:difference(tmpGTBoneOri, tmpOutBoneOri)
			mpjre = mpjre + diff:rotationAngle()
			mpjpe = mpjpe + (tmpGTBonePos-tmpOutBonePos):length()
		else
			local tmpGTBoneOri = mLoader_woH:getBoneByTreeIndex(i):getLocalFrame().rotation
			local tmpOutBoneOri = mLoader_woH_pre:getBoneByTreeIndex(i):getLocalFrame().rotation
			local tmpGTBonePos = mLoader_woH:getBoneByTreeIndex(i):getFrame().translation
			local tmpOutBonePos = mLoader_woH_pre:getBoneByTreeIndex(i):getFrame().translation
			local diff = quater()
			diff:difference(tmpGTBoneOri, tmpOutBoneOri)
			mpjre = mpjre + diff:rotationAngle()
			mpjpe = mpjpe + (tmpGTBonePos-tmpOutBonePos):length()
		end
    end

    mpjre = mpjre / num_bone
    mpjpe = mpjpe / num_bone
	
	mpjre_radian=mpjre
	mpjre = mpjre * 180 / math.pi -- rad to deg
	if i_frame==FUTURE_SIZE-1 then
		print('30 mjpe',mpjpe)
		print('30 mpre(degree)',mpjre)
		print('30 mpre(radian)',mpjre_radian)
		print('--------------------------')
		local mpjre_10, mpjre_radian_10, mpjpe_10=eval_prediction_error_10()
		print('10 mpre(degree)',mpjre_10)
		print('10 mpre(radian)',mpjre_radian_10)
		print('10 mjpe',mpjpe_10)
		print('--------------------------')
		print('--------------------------')
		RE.output2('10 mpre(degree)',mpjre_10)
		RE.output2('10 mpre(radian)',mpjre_radian_10)
		RE.output2('10 mjpe',mpjpe_10)
		RE.output2('30 mpre(degree)',mpjre)
		RE.output2('30 mpre(radian)',mpjre_radian)
		RE.output2('30 mjpe',mpjpe)
	end
end

function eval_prediction_error_10()
	local num_bone=0;
	if HAND_DRAWER then
		num_bone=mLoader:numBone()-1
	else
		num_bone=mLoader_woH:numBone()-1
	end
    local mpjre = 0.0
    local mpjre_radian = 0.0
	local mpjpe = 0.0
	local root_transf
    for i=1, num_bone do
		if HAND_DRAWER then
			local tmpGTBoneOri = mLoader_future_10:getBoneByTreeIndex(i):getLocalFrame().rotation
			local tmpOutBoneOri = mLoader_pre_10:getBoneByTreeIndex(i):getLocalFrame().rotation
			local tmpGTBonePos = mLoader_future_10:getBoneByTreeIndex(i):getFrame().translation
			local tmpOutBonePos = mLoader_pre_10:getBoneByTreeIndex(i):getFrame().translation
			local diff = quater()
			diff:difference(tmpGTBoneOri, tmpOutBoneOri)
			mpjre = mpjre + diff:rotationAngle()
			mpjpe = mpjpe + (tmpGTBonePos-tmpOutBonePos):length()
		else
			local tmpGTBoneOri = mLoader_woH_future_10:getBoneByTreeIndex(i):getLocalFrame().rotation
			local tmpOutBoneOri = mLoader_woH_pre_10:getBoneByTreeIndex(i):getLocalFrame().rotation
			local tmpGTBonePos = mLoader_woH_future_10:getBoneByTreeIndex(i):getFrame().translation
			local tmpOutBonePos = mLoader_woH_pre_10:getBoneByTreeIndex(i):getFrame().translation
			local diff = quater()
			diff:difference(tmpGTBoneOri, tmpOutBoneOri)
			mpjre = mpjre + diff:rotationAngle()
			mpjpe = mpjpe + (tmpGTBonePos-tmpOutBonePos):length()
		end
    end

    mpjre = mpjre / num_bone
    mpjpe = mpjpe / num_bone
	
	mpjre_radian=mpjre
	mpjre = mpjre * 180 / math.pi -- rad to deg
	return mpjre,mpjre_radian,mpjpe
end

