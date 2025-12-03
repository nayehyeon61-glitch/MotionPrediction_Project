require("config")
package.projectPath='../Samples/classification/'
package.path=package.path..";../Samples/classification/lua/?.lua" --;"..package.path
require("common")
require("module")
fc=require('retargetting/module/footSkateCleanup')
require('subRoutines/Timeline')
require("RigidBodyWin/subRoutines/Constraints")
require("RigidBodyWin/subRoutines/CollisionChecker")
update_encode_size = 35 pose_size = 33

-- type1 : 35, type2 : 57

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
use_vae = true
motionDataType = 1 -- 1 : dof 2 : quater
ATTACH_CAMERA=true
EVAL=false
ALIGN=false
function ctor()
	mEventReceiver = EVR()
	if gui then 
		local osm =RE.ogreSceneManager()
		osm:setFogNone()
	end
	mLoader1=MainLib.VRMLloader ("../Resource/motion/prediction/hyunwoo_lowdof_T_boxfoot.wrl")
	mLoader_predictied=MainLib.VRMLloader ("../Resource/motion/prediction/hyunwoo_lowdof_T_boxfoot.wrl")
	mLoader_future=MainLib.VRMLloader ("../Resource/motion/prediction/hyunwoo_lowdof_T_boxfoot.wrl")
	mLoader2=MainLib.VRMLloader ("../Resource/motion/prediction/hyunwoo_lowdof_T_boxfoot_parable.wrl")
	DOFcontainer = MotionDOFcontainer(mLoader1.dofInfo , "../Resource/motion/prediction/hyunwoo_lowdof_T_locomotion_hl.dof")
	--DOFcontainer = MotionDOFcontainer(mLoader1.dofInfo , "../Resource/motion/prediction/Stitch_walk.dof")
	--DOFcontainer = MotionDOFcontainer(mLoader1.dofInfo , "../Resource/motion/prediction/stitchMotion_ubi3_hyunwoo.dof")

	mMotionDOF = DOFcontainer.mot
	print("motion frames : ",mMotionDOF:numFrames())

	mSkin1 = RE.createVRMLskin(mLoader1 , false)
	mSkin1:scale(config.skinScale,config.skinScale,config.skinScale)
	mSkin1:setMaterial('red_transparent')
	mSkin1:setVisible(true)
	--mSkin1:setTranslation(-100,0,0)

	mSkin2 = RE.createVRMLskin(mLoader1 , false)
	mSkin2:scale(config.skinScale,config.skinScale,config.skinScale)
	mSkin2:setVisible(true)
	mSkin2:setMaterial("blue_transparent")
	---mSkin2:setTranslation(-100,0,0)

	mLoader2:printHierarchy()
	this:create('Check_Button','draw_tracker','draw_tracker')
	this:create('Check_Button','current motion visualization','current motion visualization')
	this:widget(0):checkButtonValue(true)
	this:create('Check_Button','attach camera','attach camera')
	this:widget(0):checkButtonValue(ATTACH_CAMERA)
	this:create('Check_Button','pose error calculating','pose error calculating')
	this:widget(0):checkButtonValue(EVAL)
	this:create('Check_Button','align current','align current')
	this:widget(0):checkButtonValue(ALIGN)

	this:updateLayout()
	latent = vectorn(100)
	latent:setAllValue(0)
	iframe = 0
	

	-- local filterSize=3*2+1 -- has to be an odd number
	-- local initialPose=vectorn()
	-- mLoader2:updateInitialBone()
	-- mLoader2:getPoseDOF(initialPose)
	-- mFilter=OnlineFilter(mLoader2, initialPose, filterSize, true)


	local initialPose=vectorn()
	mLoader1:updateInitialBone()
	mLoader1:getPoseDOF(initialPose)
	mFilter=OnlineFilter(mLoader1, initialPose, 15 , true)

	--tracker_data = All_tracker_data()

	prev_pose = vectorn(33)
	prev_pose:setAllValue(0)
	encodeBuffer = matrixn(5,update_encode_size)
	encodeBuffer:setAllValue(0)
	mTimeline=Timeline('timeline', mMotionDOF:rows()-1, config.rendering_step)
	hand_input = matrixn(3,26)

	RE.viewpoint().vpos:set(200, 400,400)
	RE.viewpoint().vat:set(200, 0, -300)
	RE.viewpoint():update()
	for i, bone in ipairs(config.trackerBones) do
		local bone2=mLoader2:getBoneByName(bone.name)
		bone.idx=bone2:treeIndex()
		bone.startT=mLoader2.dofInfo:startT(bone2:treeIndex())

		bone.idx1=mLoader1:getBoneByName(bone.name):treeIndex()
	end
	createIKsolver1(mLoader1)
	filter = -1
	use_ik = -1

	prediction_frame = 10
	mSkin_ = {}
	for i=1,prediction_frame do 
		local skin = RE.createVRMLskin(mLoader1,false)	
		skin:setMaterial("lightgrey_transparent")
		skin:scale(100,100,100)
		skin:setVisible(false)
		if i == 10 then 
			skin:setVisible(true)
			skin:setMaterial("black")
		end
		table.insert(mSkin_,skin)

	end
	prev_pose = vectorn(mMotionDOF:numDOF())
	prev_pose:assign(mMotionDOF:row(0))
	encodeBuffer = matrixn(2,update_encode_size)
	poseBuffer = matrixn(9,update_encode_size)

	--LAFAN_to_hyunwoo()

	-- 0827
	local pose, quat = getLocalTransfFromLoader(1,false)
	-- 0827 end

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
	local pose = vectorn(mMotionDOF:numDOF())
	if iframe < 2 then 
		pose:assign(mMotionDOF:row(iframe))
		mSkin1:setPoseDOF(pose)
		mSkin2:setPoseDOF(pose)
		pushBack(encodeBuffer,poseEncodeUpdate(iframe))
		for i=0,8 do 
			pushBack(poseBuffer,poseEncodeUpdate(iframe))
		end
	else 
		if use_vae then 
			local td = getTrackerData(iframe+10,draw)
			local td1 = vectorn(42)
			for i=0,5 do 
				td1:setVec3(7*i,td.pos(i))
				td1:setQuater(7*i+3,td.ori(i))
			end
			prev_pose:assign(mMotionDOF:row(iframe))
			pushBack(encodeBuffer,poseEncodeUpdate(iframe))
			pushBack(encodeBuffer,poseEncodeUpdate(iframe))
			mSkin1:setPoseDOF(mMotionDOF:row(iframe))
			for i=1,prediction_frame  do 
				local encodeData = vectorn(update_encode_size)
				local latent = vectorn(10)
				latent:setAllValue(0)
				--use reference (i+10)
				--python.F('play_mp','test_vae',latent,poseEncodeUpdate(iframe+i),encodeData)
				--recursive
				python.F('play_mp','test_vqvae',encodeBuffer:row(0),encodeBuffer:row(1),encodeData)
				pushBack(encodeBuffer,encodeData)
				pose:assign(poseDecodeUpdate(encodeData,prev_pose))
				pose:range(0,7):assign(mMotionDOF:row(iframe+i):range(0,7))
				mFilter:setCurrPose(pose)
				local renderPoseFiltered = mFilter:getFiltered()
				--renderPoseFiltered = solveIK1(renderPoseFiltered, td1)
				if not ALIGN then
					renderPoseFiltered:setVec3(0,mMotionDOF:row(iframe+prediction_frame):toVector3(0))
				else
					renderPoseFiltered:setVec3(0,mMotionDOF:row(iframe):toVector3(0))
				end
				mSkin_[i]:setPoseDOF(renderPoseFiltered)
				--mSkin_[i]:setTranslation(-100,0,0)
				prev_pose:assign(renderPoseFiltered)
				mLoader_predictied:setPoseDOF(renderPoseFiltered)
				if ATTACH_CAMERA and i==prediction_frame then
					local COM = renderPoseFiltered:toVector3(0)
					local gridposx = math.floor(COM.x/4)
					local gridposz = math.floor(COM.z/4) 
					local bgnode = RE.ogreSceneManager():getSceneNode("BackgroundNode")
					bgnode:setPosition(vector3(gridposx*400,0,gridposz*400))
	    			RE.viewpoint().vat:set(renderPoseFiltered:toVector3(0).x*100,renderPoseFiltered:toVector3(0).y*100,renderPoseFiltered:toVector3(0).z*100)
					RE.viewpoint().vpos:set(renderPoseFiltered:toVector3(0).x*100+500 , 500 ,renderPoseFiltered:toVector3(0).z*100+800 )
					RE.viewpoint():update()
				end
			end
			mSkin2:setPoseDOF(mMotionDOF:row(iframe+prediction_frame))
			mLoader_future:setPoseDOF(mMotionDOF:row(iframe+prediction_frame))
			if EVAL then	
				eval_prediction_error(mLoader_predictied,mLoader_future)
			end
		else 
			mSkin1:setPoseDOF(mMotionDOF:row(iframe))
			prev_pose:assign(mMotionDOF:row(iframe))
			local output = vectorn(35)
			local latent = vectorn(100)
			latent:setAllValue(0)
			if iframe > 10 then 
				for i=0,poseBuffer:rows()-1 do 
					pushBack(poseBuffer,poseEncodeUpdate(iframe-(10-i)))
				end
			end
			python.F('play_mp','test_predictionvae',latent,poseBuffer,output)
			local rendering_frame = 0
			local pose = vectorn(33)
			pose = poseDecodeUpdate(output,prev_pose)
			--pose:range(0,7):assign(mMotionDOF:row(iframe+11):range(0,7))
			--pose:range(7,33):assign(output)
			pose:setVec3(0,mMotionDOF:row(iframe+10):toVector3(0))
			mSkin_[11]:setPoseDOF(pose)
			mSkin_[11]:setTranslation(-100,0,0)
			--mSkin_[11]:setTranslation(-100,0,0)
			pushBack(poseBuffer,poseEncodeUpdate(iframe))
			mSkin2:setPoseDOF(mMotionDOF:row(iframe+10))
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

--0827
function All_data_frames()
	local data = matrixn(mMotionDOF:numFrames()-1,update_encode_size)
	for i=0,data:rows()-1 do
		data:row(i):assign(poseEncodeUpdate(i))
		--data:row(i):assign(poseEncodeUpdateQuater(i))
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
	discontinuity = vectorn()
	local ndof=mMotionDOF:matView():cols()
	local mot=mMotionDOF:matView()
	local isSimilar=function(d1, d2)
		return math.abs(d1-d2)<math.rad(40)
	end
	for frame=1, mot:rows()-1 do
		for i=7, ndof-1 do
			if not(isSimilar(mot(frame, i), mot(frame-1, i))) then
				discontinuity:pushBack(frame)
				--print("frame "..frame.." has some errors!!!")
				break
			end
		end
	end
	discontinuity:pushBack(mMotionDOF:numFrames())
	print(discontinuity)
	
	return discontinuity
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
	mLoader1:setPoseDOF(mMotionDOF:row(iframe))
	local pos = vector3N(6)
	local ori = quaterN(6)
	for ii, bone in ipairs(config.trackerBones) do
		local j= ii-1
		local transform = mLoader1:getBoneByName(bone.name):getFrame()*bone.trackerTransf
		pos(j):assign(transform.translation)
		ori(j):assign(transform.rotation)
		if draw then 
			dbg.draw('Axes',transform,''..ii,100)
		end
	end
	return {pos=pos, ori=ori}
end






-- Position Update code (to change with type)


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


function poseEncodeUpdateQuater(iframe)
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
	local d = getLocalTransfFromLoader(iframe, false)
	for i = 1, 12 do
		vec:setQuater(5 +i*4, d.ori(i-1))
	end
	return vec
end

function getLocalTransfFromLoader(iframe, draw)
	mLoader1:setPoseDOF(mMotionDOF:row(iframe))
	local bonenum = mLoader1:numBone()
	local pos = vector3N(bonenum)
	local ori = quaterN(bonenum)
	local rootbonetransf = mLoader1:getBoneByTreeIndex(1):getFrame()

	for i = 2, bonenum-1 do
		local transform = rootbonetransf:inverse()*mLoader1:getBoneByTreeIndex(i):getFrame()
		pos(i-1):assign(transform.translation)
		ori(i-1):assign(transform.rotation)
		if draw then 
			dbg.draw('Axes',transform,''..i-1,100)
		end
	end
	return {pos=pos, ori=ori}
end

function poseDecodeUpdateQuater(encodeData,prev_pose)
	local pose_output = vectorn(33)
	pose_output:setAllValue(0)
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
	
	local t = 7
	for i = 2, 12 do
		local q = encodeData:toQuater(5+4*i)
		local bone = mLoader1:getBoneByTreeIndex(i)
		local rochannel = bone:getRotationalChannels()
		local rot = q:getRotation(rochannel)
		local vrot = vectorn(3)
		vrot:assign(rot)
		local nt = t + string.len(rochannel)

		pose_output:range(t, nt):assign(vrot:range(0,string.len(rochannel)))
		t = nt
		
	end
	--pose_output:range(7,33):assign(encodeData:range(9,65))
	return pose_output
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
	if w:id()=='draw_tracker' then
		draw = w:checkButtonValue()	
		dbg.eraseAllDrawn()
	elseif w:id() == "current motion visualization" then 
		mSkin1:setVisible(w:checkButtonValue())
	elseif w:id() == "attach camera" then 
		ATTACH_CAMERA=w:checkButtonValue()
	elseif w:id() == "pose error calculating" then 
		EVAL=w:checkButtonValue()
	elseif w:id() == "align current" then 
		ALIGN=w:checkButtonValue()
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

function createIKsolver1(loader)
	mEffectors=MotionUtil.Effectors()
	mEffectors:resize(2)
	local kneeIndex=intvectorn(2)
	local axis=CT.vec(1,1,-1,1)
	for i=2,3 do
		local binfo=config.trackerBones[i]

		kneeIndex:set(i-2, binfo.idx)
		local eebone=loader:bone(binfo.idx+1)
		local localpos=vector3(0,0,0)
		mEffectors(i-2):init(eebone,localpos)
	end

	loader:updateInitialBone()
	mInitialPose=loader:getPoseDOF()
	mSolver=LimbIKsolver(loader.dofInfo,mEffectors, kneeIndex, axis)
end

function solveIK1(ref_pose,td)
	local poseInout=ref_pose:copy()

	local conpos=vector3N(2)
	local conori=quaterN(2)
	-- root*trackerOffset=trackerData
	mLoader1:setPoseDOF(ref_pose)
	local newRootTF=td:toTransf(0)*config.trackerBones[1].trackerTransf:inverse()
	for i=0,1 do
		-- parentGlobal*selfOffset=selfG
		-- parentG*trackerOffset=trackerData
		-- selfG*selfOffset:inverse()*trackerOffset=trackerData
		--> selfG=trackerData*trackerOffset:inverse()*selfOffset
		local eeTF=td:toTransf(7*(i+1))*config.trackerBones[i+2].trackerTransf:inverse()*mEffectors(i).bone:getOffsetTransform()
		conpos(i):assign(eeTF.translation)
		--conori(i):assign(eeTF.rotation)
		if i == 0 then 
			conori(i):assign(eeTF.rotation*mLoader1:getBoneByName("LeftAnkle"):getLocalFrame().rotation)
		else 
			conori(i):assign(eeTF.rotation*mLoader1:getBoneByName("RightAnkle"):getLocalFrame().rotation)
		end
	end

	mSolver:IKsolve3(poseInout, newRootTF, conpos, conori, CT.vec(1,1,-1,1))

	--print(ref_pose-poseInout)
	
	return poseInout
end

function eval_prediction_error(loader_sim,loader_ref)
	local num_bone=0;
	num_bone=loader_sim:numBone()-1
	assert(loader_sim:numBone()==loader_ref:numBone())

    local mpjre = 0.0
    local mpjre_radian = 0.0
	local mpjpe = 0.0
	local root_transf
    for i=1, num_bone do
		local tmpGTBoneOri = loader_sim:getBoneByTreeIndex(i):getLocalFrame().rotation
		local tmpOutBoneOri = loader_ref:getBoneByTreeIndex(i):getLocalFrame().rotation
		local tmpGTBonePos = loader_sim:getBoneByTreeIndex(i):getFrame().translation
		local tmpOutBonePos = loader_ref:getBoneByTreeIndex(i):getFrame().translation
		local diff = quater()
		diff:difference(tmpGTBoneOri, tmpOutBoneOri)
		mpjre = mpjre + diff:rotationAngle()
		mpjpe = mpjpe + (tmpGTBonePos-tmpOutBonePos):length()
    end

    mpjre = mpjre / num_bone
    mpjpe = mpjpe / num_bone
	
	mpjre_radian=mpjre
	mpjre = mpjre * 180 / math.pi -- rad to deg
	--print('frame error')
	RE.output2('mjpe(m unit)',mpjpe)
	RE.output2('mpre(degree)',mpjre)
	RE.output2('mpre(redian)',mpjre_radian)
	print('mjpe(m unit)',mpjpe)
	print('mpre(degree)',mpjre)
	print('mpre(redian)',mpjre_radian)
	print('--------------------------')
end

function LAFAN_to_hyunwoo()
	local mLoader_LAFAN=MainLib.VRMLloader ("../Resource/motion/prediction/hyunwoo_lowdof_T_boxfoot_parable.wrl")
	local mLoader_hyunwoo=MainLib.VRMLloader ("../Resource/motion/prediction/hyunwoo_lowdof_T_boxfoot.wrl")
	local LAFAN_DOFcontainer = MotionDOFcontainer(mLoader_LAFAN.dofInfo , "../Resource/motion/prediction/stitchMotion_ubi3.dof")
	local mMotionDOF_retarget=MotionDOF(mLoader_hyunwoo.dofInfo)

	print('-----------------------------------')
	print('-----------------------------------')
	print('-----------------------------------')
	mLoader_hyunwoo:printHierarchy()
	print('-----------------------------------')
	mLoader_LAFAN:printHierarchy()
	print('-----------------------------------')
	print('-----------------------------------')
	print('-----------------------------------')

	mMotionDOF_retarget:resize(LAFAN_DOFcontainer.mot:rows())
	local retarget_={
		{{0,7},{0,7}}, --root
		{{7,10},{7,10}}, --left hip
		{{10,11},{10,11}}, --left knee
		{{11,13},{11,13}}, --left ankle
		{{13,16},{13,16}}, --right hip
		{{16,17},{16,17}}, --right knee 
		{{17,19},{17,19}}, --right ankle
		{{19,22},{19,22}}, --chest
		{{22,25},{22,25}}, --left shoulder
		{{25,26},{25,26}}, --left elbow
		{{26,29},{54,57}}, --right shoulder
		{{29,30},{57,58}}, --right elbow
		{{30,33},{86,89}}, 
	}

	for i=0, mMotionDOF_retarget:rows()-1 do
		for j=1, table.count(retarget_) do  
			mMotionDOF_retarget:row(i):range(retarget_[j][1][1],retarget_[j][1][2]):assign(LAFAN_DOFcontainer.mot:row(i):range(retarget_[j][2][1],retarget_[j][2][2]))
			print(i,' frame retarget..')
		end
	end
	mMotionDOF_retarget:exportMot(package.resourcePath.."prediction/gangrae.dof")
	print('motion file exported!!')
end
