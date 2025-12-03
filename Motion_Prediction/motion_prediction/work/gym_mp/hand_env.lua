require("config")
package.projectPath='../Samples/classification/'
package.path=package.path..";../Samples/classification/lua/?.lua" --;"..package.path
require("common")
require("module")
-- require('subRoutines/classifyLib')
fc=require('retargetting/module/footSkateCleanup')
require('subRoutines/Timeline')
require("RigidBodyWin/subRoutines/Constraints")
require("RigidBodyWin/subRoutines/CollisionChecker")
framerate = 30
-- mMotionDOF =  33개의 모션 DOF값 으로 이루어짐 
fc=require('retargetting/module/footSkateCleanup')
rendering_step=1/30
gui = true
config = {"../Resource/motion/prediction/hyunwoo_lowdof_T_boxfoot.wrl",
		  "../Resource/motion/prediction/hyunwoo_lowdof_T_locomotion_hl.dof",
	trackerBones ={
		{name='Hips',  trackerTransf=transf(quater(1, 0,0,0), vector3(0,0.1,0.1))},
		{name='LeftKnee',  trackerTransf=transf(quater(1, 0,0,0), vector3(0,-0.35,0.07))},
		{name='RightKnee',   trackerTransf=transf(quater(1, 0,0,0), vector3(0,-0.35,0.07))},
		{name='LeftElbow',    trackerTransf=transf(quater(1, 0,0,0), vector3(0.2,0.05,0))},
		{name='RightElbow',   trackerTransf=transf(quater(1, 0,0,0), vector3(-0.2,0.05,0))},
		{name='Neck',    trackerTransf=transf(quater(1, 0,0,0), vector3(0,0.15,0.1))},
	},
skinScale = 100,
}

function ctor()
	mEventReceiver = EVR()
	if isMainloopInLua then 
		gui = true
	end
	if gui then 
		local osm =RE.ogreSceneManager()
		osm:setFogNone()
	end
	global = false
	RE.viewpoint():update() this:create('Button','other_latentvector','other_latentvector')
	RE.viewpoint():update() this:create('Button','local','local')
	this:updateLayout()
	mLoader=MainLib.VRMLloader ("../Resource/motion/prediction/hyunwoo_lowdof_T_boxfoot_parable.wrl")
	DOFcontainer = MotionDOFcontainer(mLoader.dofInfo , "../Resource/motion/prediction/stitchMotion_mixamo4.dof")
	mMotionDOF = DOFcontainer.mot
	mSkin1 = RE.createVRMLskin(mLoader,false)
	mSkin1:setMaterial('lightgrey_transparent')
	mSkin1:scale(config.skinScale,config.skinScale,config.skinScale)
	iframe = 0
	encode_size = 35
	prev_pose = vectorn(33)
	prev_pose:assign(mMotionDOF:row(iframe))
end

function rotationMatrix(q)
	local mat=matrix4()
	mat:setRotation(q)
	return mat
end

function getDiscontinuity()
	local  discont_frames = vectorn()
	discont_frames:pushBack(0)
	local isContinuous = function(v1,v2)
		local vel = vector3()
		vel:linearVelocity(v1,v2)
		return vel:length() > 0.005
	end
	for i=0,mMotionDOF:numFrames()-2 do 
		local before = mMotionDOF:row(i):toVector3(0)
		local after = mMotionDOF:row(i+1):toVector3(0)
		if isContinuous(before,after) then 
			discont_frames:pushBack(i)
		end
	end
	discont_frames:pushBack(mMotionDOF:numFrames())
	return discont_frames
end


-- root,hand 빼고 pose 만들어있는 데이터 --
function All_data_frames()
	--local data = matrixn(mMotionDOF:numFrames()-1,mMotionDOF:numDOF()-56-7)
	local data = matrixn(mMotionDOF:numFrames()-1,26)
	for i=0,data:rows()-1 do 
		data:row(i):assign(getOnlyPose(mMotionDOF:row(i)))
		--data:row(i):assign(extractFeatures(mLoader,mMotionDOF:row(i)))
	end
	return data
end

function All_hand_data_frames()
	local data = matrixn(mMotionDOF:numFrames()-1,56)
	for i=0,data:rows()-1 do 
		data:row(i):range(0,28):assign(mMotionDOF:row(i):range(26,54))
		data:row(i):range(28,56):assign(mMotionDOF:row(i):range(58,86))
	end
	return data
end

function All_tracker_data()
	local data = matrixn(mMotionDOF:numFrames()-1,42)
	for i=0,data:rows()-1 do
		for j=0,5 do
			data:row(i):setVec3(7*j,getTrackerData(i).pos(j))
			data:row(i):setQuater(3+7*j,getTrackerData(i).ori(j))
		end
	end
	---------- change local coord ------------
	local local_data = matrixn(data:rows(),data:cols()+6*6)
	for i=0,data:rows()-2 do 
		local_data:row(i):assign(changeTrackerLocal(data:row(i+1):range(0,7),data:row(i),data:row(i+1)))
	end
	local final_data = matrixn(data:rows(),local_data:cols()+12)
	final_data:assign(changeTrackerRotation(local_data))
	return final_data 
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
		tracker_linvel:assign(root_transf:inverse()*tracker_linvel)
		tracker_angvel:angularVelocity(t1:toQuater(7*i+3),t2:toQuater(7*i+3))
		tracker_angvel:assign(root_transf:inverse()*tracker_angvel)
		local_tracker_data:pushBack(tracker_linvel.x)
		local_tracker_data:pushBack(tracker_linvel.y)
		local_tracker_data:pushBack(tracker_linvel.z)
		local_tracker_data:pushBack(tracker_angvel.x)
		local_tracker_data:pushBack(tracker_angvel.y)
		local_tracker_data:pushBack(tracker_angvel.z)
	end
	return local_tracker_data
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



function getTrackerData(iframe,draw)
	--root,leftAnkle,rightAnkle,leftElbow,righrElbow,neck
	mLoader:setPoseDOF(mMotionDOF:row(iframe))
	local pos = vector3N(6)
	local ori = quaterN(6)
	local j = 0
	for i=1,mLoader:numBone()-1 do 
		local transform = transf()
		transform = mLoader:getBoneByTreeIndex(i):getFrame()
		if i==1 or i==4 or i==7 or i==11 or i==29 or i==45 then 
			pos(j):assign(transform.translation)
			ori(j):assign(transform.rotation)
			if draw then 
				dbg.draw('Axes',transform,''..i,100)
			end
			j = j+1
		end
	end
	return {pos=pos, ori=ori}
end


function getLocalHand(iframe)
	local vec = vectorn(18)
	mLoader:setPoseDOF(mMotionDOF:row(iframe))
	local leftHand = mLoader:getBoneByTreeIndex(11):getFrame()
	local rightHand = mLoader:getBoneByTreeIndex(29):getFrame()
	local leftSoulder = mLoader:getBoneByTreeIndex(9):getFrame()
	local rightSoulder = mLoader:getBoneByTreeIndex(27):getFrame()
	leftHand:assign(leftSoulder:inverse()*leftHand)
	rightHand:assign(rightSoulder:inverse()*rightHand)
	vec:setVec3(0,leftHand.translation)
	vec:setVec3(3,leftHand.rotation*vector3(0,1,0))
	vec:setVec3(6,leftHand.rotation*vector3(0,0,1))
	vec:setVec3(9,rightHand.translation)
	vec:setVec3(12,rightHand.rotation*vector3(0,1,0))
	vec:setVec3(15,rightHand.rotation*vector3(0,0,1))
	return vec
end


function getOnlyPose(pose)
	local vec = vectorn(26)
	vec:range(0,19):assign(pose:range(7,26))
	vec:range(19,23):assign(pose:range(54,58))
	vec:range(23,26):assign(pose:toVector3(86))
	return vec
end


function EVR:onFrameChanged(win, iframe)
end

function dtor()
end

function onCallback(w, userData)
	if w:id() == 'other_latentvector'then 
		-- python.F('play_hyunseokEnv','make_latent_vector',latent)
		-- print(latent)
		global = true
	end 
	if w:id()=='local' then 
		global = false
	end

end


function changePose2Local(centerCoord,pose_data)
	local local_pose_data = vectorn(33)
	local project_center_coord = transf()
	if lunaType(centerCoord) == 'transf' then 
		project_center_coord.translation:assign(centerCoord.translation)
		project_center_coord.rotation:assign(centerCoord.rotation)
	else --vectorn
		project_center_coord.translation:assign(centerCoord:toVector3(0))
		project_center_coord.rotation:assign(centerCoord:toQuater(3))
	end
	local global_root = transf()
	global_root.translation:assign(pose_data:toVector3(0))
	global_root.rotation:assign(pose_data:toQuater(3))
	global_root:assign(project_center_coord:project2D():inverse()*global_root)
	local_pose_data:setVec3(0,global_root.translation)
	local_pose_data:setQuater(3,global_root.rotation)
	local_pose_data:range(7,33):assign(pose_data:range(7,33))
	return local_pose_data
end


function changeTracker2Local(root_tracker,tracker_data)
	local local_tracker_data = vectorn(42)
	local project_center_coord = transf()
	if lunaType(root_tracker) == 'transf' then 
		project_center_coord.translation:assign(root_tracker.translation)
		project_center_coord.rotation:assign(root_tracker.rotation)
	else --vectorn
		project_center_coord.translation:assign(root_tracker:toVector3(0))
		project_center_coord.rotation:assign(root_tracker:toQuater(3))
	end
	for i=0,5 do 
		local other_joint = transf()
		other_joint.translation:assign(tracker_data:toVector3(7*i))
		other_joint.rotation:assign(tracker_data:toQuater(7*i+3))
		other_joint:assign(project_center_coord:project2D():inverse()*other_joint)
		local changed_data = vectorn(7)
		changed_data:setVec3(0,other_joint.translation)
		changed_data:setQuater(3,other_joint.rotation)
		local_tracker_data:range(7*i,7*i+7):assign(changed_data)
	end
	return local_tracker_data
end

function vector2trasnf(vec)
	assert(vec:size()==7,'vector2trasnf 함수에서 input data 문제')
	local transform = transf()
	transform.translation:assign(vec:toVector3(0))
	transform.rotation:assign(vec:toQuater(3))
	return transform
end

function pushBack(matrix,data)
	for i=0,matrix:rows()-2 do
		matrix:row(i):assign(matrix:row(i+1))
	end
	matrix:row(matrix:rows()-1):assign(data)
end

function extractFeatures(mLoader,pose)
	local features = vectorn(18)
	mLoader:setPoseDOF(pose)
	local leftShoulder = mLoader:getBoneByName("LeftShoulder"):getFrame()
	local rightShoulder = mLoader:getBoneByName("RightShoulder"):getFrame()
	local local_leftWrist = leftShoulder:inverse()*mLoader:getBoneByName("LeftElbow"):getFrame()*config.trackerBones[3].trackerTransf
	local local_rightWrist = rightShoulder:inverse()*mLoader:getBoneByName("RightElbow"):getFrame()*config.trackerBones[4].trackerTransf
	features:setVec3(0,local_leftWrist.translation)
	features:setVec3(3,local_leftWrist.rotation:getFrameAxis(1))
	features:setVec3(6,local_leftWrist.rotation:getFrameAxis(2))
	features:setVec3(9,local_rightWrist.translation)
	features:setVec3(12,local_rightWrist.rotation:getFrameAxis(1))
	features:setVec3(15,local_rightWrist.rotation:getFrameAxis(2))
	return features
end


elapsedTime = 0
function frameMove(fElapsedTime)
	elapsedTime = fElapsedTime + elapsedTime
	if iframe > 20 then 
		mSkin1:setPoseDOF(mMotionDOF:row(iframe))
		mLoader:setPoseDOF(mMotionDOF:row(iframe))
		local hand_pos = mLoader:getBoneByName("LeftWrist"):getFrame().translation*100
		RE.viewpoint().vpos:set(hand_pos.x,hand_pos.y,hand_pos.z+80)
		RE.viewpoint().vat:set(hand_pos.x,hand_pos.y,hand_pos.z)
		RE.viewpoint():update()
		prev_pose:assign(mMotionDOF:row(iframe))
	else 
	end
	iframe = iframe + 1
end



