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

config = {"../Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T_boxfoot.wrl", "../Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T_locomotion_hl.dof",
{
		{'LeftKnee', 'LeftAnkle', vector3(0,0,0), reversed=false},
		{'RightKnee', 'RightAnkle', vector3(0,0,0), reversed=false},
		{'LeftShoulder', 'LeftElbow', vector3(0,0,0), reversed=false},
		{'RightShoulder', 'RightElbow', vector3(0,0,0), reversed=false},
		{'Chest', 'Neck', vector3(0,0,0), reversed=false},
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
	RE.viewpoint():update() this:create('Button','other_latentvector','other_latentvector')
	this:updateLayout()
	mLoader=MainLib.VRMLloader ("../Resource/motion/prediction/hyunwoo_lowdof_T_boxfoot.wrl")
	DOFcontainer = MotionDOFcontainer(mLoader.dofInfo , "../Resource/motion/prediction/hyunwoo_lowdof_T_locomotion_hl.dof")
	mMotionDOF = DOFcontainer.mot
	mTree = MotionUtil.LoaderToTree(mLoader,false,false)
	DmMotionDOF = calcDerivative(mMotionDOF)
    mSkin1 = RE.createVRMLskin(mLoader , false)
	mSkin1:scale(config.skinScale,config.skinScale,config.skinScale)
    start_frame = 0
    iframe = start_frame
	pose_size = mMotionDOF:numDOF()
	prev_pose = vectorn(mMotionDOF:numDOF())
	prev_pose:assign(mMotionDOF:row(start_frame))
	encode_size = 35
	mTree:setLinkData(mMotionDOF:row(start_frame),DmMotionDOF:row(start_frame))
	encodeBuffer = matrixn(2,encode_size)
	encodeBuffer:setAllValue(0)
	latent = vectorn(10)
	latent:setAllValue(0)

	motionMatrix = matrixn(5000,33)
end

function EVR:onFrameChanged(win, iframe)
end

function rotationMatrix(q)
	local mat=matrix4()
	mat:setRotation(q)
	return mat
end

function poseEncodeUpdate(iframe)
    local vec = vectorn(encode_size)
    local pose1 = mMotionDOF:row(iframe)
    local pose2 = mMotionDOF:row(iframe+1)
    local rotY = pose1:toQuater(3):rotationY()
    local dv = rotY:inverse()*(pose2:toVector3(0) - pose1:toVector3(0))
	local local_quater = rotY:inverse()*pose2:toQuater(3)
	vec:set(0,dv.x)
	vec:set(1,dv.z)
	vec:set(2,pose2:toVector3(0).y)
	vec:setVec3(3,local_quater:getFrameAxis(1))
	vec:setVec3(6,local_quater:getFrameAxis(2))
	vec:range(9,35):assign(pose2:range(7,33))
	return vec
end

function poseDecodeUpdate(encodeData,prev_pose)
	local pose_output = vectorn(33)
	local prev_root_pos = prev_pose:toVector3(0)
	local prev_root_ori = prev_pose:toQuater(3)
	local rotY = prev_root_ori:rotationY()
	local new_root_pos = prev_root_pos + rotY*vector3(encodeData(0),0,encodeData(1))
	new_root_pos.y = encodeData(2)
	local new_root_ori = quater()
	new_root_ori:setFrameAxesYZ(encodeData:toVector3(3),encodeData:toVector3(6))
	new_root_ori = rotY*new_root_ori
	pose_output:setVec3(0,new_root_pos)
	pose_output:setQuater(3,new_root_ori)
	pose_output:range(7,33):assign(encodeData:range(9,35))
	return pose_output
end



function onCallback(w, userData)
	if w:id() == 'other_latentvector'then 
		python.F('play_hyunseokEnv','make_latent_vector',latent)
		print(latent)
	end 
end

function dtor()
end

function pushBack(matrix,data)
	for i=0,matrix:rows()-2 do
		matrix:row(i):assign(matrix:row(i+1))
	end
	matrix:row(matrix:rows()-1):assign(data)
end
	
function getJointStateFromRefTree(mTree)
	local numBone = mLoader:numBone()
	local joint_pos = vector3N(numBone)
	local joint_ori = quaterN(numBone)
	for i=1,numBone-1 do
		local sg = mTree:globalFrame(i)
		joint_pos(i):assign(sg.translation)
		joint_ori(i):assign(sg.rotation)
	end

	return {pos = joint_pos ,ori = joint_ori }
end

function getDiscontinuity()
	local discont = vectorn()
	local isSimilar=function(d1,d2)
		return math.abs(d1-d2) < math.rad(40)
	end
	local ndof = mMotionDOF:matView():cols()
	local mot = mMotionDOF:matView()
	for frame=1,mMotionDOF:rows()-1 do 
		for i=7,ndof-1 do 
			if not(isSimilar(mot(frame,i),mot(frame-1,i))) then 
				discont:pushBack(frame)
				break
			end
		end
	end
	discont:pushBack(mMotionDOF:numFrames())
	return  discont
end

function All_data_frames()
	local data = matrixn(mMotionDOF:numFrames()-1,encode_size)
	for i=0,data:rows()-1 do
		data:row(i):assign(poseEncodeUpdate(i))
	end
	return data
end


elapsedTime = 0
function frameMove(fElapsedTime)
	elapsedTime = fElapsedTime + elapsedTime
	local pose = vectorn(pose_size)
	if iframe < 2 then 
		pose:assign(mMotionDOF:row(iframe))
		mSkin1:setPoseDOF(pose)
		pushBack(encodeBuffer,poseEncodeUpdate(iframe))
		prev_pose:assign(pose)
	else 
		local encodeData = vectorn(encode_size)
		encodeData:setAllValue(0)
		python.F('play_mp','make_latent_vector',latent)
		python.F('play_mp','test_vae',latent,encodeBuffer:row(1),encodeData)
		pushBack(encodeBuffer,encodeData)
		pose = poseDecodeUpdate(encodeData,prev_pose)
		if gui then 
			local com = pose:toVector3(0)
			RE.viewpoint().vat:set(com.x*100,com.y*100,com.z*100)
			RE.viewpoint().vpos:set(com.x*100-800,com.y*100+500,com.z*100)
			RE.viewpoint():update()
		end
		mSkin1:setPoseDOF(pose)
		prev_pose:assign(pose)
	end
    iframe = iframe + 1
end
