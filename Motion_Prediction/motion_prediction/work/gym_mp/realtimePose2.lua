require("config")
require("common")
package.projectPath='../Samples/QP_controller/'
package.path=package.path..";../Samples/QP_controller/lua/?.lua" --;"..package.path
package.path=package.path..";../Samples/classification/lua/?.lua" --;"..package.path
package.path=package.path..";../Resource/classification/lua/?.lua" --;"..package.path
package.path=package.path..";../../taesooLib/Samples/classification/lua/xml2lua/?.lua" --;"..package.path
require("gym_gang/gang_lib")
require("lib/OsimParser")
require("gym_mp/retarget_config")
PoseTransfer2=require("subRoutines/PoseTransfer2")

target_motions={
	target_model=package.resourcePath.."prediction/hyunwoo_lowdof_T_boxfoot.wrl",
}
skinScale=1
use1Dknees=false
-- minimal defaults for prediction model configuration
model_config={}
model_config.prediction_frame=10
model_config.update_encode_size=35
model_config.pose_size=33
model_config.input_frames=10
model_config.useFilter=false
model_config.ALIGN=false
model_config.ATTACH_CAMERA=false
model_config.whole_prediction=false

origin_bones={
	hips='pelvis',
	left_hip='l_hip',
	left_knee='l_knee',
	left_heel='l_ankle',

	right_hip='r_hip',
	right_knee='r_knee',
	right_heel='r_ankle',

	chest='waist',
	
	left_shoulder='l_shoulder',

	right_shoulder='r_shoulder',

	neck='neck',
	head='forehead',
}

target_bones={
	hips='Hips',
	left_hip='LeftHip',
	left_knee='LeftKnee',
	left_heel='LeftAnkle',

	right_hip='RightHip',
	right_knee='RightKnee',
	right_heel='RightAnkle',

	chest='Chest',
	
	left_shoulder='LeftShoulder',
	left_elbow='LeftElbow',

	right_shoulder='RightShoulder',
	right_elbow='RightElbow',

	neck='neck',
	head='neck',
}


-- simple ring-buffer push for matrixn history (shift up, append vec to last row)
function pushBack(buf, vec)
	local rows = buf:rows()
	if rows<=0 then return end
	for i=0, rows-2 do
		buf:row(i):assign(buf:row(i+1))
	end
	buf:row(rows-1):assign(vec)
end

function ctor()
end

function prepareEnv(source_loader, source_skin)
	mOrigLoader=source_loader
	-- skip wrist name scan (not required and may error in some loaders)
	--for i,v in ipairs(bone_preprocessing) do
	--	mOrigLoader:getBoneByTreeIndex(bone_preprocessing[i][1]):setName(bone_preprocessing[i][2])	
	--end
	mOrigLoader:setVoca(origin_bones)
	mOrigSkin=source_skin
	--mOrigLoader:_changeVoca(strToVoca[k], self:getBoneByName(v))
	mTargetLoader=MainLib.VRMLloader(target_motions.target_model)
	mTargetLoader:Scale(100)
	mTargetLoader:setVoca(target_bones)
    mTargetSkin=RE.createVRMLskin(mTargetLoader, false)
	mTargetSkin:scale(skinScale,skinScale,skinScale)
	mTargetSkin:setMaterial("lightgrey_verytransparent")
	mTargetSkin:setTranslation(-100,0,0)
    mLoader_predictied=MainLib.VRMLloader ("../Resource/motion/prediction/hyunwoo_lowdof_T_boxfoot.wrl")
    mSkin_ = {}
	for i=1,10 do 
		local skin = RE.createVRMLskin(mTargetLoader,false)	
		skin:setMaterial("green_transparent")
		skin:scale(skinScale,skinScale,skinScale)
		skin:setVisible(false)
		if i == 10 then 
			skin:setVisible(true)
			skin:setMaterial("green_transparent")
		end
		table.insert(mSkin_,skin)
	end
    -- initialize buffers for prediction BEFORE matching_correspondence (which calls transferPose)
    prev_pose = vectorn(33)
    local mMot = mTargetLoader:getPoseDOF()
	prev_pose:assign(mMot)
	inputBuffer = matrixn(10,35)
	setTpose_enforce()
	matching_correspondence()
end


function dtor()
end

if EventReceiver then
	EVR=LUAclass(EventReceiver)
	function EVR:__init(graph)
		self.currFrame=0
		self.cameraInfo={}
	end
end

function EVR:onFrameChanged(win, iframe)
end

function frameMove(fElapsedTime)
end

function setTpose_enforce()
	local M=require("subRoutines/retarget_common")
	dbg.line()
	print('set bone dictionary: Original model')
	M.setVoca(mOrigLoader)
	if use1Dknees then
	do 
		local treeIndicesShoulder=intvectorn(4)
		
		treeIndicesShoulder:set(0, mTargetLoader:getTreeIndexByVoca(MotionLoader.LEFTSHOULDER))
		treeIndicesShoulder:set(1, mTargetLoader:getTreeIndexByVoca(MotionLoader.RIGHTSHOULDER))
		treeIndicesShoulder:set(2, mTargetLoader:getTreeIndexByVoca(MotionLoader.LEFTHIP))
		treeIndicesShoulder:set(3, mTargetLoader:getTreeIndexByVoca(MotionLoader.RIGHTHIP))
		local treeIndicesElbow=intvectorn(4)
		treeIndicesElbow:set(0, mTargetLoader:getTreeIndexByVoca(MotionLoader.LEFTELBOW))
		treeIndicesElbow:set(1, mTargetLoader:getTreeIndexByVoca(MotionLoader.RIGHTELBOW))
		treeIndicesElbow:set(2, mTargetLoader:getTreeIndexByVoca(MotionLoader.LEFTKNEE))
		treeIndicesElbow:set(3, mTargetLoader:getTreeIndexByVoca(MotionLoader.RIGHTKNEE))
		local treeIndicesWrist=intvectorn(4)
		treeIndicesWrist:set(0, mTargetLoader:getTreeIndexByVoca(MotionLoader.LEFTWRIST))
		treeIndicesWrist:set(1, mTargetLoader:getTreeIndexByVoca(MotionLoader.RIGHTWRIST))
		treeIndicesWrist:set(2, mTargetLoader:getTreeIndexByVoca(MotionLoader.LEFTANKLE))
		treeIndicesWrist:set(3, mTargetLoader:getTreeIndexByVoca(MotionLoader.RIGHTANKLE))
		local fi=treeIndicesWrist:findFirstIndex(-1)
		if fi==0 then
			-- has no wrist
			treeIndicesShoulder=treeIndicesShoulder:range(2,4):copy()
			treeIndicesElbow=treeIndicesElbow:range(2,4):copy()
			treeIndicesWrist=treeIndicesWrist:range(2,4):copy()
		end
		assert(treeIndicesShoulder:findFirstIndex(-1)==-1)
		assert(treeIndicesElbow:findFirstIndex(-1)==-1)
		assert(treeIndicesWrist:findFirstIndex(-1)==-1)
		use1Dknees={treeIndicesShoulder, treeIndicesElbow, treeIndicesWrist}
		local motdofB=MotionDOF(mTargetLoader.dofInfo)
		

		local gangraepark=vectorn()
		mTargetLoader:getPoseDOF(gangraepark)
		local gg_=MotionDOFcontainer(mTargetLoader.dofInfo, gangraepark)
		
		--motdofB:resize(mMotionDOF_ori:numFrames())
		if use1Dknees then
			motdofB:set(Motion(gg_.mot), use1Dknees[1], use1Dknees[2], use1Dknees[3])
		else
			motdofB:set(Motion(gg_.mot))
		end
	end	
	end

	M.gotoTpose(mOrigLoader,true,false)
	dbg.line()
	print('set bone dictionary: Target model (muscle-model)')
	M.setVoca(mTargetLoader)
	dbg.line()
	--if mOrigLoader:getTreeIndexByVoca(MotionLoader.LThWrist)==-1 then

	if false then
		M.gotoTpose(mTargetLoader, true)-- do not change finger configurations
	else
		M.gotoTpose(mTargetLoader, true, false)
		if true then
			--toe
			local gangrae=vectorn()
			mTargetLoader:getPoseDOF(gangrae)
			gangrae:set(source_redundant[1],0.4)
			gangrae:set(source_redundant[2],0.4)
			mTargetLoader:setPoseDOF(gangrae)

		end
	end
	updateSkins()
end

function updateSkins()
	local pose=Pose()
	mOrigLoader:getPose(pose)
	mOrigSkin:_setPose(pose, mOrigLoader)
	mTargetLoader:getPose(pose)
	mTargetSkin:_setPose(pose, mTargetLoader)
end

function matching_correspondence()
	local M=require("subRoutines/retarget_common")
	local c=M.setCorrespondences(mOrigLoader, mTargetLoader)
	do
			-- test if all bones are automatically detected
		local identified={}
		for i, cc in ipairs(c) do
			identified[cc[1]]=true
		end
		dbg.line()
		for i=1, mOrigLoader:numBone()-1 do
			local name=mOrigLoader:bone(i):name()
			--if not identified[name] then
			--	print( i, name..' : unidentified')
			--	print('This is undesirable! still you can type cont to continue.')
			--	dbg.console()
			--end
		end
	end

	local debugDraw=vector3N()
	for i, v in ipairs(c) do
		local A=mOrigLoader:getBoneByName(v[1]):getFrame().translation+vector3(-100/skinScale,0,0)
		local B=mTargetLoader:getBoneByName(v[2]):getFrame().translation
		debugDraw:pushBack(A)
		debugDraw:pushBack(A*0.5+B*0.5+vector3(0, A:distance(B)*0.4, 0))
		debugDraw:pushBack(A*0.5+B*0.5+vector3(0, A:distance(B)*0.4, 0))
		debugDraw:pushBack(B)
	end

	local thickness=10 -- 10 cm
	dbg.timedDraw(5, 'Traj', debugDraw:matView()*skinScale , 'solidred', thickness, 'LineList' )
	print("correspondences.. \n")
	g_convAB={TStrings(), TStrings()}
	for i, v in ipairs(c) do
		g_convAB[1]:pushBack(v[1])
		g_convAB[2]:pushBack(v[2])
		print(v[1], v[2])
	end

	if correspondencesManualFix then
		for i,v in ipairs(correspondencesManualFix .add) do
			g_convAB[1]:pushBack(v[1])
			g_convAB[2]:pushBack(v[2])
			print('manual: ', v[1], v[2])
		end
	end

	if not g_convAB then
		util.msgBox("set correspondences first!")
	else
		currentMode='retarget' -- for GUI 
		if false then
			PT=PoseTransferT(mLoaderA, mLoaderB, g_convAB[1], g_convAB[2])
		else
			PT=PoseTransfer2(mOrigLoader, mTargetLoader, g_convAB[1], g_convAB[2])
		end
		transferPose()
	end
end

function transferPose()
	local curr_src_pose=vectorn()
	mOrigLoader:getPoseDOF(curr_src_pose)
	PT:setTargetSkeleton(curr_src_pose)
	if true then
		local p=Pose()
		mTargetLoader:getPose(p)
		mTargetSkin:_setPose(p, mTargetLoader)
        local curr_dof = mTargetLoader:getPoseDOF()
        local enc = posencodeUpdateVec(curr_dof, prev_pose)
        pushBack(inputBuffer, enc)
        prev_pose:assign(curr_dof)
	end
end

function transferPose2()
	local curr_src_pose=vectorn()
	mOrigLoader:getPoseDOF(curr_src_pose)
	PT:setTargetSkeleton(curr_src_pose)
    local pose = vectorn(35)
	local tmp_pose = vectorn(35)
	if true then
		local p=Pose()
		mTargetLoader:getPose(p)
		mTargetSkin:_setPose(p, mTargetLoader)
        local curr_dof = mTargetLoader:getPoseDOF()
        local enc = posencodeUpdateVec(curr_dof, prev_pose)
        pushBack(inputBuffer, enc)
        prev_pose:assign(curr_dof)
	end
    for i=1,model_config.prediction_frame  do 
			
			
        local encodeData = vectorn(model_config.update_encode_size)
        local tempData =vectorn(model_config.update_encode_size)
        local latent = vectorn(35)
                   latent:setAllValue(0)
                   
        

        -- run model
           --python.F('train_motionprediction_dif16','test_dif16',inputBuffer:row(0),inputBuffer:toVector(),latent,encodeData)
            --python.F('train_motionprediction_dif16','test_dif16',inputBuffer:toVector(),inputBuffer:row(9),latent,encodeData)
          --python.F('train_motionprediction_dif16','test_dif16',inputBuffer:row(0),inputBuffer:row(0),latent,encodeData)
            --python.F('train_motionprediction_dif16','test_dif16',inputBuffer:toVector(),poseEncodeUpdate(iframe + i - 1),latent,encodeData)
            --python.F('train_motionprediction_dif16','test_dif1016',inputBuffer:row(0),inputBuffer:row(1),inputBuffer:row(2),inputBuffer:row(3),inputBuffer:row(4),inputBuffer:row(5),inputBuffer:row(6),inputBuffer:row(7),inputBuffer:row(8),inputBuffer:row(9),latent,encodeData)
            --python.F('train_motionprediction_dif16','test_dif516',inputBuffer:row(0),inputBuffer:row(1),inputBuffer:row(2),inputBuffer:row(3),inputBuffer:row(4),inputBuffer:row(5),inputBuffer:row(6),inputBuffer:row(7),inputBuffer:row(8),inputBuffer:row(9),latent,encodeData)
           
            --for i=1,3 do
            python.F('train_motionprediction_dif16','test_dif316',inputBuffer:row(0),inputBuffer:row(1),inputBuffer:row(2),inputBuffer:row(3),inputBuffer:row(4),inputBuffer:row(5),inputBuffer:row(6),inputBuffer:row(7),inputBuffer:row(8),inputBuffer:row(9),latent,encodeData)
            --tempData=encodeData+tempData
            --end
            --tempData=tempData/3
            
            pushBack(inputBuffer, encodeData) -- ok than pushback the encode to input Buffer
            
            
        pose:assign(poseDecodeUpdate(encodeData,prev_pose))
        -- pose:range(0,7):assign(mMotionDOF:row(iframe+i):range(0,7))

        -- filter process (optional)
        local pp_pose = vectorn(35)
        if model_config.useFilter and mFilter then
            mFilter:setCurrPose(pose)
            local renderPoseFiltered = mFilter:getFiltered()
            pp_pose:assign(renderPoseFiltered)
        else
            pp_pose:assign(pose)
        end

        -- in realtime bin->motion path, mMotionDOF may be unavailable; keep previous root alignment
        pp_pose:setVec3(0, prev_pose:toVector3(0))
        mSkin_[i]:setPoseDOF(pp_pose)
        mSkin_[i]:setTranslation(-100,-5,20)
        prev_pose:assign(pp_pose)
        mLoader_predictied:setPoseDOF(pp_pose)


end
end
function generate_exportfile(motion_length)
	mMotionDOF=MotionDOF(mTargetLoader.dofInfo)
	mMotionDOF:resize(motion_length)
end

function setting_pose(row_)
	assert(mTargetLoader)
	transferPose()
	local target_pose=vectorn()
	target_pose=mTargetLoader:getPoseDOF()
	target_pose:range(0,3):rdiv(100)
	mMotionDOF:row(row_):assign(target_pose)
end
function poseEncodeUpdate(iframe)
    local vec = vectorn(model_config.update_encode_size)
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

function posencodeUpdateVec(cur, past)
	local vec = vectorn(model_config.update_encode_size)
    local pose1 = past
    local pose2 = cur
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

-- poseEncodeEnd
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
		model_config.draw = w:checkButtonValue()	
		dbg.eraseAllDrawn()
	elseif w:id() == "current motion visualization" then 
		mSkin1:setVisible(w:checkButtonValue())
	elseif w:id() == "attach camera" then 
		model_config.ATTACH_CAMERA=w:checkButtonValue()
	elseif w:id() == "pose error calculating" then 
		model_config.EVAL=w:checkButtonValue()
	elseif w:id() == "align current" then 
		model_config.ALIGN=w:checkButtonValue()
	elseif w:id() == "show whole prediction" then
		model_config.whole_prediction = w:checkButtonValue()
		for i = 1, 9 do
			mSkin_[i]:setVisible(model_config.whole_prediction)
		end
	elseif w:id() == "useFilter" then
		model_config.useFilter = w:checkButtonValue()
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

function export_motfile(file_path_)
    local new_motion_name=this:findWidget('file_name_'):inputValue()
	if new_motion_name:len()==0 then
	end
	local index_=file_path_:findLastOf("/")
	local origin_name=file_path_:sub(index_+1,-5)

	local mot_path="../python/ETRI_viewer/"
	local file_name=mot_path..origin_name.."_"..new_motion_name..".dof"
	mMotionDOF:exportMot(file_name)
	print("======================================================")
	print("Exporting motion done! (.bin to .dof) named file_name"..file_name)
	print("======================================================")
end
