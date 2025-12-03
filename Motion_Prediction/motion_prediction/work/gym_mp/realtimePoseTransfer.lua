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


function ctor()
end

function prepareEnv(source_loader, source_skin)
	mOrigLoader=source_loader
	for i=1, mOrigLoader:numBone()-1 do
		if mOrigLoader:getBoneByTreeIndex(i):name():find("wrist") then
		end
	end
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
	end
end

-- placeholder for advanced transfer using prediction over 10-bin motion
-- currently falls back to basic transfer until prediction integration is added
function transferPose2()
	transferPose()
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
