package.path=package.path..";../Samples/scripts/RigidBodyWin/motionFormatConversion/?.lua" --;"..package.path
require("RetargetDOFtoVRML")

local wrlpath, bvhpath, dofpath
do
	---- gait25*
	--wrlpath = '../Resource/motion/opensim/gait2592_modified.wrl'

	----bvhpath = '../Resource/motion/opensim/gait_repeat.bvh'
	----dofpath = '../Resource/motion/opensim/gait25_gait_repeat.dof'

	----bvhpath = '../Resource/motion/opensim/gait19_soldier_repeat.bvh'
	----dofpath = '../Resource/motion/opensim/gait25_soldier_repeat.dof'

	--bvhpath = '../Resource/motion/opensim/gait19_tong_repeat.bvh'
	--dofpath = '../Resource/motion/opensim/gait25_tong_repeat.dof'
end

do
	-- full2*
	wrlpath = '../Resource/motion/opensim/FullBody2_lee.wrl'

	--bvhpath = '../Resource/motion/opensim/full_soldier_repeat.bvh'
	--dofpath = '../Resource/motion/opensim/full2_soldier_repeat.dof'
	
	--bvhpath = '../Resource/motion/opensim/full_tong_repeat.bvh'
	--dofpath = '../Resource/motion/opensim/full2_tong_repeat.dof'
	
	--bvhpath = '../Resource/motion/opensim/full_lean_repeat.bvh'
	--dofpath = '../Resource/motion/opensim/full2_lean_repeat.dof'
	
	--bvhpath = '../Resource/motion/opensim/full_same_repeat.bvh'
	--dofpath = '../Resource/motion/opensim/full2_same_repeat.dof'

	--bvhpath = '../Resource/motion/opensim/full_ipnrun_repeat.bvh'
	--dofpath = '../Resource/motion/opensim/full2_ipnrun_repeat.dof'
	
	--bvhpath = '../Resource/motion/opensim/full_ipfrun_repeat.bvh'
	--dofpath = '../Resource/motion/opensim/full2_ipfrun_repeat.dof'

	bvhpath = '../Resource/motion/opensim/full_sit_repeat.bvh'
	dofpath = '../Resource/motion/opensim/full2_sit_repeat.dof'
end

do
	---- gait19*
	--wrlpath = '../Resource/motion/opensim/gait1956_render.wrl'

	----bvhpath = '../Resource/motion/opensim/gait_repeat.bvh'
	----dofpath = '../Resource/motion/opensim/gait19_gait_repeat.dof'

	----bvhpath = '../Resource/motion/opensim/gait19_soldier_repeat.bvh'
	----dofpath = '../Resource/motion/opensim/gait19_soldier_repeat.dof'

	--bvhpath = '../Resource/motion/opensim/gait19_tong_repeat.bvh'
	--dofpath = '../Resource/motion/opensim/gait19_tong_repeat.dof'
end

do
	---- full*
	--wrlpath = '../Resource/motion/opensim/FullBody_lee.wrl'

	----bvhpath = '../Resource/motion/opensim/full_soldier_repeat.bvh'
	----dofpath = '../Resource/motion/opensim/full_soldier_repeat.dof'

	----bvhpath = '../Resource/motion/opensim/full_sukiko_repeat.bvh'
	----dofpath = '../Resource/motion/opensim/full_sukiko_repeat.dof'

	----bvhpath = '../Resource/motion/opensim/full_tong_repeat.bvh'
	----dofpath = '../Resource/motion/opensim/full_tong_repeat.dof'
	
	----bvhpath = '../Resource/motion/opensim/full_lean_repeat.bvh'
	----dofpath = '../Resource/motion/opensim/full_lean_repeat.dof'
	
	----bvhpath = '../Resource/motion/opensim/full_same_repeat.bvh'
	----dofpath = '../Resource/motion/opensim/full_same_repeat.dof'

	----bvhpath = '../Resource/motion/opensim/full_sit_repeat.bvh'
	----dofpath = '../Resource/motion/opensim/full_sit_repeat.dof'
	
	----bvhpath = '../Resource/motion/opensim/full_ipnrun_repeat.bvh'
	----dofpath = '../Resource/motion/opensim/full_ipnrun_repeat.dof'
	
	--bvhpath = '../Resource/motion/opensim/full_ipfrun_repeat.bvh'
	--dofpath = '../Resource/motion/opensim/full_ipfrun_repeat.dof'
end


targets={"general_bvh"}

Motions.general_bvh=deepCopyTable(Motions.default)
Motions.general_bvh.conversionMethod=conversionMethod_T.useAllLocalAxis
Motions.general_bvh.src_skel_file=bvhpath
Motions.general_bvh.scale=1	-- change inch to METER unit system
Motions.general_bvh.out_file=dofpath
Motions.general_bvh.wrl_file=wrlpath
