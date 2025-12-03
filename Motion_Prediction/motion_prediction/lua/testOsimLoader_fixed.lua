require("config")
require("module")
package.path=package.path..";lib/?.lua" --;"..package.path
require("subRoutines/Timeline")

projectDir=os.parentPath(util.getScriptPath())
package.path=package.path..";"..projectDir..'/lua/?.lua' --;"..package.path
require("lib/OsimParser")

function ctor()
	RE.ogreSceneManager():setFogNone()
	mEventReceiver=EVR()

	parser_origin=OsimParser(package.resourcePath..'ETRI/Rajagopal_2015_ETRI.osim', {debugDraw=false})
	util.saveTableToLua(parser_origin:toTable(), 'Rajagopal_2015_ETRI.osim.lua')

	local fixed_model=Fltk.chooseFile('choose a model', '../Resource/motion/ETRI/' ,'*.osim.lua', false)

	parser_fixed=OsimParser(util.loadTableFromLua(fixed_model), {debugDraw=false})
	local pose=parser_origin.loader:pose()
	local pose_fixed=parser_fixed.loader:pose()
	pose.translations(0).y=1.0
	pose_fixed.translations(0).y=1.0

	parser_origin.loader:setPose(pose)
	parser_fixed.loader:setPose(pose_fixed)
	skin=RE.createVRMLskin(parser_origin.loader, false)
	skin:setScale(100,100,100)
	skin:setSamePose(parser_origin.loader:fkSolver())
	skin_fixed=RE.createVRMLskin(parser_fixed.loader, false)
	skin_fixed:setScale(100,100,100)
	skin_fixed:setSamePose(parser_fixed.loader:fkSolver())
	skin_fixed:setTranslation(0,0,-100)

	if true then
		-- draw muscles
		package.path=package.path..";../lua/?.lua" 
		package.path=package.path..";../lua/models/?.lua" 
		require("lib/IPC_based/common")
		require("useCasesMuscle")

		local MS=require('subRoutines/MuscleSim')
		do 
			model=model_files.full_ipfrun_repeat
			
			----------------------
			model.file_name='FullBodyModel-4.0/Rajagopal2015.osim'
			model.bones={}
			model.luamsclpath=nil -- will be later set
			model.mot_file=nil
			model.initialHeight=1.1
			----------------------

			model.rendering_step=1/30 -- real-time speed
			-- muscle is simulated at 840hz so use an integer multiple
			model.timestep=1/420 -- when require('PDservo_spd') that works only for physX
			model.frame_rate=30 -- mocap frame rate
			model.k_p_PD=0
			model.k_d_PD=1
			--model.numSolverIterations=2000
		end
		model.luamsclpath=parser_origin.muscles 
		mLoader=parser_origin.loader
		osim=OsModel(parser_origin.loader, model.luamsclpath, model, {}) 
		objectList=Ogre.ObjectList()
		local lineWidth=1
		osim:init_A_U()

		osim:invalidatePPIndicesBasedInfo()
		osim:invalidatePPPoszBasedInfo()
		osim:setBoneForwardKinematics(mLoader:fkSolver())

		-- blue to red (0==blue, red ==1)
		osim.a:vecView():setAllValue(0.0)
		osim:drawMuscles(nil, objectList, lineWidth)

		--------------------------------------------------------			
		mLoader_fixed=parser_fixed.loader
		local model_fixed=model
		model_fixed.luamsclpath=parser_fixed.muscles
		osim_fixed=OsModel(parser_fixed.loader, model_fixed.luamsclpath, model_fixed, {})
		objectList=Ogre.ObjectList()
		local lineWidth=1
		osim_fixed:init_A_U()

		osim_fixed:invalidatePPIndicesBasedInfo()
		osim_fixed:invalidatePPPoszBasedInfo()
		osim_fixed:setBoneForwardKinematics(mLoader_fixed:fkSolver())

		-- blue to red (0==blue, red ==1)
		osim_fixed.a:vecView():setAllValue(0.0)
		osim_fixed:drawMuscles(nil, objectList, lineWidth)

	end
	RE.viewpoint().vpos:set(341.9946544,105.0894399,-50.66303655)
	RE.viewpoint().vat:set(-40.55202699,93.60099849,-35.85557173)
	dbg.draw("Text", vector3(0,200,100), "textfield1", vector3(0,0,0), 15, "Origin Rajagopal model")
	dbg.draw("Text", vector3(0,200,-100), "textfield12", vector3(0,0,0), 15, "New Rajagopal model")
	RE.viewpoint():update()     
end
function dtor()
   if mTimeline then
	   mTimeline:dtor()
       mTimeline=nil
   end
   -- remove objects that are owned by LUA
   collectgarbage()
end
function onCallback(w, userData)
end
function frameMove(fElapsedTime)
	osim:drawMuscles(nil, objectList, lineWidth)
	osim_fixed:drawMuscles(nil, objectList, lineWidth,vector3(0,0,-100))
	return 0
end
function handleRendererEvent(fElapsedTime)
	return 0
end
