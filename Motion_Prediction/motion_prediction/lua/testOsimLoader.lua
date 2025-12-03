
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

	parser=OsimParser(projectDir..'/Resource/motion/ETRI/Rajagopal_2015_ETRI.osim', {debugDraw=false})

	local pose=parser.loader:pose()
	pose.translations(0).y=1.0
	parser.loader:setPose(pose)
	skin=RE.createVRMLskin(parser.loader, false)
	skin:setScale(100,100,100)
	skin:setSamePose(parser.loader:fkSolver())

	print(parser.actuated)
	for i=1, parser.loader:numBone()-1 do
		if parser.actuated(i) then
			print(parser.loader:bone(i):name())
		end
	end
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
		model.luamsclpath=parser.muscles 
		mLoader=parser.loader
		--mLoader:convertToConvexes()
		osim=OsModel(parser.loader, model.luamsclpath, model, {}) 
		objectList=Ogre.ObjectList()
		local lineWidth=1
		osim:init_A_U()

		osim:invalidatePPIndicesBasedInfo()
		osim:invalidatePPPoszBasedInfo()
		osim:setBoneForwardKinematics(mLoader:fkSolver())

		-- blue to red (0==blue, red ==1)
		osim.a:vecView():setAllValue(0.0)
		osim:drawMuscles(nil, objectList, lineWidth)
	end
	RE.viewpoint().vpos:set(239.1966796,96.94216219,3.158010758)
	RE.viewpoint().vat:set(-38.47578426,88.60324832,13.90604607)
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
	osim:drawMuscles()
	return 0
end
function handleRendererEvent(fElapsedTime)
	return 0
end
