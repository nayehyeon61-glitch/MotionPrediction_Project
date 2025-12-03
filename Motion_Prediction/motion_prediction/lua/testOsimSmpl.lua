require("config")
require("module")
package.path=package.path..";../../taesooLib/Samples/classification/lua/xml2lua/?.lua" --;"..package.path
require("subRoutines/Timeline")


projectDir=os.parentPath(util.getScriptPath())
package.path=package.path..";"..projectDir..'/lua/?.lua' --;"..package.path
require("lib/OsimParser")


smplDir=os.parentPath(projectDir)..'/sample_SAMP/lua'
if not os.isFileExist(smplDir..'/smpl/smpl.lua') then
	util.msgBox('Error! '..smplDir.."/smpl/smpl.lua doesn't exist")
end
package.path=package.path..";"..smplDir..'/?.lua' --;"..package.path
FBXloader=require("FBXloader")
SMPL=require("smpl/smpl")
require("Kinematics/meshTools")

SMPL_MALE=true
SMPL_FEMALE=not SMPL_MALE
FRONT_VIEW=true
function redrawMuscles(option_)
	osim:invalidatePPIndicesBasedInfo()
	osim:invalidatePPPoszBasedInfo()
	osim:setBoneForwardKinematics(parser.loader:fkSolver())
	
	local l_mt=osim:getTendonMuscleLengths()
	local l_m_opt=osim.l_m_opt
	--print('fiber', 'tendon', 'muscle-tendon')
	for i=0,osim.l_m:vecView():size()-1 do
		if option_==nil then
			--print(i,osim.l_m:vecView():get(i),osim.l_t_sl:vecView():get(i), l_mt:vecView():get(i))
		--	RE.output2("origin l_m (fiber length)")
		end
		if osim.l_m:vecView():get(i)==0 then
			osim.a:vecView():set(i,1.0)
			--osim.a:vecView():set(i+(((n_muscle-1)-6)/2),1.0)
		else
			osim.a:vecView():set(i,osim.l_m:vecView():get(i))
		end
		osim.a:vecView():set(i,0.0)
	end
	if option_~=nil then
		for i=0,osim.l_m:vecView():size()-1 do
			osim.a:vecView():set(i,0)
		end
		osim.a:vecView():set(option_,1)
	end
	osim:drawMuscles(nil, objectList, lineWidth, option_)
end
function redrawSMPL()
	SMPL.redraw(dd, meshToEntity, node)
end

-- fit SMPL beta parameters to match osimLoader

function ctor()
	RE.turnOffSoftShadows()
	RE.ogreSceneManager():setFogNone()
	mEventReceiver=EVR()

	--bm_path='/smpl/models/basicModel_f_lbs_10_207_0_v1.0.0.npz'
	bm_path='/smpl/models/basicModel_m_lbs_10_207_0_v1.0.0.npz'

	dd=SMPL.init(smplDir..bm_path)
	mesh=SMPL.createMesh(dd)
	meshToEntity, node=mesh:drawMesh("lightgrey_verytransparent")
	--meshToEntity, node=mesh:drawMesh("white")
	node:scale(100,100,100)
	parser=OsimParser(projectDir..'/Resource/motion/ETRI/Rajagopal_2015_ETRI.osim', {debugDraw=false})
	OsimSMPL=require('OsimSmpl')
	originalOsimModel=parser:toTable()

	if false then
		util.saveTableToLua(parser:toTable(), 'Rajagopal_2015_ETRI.osim.lua')
		parser=nil

		local tbl=util.loadTableFromLua('Rajagopal_2015_ETRI.osim.lua')


		local scaleInfo={
			default={ 
				geomTransform=matrix4(),
				massScale=2*2*2,
				lengthScale=2,
			},
		}
		scaleInfo.default.geomTransform:setScaling(2,2,2)
		OsimSMPL.fitOsimParser(tbl, scaleInfo)

		parser=OsimParser(tbl, {debugDraw=false})
	end

	OsimSMPL.gotoSMPL_Tpose(parser.loader)
	OsimSMPL.findBetas(dd, parser.loader)
	OsimSMPL.alignPoses(dd, parser.loader)
	originalOsimJointPos=vector3N(parser.loader:numBone())
	for i=1, parser.loader:numBone()-1 do
		originalOsimJointPos(i):assign(parser.loader:bone(i):getFrame().translation)
	end
	local skinScale=100
	for i=0, dd.betas:size() -1 do
		this:addFloatSlider('beta'..tonumber(i), dd.betas(i), -3, 3)
	end
	this:addButton('fitting') 
	this:create("Value_Slider", "muscle", "muscle");
	this:widget(0):sliderRange(0,45);
	this:widget(0):sliderStep(1)
	this:widget(0):sliderValue(0);
	this:addButton('change view') 


	this:create("Check_Button", "Male", "Male")
	this:widget(0):checkButtonValue(SMPL_MALE) 
	this:create("Check_Button", "Female", "Female")
	this:widget(0):checkButtonValue(SMPL_FEMALE) 

	this:updateLayout()
	skin=RE.createVRMLskin(parser.loader, false)
	skin:setScale(100,100,100)
	skin:setSamePose(parser.loader:fkSolver())
	--skin:setVisible(parser.loader:fkSolver())

	print(parser.actuated)
	for i=1, parser.loader:numBone()-1 do
		if parser.actuated(i) then
			print(parser.loader:bone(i):name())
		end
	end
	print('total mass:', parser.loader:calcTotalMass())
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
		--mLoader:convertToConvexes() -- 이거는 시뮬레이션 할때는 이거 한번 호출해주면 충돌 메시가 간결해져서 빠르고 정확할거에요. 기존 학습코드에도 적용가능. (렌더링 할때는 original loader를 사용하는게 더 좋겠네요.)
		osim=OsModel(parser.loader, model.luamsclpath, model, {})  
		n_muscle=osim:getNumMuscles()
		objectList=Ogre.ObjectList()
		local lineWidth=1
		osim:init_A_U()
	end
	redrawMuscles()
	origin_muscle_lm=osim:getFiberLengths():vecView():copy()
	origin_l_mt=osim:getTendonMuscleLengths():vecView():copy()
	origin_l_t=(osim:getTendonMuscleLengths()-osim:getFiberLengths()):vecView():copy()
	origin_lm_opt=osim.l_m_opt:vecView():copy()

	RE.viewpoint().vpos:assign(vector3(12, 107, 335))
	RE.viewpoint().vat:assign(vector3(-10.5, 86,0.8))
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
	if w:id():sub(1,4)=='beta' then
		local i=tonumber(w:id():sub(5))
		dd.betas:set(i, w:sliderValue())
		SMPL.updateShape(dd, mesh)
		redrawSMPL()

	elseif w:id()=='fitting' then
		skin=nil
		local tbl=deepCopyTable(originalOsimModel)
		local osimLoader=parser.loader
		local function genPair(n1, n2, offset)
			return { array.locate(SMPL.SMPL_JOINT_NAMES, n1)-1, osimLoader:getTreeIndexByName(n2), offset }
		end
		local alignInfo={
			{
				genPair("left_shoulder",'humerus_l'),
				genPair("left_elbow",'radius_l'),
				genPair("left_wrist",'hand_l'),
			},
			{
				genPair("right_shoulder",'humerus_r'),
				genPair("right_elbow",'radius_r'),
				genPair("right_wrist",'hand_r'),
			},
			{
				genPair("left_hip",'femur_l'),
				genPair("left_knee",'tibia_l'),
				genPair("left_ankle",'talus_l'),
				--genPair("left_foot", "toes_l"),
			},
			{
				genPair("right_hip",'femur_r'),
				genPair("right_knee",'tibia_r'),
				genPair("right_ankle",'talus_r'),
				--genPair("right_foot",'toes_r'),
			},
		}
		local function getPos(i,j)
			return originalOsimJointPos(alignInfo[i][j][2])
		end
		local globalMassScale=sop.piecewiseLinearMap(dd.betas(1),{-3, -1.5, 1.5, 3}, 
		{0.129/0.092, 0.108/0.092, 0.076/0.092, 0.0661/0.092})
			
		local sh_to_hip1=dd.J:row(alignInfo[1][1][1])-dd.J:row(alignInfo[3][1][1])
		local sh_to_hip2=dd.J:row(alignInfo[2][1][1])-dd.J:row(alignInfo[4][1][1])
		local sh_to_hip_smpl=sh_to_hip1:toVector3()*0.5+sh_to_hip2:toVector3()*0.5
		local sh_to_hip_osim=(getPos(1,1)-getPos(3,1))*0.5+(getPos(2,1)-getPos(4,1))*0.5

		local sf=sh_to_hip_smpl:length()/sh_to_hip_osim:length()
		local scaleInfo={
			default={ 
				geomTransform=matrix4(),
				massScale=sf*sf*sf*globalMassScale,
				lengthScale=sf,
			},
		}
		scaleInfo.default.geomTransform:setScaling(sf,sf,sf)

		for i, v in ipairs(alignInfo) do

			for j=2, #v do
				local parent_to_child_smpl=dd.J:row(v[j][1])-dd.J:row(v[j-1][1])
				local parent_to_child_osim=getPos(i, j)-getPos(i, j-1)

				local sf=parent_to_child_smpl:length()/parent_to_child_osim:length()

				local si={
					geomTransform=matrix4(),
					massScale=sf*sf*sf*globalMassScale,
					lengthScale=sf,
				}
				si.geomTransform:setScaling(sf,sf,sf)
				scaleInfo[parser.loader:bone(v[j-1][2]):name()]=si
				print(parser.loader:bone(v[j-1][2]):name(), sf)
			end
		end
		OsimSMPL.fitOsimParser(tbl, scaleInfo)
		tbl.betas=dd.betas
		local new_model=deepCopyTable(tbl)
		parser=OsimParser(tbl, {debugDraw=false})

		OsimSMPL.gotoSMPL_Tpose(parser.loader)
		--OsimSMPL.findBetas(dd, parser.loader)
		OsimSMPL.alignPoses(dd, parser.loader)
		skin=RE.createVRMLskin(parser.loader, false)
		skin:setScale(100,100,100)
		skin:setSamePose(parser.loader:fkSolver())
		model.luamsclpath=parser.muscles    -- 이거는 루아 파일이름 대신에 테이블을 저장. 

		osim=OsModel(parser.loader, model.luamsclpath, model, {}) --이런식으로 위에서 저장한 근육정보 테이블이 사용됩니다. 
		--osim:invalidatePPIndicesBasedInfo()
		--osim:invalidatePPPoszBasedInfo()
		--osim:setBoneForwardKinematics(parser.loader:fkSolver())
		
		local function fix_optimal_fiber_length()
			local scale_=VectorXd()
			local fixed_muscle_lm=osim.l_m
			scale_=fixed_muscle_lm/origin_muscle_lm
			for i, muscle in ipairs(new_model.muscles) do
				--print('scaling...(',scale_:vecView():get(i-1),')',i-1,'origin', origin_muscle_lm:vecView():get(i-1),'fixed',fixed_muscle_lm:vecView():get(i-1))
				muscle.optimal_fiber_length=muscle.optimal_fiber_length*scale_:vecView():get(i-1)
				if muscle.FmaxMuscleStrain==nil then 
					muscle.FmaxMuscleStrain=0.6*scale_:vecView():get(i-1)
					muscle.FmaxTendonStrain=0.033*scale_:vecView():get(i-1)
				else
					muscle.FmaxMuscleStrain=muscle.FmaxMuscleStrain*scale_:vecView():get(i-1)
					muscle.FmaxTendonStrain=muscle.FmaxTendonStrain*scale_:vecView():get(i-1)
				end
				--muscle.KshapePassive=4.0*scale_:vecView():get(i-1)
				if muscle.optimal_fiber_length==0 then
					print('scaling error',i-1,muscle.optimal_fiber_length)
					--print('scaling error',i-1, muscle.optimal_fiber_length,'(',fixed_muscle_lm:vecView():get(i-1),')')
				end
			end
		end
		local function fix_optimal_mt_length()
			local scale_=VectorXd()
			local fixed_muscle_lt=osim:getTendonMuscleLengths()-osim:getFiberLengths()
			scale_=fixed_muscle_lt/origin_l_t
			for i, muscle in ipairs(new_model.muscles) do
				print(i-1,'origin', origin_l_t:vecView():get(i-1),'fixed',fixed_muscle_lt:vecView():get(i-1))
				muscle.tendon_slack_length=muscle.tendon_slack_length*scale_:vecView():get(i-1)
				if muscle.FmaxMuscleStrain==nil then 
					muscle.FmaxMuscleStrain=0.6*scale_:vecView():get(i-1)
					muscle.FmaxTendonStrain=0.033*scale_:vecView():get(i-1)
				else
					muscle.FmaxMuscleStrain=muscle.FmaxMuscleStrain*scale_:vecView():get(i-1)
					muscle.FmaxTendonStrain=muscle.FmaxTendonStrain*scale_:vecView():get(i-1)
				end
				--muscle.KshapePassive=4.0*scale_:vecView():get(i-1)
				if muscle.optimal_fiber_length==0 then
					print('scaling error',i-1,muscle.optimal_fiber_length)
					--print('scaling error',i-1, muscle.optimal_fiber_length,'(',fixed_muscle_lm:vecView():get(i-1),')')
				end
			end
		end

		local lineWidth=1
		--osim:init_A_U()
		--fix_optimal_fiber_length()
		--redrawMuscles()
		--fix_optimal_mt_length()
		redrawMuscles()

		local new_model_name="Rajagopal_2015_ETRI_fixed"
		if SMPL_MALE then
			new_model_name=new_model_name..'_M.osim.lua'
			util.saveTableToLua(new_model, projectDir..'/Resource/motion/ETRI/'..new_model_name)
		else
			new_model_name=new_model_name..'_F.osim.lua'
			util.saveTableToLua(new_model, projectDir..'/Resource/motion/ETRI/'..new_model_name)
		end
		print("fixed model is saved.... ")
		print("retargetting done!")
		print("location : ../Resource/motion/ETRI/"..new_model_name)
		print("New model totalMass : ", parser.loader:calcTotalMass())
	elseif w:id()=='muscle' then
		redrawMuscles(w:sliderValue())
	elseif w:id()=='Male' then
		SMPL_MALE=w:checkButtonValue()
		SMPL_FEMALE=not w:checkButtonValue()
		this:findWidget("Female"):checkButtonValue(SMPL_FEMALE)
		model_change()
	elseif w:id()=='Female' then
		SMPL_FEMALE=w:checkButtonValue()
		SMPL_MALE=not w:checkButtonValue()
		this:findWidget("Male"):checkButtonValue(SMPL_MALE)
		model_change()
		--this:removeWidgets(0)
	elseif w:id()=='change view' then
		if FRONT_VIEW then
			RE.viewpoint().vpos:assign(vector3(-285.8401448,121.3999705,8.099985091))
			RE.viewpoint().vat:assign(vector3(-9.783222491,92.96317951,0.7809963595))
			RE.viewpoint():update()     
			FRONT_VIEW=not FRONT_VIEW
		else
			RE.viewpoint().vpos:assign(vector3(12, 107, 335))
			RE.viewpoint().vat:assign(vector3(-10.5, 86,0.8))
			RE.viewpoint():update()
			FRONT_VIEW=not FRONT_VIEW
		end
	end
end

function frameMove(fElapsedTime)
	dbg.updateBillboards(fElapsedTime)
	return 0
end
function handleRendererEvent(fElapsedTime)
	return 0
end

function model_change(option_)
	bm_path=nil;
	dd=nil;
	mesh=nil;
	meshToEntity=nil;
	node=nil;
	parser=nil; originalOsimModel=nil; skin=nil;
	if SMPL_MALE then
		bm_path='/smpl/models/basicModel_m_lbs_10_207_0_v1.0.0.npz'
	elseif SMPL_FEMALE then
		bm_path='/smpl/models/basicModel_f_lbs_10_207_0_v1.0.0.npz'
	end

	dd=SMPL.init(smplDir..bm_path)
	mesh=SMPL.createMesh(dd)
	meshToEntity, node=mesh:drawMesh("lightgrey_verytransparent")
	--meshToEntity, node=mesh:drawMesh("white")
	node:scale(100,100,100)

	parser=OsimParser(projectDir..'/Resource/motion/ETRI/Rajagopal_2015_ETRI.osim', {debugDraw=false})
	originalOsimModel=parser:toTable()


	OsimSMPL.gotoSMPL_Tpose(parser.loader)
	OsimSMPL.findBetas(dd, parser.loader)
	OsimSMPL.alignPoses(dd, parser.loader)
	originalOsimJointPos=vector3N(parser.loader:numBone())
	for i=1, parser.loader:numBone()-1 do
		originalOsimJointPos(i):assign(parser.loader:bone(i):getFrame().translation)
	end
	
	local skinScale=100
	for i=0, dd.betas:size() -1 do
		this:findWidget("beta"..i):sliderValue(dd.betas(i))
	end
	skin=RE.createVRMLskin(parser.loader, false)
	skin:setScale(100,100,100)
	skin:setSamePose(parser.loader:fkSolver())

	redrawMuscles()
	origin_muscle_lm=osim:getFiberLengths():vecView():copy()
	origin_l_mt=osim:getTendonMuscleLengths():vecView():copy()
	origin_l_t=(osim:getTendonMuscleLengths()-osim:getFiberLengths()):vecView():copy()
	origin_lm_opt=osim.l_m_opt:vecView():copy()
end




