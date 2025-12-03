require('IPC_based/useCase_walk3')
do -- default settings
--			seg={'lowerUB','throw','flight','landL','handoff','l','B', 'F', 'l2', 'B2', 'r','B3','SL','L','LR','R2', },
	-- roundoff 2
	useCases.handstand=deepCopyTable(useCases.roundoff_common)

	local useCase=useCases.handstand
	useCase.useQPsolver=true
	--useCase.noCOMjoint=true
	useCase.footPosFeedbackMethod=2 -- experimental
	useCase.turnGain=10
	useCase.turnFeedbackTorque=60
	useCase.upperBoundLocal=6
	useCase.amt_sideway_vel=1.94 -- 0.94 is optimal for kinematic controller
	useCase.initialHeight=0.07
	useCase.model_file_name='../Resource/motion/gymnist/gymnist_handstand.wrl' -- override the default defined in common.lua
	--useCase.keyframes.footLmod={numKey=3, isContinuous=false,default=vector3(0,0,0)}
	--useCase.keyframes.footRmod={numKey=3, isContinuous=false,default=vector3(0,0,0)}
	--useCase.keyframes.footLmocapMod={numKey=3, isContinuous=false,default=vector3(0,0,0)}
	--useCase.keyframes.footRmocapMod={numKey=3, isContinuous=false,default=vector3(0,0,0)}
	--useCase.keyframes.asymmetricFootBasis={numKey=3, isContinuous=false,default=0}
	--useCase.keyframes.asymmetricFootMask={numKey=3, isContinuous=false,default=0}

	useCase.asymmetricFoot=vector3(0,0,0)
	useCase.asymmetricFootTimeDependent=vector3(0,0,0)
	useCase.attachCamera=true
	useCase.useAnalyticIK=true
	useCase.useMomentumPreservingIK=false

	useCase.init_globals= function()
		useCases.roundoff_common.init_globals()
		model.k_p_ID=150
		model.k_d_ID=10
		model.k_p_PD=0
		model.k_d_PD=0
		model.clampTorqueID=800
		model.penaltyForceStiffness=20000
		model.penaltyForceDamp=2000
		model.k_scale_active_pd.default={1,1,1}
		model.k_scale_active_pd.collar={1,1,1}
		model.k_scale_active_pd.shoulder={1,1,1}
		model.k_scale_active_pd.elbow={1,1,1}
		model.k_scale_active_pd.knee={1,1,1}
		model.k_scale_active_pd.hip={1,1,1}
		model.k_scale_active_pd.chest={1,1,1} model.k_scale_active_pd.ankle={1,1,1}
		model.k_scale_active_pd.toes={1,1,1}
	end
	useCase.maxContactForceY=480
	useCase.grpName='handstand'
	useCase.grpNames={'handstand', 'walk3'}
	useCase.segNames=useCase.graphParam.handstand.seg
	useCase.segmentation=useCase.segmentations.handstand

	useCase.pendOptExcludePattern= {}

	useCase.mapControlParam=function(graph, title, param)
		local out={}

		assert(string.sub(title, 1,4)=="map,")
		local tokens=string.tokenize(title,',')
		local idx=tonumber(tokens[2])
		local name=tokens[3]

		local convertMap=function(name, mirror)
			local HandIdx=string.find(name, "Hand")
			local key=string.sub(name, 1, HandIdx+3)
			local id=string.lower(string.sub(name, HandIdx+4, HandIdx+4))..string.sub(name, HandIdx+5)
			local convertMap
			if mirror then
				convertMap=
				{
					swingHand={',handR', ',LH'}, 
					spprtHand={',handL', ',LH'},
					landingHand={',handR', ',LRH'},
					kickingHand={',handL', ',LRH'},
				}
			else
				convertMap=
				{
					swingHand={',handL', ',RH'}, 
					spprtHand={',handR', ',RH'},
					landingHand={',handL', ',RLH'},
					kickingHand={',handR', ',RLH'},
				}
			end
			return {convertMap[key][1]..id..",", convertMap[key][2]}
		end

		local grp=tokens[4]
		local axis=tokens[5]
		local cmap=convertMap(name)
		local cmapm=convertMap(name,true)
		local name2='keyframe,'..idx..cmap[1]..grp..cmap[2]
		local name2_mirror='keyframe,'..idx..cmapm[1]..grp..cmapm[2]

		if axis~=nil then
			array.pushBack(out, {name2..","..axis,param}) 
			if axis=='x' then
				array.pushBack(out, {name2_mirror..","..axis,param*-1}) 
			else
				array.pushBack(out, {name2_mirror..","..axis,param}) 
			end
		else
			local param2=param:copy()
			array.pushBack(out, {name2, param2})
			param2.x=param2.x*-1
			array.pushBack(out, {name2_mirror, param2})
		end

		return out
	end
	useCase.graph=table.ijoin(useCases.walk3.graph, {
	--	{
	--		"addInterpolatedSegment",
	--		grpName="handstand",
	--		name="LH",
	--		seg0={"handstand", "LH1"},
	--		seg1={"handstand", "LH0"},
	--		startWeight=0, endWeight=1
	--	},
		{
			"addInterpolatedSegment",
			grpName="handstand",
			name="LRH",
			seg0={"handstand", "LRH1"},
			seg1={"handstand", "LRH0"},
			startWeight=0, endWeight=1
		},
		{"connectMulti",'handstand',  'R',   'RL', 'L',  'LB','B',  'LH0', 'LRH0','RH','RLH','LH','LRH','RH'}, --'LH1','LRH2','RH2','RLH2','LH','LRH2'},
		{"initialSegment", "handstand", "R"}
		--{"initialSegment", "handstand", "RL"}
		--{"initialSegment", "handstand", "LRH0"}
	})

	useCase.keyframes.pendDesiredVel.default=vector3(0,0,0)
	-- the initial values for the follwing two were automatically generated using l ipco_prep
	-- l ipco_prep "grpName='handstand'"

	useCase.pendControlParam={
		['keyframe,0,pendDesiredVel,handstand,R,z']=0,['keyframe,0,pendDesiredVel,handstand,R,x']=0,['keyframe,0,pendDesiredVel,handstand,RL,z']=0,['keyframe,0,pendDesiredVel,handstand,RL,x']=0,['keyframe,0,pendDesiredVel,handstand,L,z']=0,['keyframe,0,pendDesiredVel,handstand,L,x']=0,['keyframe,0,pendDesiredVel,handstand,LB,z']=0,['keyframe,0,pendDesiredVel,handstand,LB,x']=0,['keyframe,0,pendDesiredVel,handstand,B,z']=0,['keyframe,0,pendDesiredVel,handstand,B,x']=0,['keyframe,0,pendDesiredVel,handstand,LH0,z']=0,['keyframe,0,pendDesiredVel,handstand,LH0,x']=0,['keyframe,0,pendDesiredVel,handstand,LRH,z']=0,['keyframe,0,pendDesiredVel,handstand,LRH,x']=0,['keyframe,0,pendDesiredVel,handstand,RH,z']=0,['keyframe,0,pendDesiredVel,handstand,RH,x']=0,['keyframe,0,pendDesiredVel,handstand,RLH,z']=0,['keyframe,0,pendDesiredVel,handstand,RLH,x']=0,['keyframe,0,pendDesiredVel,handstand,LH1,z']=0,['keyframe,0,pendDesiredVel,handstand,LH1,x']=0,['keyframe,0,pendDesiredVel,handstand,LRH2,z']=0,['keyframe,0,pendDesiredVel,handstand,LRH2,x']=0,['keyframe,0,pendDesiredVel,handstand,RH2,z']=0,['keyframe,0,pendDesiredVel,handstand,RH2,x']=0,['keyframe,0,pendDesiredVel,handstand,RLH2,z']=0,['keyframe,0,pendDesiredVel,handstand,RLH2,x']=0,['keyframe,0,pendDesiredVel,handstand,LH2,z']=0,['keyframe,0,pendDesiredVel,handstand,LH2,x']=0,['keyframe,0,pendDesiredVel,handstand,LRH3,z']=0,['keyframe,0,pendDesiredVel,handstand,LRH3,x']=0,['keyframe,0,pendDesiredVel,handstand,RH3,z']=0,['keyframe,0,pendDesiredVel,handstand,RH3,x']=0,['keyframe,0,pendDesiredVel,handstand,RLH3,z']=0,['keyframe,0,pendDesiredVel,handstand,RLH3,x']=0,['keyframe,0,pendDesiredVel,handstand,LH3,z']=0,['keyframe,0,pendDesiredVel,handstand,LH3,x']=0,['keyframe,0,pendDesiredVel,handstand,LRH4,z']=0,['keyframe,0,pendDesiredVel,handstand,LRH4,x']=0,['keyframe,0,pendDesiredVel,handstand,RH4,z']=0,['keyframe,0,pendDesiredVel,handstand,RH4,x']=0,['keyframe,0,pendDesiredVel,handstand,RLH4,z']=0,['keyframe,0,pendDesiredVel,handstand,RLH4,x']=0,['keyframe,0,pendDesiredVel,handstand,LH4,z']=0,['keyframe,0,pendDesiredVel,handstand,LH4,x']=0,['keyframe,0,pendDesiredVel,handstand,B2,z']=0,['keyframe,0,pendDesiredVel,handstand,B2,x']=0,['keyframe,0,pendDesiredVel,handstand,RH5,z']=0,['keyframe,0,pendDesiredVel,handstand,RH5,x']=0,['keyframe,0,pendDesiredVel,handstand,BL,z']=0,['keyframe,0,pendDesiredVel,handstand,BL,x']=0,['keyframe,0,pendDesiredVel,handstand,BLL,z']=0,['keyframe,0,pendDesiredVel,handstand,BLL,x']=0,['keyframe,0,pendDesiredVel,handstand,L2,z']=0,['keyframe,0,pendDesiredVel,handstand,L2,x']=0,['keyframe,0,pendDesiredVel,handstand,LR2,z']=0,['keyframe,0,pendDesiredVel,handstand,LR2,x']=0,['keyframe,0,pendDesiredVel,handstand,R2,z']=0,['keyframe,0,pendDesiredVel,handstand,R2,x']=0,['keyframe,0,pendDesiredVel,ignore,0,z']=0,['keyframe,0,pendDesiredVel,ignore,0,x']=0,['keyframe,0,pendDesiredVel,ignore,1,z']=0,['keyframe,0,pendDesiredVel,ignore,1,x']=0,
        ['keyframe,0,pendDesiredVel,ignore,0,x']= 0.14106972745576, ['keyframe,0,pendDesiredVel,ignore,0,z']= -0.74547252721685, ['keyframe,0,pendDesiredVel,handstand,R,x']= 0.13334641682878, ['keyframe,0,pendDesiredVel,handstand,R,z']= 1.3420870113356, ['keyframe,0,pendDesiredVel,handstand,RL,x']= 0.021270333021046, ['keyframe,0,pendDesiredVel,handstand,RL,z']= 0.096549160544586, ['keyframe,0,pendDesiredVel,handstand,L,x']= -0.048421770255315, ['keyframe,0,pendDesiredVel,handstand,L,z']= -0.15664200666501, ['keyframe,0,pendDesiredVel,handstand,LB,x']= -0.009039710963318, ['keyframe,0,pendDesiredVel,handstand,LB,z']= -0.01341552702107, 
		['keyframe,0,pendDesiredVel,handstand,LB,x']= 3.0410212019134, ['keyframe,0,pendDesiredVel,handstand,LB,z']= 1.5183374290292, ['keyframe,0,pendDesiredVel,handstand,B,x']= -0.2408790947876, ['keyframe,0,pendDesiredVel,handstand,B,z']= 0.22337781674446, ['keyframe,0,pendDesiredVel,handstand,LH0,x']= 0.088646922945729, ['keyframe,0,pendDesiredVel,handstand,LH0,z']= 1.2353743481005, ['keyframe,0,pendDesiredVel,handstand,LRH,x']= 0.48964416678199, ['keyframe,0,pendDesiredVel,handstand,LRH,z']= 0.71442517693779, ['keyframe,0,pendDesiredVel,handstand,RH,x']= 0.08486323114335, ['keyframe,0,pendDesiredVel,handstand,RH,z']= 0.029241618398489, ['keyframe,0,pendDesiredVel,handstand,RLH,x']= -0.066734094943114, ['keyframe,0,pendDesiredVel,handstand,RLH,z']= -0.12329138038641, 
		['keyframe,0,pendDesiredVel,handstand,RLH,x']= -1.1857714379795, ['keyframe,0,pendDesiredVel,handstand,RLH,z']= 2.1061216052177, ['keyframe,0,pendDesiredVel,handstand,LH1,x']= -0.0817702230406, ['keyframe,0,pendDesiredVel,handstand,LH1,z']= 0.42259083774854, ['keyframe,0,pendDesiredVel,handstand,LRH2,x']= 0.78495527323879, ['keyframe,0,pendDesiredVel,handstand,LRH2,z']= -0.30479844204228, ['keyframe,0,pendDesiredVel,handstand,RH2,x']= 0.24516767329499, ['keyframe,0,pendDesiredVel,handstand,RH2,z']= 0.23676096267111, ['keyframe,0,pendDesiredVel,handstand,RLH2,x']= -0.97034077757575, ['keyframe,0,pendDesiredVel,handstand,RLH2,z']= 0.75107488691897, ['keyframe,0,pendDesiredVel,handstand,LH2,x']= 0.069177618251955, ['keyframe,0,pendDesiredVel,handstand,LH2,z']= -0.1463591636769, 
		['keyframe,0,pendDesiredVel,handstand,LH2,x']= 0.49656215233664, ['keyframe,0,pendDesiredVel,handstand,LH2,z']= 0.9395874340057, ['keyframe,0,pendDesiredVel,handstand,LRH3,x']= 0.20597154509352, ['keyframe,0,pendDesiredVel,handstand,LRH3,z']= 0.70346818577941, ['keyframe,0,pendDesiredVel,handstand,RH3,x']= -0.14749740906746, ['keyframe,0,pendDesiredVel,handstand,RH3,z']= 0.92685506527325, ['keyframe,0,pendDesiredVel,handstand,RLH3,x']= -0.033301552736068, ['keyframe,0,pendDesiredVel,handstand,RLH3,z']= 0.31360469668961, ['keyframe,0,pendDesiredVel,handstand,LH3,x']= -0.18247607426231, ['keyframe,0,pendDesiredVel,handstand,LH3,z']= 0.62418836231065, ['keyframe,0,pendDesiredVel,handstand,LRH4,x']= -0.064249203270578, ['keyframe,0,pendDesiredVel,handstand,LRH4,z']= 0.22831053940919, ['keyframe,0,pendDesiredVel,handstand,RH4,x']= -0.15858308775138, ['keyframe,0,pendDesiredVel,handstand,RH4,z']= 0.13247577880827, ['keyframe,0,pendDesiredVel,handstand,RLH4,x']= -0.059912110708138, ['keyframe,0,pendDesiredVel,handstand,RLH4,z']= 0.064903551178492, ['keyframe,0,pendDesiredVel,handstand,LH4,x']= -0.068348640629904, ['keyframe,0,pendDesiredVel,handstand,LH4,z']= 0.043437309921818, ['keyframe,0,pendDesiredVel,handstand,B2,x']= -0.026709026535115, ['keyframe,0,pendDesiredVel,handstand,B2,z']= 0.048009335497024, ['keyframe,0,pendDesiredVel,handstand,RH5,x']= 0.047392082343596, ['keyframe,0,pendDesiredVel,handstand,RH5,z']= 0.069692532745187, ['keyframe,0,pendDesiredVel,handstand,BL,x']= -0.349505733397, ['keyframe,0,pendDesiredVel,handstand,BL,z']= -0.7786529476622, ['keyframe,0,pendDesiredVel,handstand,BLL,x']= 0.022733959101499, ['keyframe,0,pendDesiredVel,handstand,BLL,z']= -0.023093896782886, ['keyframe,0,pendDesiredVel,handstand,L2,x']= 0.063884600344583, ['keyframe,0,pendDesiredVel,handstand,L2,z']= -0.070115137377596, ['keyframe,0,pendDesiredVel,handstand,LR2,x']= -0.062922876203261, ['keyframe,0,pendDesiredVel,handstand,LR2,z']= -0.2350659878443, ['keyframe,0,pendDesiredVel,handstand,R2,x']= -0.019506953957647, ['keyframe,0,pendDesiredVel,handstand,R2,z']= -0.10915963924183, ['keyframe,0,pendDesiredVel,ignore,1,x']= -0.0027247681353956, ['keyframe,0,pendDesiredVel,ignore,1,z']= 0.0086271883605796, 
		['keyframe,0,pendDesiredVel,ignore,0,x']= 0.14109557062104, ['keyframe,0,pendDesiredVel,ignore,0,z']= -0.74533950185994, ['keyframe,0,pendDesiredVel,handstand,R,x']= 0.13340677934322, ['keyframe,0,pendDesiredVel,handstand,R,z']= 1.3426828481662, ['keyframe,0,pendDesiredVel,handstand,RL,x']= 0.021253934155772, ['keyframe,0,pendDesiredVel,handstand,RL,z']= 0.096532222842295, ['keyframe,0,pendDesiredVel,handstand,L,x']= -0.048400019609399, ['keyframe,0,pendDesiredVel,handstand,L,z']= -0.15662437024072, ['keyframe,0,pendDesiredVel,handstand,LB,x']= 3.041024701107, ['keyframe,0,pendDesiredVel,handstand,LB,z']= 1.5183395724802, 
		['keyframe,0,pendDesiredVel,ignore,0,x']= 0.13859873674144, ['keyframe,0,pendDesiredVel,ignore,0,z']= -0.74847885297062, ['keyframe,0,pendDesiredVel,handstand,R,x']= 0.14602368311335, ['keyframe,0,pendDesiredVel,handstand,R,z']= 1.3484487247027, ['keyframe,0,pendDesiredVel,handstand,RL,x']= 0.018362781711896, ['keyframe,0,pendDesiredVel,handstand,RL,z']= 0.093047445153452, ['keyframe,0,pendDesiredVel,handstand,L,x']= -0.051295501584671, ['keyframe,0,pendDesiredVel,handstand,L,z']= -0.16014978137105, ['keyframe,0,pendDesiredVel,handstand,LB,x']= 3.0392480833881, ['keyframe,0,pendDesiredVel,handstand,LB,z']= 1.5174401774272, ['keyframe,0,pendDesiredVel,handstand,B,x']= -0.24470900927716, ['keyframe,0,pendDesiredVel,handstand,B,z']= 0.22459579848127, ['keyframe,0,pendDesiredVel,handstand,LH0,x']= 0.088118030805116, ['keyframe,0,pendDesiredVel,handstand,LH0,z']= 1.2309639931745, ['keyframe,0,pendDesiredVel,handstand,LRH,x']= 0.48911430208492, ['keyframe,0,pendDesiredVel,handstand,LRH,z']= 0.71082077322036, ['keyframe,0,pendDesiredVel,handstand,RH,x']= 0.085806430514341, ['keyframe,0,pendDesiredVel,handstand,RH,z']= 0.024897060081271, ['keyframe,0,pendDesiredVel,handstand,RLH,x']= -1.186167995837, ['keyframe,0,pendDesiredVel,handstand,RLH,z']= 2.1032441649435, ['keyframe,0,pendDesiredVel,handstand,LH1,x']= -0.082620076212238, ['keyframe,0,pendDesiredVel,handstand,LH1,z']= 0.41808938153006, ['keyframe,0,pendDesiredVel,handstand,LRH2,x']= 0.78395464079977, ['keyframe,0,pendDesiredVel,handstand,LRH2,z']= -0.30745302302949, ['keyframe,0,pendDesiredVel,handstand,RH2,x']= 0.24577161526914, ['keyframe,0,pendDesiredVel,handstand,RH2,z']= 0.2323400625319, ['keyframe,0,pendDesiredVel,handstand,RLH2,x']= -0.96921313534941, ['keyframe,0,pendDesiredVel,handstand,RLH2,z']= 0.74659865122844, ['keyframe,0,pendDesiredVel,handstand,LH2,x']= 0.49546627802188, ['keyframe,0,pendDesiredVel,handstand,LH2,z']= 0.93781896000551, ['keyframe,0,pendDesiredVel,handstand,LRH3,x']= 0.20537779112713, ['keyframe,0,pendDesiredVel,handstand,LRH3,z']= 0.7023466500488, ['keyframe,0,pendDesiredVel,handstand,RH3,x']= -0.14713535213952, ['keyframe,0,pendDesiredVel,handstand,RH3,z']= 0.92580952448882, ['keyframe,0,pendDesiredVel,handstand,RLH3,x']= -0.03336329284673, ['keyframe,0,pendDesiredVel,handstand,RLH3,z']= 0.31321186263504, ['keyframe,0,pendDesiredVel,handstand,LH3,x']= -0.18197929574389, ['keyframe,0,pendDesiredVel,handstand,LH3,z']= 0.6246545104387, ['keyframe,0,pendDesiredVel,handstand,LRH4,x']= -0.064447018934756, ['keyframe,0,pendDesiredVel,handstand,LRH4,z']= 0.22850334241719, ['keyframe,0,pendDesiredVel,handstand,RH4,x']= -0.1586637249334, ['keyframe,0,pendDesiredVel,handstand,RH4,z']= 0.13272764499766, ['keyframe,0,pendDesiredVel,handstand,RLH4,x']= -0.060092532985527, ['keyframe,0,pendDesiredVel,handstand,RLH4,z']= 0.064968361696592, ['keyframe,0,pendDesiredVel,handstand,LH4,x']= -0.068479496069573, ['keyframe,0,pendDesiredVel,handstand,LH4,z']= 0.043588972138202, ['keyframe,0,pendDesiredVel,handstand,B2,x']= -0.027001672371552, ['keyframe,0,pendDesiredVel,handstand,B2,z']= 0.047699152614741, ['keyframe,0,pendDesiredVel,handstand,RH5,x']= 0.046735981836527, ['keyframe,0,pendDesiredVel,handstand,RH5,z']= 0.069011476518519, ['keyframe,0,pendDesiredVel,handstand,BL,x']= -0.34696955302823, ['keyframe,0,pendDesiredVel,handstand,BL,z']= -0.77242607859108, ['keyframe,0,pendDesiredVel,handstand,BLL,x']= 0.022844405042025, ['keyframe,0,pendDesiredVel,handstand,BLL,z']= -0.022839714102458, ['keyframe,0,pendDesiredVel,handstand,L2,x']= 0.064769429409476, ['keyframe,0,pendDesiredVel,handstand,L2,z']= -0.068380499404554, ['keyframe,0,pendDesiredVel,handstand,LR2,x']= -0.060375399881301, ['keyframe,0,pendDesiredVel,handstand,LR2,z']= -0.22995344529581, ['keyframe,0,pendDesiredVel,handstand,R2,x']= -0.019261698849804, ['keyframe,0,pendDesiredVel,handstand,R2,z']= -0.10821351796755, ['keyframe,0,pendDesiredVel,ignore,1,x']= -0.002707043804668, ['keyframe,0,pendDesiredVel,ignore,1,z']= 0.0085494479840587, 
	}
	table.mergeInPlace(useCase.pendControlParam, 
	{
		['keyframe,0,pendDesiredVel,handstand,RH,x']=  0.08,
		['keyframe,0,pendDesiredVel,handstand,RH,z']=  0.5,
		['keyframe,0,pendDesiredVel,handstand,LH,x']= -0.08,
		['keyframe,0,pendDesiredVel,handstand,LH,z']= 0.5,
		['keyframe,0,pendDesiredVel,handstand,LH0,x']= -0.08,
		['keyframe,0,pendDesiredVel,handstand,LH0,z']= 0.5,
		['keyframe,0,pendDesiredVel,handstand,LRH0,x']= 0,
		['keyframe,0,pendDesiredVel,handstand,LRH0,z']= 0.5,
		['keyframe,0,pendDesiredVel,handstand,LRH1,x']= 0,
		['keyframe,0,pendDesiredVel,handstand,LRH1,z']= 0.5,
	}, true
	)
	useCase.pendOptimizationPath=
	{
		firstFrames={4392,  4410,        4540,           4565,        4594,          4604,         4689,           4715,          4735,           4761,            4777,          4811,            4829,            4862,          4912,            4939,            4957,            4987,           4998,          5037,              5058,          5085,             5103,          5132,          5165,           5191,          5333,           5345,          5396,           5543,         5604,       5636,},
		segments={'ignore,0','handstand,R','handstand,RL','handstand,L','handstand,LB','handstand,B','handstand,LH0','handstand,LRH0','handstand,RH','handstand,RLH','handstand,LH','handstand,LRH1','handstand,RH2','handstand,RLH2','handstand,LH2','handstand,LRH3','handstand,RH3','handstand,RLH3','handstand,LH3','handstand,LRH4','handstand,RH4','handstand,RLH4','handstand,LH4','handstand,B2','handstand,RH5','handstand,BL','handstand,BLL','handstand,L2','handstand,LR2','handstand,R2','ignore,1'},
	}
	useCases.handstand.pendOptExcludePattern= { '.*rl.*','.*lr.*','.*RL.*','.*LR.*'}

	function useCase.funcUpdateConstraints(graph)
		useCases.defaultFuncUpdateConstraintsSub(useCase,graph,'pendDesiredVel',1,useCases.handstand.pendOptExcludePattern)
		useCases.defaultFuncUpdateConstraints(useCase,graph,
		{
			footLmod=useCase.keyframes.footLmod,
			footRmod=useCase.keyframes.footRmod,
			handLmod=useCase.keyframes.handLmod,
			handRmod=useCase.keyframes.handRmod,
			footLmocapMod=useCase.keyframes.footLmocapMod,
			footRmocapMod=useCase.keyframes.footRmocapMod,
			desiredHeadAcc=useCase.keyframes.desiredHeadAcc,
			desiredMomentum=useCase.keyframes.desiredMomentum,
		})
	end
	useCase.measureOptCost=useCases.measureOptCost
	useCase.measureOptCost=nil
	
end

do -- prepare full-body optimization
	local useCase=useCases.handstand
	local objCost='objCost_backflip'

	--List=require("functional/list")
	local genOpt=function(starts, ends, ps, pe, name, axis, others)
		local out= useCases.genOpt(useCase, starts, ends, ps, pe, name, axis, others)
		out.objCost=objCost
		return out
	end

	useCases.rectifyGraph(useCases.handstand)

	local function makeOptParam(numKey,...)
		local gtarget=array:new()
		local prec=0.01
		local symmetricMapUsed=true
		for iid,id in ipairs({...}) do
			for i=0, numKey-1 do
				if string.sub(id,1,3)=="MAP" then
					gtarget:pushBack({"map,"..i..","..string.sub(id,4)..",x",prec})
					gtarget:pushBack({"map,"..i..","..string.sub(id,4)..",y",prec})
					gtarget:pushBack({"map,"..i..","..string.sub(id,4)..",z",prec})
					symmetricMapUsed=true
				else
					gtarget:pushBack({"keyframe,"..i..","..id..",x",prec})
					gtarget:pushBack({"keyframe,"..i..","..id..",y",prec})
					gtarget:pushBack({"keyframe,"..i..","..id..",z",prec})
				end
			end
		end
		return gtarget
	end
	local function makeOptParamAxis(numKey,axis,...)
		local gtarget=array:new()
		local prec=0.01
		local symmetricMapUsed=false
		for iid,id in ipairs({...}) do
			for i=0, numKey-1 do
				if string.sub(id,1,3)=="MAP" then
					gtarget:pushBack({"map,"..i..","..string.sub(id,4)..","..axis,prec})
					--symmetricMapUsed=true
				else
					gtarget:pushBack({"keyframe,"..i..","..id..","..axis,prec})
				end
			end
		end
		--[[if symmetricMapUsed then
			gtarget:pushBack({"useCases,walk3,asymmetricFoot,"..axis,prec})
			if turning then
				gtarget:pushBack({"useCases,walk3,asymmetricFootTimeDependent,x",prec})
				gtarget:pushBack({"useCases,walk3,asymmetricFootTimeDependent,y",prec})
				--gtarget:pushBack({'useCases,walk3,turnGain',0.4})
				--gtarget:pushBack({'useCases,walk3,turnFeedbackTorque',1})
			end
			gtarget:pushBack({"useCases,walk3,asymmetricFootTimeDependent,z",prec})
		end]]--

		return gtarget
	end
	-- now let's define optimization stages
	function useCases.handstand:updateStageParam(stage, stageParam, best_eval, best_eval_unscaled)
		local endSegW=29
		if false then
			stageParam[stage]={startSeg=1, endSeg=endSegW+stage, nvar=2,setStageFunc="setStage_param", param=useCases.walk3._makeOptParam(3,"MAPswingHandMod,handstand","MAPspprtHandMod,handstand"), objCost=objCost,baseStage=1}
		elseif false and (stage==1 or stage==3) then
			stageParam[stage]=useCases.genOpt(useCases.handstand, 9, endSegW, 5, 6,'handstand')
		elseif false and stage==1 then
			stageParam[stage]=useCases.genOpt(useCases.handstand, 1, 5, 1, 1,'handstand', nil, {'desiredLeaning'})
		elseif false then
			-- doesn't assume symmetry
			local mod=math.mod(stage-1,4)
			if mod==0 then 
				-- footall
	--			stageParam[stage]=useCases.genOpt(useCases.handstand, 8, endSegW, 6+6, 8+6,'handstand', nil, {'handLmod_all','handRmod_all'})
				stageParam[stage]=useCases.genOpt(useCases.handstand, 8, endSegW, 7, 10,'handstand', {'y'}, {'handLmod', 'handRmod'})
			elseif mod==1 then
				stageParam[stage]=useCases.genOpt(useCases.handstand, 8, endSegW, 7, 10,'handstand',{'x','y'}, {'handLmod', 'handRmod'})
			elseif mod==2 then
				stageParam[stage]=useCases.genOpt(useCases.handstand, 8, endSegW, 7, 10,'handstand',{'x','z'}, {'handLmod', 'handRmod'})
			else
				stageParam[stage]=useCases.genOpt(useCases.handstand, 8, endSegW, 7, 10,'handstand',{'y','z'}, {'handLmod', 'handRmod'})
			end
		elseif true then
			--stageParam[stage]={startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingHandMod,handstand","MAPspprtHandMod,handstand"), objCost=objCost,baseStage=1}
			local mod=math.mod(stage-1,4)
			if mod==0 then
				stageParam[stage]={startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", param=makeOptParam(3,"MAPswingHandMod,handstand","MAPspprtHandMod,handstand"), objCost=objCost,baseStage=1}
			elseif mod==1 then
				stageParam[stage]={startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", param=makeOptParamAxis(3,'z',"MAPswingHandMod,handstand","MAPspprtHandMod,handstand"), objCost=objCost,baseStage=1}
			elseif mod==2 then
				stageParam[stage]={startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", param=makeOptParamAxis(3,'x',"MAPswingHandMod,handstand","MAPspprtHandMod,handstand"), objCost=objCost,baseStage=1}
			elseif mod==3 then
				stageParam[stage]={startSeg=1, endSeg=endSegW, nvar=2,setStageFunc="setStage_param", param=makeOptParamAxis(3,'y',"MAPswingHandMod,handstand","MAPspprtHandMod,handstand"), objCost=objCost,baseStage=1}
			end
		else -- 
			local working=3
			--local working=9
			local startOpt=5
			useCases.updateStageParam(self, stage, stageParam, working, genOpt, best_eval, best_eval_unscaled, startOpt)
		end
	end
end
do
	-- opt result
	local function accumulate(cp_mod)
		local useCase=useCases.handstand
		useCases.accumulate(useCase, cp_mod)
	end
	table.mergeInPlace(useCases.handstand.controlParam, {
	['keyframe,0,desiredLeaning,handstand,R,x']=0.041115976717226, 
	['keyframe,0,desiredLeaning,handstand,R,z']=-0.0066649614049112, 
	--['keyframe,0,desiredMomentum,handstand,B,x']=0.030623925842745, 
	--['keyframe,0,desiredMomentum,handstand,B,y']=-0.072463346138439, 
	--['keyframe,0,desiredMomentum,handstand,B,z']=0.071942285798808, 
	--['keyframe,0,desiredMomentum,handstand,L,x']=-0.20106313605895, 
	--['keyframe,0,desiredMomentum,handstand,L,y']=0.37675040657354, 
	--['keyframe,0,desiredMomentum,handstand,L,z']=-0.32834636178547, 
	--['keyframe,0,desiredMomentum,handstand,LH,x']=0.05, 
	--['keyframe,0,desiredMomentum,handstand,LH,y']=-0.038850295826823, 
	--['keyframe,0,desiredMomentum,handstand,LH,z']=0.05, 
	--['keyframe,0,desiredMomentum,handstand,LH0,x']=0.14927062266112, 
	--['keyframe,0,desiredMomentum,handstand,LH0,y']=-0.21787810089516, 
	--['keyframe,0,desiredMomentum,handstand,LH0,z']=-0.24324140732359, 
	--['keyframe,0,desiredMomentum,handstand,LH1,x']=38.985938821633, 
	--['keyframe,0,desiredMomentum,handstand,LH1,y']=42.16423946864, 
	--['keyframe,0,desiredMomentum,handstand,LH1,z']=-19.478774393543, 
	--['keyframe,0,desiredMomentum,handstand,R,x']=0.15818149499495, 
	--['keyframe,0,desiredMomentum,handstand,R,y']=0.13207476652942, 
	--['keyframe,0,desiredMomentum,handstand,R,z']=-0.013668877915112, 
	--['keyframe,0,desiredMomentum,handstand,RH,x']=7.4594470887694, 
	--['keyframe,0,desiredMomentum,handstand,RH,y']=-9.7488150604878, 
	--['keyframe,0,desiredMomentum,handstand,RH,z']=11.933299135737, 
	--['keyframe,0,desiredMomentum,handstand,RH2,x']=-7.3348631660017, 
	--['keyframe,0,desiredMomentum,handstand,RH2,y']=-5.5313087679787, 
	--['keyframe,0,desiredMomentum,handstand,RH2,z']=-1.0017117012513, 
	['keyframe,0,footLmod,handstand,B,x']=-0.0071914128284834, 
	['keyframe,0,footLmod,handstand,B,y']=0.0044794553270068, 
	['keyframe,0,footLmod,handstand,B,z']=0.12108031101752, 
	['keyframe,0,footLmod,handstand,L,x']=0.020812998608348, 
	['keyframe,0,footLmod,handstand,L,y']=0.0047143251291175, 
	['keyframe,0,footLmod,handstand,L,z']=0.0014178520489586, 
	['keyframe,0,footLmod,handstand,R,x']=0.024433597290204, 
	['keyframe,0,footLmod,handstand,R,y']=-0.01641407552648, 
	['keyframe,0,footLmod,handstand,R,z']=-0.0017248391684289, 
	['keyframe,0,footRmod,handstand,L,x']=0.026484844478812, 
	['keyframe,0,footRmod,handstand,L,y']=-0.012511694889439, 
	['keyframe,0,footRmod,handstand,L,z']=-0.020162215333506, 
	['keyframe,0,footRmod,handstand,R,x']=-0.023771382428324, 
	['keyframe,0,footRmod,handstand,R,y']=-0.042358960870446, 
	['keyframe,0,footRmod,handstand,R,z']=-0.024748942004237, 
	['keyframe,0,handLmod,handstand,B,x']=-0.012783219482239, 
	['keyframe,0,handLmod,handstand,B,y']=-0.015120668652124, 
	['keyframe,0,handLmod,handstand,B,z']=-0.0017486687357396, 
	['keyframe,0,handLmod,handstand,LH,x']=0.0045504072241685, 
	['keyframe,0,handLmod,handstand,LH,y']=0.020840329102037, 
	['keyframe,0,handLmod,handstand,LH,z']=0.045392897760709, 
	['keyframe,0,handLmod,handstand,LH0,x']=0.060074055729317, 
	['keyframe,0,handLmod,handstand,LH0,y']=-0.061776160276512, 
	['keyframe,0,handLmod,handstand,LH0,z']=-0.01362961162517, 
	['keyframe,0,handLmod,handstand,LH1,x']=-0.027286041720815, 
	['keyframe,0,handLmod,handstand,LH1,y']=0.020436560129028, 
	['keyframe,0,handLmod,handstand,LH1,z']=-0.012961330120569, 
	['keyframe,0,handLmod,handstand,RH,x']=0.0058970100534626, 
	['keyframe,0,handLmod,handstand,RH,y']=0.19081590625921, 
	['keyframe,0,handLmod,handstand,RH,z']=-0.022240975551638, 
	['keyframe,0,handLmod,handstand,RH2,x']=-0.021394798992917, 
	['keyframe,0,handLmod,handstand,RH2,y']=0.020412801418176, 
	['keyframe,0,handLmod,handstand,RH2,z']=0.01104721300924, 
	['keyframe,0,handRmod,handstand,B,x']=0.025552410089029, 
	['keyframe,0,handRmod,handstand,B,y']=0.031867093199896, 
	['keyframe,0,handRmod,handstand,B,z']=0.14062523656416, 
	['keyframe,0,handRmod,handstand,LH,x']=-0.026194379818699, 
	['keyframe,0,handRmod,handstand,LH,y']=0.0098781256369107, 
	['keyframe,0,handRmod,handstand,LH,z']=-0.0057876079775072, 
	['keyframe,0,handRmod,handstand,LH0,x']=-0.033504093130224, 
	['keyframe,0,handRmod,handstand,LH0,y']=0.016884653313291, 
	['keyframe,0,handRmod,handstand,LH0,z']=0.013972301885838, 
	['keyframe,0,handRmod,handstand,LH1,x']=0.068956960282723, 
	['keyframe,0,handRmod,handstand,LH1,y']=0.055353816058648, 
	['keyframe,0,handRmod,handstand,LH1,z']=-0.12799843870275, 
	['keyframe,0,handRmod,handstand,RH,x']=-0.016610036292982, 
	['keyframe,0,handRmod,handstand,RH,y']=0.012407719145849, 
	['keyframe,0,handRmod,handstand,RH,z']=-0.012216360831408, 
	['keyframe,0,handRmod,handstand,RH2,x']=0.00556369635584, 
	['keyframe,0,handRmod,handstand,RH2,y']=0.00091709351134316, 
	['keyframe,0,handRmod,handstand,RH2,z']=0.0097129689975316, 
	--['keyframe,1,desiredMomentum,handstand,L,x']=-0.33504017183971, 
	--['keyframe,1,desiredMomentum,handstand,L,y']=-0.25839143496886, 
	--['keyframe,1,desiredMomentum,handstand,L,z']=0.29926013233725, 
	--['keyframe,1,desiredMomentum,handstand,LH,x']=-0.05, 
	--['keyframe,1,desiredMomentum,handstand,LH,y']=0.05, 
	--['keyframe,1,desiredMomentum,handstand,LH,z']=-0.05, 
	--['keyframe,1,desiredMomentum,handstand,LH0,x']=0.3834638788034, 
	--['keyframe,1,desiredMomentum,handstand,LH0,y']=-0.096501564609967, 
	--['keyframe,1,desiredMomentum,handstand,LH0,z']=0.19185453126648, 
	--['keyframe,1,desiredMomentum,handstand,LH1,x']=-4.6376919087666, 
	--['keyframe,1,desiredMomentum,handstand,LH1,y']=-0.62157855829656, 
	--['keyframe,1,desiredMomentum,handstand,LH1,z']=-1.8160733180702, 
	--['keyframe,1,desiredMomentum,handstand,R,x']=-0.18109868989273, 
	--['keyframe,1,desiredMomentum,handstand,R,y']=-0.24856873832193, 
	--['keyframe,1,desiredMomentum,handstand,R,z']=0.15923465974856, 
	--['keyframe,1,desiredMomentum,handstand,RH,x']=7.1371686929548, 
	--['keyframe,1,desiredMomentum,handstand,RH,y']=-14.646024908833, 
	--['keyframe,1,desiredMomentum,handstand,RH,z']=-1.3732755103964, 
	--['keyframe,1,desiredMomentum,handstand,RH2,x']=56.452179806397, 
	--['keyframe,1,desiredMomentum,handstand,RH2,y']=0.82235078734168, 
	--['keyframe,1,desiredMomentum,handstand,RH2,z']=10.446271830733, 
	['keyframe,1,footLmod,handstand,L,x']=0.0043551471513481, 
	['keyframe,1,footLmod,handstand,L,y']=-0.00019527685830161, 
	['keyframe,1,footLmod,handstand,L,z']=0.0041497425831664, 
	['keyframe,1,footLmod,handstand,R,x']=0.031365390593772, 
	['keyframe,1,footLmod,handstand,R,y']=-0.024200779090238, 
	['keyframe,1,footLmod,handstand,R,z']=0.0014129002256806, 
	['keyframe,1,footRmod,handstand,L,x']=0.0038389042497842, 
	['keyframe,1,footRmod,handstand,L,y']=-0.0025584109864192, 
	['keyframe,1,footRmod,handstand,L,z']=-0.005419144676737, 
	['keyframe,1,footRmod,handstand,R,x']=-0.013722632268005, 
	['keyframe,1,footRmod,handstand,R,y']=-0.013262928340321, 
	['keyframe,1,footRmod,handstand,R,z']=-0.025939594940479, 
	['keyframe,1,handLmod,handstand,LH,x']=0.024147210559189, 
	['keyframe,1,handLmod,handstand,LH,y']=-0.01060362767864, 
	['keyframe,1,handLmod,handstand,LH,z']=-0.01355660212307, 
	['keyframe,1,handLmod,handstand,LH0,x']=0.044640763964107, 
	['keyframe,1,handLmod,handstand,LH0,y']=-0.089843474485899, 
	['keyframe,1,handLmod,handstand,LH0,z']=-0.0080027116002891, 
	['keyframe,1,handLmod,handstand,LH1,x']=-0.40784790442703, 
	['keyframe,1,handLmod,handstand,LH1,y']=0.24050885499893, 
	['keyframe,1,handLmod,handstand,LH1,z']=0.049612496455093, 
	['keyframe,1,handLmod,handstand,RH,x']=-0.018531060303731, 
	['keyframe,1,handLmod,handstand,RH,y']=-0.023391841927612, 
	['keyframe,1,handLmod,handstand,RH,z']=-0.0074454417924282, 
	['keyframe,1,handLmod,handstand,RH2,x']=0.018583678925406, 
	['keyframe,1,handLmod,handstand,RH2,y']=-0.027184315921988, 
	['keyframe,1,handLmod,handstand,RH2,z']=-0.0085093425406411, 
	['keyframe,1,handRmod,handstand,LH,x']=-0.023159704967765, 
	['keyframe,1,handRmod,handstand,LH,y']=0.0053814790347483, 
	['keyframe,1,handRmod,handstand,LH,z']=-0.0067541771739996, 
	['keyframe,1,handRmod,handstand,LH0,x']=-0.032800282997232, 
	['keyframe,1,handRmod,handstand,LH0,y']=0.0049412837491841, 
	['keyframe,1,handRmod,handstand,LH0,z']=0.0012507701875922, 
	['keyframe,1,handRmod,handstand,LH1,x']=-0.40505643598686, 
	['keyframe,1,handRmod,handstand,LH1,y']=0.1235144212967, 
	['keyframe,1,handRmod,handstand,LH1,z']=-0.014953920764743, 
	['keyframe,1,handRmod,handstand,RH,x']=0.015441916890318, 
	['keyframe,1,handRmod,handstand,RH,y']=-0.061095930616145, 
	['keyframe,1,handRmod,handstand,RH,z']=0.014304647544181, 
	['keyframe,1,handRmod,handstand,RH2,x']=0.0083540145064649, 
	['keyframe,1,handRmod,handstand,RH2,y']=-0.036270064881041, 
	['keyframe,1,handRmod,handstand,RH2,z']=-0.003354774948099, 
	--['keyframe,2,desiredMomentum,handstand,L,x']=0.26744461443047, 
	--['keyframe,2,desiredMomentum,handstand,L,y']=0.14148816626198, 
	--['keyframe,2,desiredMomentum,handstand,L,z']=0.16562589922469, 
	--['keyframe,2,desiredMomentum,handstand,LH,x']=-0.05, 
	--['keyframe,2,desiredMomentum,handstand,LH,y']=0.05, 
	--['keyframe,2,desiredMomentum,handstand,LH,z']=0.049382684878636, 
	--['keyframe,2,desiredMomentum,handstand,LH0,x']=0.38323551532997, 
	--['keyframe,2,desiredMomentum,handstand,LH0,y']=0.17061153315504, 
	--['keyframe,2,desiredMomentum,handstand,LH0,z']=-0.17110452545989, 
	--['keyframe,2,desiredMomentum,handstand,R,x']=0.049445209033986, 
	--['keyframe,2,desiredMomentum,handstand,R,y']=-0.019030258279676, 
	--['keyframe,2,desiredMomentum,handstand,R,z']=-0.25119174548343, 
	--['keyframe,2,desiredMomentum,handstand,RH,x']=-7.3545039263631, 
	--['keyframe,2,desiredMomentum,handstand,RH,y']=5.2499308898996, 
	--['keyframe,2,desiredMomentum,handstand,RH,z']=0.60957462455032, 
	--['keyframe,2,desiredMomentum,handstand,RH2,x']=-9.0704334890321, 
	--['keyframe,2,desiredMomentum,handstand,RH2,y']=-6.0859016101127, 
	--['keyframe,2,desiredMomentum,handstand,RH2,z']=11.752541017672, 
	['keyframe,2,footLmod,handstand,L,x']=0.005788806660972, 
	['keyframe,2,footLmod,handstand,L,y']=0.027732470333489, 
	['keyframe,2,footLmod,handstand,L,z']=-0.014114853388075, 
	['keyframe,2,footLmod,handstand,R,x']=0.016454194143006, 
	['keyframe,2,footLmod,handstand,R,y']=-0.027698354455919, 
	['keyframe,2,footLmod,handstand,R,z']=0.0048377561411194, 
	['keyframe,2,footRmod,handstand,L,x']=0.0038389042497842, 
	['keyframe,2,footRmod,handstand,L,y']=-0.0025584109864192, 
	['keyframe,2,footRmod,handstand,L,z']=-0.005419144676737, 
	['keyframe,2,footRmod,handstand,R,x']=-0.009138726402264, 
	['keyframe,2,footRmod,handstand,R,y']=-0.021123966087703, 
	['keyframe,2,footRmod,handstand,R,z']=0.0041745145072535, 
	['keyframe,2,handLmod,handstand,LH,x']=0.0043093912519029, 
	['keyframe,2,handLmod,handstand,LH,y']=-0.015032251021214, 
	['keyframe,2,handLmod,handstand,LH,z']=0.0029095970785754, 
	['keyframe,2,handLmod,handstand,LH0,x']=0.04734372103738, 
	['keyframe,2,handLmod,handstand,LH0,y']=-0.085629899033128, 
	['keyframe,2,handLmod,handstand,LH0,z']=-0.016799207997088, 
	['keyframe,2,handLmod,handstand,RH,x']=0.17906888296881, 
	['keyframe,2,handLmod,handstand,RH,y']=-0.021532638328541, 
	['keyframe,2,handLmod,handstand,RH,z']=-0.027378640560885, 
	['keyframe,2,handLmod,handstand,RH2,x']=-0.015297161192177, 
	['keyframe,2,handLmod,handstand,RH2,y']=-0.019150542739454, 
	['keyframe,2,handLmod,handstand,RH2,z']=-0.063180146254064, 
	['keyframe,2,handRmod,handstand,LH,x']=0.0062862866855816, 
	['keyframe,2,handRmod,handstand,LH,y']=-0.010075734475372, 
	['keyframe,2,handRmod,handstand,LH,z']=-0.032197744066482, 
	['keyframe,2,handRmod,handstand,LH0,x']=-0.0040381143149253, 
	['keyframe,2,handRmod,handstand,LH0,y']=0.0025189671360569, 
	['keyframe,2,handRmod,handstand,LH0,z']=0.024587740947909, 
	['keyframe,2,handRmod,handstand,RH,x']=-0.2199922721946, 
	['keyframe,2,handRmod,handstand,RH,y']=0.024549409418911, 
	['keyframe,2,handRmod,handstand,RH,z']=-0.028732646317013, 
	['keyframe,2,handRmod,handstand,RH2,x']=-0.038379882223069, 
	['keyframe,2,handRmod,handstand,RH2,y']=-0.013343451107013, 
	['keyframe,2,handRmod,handstand,RH2,z']=-0.02791733537745, 
	['useCases,handstand,COMobjWeight']=0, 
	['useCases,handstand,EEobjAngular']=6000, 
	['useCases,handstand,EEobjWeight']=60000, 
	['useCases,handstand,EEobjWeightAngular']=60000, 
	['useCases,handstand,contactMargin']=0.01, 
	['useCases,handstand,ddqObjWeight']=10000, 
	['useCases,handstand,dotMomentumScale']=1, 
	['useCases,handstand,excludeRoot']=true, 
	['useCases,handstand,headControlWeight']=6000, 
	['useCases,handstand,k_d_EE']=44, 
	['useCases,handstand,k_d_ID']=25, 
	['useCases,handstand,k_d_momentum']=12, 
	['useCases,handstand,k_p_EE']=250, 
	['useCases,handstand,k_p_ID']=250, 
	['useCases,handstand,lambdaObjWeight']=0.0001, 
	['useCases,handstand,linearMomentumWeight']=6000, 
	['useCases,handstand,modifyFootSim']=true, 
	['useCases,handstand,momentumThr']=100, 
	['useCases,handstand,momentumWeight']=6000, 
	['useCases,handstand,tauObjWeight']=0.0001, 
	['useCases,handstand,useBulletColdet']=false, 
}
)
accumulate({
	['useCases,handstand,headControlWeight']=0, 
	['useCases,handstand,k_d_momentum']=10, 
	['useCases,handstand,maxPenetratingVel']=0, 
	['useCases,handstand,k_p_EE']=120*0.7, 
	['useCases,handstand,k_d_EE']=24*0.5, 
	['useCases,handstand,k_p_ID']=70*1.4, 
	['useCases,handstand,k_d_ID']=12, 
	--['useCases,handstand,momentumThr']=10, 
	['useCases,handstand,numericalDerivDmot']=true, 
	['useCases,handstand,perClassContactMargin']=1, 
	['useCases,handstand,velMarginOffset']=0, 
	--['useCases,handstand,momentumWeight']=3000, 
	--['useCases,handstand,useBulletColdet']=true, 
	['useCases,handstand,dotMomentumScale']=0.3, 
	['useCases,handstand,contactMargin']=0.0001, 
})
	useCases.clampParam(useCases.handstand, 'handRmod,handstand', 0.2)
	useCases.clampParam(useCases.handstand, 'handLmod,handstand', 0.2)
	local function modify(cp_mod)
		local useCase=useCases.handstand
		for k,v in pairs(cp_mod) do
			if(useCase.controlParam[k]==nil) then
				--print('warning: setting controlParam['..k..']=0')
				useCase.controlParam[k]=0
			end
			useCase.controlParam[k]=useCase.controlParam[k]+v
		end
	end
	modify({
		['keyframe,0,handLmod,handstand,B,y']=0,
		['keyframe,0,handRmod,handstand,B,y']=0,
	})
-- tropt.lua: 	Tue Aug 27 16:17:34 2013

--	------------------New optimization-------- [1e+100,-1e+100] -> oeval:131908.62851823 dim:20}) -- skipped oeval:131908.62851823 
--0.000 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 
--Tue Aug 27 10:39:55 2013 setStage2 startSeg:1 endSeg:10 2 LRLean1 [8.8924645792374e-06,1.0283088206616e-05] -> oeval:131908.62851823 dim:0}) -- skipped oeval:131908.62851823 

accumulate({ ['keyframe,0,footLmod,handstand,R,x']= 0.02474263254642, ['keyframe,0,footLmod,handstand,R,y']= -0.016936205133505, ['keyframe,0,footLmod,handstand,R,z']= -0.0018930155850212, ['keyframe,1,footLmod,handstand,R,x']= 0.030572816958843, ['keyframe,1,footLmod,handstand,R,y']= -0.025146246131325, ['keyframe,1,footLmod,handstand,R,z']= 0.0011816817086188, ['keyframe,2,footLmod,handstand,R,x']= 0.016372586650929, ['keyframe,2,footLmod,handstand,R,y']= -0.027916286386128, ['keyframe,2,footLmod,handstand,R,z']= 0.0044954813322229, ['keyframe,0,footRmod,handstand,R,x']= -0.023629183513127, ['keyframe,0,footRmod,handstand,R,y']= -0.042575657115822, ['keyframe,0,footRmod,handstand,R,z']= -0.024234694706492, ['keyframe,1,footRmod,handstand,R,x']= -0.01419640559971, ['keyframe,1,footRmod,handstand,R,y']= -0.0133757254029, ['keyframe,1,footRmod,handstand,R,z']= -0.026126547735081, ['keyframe,2,footRmod,handstand,R,x']= -0.0093799368961062, ['keyframe,2,footRmod,handstand,R,y']= -0.020823235938254, ['keyframe,2,footRmod,handstand,R,z']= 0.003428080817127, ['keyframe,0,desiredLeaning,handstand,R,x']= 0.041216025039154, ['keyframe,0,desiredLeaning,handstand,R,z']= -0.0065604860682141, })

--Tue Aug 27 10:41:31 2013 setStage3 startSeg:1 endSeg:10 3 LRLean1 [1e+100,-1e+100] -> oeval:131908.62851823 dim:12}) -- skipped oeval:131908.62851823 

accumulate({ }) -- skipped iter :-1 
--1.003 --0.000 --0.892 --0.000 --0.000 --0.892 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 
--Tue Aug 27 12:36:47 2013 setStage4 startSeg:1 endSeg:10 4 LRLean1 [1.5876493548064e-05,1.7161872187822e-05] -> oeval:131908.62851823 dim:0}) -- skipped oeval:131908.62851823 

accumulate({ ['keyframe,0,footLmod,handstand,L,x']= 0.019486981969167, ['keyframe,0,footLmod,handstand,L,y']= -0.021787159547639, ['keyframe,0,footLmod,handstand,L,z']= -3.9318761752533e-05, ['keyframe,1,footLmod,handstand,L,x']= 0.0047433647731906, ['keyframe,1,footLmod,handstand,L,y']= -0.00022777873177317, ['keyframe,1,footLmod,handstand,L,z']= 0.0072180183723209, ['keyframe,2,footLmod,handstand,L,x']= 0.0061531366649318, ['keyframe,2,footLmod,handstand,L,y']= 0.027010916525098, ['keyframe,2,footLmod,handstand,L,z']= -0.013880446941262, ['keyframe,0,footRmod,handstand,L,x']= 0.029642200529848, ['keyframe,0,footRmod,handstand,L,y']= -0.015779636329432, ['keyframe,0,footRmod,handstand,L,z']= -0.014086867159691, })

--Tue Aug 27 12:38:28 2013 setStage5 startSeg:1 endSeg:10 5 LRLean1 [1e+100,-1e+100] -> oeval:131908.62851823 dim:9}) -- skipped oeval:131908.62851823 

accumulate({ }) -- skipped iter :-1 
--1.000 --1.000 --0.997 --0.994 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 
--Tue Aug 27 14:26:17 2013 setStage6 startSeg:1 endSeg:10 6 LRLean1 [1.8664977106546e-05,1.930519627394e-05] -> oeval:131908.62851823 dim:18}) -- skipped oeval:131908.62851823 

accumulate({ ['keyframe,0,footLmod,handstand,B,x']= -0.009308128934378, ['keyframe,0,footLmod,handstand,B,y']= 0.021351022490957, ['keyframe,0,footLmod,handstand,B,z']= 0.041901171721839, ['keyframe,0,handLmod,handstand,B,x']= -0.050600272136476, ['keyframe,0,handLmod,handstand,B,y']= -0.063625595330544, ['keyframe,0,handLmod,handstand,B,z']= 0.49709366568234, ['keyframe,0,handRmod,handstand,B,x']= 0.02932365679779, ['keyframe,0,handRmod,handstand,B,y']= -0.00041565542683546, ['keyframe,0,handRmod,handstand,B,z']= 0.13908647045949, })
--1.000 
--Tue Aug 27 14:28:50 2013 setStage7 startSeg:1 endSeg:10 7 LRLean1 [131908.62851823,131908.62851823] -> oeval:131908.62851823 dim:0}) -- skipped oeval:131908.62851823 

accumulate({ ['keyframe,0,handLmod,handstand,LH0,x']= 0.060424903598208, ['keyframe,0,handLmod,handstand,LH0,y']= -0.061764675039001, ['keyframe,0,handLmod,handstand,LH0,z']= -0.013654586992436, ['keyframe,1,handLmod,handstand,LH0,x']= 0.044422409286466, ['keyframe,1,handLmod,handstand,LH0,y']= -0.089636594672295, ['keyframe,1,handLmod,handstand,LH0,z']= -0.0078912871433926, ['keyframe,2,handLmod,handstand,LH0,x']= 0.047393353343975, ['keyframe,2,handLmod,handstand,LH0,y']= -0.085652627288794, ['keyframe,2,handLmod,handstand,LH0,z']= -0.017040579548417, ['keyframe,0,handRmod,handstand,LH0,x']= -0.033721394957779, ['keyframe,0,handRmod,handstand,LH0,y']= 0.016668202275268, ['keyframe,0,handRmod,handstand,LH0,z']= 0.013590275778774, ['keyframe,1,handRmod,handstand,LH0,x']= -0.032969354905675, ['keyframe,1,handRmod,handstand,LH0,y']= 0.0049460943352623, ['keyframe,1,handRmod,handstand,LH0,z']= 0.0014234113987495, ['keyframe,2,handRmod,handstand,LH0,x']= -0.0037857632904632, ['keyframe,2,handRmod,handstand,LH0,y']= 0.0023665984989577, ['keyframe,2,handRmod,handstand,LH0,z']= 0.024379771104345, })

--Tue Aug 27 14:30:30 2013 setStage8 startSeg:1 endSeg:10 8 LRLean1 [1e+100,-1e+100] -> oeval:131908.62851823 dim:18}) -- skipped oeval:131908.62851823 

accumulate({ }) -- skipped iter :-1 
--1.000 
--Tue Aug 27 14:33:08 2013 setStage9 startSeg:1 endSeg:10 9 LRLean1 [131908.62851823,131908.62851823] -> oeval:131908.62851823 dim:0}) -- skipped oeval:131908.62851823 

accumulate({ }) -- skipped iter :-1 ['keyframe,0,handLmod,handstand,RH,x']= 0.0064253730768144, ['keyframe,0,handLmod,handstand,RH,y']= 0.1907471871274, ['keyframe,0,handLmod,handstand,RH,z']= -0.022142531185883, ['keyframe,1,handLmod,handstand,RH,x']= -0.018843132319516, ['keyframe,1,handLmod,handstand,RH,y']= -0.023692856331136, ['keyframe,1,handLmod,handstand,RH,z']= -0.0077091199817354, ['keyframe,2,handLmod,handstand,RH,x']= 0.17900989430345, ['keyframe,2,handLmod,handstand,RH,y']= -0.021423916911318, ['keyframe,2,handLmod,handstand,RH,z']= -0.027542235711294, ['keyframe,0,handRmod,handstand,RH,x']= -0.016639350411915, ['keyframe,0,handRmod,handstand,RH,y']= 0.012569641858782, ['keyframe,0,handRmod,handstand,RH,z']= -0.012168473065096, ['keyframe,1,handRmod,handstand,RH,x']= 0.015341977813188, ['keyframe,1,handRmod,handstand,RH,y']= -0.061002065660545, ['keyframe,1,handRmod,handstand,RH,z']= 0.01411823313161, ['keyframe,2,handRmod,handstand,RH,x']= -0.19999360149319, ['keyframe,2,handRmod,handstand,RH,y']= 0.024539728582607, ['keyframe,2,handRmod,handstand,RH,z']= -0.028595357114022, 

--Tue Aug 27 14:34:54 2013 setStage10 startSeg:1 endSeg:10 1 LRLean1 [1e+100,-1e+100] -> oeval:131908.62851823 dim:20}) -- skipped oeval:131908.62851823 

accumulate({ }) -- skipped iter :-1 
--0.000 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 --0.753 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 
accumulate({ ['keyframe,0,footLmod,handstand,R,x']= 0.025108389685064, ['keyframe,0,footLmod,handstand,R,y']= -0.017282443784858, ['keyframe,0,footLmod,handstand,R,z']= -0.0016079522432582, ['keyframe,1,footLmod,handstand,R,x']= 0.031557679542577, ['keyframe,1,footLmod,handstand,R,y']= -0.026629475689325, ['keyframe,1,footLmod,handstand,R,z']= 0.0020046977635041, ['keyframe,2,footLmod,handstand,R,x']= 0.016459826336798, ['keyframe,2,footLmod,handstand,R,y']= -0.027939264414444, ['keyframe,2,footLmod,handstand,R,z']= 0.0045272287015073, ['keyframe,0,footRmod,handstand,R,x']= -0.023768171244839, ['keyframe,0,footRmod,handstand,R,y']= -0.04311255397782, ['keyframe,0,footRmod,handstand,R,z']= -0.02448065948378, ['keyframe,1,footRmod,handstand,R,x']= -0.016071149535612, ['keyframe,1,footRmod,handstand,R,y']= -0.013446122227443, ['keyframe,1,footRmod,handstand,R,z']= -0.025443696219892, ['keyframe,2,footRmod,handstand,R,x']= -0.0099198054151059, ['keyframe,2,footRmod,handstand,R,y']= -0.020655186290584, ['keyframe,2,footRmod,handstand,R,z']= 0.0030663361794297, ['keyframe,0,desiredLeaning,handstand,R,x']= 0.042089268494236, ['keyframe,0,desiredLeaning,handstand,R,z']= -0.0042873544650129, })

--Wed Aug 28 02:17:47 2013 setStage3 startSeg:1 endSeg:10 3 LRLean1 [1e+100,-1e+100] -> oeval:1.4494500719468e-05 dim:12}) -- skipped oeval:1.4494500719468e-05 

accumulate({ }) -- skipped iter :-1 
--1.122 --0.647 --0.812 --0.769 --0.673 --0.644 --0.803 --0.766 --0.644 --0.635 --0.624 --0.633 --0.624 --0.631 --0.633 --0.637 --0.637 --0.634 --0.636 --0.624 
--Wed Aug 28 04:30:51 2013 setStage4 startSeg:1 endSeg:10 4 LRLean1 [9.0460923801649e-06,9.7527786675036e-06] -> oeval:1.4494500719468e-05 dim:0}) -- skipped oeval:1.4494500719468e-05 

accumulate({ ['keyframe,0,footLmod,handstand,L,x']= 0.019184162616103, ['keyframe,0,footLmod,handstand,L,y']= -0.02333752941456, ['keyframe,0,footLmod,handstand,L,z']= 6.0286128275668e-05, ['keyframe,1,footLmod,handstand,L,x']= 0.0051780266138264, ['keyframe,1,footLmod,handstand,L,y']= -2.3224459645139e-05, ['keyframe,1,footLmod,handstand,L,z']= 0.0064835483291606, ['keyframe,2,footLmod,handstand,L,x']= 0.0058577011545671, ['keyframe,2,footLmod,handstand,L,y']= 0.027013009605291, ['keyframe,2,footLmod,handstand,L,z']= -0.014481543523805, ['keyframe,0,footRmod,handstand,L,x']= 0.029409237963722, ['keyframe,0,footRmod,handstand,L,y']= -0.015931165883945, ['keyframe,0,footRmod,handstand,L,z']= -0.014506922264691, })

--Wed Aug 28 04:32:30 2013 setStage5 startSeg:1 endSeg:10 5 LRLean1 [1e+100,-1e+100] -> oeval:1.4494500719468e-05 dim:9}) -- skipped oeval:1.4494500719468e-05 

accumulate({ }) -- skipped iter :-1 
--1.047 --0.906 --0.895 --0.868 --0.867 --0.867 --0.867 --0.867 --0.939 --0.956 --0.867 --0.867 --0.866 --0.866 --0.866 --0.866 --0.866 --0.866 --0.866 --0.866 
--Wed Aug 28 06:44:38 2013 setStage6 startSeg:1 endSeg:10 6 LRLean1 [1.2460813803458e-05,1.6565400359478e-05] -> oeval:1.4494500719468e-05 dim:18}) -- skipped oeval:1.4494500719468e-05 

accumulate({ ['keyframe,0,footLmod,handstand,B,x']= -0.0064013643463661, ['keyframe,0,footLmod,handstand,B,y']= 0.020969906581794, ['keyframe,0,footLmod,handstand,B,z']= 0.0400100182209, ['keyframe,0,handLmod,handstand,B,x']= -0.050881774069298, ['keyframe,0,handLmod,handstand,B,y']= -0.063840116224145, ['keyframe,0,handLmod,handstand,B,z']= 0.49679944358424, ['keyframe,0,handRmod,handstand,B,x']= 0.030692112473089, ['keyframe,0,handRmod,handstand,B,y']= -0.0026419434957121, ['keyframe,0,handRmod,handstand,B,z']= 0.13875715987153, })
-- tropt.lua: 	Thu Aug 29 08:41:32 2013

--	------------------New optimization-------- [1e+100,-1e+100] -> oeval:2.2948830858893e-05 dim:18}) -- skipped oeval:2.2948830858893e-05 
--0.607 --0.586 --0.656 --0.656 --0.547 --0.533 --0.588 --0.584 --0.528 --0.529 --0.526 --0.526 --0.525 --0.525 --0.596 --0.520 --0.513 --0.553 --0.667 --0.506 
--Wed Aug 28 17:04:07 2013 setStage2 startSeg:1 endSeg:19 : [1.1620624822467e-05,1.5541356902049e-05] -> oeval:1.1296988877858e-05 dim:18}) -- skipped oeval:1.1296988877858e-05 

accumulate({ ['map,0,swingHandMod,handstand,x']= 0.015185580369832, ['map,0,swingHandMod,handstand,y']= 0.10115544413635, ['map,0,swingHandMod,handstand,z']= -0.01459454643921, ['map,1,swingHandMod,handstand,x']= 0.0025512791191884, ['map,1,swingHandMod,handstand,y']= -0.0088940924246746, ['map,1,swingHandMod,handstand,z']= -0.0072058077414896, ['map,2,swingHandMod,handstand,x']= 0.086287396334922, ['map,2,swingHandMod,handstand,y']= -0.015812873035899, ['map,2,swingHandMod,handstand,z']= -0.03015525804595, ['map,0,spprtHandMod,handstand,x']= -0.010543610972169, ['map,0,spprtHandMod,handstand,y']= 0.016760220526403, ['map,0,spprtHandMod,handstand,z']= 0.01677915120733, ['map,1,spprtHandMod,handstand,x']= -0.0050513225593107, ['map,1,spprtHandMod,handstand,y']= -0.03614545674311, ['map,1,spprtHandMod,handstand,z']= 0.00044441630195, ['map,2,spprtHandMod,handstand,x']= -0.10243799167004, ['map,2,spprtHandMod,handstand,y']= 0.0051811355277241, ['map,2,spprtHandMod,handstand,z']= -0.01317808782121, })
useCases.unmapControlParam(useCases.handstand)

accumulate({ ['map,0,swingHandMod,handstand,x']= 0.015470075832621, ['map,0,swingHandMod,handstand,y']= 0.10111984087058, ['map,0,swingHandMod,handstand,z']= -0.013920419823627, ['map,1,swingHandMod,handstand,x']= 0.0027535874726085, ['map,1,swingHandMod,handstand,y']= -0.0084953129285016, ['map,1,swingHandMod,handstand,z']= -0.0070231723575581, ['map,2,swingHandMod,handstand,x']= 0.086533189466734, ['map,2,swingHandMod,handstand,y']= -0.016634562187913, ['map,2,swingHandMod,handstand,z']= -0.031332970353413, ['map,0,spprtHandMod,handstand,x']= -0.0099909274283943, ['map,0,spprtHandMod,handstand,y']= 0.015669341421367, ['map,0,spprtHandMod,handstand,z']= 0.017113375231838, ['map,1,spprtHandMod,handstand,x']= -0.0038204606056335, ['map,1,spprtHandMod,handstand,y']= -0.036594664850002, ['map,1,spprtHandMod,handstand,z']= 8.4671260636924e-05, ['map,2,spprtHandMod,handstand,x']= -0.10288106569981, ['map,2,spprtHandMod,handstand,y']= 0.0046604620155005, ['map,2,spprtHandMod,handstand,z']= -0.012975925679459, })
useCases.unmapControlParam(useCases.handstand)
--1.384 --1.193 --1.254 --1.273 --1.051 --1.060 --1.060 --1.071 --1.171 --1.036 --1.037 --1.165 --1.092 --1.099 --1.019 --1.019 --1.019 
--useCases.handstand.useSoftContactModel=true
accumulate({ ['keyframe,0,footLmod,handstand,R,x']= 0.026135155560944, ['keyframe,0,footLmod,handstand,R,y']= -0.019363754955619, ['keyframe,0,footLmod,handstand,R,z']= 0.001034126515008, ['keyframe,1,footLmod,handstand,R,x']= 0.031855753959725, ['keyframe,1,footLmod,handstand,R,y']= -0.027072997386585, ['keyframe,1,footLmod,handstand,R,z']= 0.0022036294612988, ['keyframe,2,footLmod,handstand,R,x']= 0.016583362958593, ['keyframe,2,footLmod,handstand,R,y']= -0.027780693263234, ['keyframe,2,footLmod,handstand,R,z']= 0.0037194336461375, ['keyframe,0,footRmod,handstand,R,x']= -0.022780846024387, ['keyframe,0,footRmod,handstand,R,y']= -0.042763556290747, ['keyframe,0,footRmod,handstand,R,z']= -0.024703346071482, ['keyframe,1,footRmod,handstand,R,x']= -0.015483417394909, ['keyframe,1,footRmod,handstand,R,y']= -0.012852137031498, ['keyframe,1,footRmod,handstand,R,z']= -0.025609847806099, ['keyframe,2,footRmod,handstand,R,x']= -0.010575768321748, ['keyframe,2,footRmod,handstand,R,y']= -0.020829753055693, ['keyframe,2,footRmod,handstand,R,z']= 0.0035281485977037, ['keyframe,0,desiredLeaning,handstand,R,x']= 0.042373233710097, ['keyframe,0,desiredLeaning,handstand,R,z']= -0.0043391515473133, })
-- tropt.lua: 	Fri Aug 30 13:17:35 2013

--	------------------New optimization-------- [1e+100,-1e+100] -> oeval:1.7823670743377e-05 dim:18}) -- skipped oeval:1.7823670743377e-05 
--0.960 --0.850 --0.811 --0.735 --0.797 --0.820 --0.844 --0.775 --0.865 --0.763 
--Fri Aug 30 10:55:38 2013 setStage2 startSeg:1 endSeg:19 : [1.2740674272602e-05,1.8995525367573e-05] -> oeval:1.2467891764166e-05 dim:18}) -- skipped oeval:1.2467891764166e-05 

accumulate({ ['map,0,swingHandMod,handstand,x']= 0.015557654268979, ['map,0,swingHandMod,handstand,y']= 0.10018416897715, ['map,0,swingHandMod,handstand,z']= -0.013195805839577, ['map,1,swingHandMod,handstand,x']= 0.0025463302434301, ['map,1,swingHandMod,handstand,y']= -0.0085706892627338, ['map,1,swingHandMod,handstand,z']= -0.0075559172987989, ['map,2,swingHandMod,handstand,x']= 0.085065602779687, ['map,2,swingHandMod,handstand,y']= -0.016105926929699, ['map,2,swingHandMod,handstand,z']= -0.032070641811596, ['map,0,spprtHandMod,handstand,x']= -0.011068957900935, ['map,0,spprtHandMod,handstand,y']= 0.015764538956575, ['map,0,spprtHandMod,handstand,z']= 0.018009996480427, ['map,1,spprtHandMod,handstand,x']= -0.0038931709308074, ['map,1,spprtHandMod,handstand,y']= -0.035219467989672, ['map,1,spprtHandMod,handstand,z']= -0.0003222998430949, ['map,2,spprtHandMod,handstand,x']= -0.1038551322689, ['map,2,spprtHandMod,handstand,y']= 0.0044262945197655, ['map,2,spprtHandMod,handstand,z']= -0.01260469674762, })
useCases.unmapControlParam(useCases.handstand)

useCases.clearParam(useCases.handstand, '.mod,handstand,LH1,')
useCases.clearParam(useCases.handstand, '.mod,handstand,LRH2,')
useCases.clearParam(useCases.handstand, '.mod,handstand,RH2,')
useCases.clearParam(useCases.handstand, '.mod,handstand,RLH2,')
--	------------------New optimization-------- [1e+100,-1e+100] -> oeval:1.4148678673357e-05 dim:12}) -- skipped oeval:1.4148678673357e-05 
--0.865 --0.747 --0.882 --0.732 --0.729 --0.731 --0.730 --0.731 --0.730 --0.730 
--Sat Aug 31 04:03:59 2013 setStage2 startSeg:1 endSeg:16 4 LRLean [1.031100380845e-05,1.3424021759063e-05] -> oeval:9.6415013539923e-06 dim:9}) -- skipped oeval:9.6415013539923e-06 

accumulate({ ['keyframe,0,footLmod,handstand,L,x']= 0.019097939312825, ['keyframe,0,footLmod,handstand,L,y']= -0.023851671366619, ['keyframe,0,footLmod,handstand,L,z']= -0.0014271408527385, ['keyframe,1,footLmod,handstand,L,x']= 0.005804290835212, ['keyframe,1,footLmod,handstand,L,y']= -0.00016976270733129, ['keyframe,1,footLmod,handstand,L,z']= 0.006389837645676, ['keyframe,2,footLmod,handstand,L,x']= 0.0054460578028605, ['keyframe,2,footLmod,handstand,L,y']= 0.027791398707741, ['keyframe,2,footLmod,handstand,L,z']= -0.013934522198625, ['keyframe,0,footRmod,handstand,L,x']= 0.028104138587626, ['keyframe,0,footRmod,handstand,L,y']= -0.015175964913089, ['keyframe,0,footRmod,handstand,L,z']= -0.014795763440253, })
	accumulate({
		--['useCases,handstand,perClassContactMargin']=0, 
	['useCases,handstand,momentumThr']=50, 
	['useCases,handstand,k_p_EE']=120*0.5, 
	['useCases,handstand,k_d_EE']=24*0.5, 
	['useCases,handstand,k_p_ID']=70*1.8, 
	['useCases,handstand,k_d_ID']=20, 
			})


useCases.clampParam(useCases.handstand, '.mod,handstand,B,',0.001)
useCases.clampParam(useCases.handstand, '.mod,handstand,L,',0.001)
--accumulate({['keyframe,0,footLmod,handstand,R,x']= 0.02680665865696, ['keyframe,0,footLmod,handstand,R,y']= -0.021624337125761, ['keyframe,0,footLmod,handstand,R,z']= 0.0024344266786188, ['keyframe,1,footLmod,handstand,R,x']= 0.032018531342119, ['keyframe,1,footLmod,handstand,R,y']= -0.028019651541214, ['keyframe,1,footLmod,handstand,R,z']= 0.00067896932314679, ['keyframe,2,footLmod,handstand,R,x']= 0.015422023614963, ['keyframe,2,footLmod,handstand,R,y']= -0.024827176914567, ['keyframe,2,footLmod,handstand,R,z']= 0.0027307324659357, ['keyframe,0,footRmod,handstand,R,x']= -0.022868764014171, ['keyframe,0,footRmod,handstand,R,y']= -0.046137609719315, ['keyframe,0,footRmod,handstand,R,z']= -0.024421295353927, ['keyframe,1,footRmod,handstand,R,x']= -0.015771177153072, ['keyframe,1,footRmod,handstand,R,y']= -0.01151771589968, ['keyframe,1,footRmod,handstand,R,z']= -0.024072236018107, ['keyframe,2,footRmod,handstand,R,x']= -0.01296465469612, ['keyframe,2,footRmod,handstand,R,y']= -0.02156420142079, ['keyframe,2,footRmod,handstand,R,z']= 0.003221776345974, ['keyframe,0,desiredLeaning,handstand,R,x']= 0.042099151279887, ['keyframe,0,desiredLeaning,handstand,R,z']= -0.004722219037305, })
--accumulate({['map,0,swingHandMod,handstand,x']= 0.016339187814883, ['map,0,swingHandMod,handstand,y']= 0.10066789735275, ['map,0,swingHandMod,handstand,z']= -0.013053496211711, ['map,1,swingHandMod,handstand,x']= 0.0028509213784503, ['map,1,swingHandMod,handstand,y']= -0.0086755992641528, ['map,1,swingHandMod,handstand,z']= -0.0075320866638087, ['map,2,swingHandMod,handstand,x']= 0.084945721556551, ['map,2,swingHandMod,handstand,y']= -0.016317640397953, ['map,2,swingHandMod,handstand,z']= -0.032108630770627, ['map,0,spprtHandMod,handstand,x']= -0.010524943247464, ['map,0,spprtHandMod,handstand,y']= 0.016020349426672, ['map,0,spprtHandMod,handstand,z']= 0.018166833834521, ['map,1,spprtHandMod,handstand,x']= -0.0045790973246872, ['map,1,spprtHandMod,handstand,y']= -0.035573784170516, ['map,1,spprtHandMod,handstand,z']= 4.7686334490018e-05, ['map,2,spprtHandMod,handstand,x']= -0.10382128486389, ['map,2,spprtHandMod,handstand,y']= 0.0043013037899516, ['map,2,spprtHandMod,handstand,z']= -0.012600818121225, })
--accumulate({['map,0,swingHandMod,handstand,x']= 0.01670419142795, ['map,0,swingHandMod,handstand,y']= 0.10045283164956, ['map,0,swingHandMod,handstand,z']= -0.012952251602286, ['map,1,swingHandMod,handstand,x']= 0.0029042667001395, ['map,1,swingHandMod,handstand,y']= -0.0088910477057878, ['map,1,swingHandMod,handstand,z']= -0.0076407882756762, ['map,2,swingHandMod,handstand,x']= 0.085100063666603, ['map,2,swingHandMod,handstand,y']= -0.016258088909143, ['map,2,swingHandMod,handstand,z']= -0.032368587265718, ['map,0,spprtHandMod,handstand,x']= -0.010584095587391, ['map,0,spprtHandMod,handstand,y']= 0.016125443431149, ['map,0,spprtHandMod,handstand,z']= 0.018391668476467, ['map,1,spprtHandMod,handstand,x']= -0.0045640361579288, ['map,1,spprtHandMod,handstand,y']= -0.035588466137836, ['map,1,spprtHandMod,handstand,z']= -7.8815157931869e-05, ['map,2,spprtHandMod,handstand,x']= -0.10390532351803, ['map,2,spprtHandMod,handstand,y']= 0.0042841473928827, ['map,2,spprtHandMod,handstand,z']= -0.012883204053415, })
--accumulate({['map,0,swingHandMod,handstand,x']= 0.01652562897673, ['map,0,swingHandMod,handstand,y']= 0.10024828715222, ['map,0,swingHandMod,handstand,z']= -0.013133464939021, ['map,1,swingHandMod,handstand,x']= 0.0029035040369311, ['map,1,swingHandMod,handstand,y']= -0.0091838609141621, ['map,1,swingHandMod,handstand,z']= -0.007686434374622, ['map,2,swingHandMod,handstand,x']= 0.085348349170614, ['map,2,swingHandMod,handstand,y']= -0.016243614511086, ['map,2,swingHandMod,handstand,z']= -0.032081826131967, ['map,0,spprtHandMod,handstand,x']= -0.0107815983686, ['map,0,spprtHandMod,handstand,y']= 0.016270365164425, ['map,0,spprtHandMod,handstand,z']= 0.018006738631673, ['map,1,spprtHandMod,handstand,x']= -0.0042725263976432, ['map,1,spprtHandMod,handstand,y']= -0.035390471423565, ['map,1,spprtHandMod,handstand,z']= -0.00023318763891799, ['map,2,spprtHandMod,handstand,x']= -0.10339107516005, ['map,2,spprtHandMod,handstand,y']= 0.0036361386416345, ['map,2,spprtHandMod,handstand,z']= -0.012761054685159, })
--accumulate({['map,0,swingHandMod,handstand,x']= 0.017168466746212, ['map,0,swingHandMod,handstand,y']= 0.10047370468553, ['map,0,swingHandMod,handstand,z']= -0.014109161638272, ['map,1,swingHandMod,handstand,x']= 0.0020788733902827, ['map,1,swingHandMod,handstand,y']= -0.009863203196835, ['map,1,swingHandMod,handstand,z']= -0.0072915591374481, ['map,2,swingHandMod,handstand,x']= 0.086447243312201, ['map,2,swingHandMod,handstand,y']= -0.016071122726085, ['map,2,swingHandMod,handstand,z']= -0.031707229303984, ['map,0,spprtHandMod,handstand,x']= -0.011518884551532, ['map,0,spprtHandMod,handstand,y']= 0.016827676966062, ['map,0,spprtHandMod,handstand,z']= 0.01792787620986, ['map,1,spprtHandMod,handstand,x']= -0.0040868930454372, ['map,1,spprtHandMod,handstand,y']= -0.035299590704576, ['map,1,spprtHandMod,handstand,z']= -0.0006547919841974, ['map,2,spprtHandMod,handstand,x']= -0.10366462765736, ['map,2,spprtHandMod,handstand,y']= 0.0033550446257678, ['map,2,spprtHandMod,handstand,z']= -0.012695834112207, })
--useCases.unmapControlParam(useCases.handstand)
accumulate({
	['useCases,handstand,linearMomentumWeight']=0, 
	['useCases,handstand,momentumWeight']=8000, 
	--['keyframe,0,desiredLeaning,handstand,R,x']=0.041115976717226, 
	--['keyframe,0,desiredLeaning,handstand,R,z']=0.066649614049112, 
})
accumulate({['keyframe,0,footLmod,handstand,R,x']= 0.02661078423852, ['keyframe,0,footLmod,handstand,R,y']= -0.018527511180011, ['keyframe,0,footLmod,handstand,R,z']= 0.00065917436995214, ['keyframe,1,footLmod,handstand,R,x']= 0.032594903266797, ['keyframe,1,footLmod,handstand,R,y']= -0.026311149040746, ['keyframe,1,footLmod,handstand,R,z']= 0.0011235521497024, ['keyframe,2,footLmod,handstand,R,x']= 0.016942349217067, ['keyframe,2,footLmod,handstand,R,y']= -0.02896016383786, ['keyframe,2,footLmod,handstand,R,z']= 0.0016276836708673, ['keyframe,0,footRmod,handstand,R,x']= -0.022186078710834, ['keyframe,0,footRmod,handstand,R,y']= -0.042185298569845, ['keyframe,0,footRmod,handstand,R,z']= -0.024361290005698, ['keyframe,1,footRmod,handstand,R,x']= -0.017084017066879, ['keyframe,1,footRmod,handstand,R,y']= -0.013147575121561, ['keyframe,1,footRmod,handstand,R,z']= -0.024377996706904, ['keyframe,2,footRmod,handstand,R,x']= -0.010822099688817, ['keyframe,2,footRmod,handstand,R,y']= -0.021162892080563, ['keyframe,2,footRmod,handstand,R,z']= 0.0046109110766529, ['keyframe,0,desiredLeaning,handstand,R,x']= 0.042502094703889, ['keyframe,0,desiredLeaning,handstand,R,z']= -0.0042806891476341, })
accumulate({['keyframe,0,footLmod,handstand,L,x']= 0.007059839856266, ['keyframe,0,footLmod,handstand,L,y']= -0.0057982420328116, ['keyframe,0,footLmod,handstand,L,z']= -0.0033236331619443, ['keyframe,1,footLmod,handstand,L,x']= -0.013364673658133, ['keyframe,1,footLmod,handstand,L,y']= 0.0050875323842585, ['keyframe,1,footLmod,handstand,L,z']= 0.0017136165533935, ['keyframe,2,footLmod,handstand,L,x']= 0.0071604302380284, ['keyframe,2,footLmod,handstand,L,y']= 0.0079106268562243, ['keyframe,2,footLmod,handstand,L,z']= -0.0083996808221056, ['keyframe,0,footRmod,handstand,L,x']= 0.0040481692495603, ['keyframe,0,footRmod,handstand,L,y']= 0.0070764862972725, ['keyframe,0,footRmod,handstand,L,z']= 0.013575757563333, })
-- tropt.lua: 	Tue Sep  3 07:21:46 2013

--	------------------New optimization-------- [1e+100,-1e+100] -> oeval:156884.6154186 dim:18}) -- skipped oeval:156884.6154186 
--1.032 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 --0.000 --0.792 --0.000 
--Tue Sep  3 01:41:36 2013 setStage2 startSeg:1 endSeg:24 : [9.9156339777402e-06,129884.61540627] -> oeval:1.7371621902331e-05 dim:18}) -- skipped oeval:1.7371621902331e-05 

accumulate({ ['map,0,swingHandMod,handstand,x']= 0.016023261598265, ['map,0,swingHandMod,handstand,y']= 0.10138753257181, ['map,0,swingHandMod,handstand,z']= -0.013767605703189, ['map,1,swingHandMod,handstand,x']= 0.0028191250559968, ['map,1,swingHandMod,handstand,y']= -0.0080658814027412, ['map,1,swingHandMod,handstand,z']= -0.0066699377812152, ['map,2,swingHandMod,handstand,x']= 0.085110089635226, ['map,2,swingHandMod,handstand,y']= -0.01604072010254, ['map,2,swingHandMod,handstand,z']= -0.031951189431895, ['map,0,spprtHandMod,handstand,x']= -0.010839327403303, ['map,0,spprtHandMod,handstand,y']= 0.015378806551484, ['map,0,spprtHandMod,handstand,z']= 0.017877757033614, ['map,1,spprtHandMod,handstand,x']= -0.0035186070050201, ['map,1,spprtHandMod,handstand,y']= -0.034597877696206, ['map,1,spprtHandMod,handstand,z']= 0.00020920193905352, ['map,2,spprtHandMod,handstand,x']= -0.10373663171436, ['map,2,spprtHandMod,handstand,y']= 0.0040546202903871, ['map,2,spprtHandMod,handstand,z']= -0.012879335089041, })
--0.929 --0.476 --0.608 --0.681 --0.925 --0.745 --0.642 --0.695 --0.689 --0.781 
--Tue Sep  3 03:15:11 2013 setStage3 startSeg:1 endSeg:24 : [8.7617690564651e-06,155384.61540827] -> oeval:1.4024068857861e-05 dim:18}) -- skipped oeval:1.4024068857861e-05 

useCases.unmapControlParam(useCases.handstand)
accumulate({['map,0,swingHandMod,handstand,x']= 0.016141447442931, ['map,0,swingHandMod,handstand,y']= 0.10156614805116, ['map,0,swingHandMod,handstand,z']= -0.013935645403173, ['map,1,swingHandMod,handstand,x']= 0.0022957962093915, ['map,1,swingHandMod,handstand,y']= -0.0078194148988375, ['map,1,swingHandMod,handstand,z']= -0.0070604373949034, ['map,2,swingHandMod,handstand,x']= 0.085011314533562, ['map,2,swingHandMod,handstand,y']= -0.016204805104788, ['map,2,swingHandMod,handstand,z']= -0.032181416897695, ['map,0,spprtHandMod,handstand,x']= -0.011028085966197, ['map,0,spprtHandMod,handstand,y']= 0.015115482585023, ['map,0,spprtHandMod,handstand,z']= 0.017944435736603, ['map,1,spprtHandMod,handstand,x']= -0.0031622430553707, ['map,1,spprtHandMod,handstand,y']= -0.034483268816434, ['map,1,spprtHandMod,handstand,z']= 0.00065721926474622, ['map,2,spprtHandMod,handstand,x']= -0.10276110141562, ['map,2,spprtHandMod,handstand,y']= 0.0037418816660338, ['map,2,spprtHandMod,handstand,z']= -0.012366570695792, })
useCases.unmapControlParam(useCases.handstand)
accumulate({['map,0,swingHandMod,handstand,x']= 0.016357748415291, ['map,0,swingHandMod,handstand,y']= 0.10138654081078, ['map,0,swingHandMod,handstand,z']= -0.013912245490687, ['map,1,swingHandMod,handstand,x']= 0.0021077722312206, ['map,1,swingHandMod,handstand,y']= -0.0079720481055572, ['map,1,swingHandMod,handstand,z']= -0.0069223863132892, ['map,2,swingHandMod,handstand,x']= 0.08499557310352, ['map,2,swingHandMod,handstand,y']= -0.016478940737365, ['map,2,swingHandMod,handstand,z']= -0.032167932225292, ['map,0,spprtHandMod,handstand,x']= -0.011116812660813, ['map,0,spprtHandMod,handstand,y']= 0.015303456643706, ['map,0,spprtHandMod,handstand,z']= 0.018429515186645, ['map,1,spprtHandMod,handstand,x']= -0.0032426688719402, ['map,1,spprtHandMod,handstand,y']= -0.034190034553615, ['map,1,spprtHandMod,handstand,z']= 0.00037214355481529, ['map,2,spprtHandMod,handstand,x']= -0.10262053066186, ['map,2,spprtHandMod,handstand,y']= 0.0033546098742164, ['map,2,spprtHandMod,handstand,z']= -0.012353937760066, })
useCases.unmapControlParam(useCases.handstand)
accumulate({
	--['useCases,handstand,useBulletColdet']=true, 
	--['useCases,handstand,EEobjWeight']=60000, 
	--['useCases,handstand,EEobjWeightAngular']=60000, 
	--['useCases,handstand,k_p_EE']=120, 
	--['useCases,handstand,k_d_EE']=24, 
	--['useCases,handstand,dotMomentumScale']=0.3, 
	--['useCases,handstand,noComvelDependentFootAdjustment']=true, 
	['useCases,handstand,contactMargin']=0.01, 
	['useCases,handstand,momentumThr']=50, 
	['useCases,handstand,headControlWeight']=0, 
	['useCases,handstand,useSoftContactModel']=true,
	--['useCases,handstand,desiredAccThr']= 100
	['useCases,handstand,lambdaObjWeight']=10, 
})

accumulate({['map,0,swingHandMod,handstand,z']= -0.040761001386254, ['map,1,swingHandMod,handstand,z']= -0.0069221199473657, ['map,2,swingHandMod,handstand,z']= -0.028641251413821, ['map,0,spprtHandMod,handstand,z']= 0.026133475146212, ['map,1,spprtHandMod,handstand,z']= 0.014749346454333, ['map,2,spprtHandMod,handstand,z']= -2.701215977977e-05, })
accumulate({['map,0,swingHandMod,handstand,x']= 0.01711850424675, ['map,1,swingHandMod,handstand,x']= 0.0014561303523152, ['map,2,swingHandMod,handstand,x']= 0.080647436369692, ['map,0,spprtHandMod,handstand,x']= -0.0074024396261131, ['map,1,spprtHandMod,handstand,x']= -0.0018421355044474, ['map,2,spprtHandMod,handstand,x']= -0.10178003716704, })
accumulate({['map,0,swingHandMod,handstand,y']= 0.092159865870603, ['map,1,swingHandMod,handstand,y']= -0.0042547607145755, ['map,2,swingHandMod,handstand,y']= -0.028021214449063, ['map,0,spprtHandMod,handstand,y']= 0.031693453412205, ['map,1,spprtHandMod,handstand,y']= -0.033992734149961, ['map,2,spprtHandMod,handstand,y']= -0.003297192669068, })
useCases.unmapControlParam(useCases.handstand)

accumulate({['map,0,swingHandMod,handstand,x']= 0.017501800965553, ['map,0,swingHandMod,handstand,y']= 0.093084032081122, ['map,0,swingHandMod,handstand,z']= -0.041044003127883, ['map,1,swingHandMod,handstand,x']= 0.0016679746056319, ['map,1,swingHandMod,handstand,y']= -0.0051777860676322, ['map,1,swingHandMod,handstand,z']= -0.0065505852736172, ['map,2,swingHandMod,handstand,x']= 0.079924166068212, ['map,2,swingHandMod,handstand,y']= -0.026571392126743, ['map,2,swingHandMod,handstand,z']= -0.028516018672996, ['map,0,spprtHandMod,handstand,x']= -0.0081784155555612, ['map,0,spprtHandMod,handstand,y']= 0.026737676241448, ['map,0,spprtHandMod,handstand,z']= 0.025917239349666, ['map,1,spprtHandMod,handstand,x']= -0.0039347980589926, ['map,1,spprtHandMod,handstand,y']= -0.0327963105905, ['map,1,spprtHandMod,handstand,z']= 0.012100214366447, ['map,2,spprtHandMod,handstand,x']= -0.10511962417964, ['map,2,spprtHandMod,handstand,y']= -0.0058598618376925, ['map,2,spprtHandMod,handstand,z']= -0.0023113306742713, })
	modify({
		['map,0,swingHandMod,handstand,y']=0.04,
		['map,1,swingHandMod,handstand,y']=0.04,
		['map,2,swingHandMod,handstand,y']=0.04,
	})
useCases.unmapControlParam(useCases.handstand)
	accumulate({
		['useCases,handstand,k_p_EE']=250, 
		['useCases,handstand,k_d_EE']=24, 
		['useCases,handstand,modifyFootSim']=false, 
	})

accumulate({['map,0,swingHandMod,handstand,x']= 0.017501800965553, ['map,0,swingHandMod,handstand,y']= 0.093084032081122, ['map,0,swingHandMod,handstand,z']= -0.041044003127883, ['map,1,swingHandMod,handstand,x']= 0.0016679746056319, ['map,1,swingHandMod,handstand,y']= -0.0051777860676322, ['map,1,swingHandMod,handstand,z']= -0.0065505852736172, ['map,2,swingHandMod,handstand,x']= 0.079924166068212, ['map,2,swingHandMod,handstand,y']= -0.026571392126743, ['map,2,swingHandMod,handstand,z']= -0.028516018672996, ['map,0,spprtHandMod,handstand,x']= -0.0081784155555612, ['map,0,spprtHandMod,handstand,y']= 0.026737676241448, ['map,0,spprtHandMod,handstand,z']= 0.025917239349666, ['map,1,spprtHandMod,handstand,x']= -0.0039347980589926, ['map,1,spprtHandMod,handstand,y']= -0.0327963105905, ['map,1,spprtHandMod,handstand,z']= 0.012100214366447, ['map,2,spprtHandMod,handstand,x']= -0.10511962417964, ['map,2,spprtHandMod,handstand,y']= -0.0058598618376925, ['map,2,spprtHandMod,handstand,z']= -0.0023113306742713, })
useCases.unmapControlParam(useCases.handstand)

--accumulate({['map,0,swingHandMod,handstand,x']= 0.0077685589957861, ['map,0,swingHandMod,handstand,y']= 0.088299268226931, ['map,0,swingHandMod,handstand,z']= -0.045099716368629, ['map,1,swingHandMod,handstand,x']= 0.0037470942586644, ['map,1,swingHandMod,handstand,y']= -0.0024313659048103, ['map,1,swingHandMod,handstand,z']= -0.0014023440953933, ['map,2,swingHandMod,handstand,x']= 0.081175524160665, ['map,2,swingHandMod,handstand,y']= -0.025631528750118, ['map,2,swingHandMod,handstand,z']= -0.027174323466467, ['map,0,spprtHandMod,handstand,x']= -0.0051800761659673, ['map,0,spprtHandMod,handstand,y']= 0.019018004311483, ['map,0,spprtHandMod,handstand,z']= 0.023780685303873, ['map,1,spprtHandMod,handstand,x']= 0.0018945811982324, ['map,1,spprtHandMod,handstand,y']= -0.0326731096334, ['map,1,spprtHandMod,handstand,z']= 0.0065361782200904, ['map,2,spprtHandMod,handstand,x']= -0.10665465007233, ['map,2,spprtHandMod,handstand,y']= 0.0013393318698448, ['map,2,spprtHandMod,handstand,z']= -0.0018191004488101, })
--accumulate({['map,0,swingHandMod,handstand,z']= -0.065377765263672, ['map,1,swingHandMod,handstand,z']= -0.016262940175591, ['map,2,swingHandMod,handstand,z']= -0.026368962820604, ['map,0,spprtHandMod,handstand,z']= 0.038468350753865, ['map,1,spprtHandMod,handstand,z']= 0.014678783313442, ['map,2,spprtHandMod,handstand,z']= 0.0075631628840146, })
--accumulate({['map,0,swingHandMod,handstand,x']= 0.011251452686426, ['map,1,swingHandMod,handstand,x']= 0.015266975730572, ['map,2,swingHandMod,handstand,x']= 0.081506011891278, ['map,0,spprtHandMod,handstand,x']= -0.0059066867264809, ['map,1,spprtHandMod,handstand,x']= 0.0034933305055194, ['map,2,spprtHandMod,handstand,x']= -0.090323895035053, })
accumulate({ ['map,0,swingHandMod,handstand,x']= 0.01059309681612, ['map,0,swingHandMod,handstand,y']= 0.087950169259358, ['map,0,swingHandMod,handstand,z']= -0.044511230885146, ['map,1,swingHandMod,handstand,x']= 0.0050045668683165, ['map,1,swingHandMod,handstand,y']= -0.0042628182906453, ['map,1,swingHandMod,handstand,z']= -0.0023045217432474, ['map,2,swingHandMod,handstand,x']= 0.080752195707361, ['map,2,swingHandMod,handstand,y']= -0.027097733571499, ['map,2,swingHandMod,handstand,z']= -0.023524031277099, ['map,0,spprtHandMod,handstand,x']= -0.0056963725254497, ['map,0,spprtHandMod,handstand,y']= 0.018064395089442, ['map,0,spprtHandMod,handstand,z']= 0.024673237736023, ['map,1,spprtHandMod,handstand,x']= 0.0015006858762389, ['map,1,spprtHandMod,handstand,y']= -0.031370393528668, ['map,1,spprtHandMod,handstand,z']= 0.0086791050516302, ['map,2,spprtHandMod,handstand,x']= -0.10511742135549, ['map,2,spprtHandMod,handstand,y']= 6.6597552644547e-06, ['map,2,spprtHandMod,handstand,z']= -0.0015860260518493, })
accumulate({ ['map,0,swingHandMod,handstand,z']= -0.065797228889562, ['map,1,swingHandMod,handstand,z']= -0.014169459178778, ['map,2,swingHandMod,handstand,z']= -0.026843269434622, ['map,0,spprtHandMod,handstand,z']= 0.046917253877152, ['map,1,spprtHandMod,handstand,z']= 0.017702733288664, ['map,2,spprtHandMod,handstand,z']= 0.0071062765958296, })
accumulate({ ['map,0,swingHandMod,handstand,x']= 0.011152068097398, ['map,1,swingHandMod,handstand,x']= 0.0094901306774208, ['map,2,swingHandMod,handstand,x']= 0.08125207853083, ['map,0,spprtHandMod,handstand,x']= -0.0059427854160506, ['map,1,spprtHandMod,handstand,x']= 0.0034171387005461, ['map,2,spprtHandMod,handstand,x']= -0.098708516312903, })
useCases.unmapControlParam(useCases.handstand)
-- Sat Sep  7 21:50:13 2013eval:3.7660249958163e-06 
--accumulate({['map,0,swingHandMod,handstand,x']= 0.019870713365033, ['map,0,swingHandMod,handstand,y']= 0.089477347247843, ['map,0,swingHandMod,handstand,z']= -0.070011989374807, ['map,1,swingHandMod,handstand,x']= 0.0031518390943079, ['map,1,swingHandMod,handstand,y']= -0.0086974528801297, ['map,1,swingHandMod,handstand,z']= -0.011395481764097, ['map,2,swingHandMod,handstand,x']= 0.079810430447645, ['map,2,swingHandMod,handstand,y']= -0.026623731406135, ['map,2,swingHandMod,handstand,z']= -0.024799274242132, ['map,0,spprtHandMod,handstand,x']= -0.0077894571756947, ['map,0,spprtHandMod,handstand,y']= 0.020118948085538, ['map,0,spprtHandMod,handstand,z']= 0.041594718123002, ['map,1,spprtHandMod,handstand,x']= 0.00084842553292399, ['map,1,spprtHandMod,handstand,y']= -0.035273582638238, ['map,1,spprtHandMod,handstand,z']= 0.022237460734591, ['map,2,spprtHandMod,handstand,x']= -0.10113883663321, ['map,2,spprtHandMod,handstand,y']= 0.0028160599684989, ['map,2,spprtHandMod,handstand,z']= 0.0087427589327896, })
--useCases.unmapControlParam(useCases.handstand)
accumulate({['map,0,swingHandMod,handstand,x']= 0.012100601195521, ['map,0,swingHandMod,handstand,y']= 0.087403155290566, ['map,0,swingHandMod,handstand,z']= -0.065140276903066, ['map,1,swingHandMod,handstand,x']= 0.0093724370002283, ['map,1,swingHandMod,handstand,y']= -0.0044899686535598, ['map,1,swingHandMod,handstand,z']= -0.013862210912401, ['map,2,swingHandMod,handstand,x']= 0.081633388800547, ['map,2,swingHandMod,handstand,y']= -0.027190354770017, ['map,2,swingHandMod,handstand,z']= -0.027902406962596, ['map,0,spprtHandMod,handstand,x']= -0.0062238315299515, ['map,0,spprtHandMod,handstand,y']= 0.017363722522023, ['map,0,spprtHandMod,handstand,z']= 0.047551675674298, ['map,1,spprtHandMod,handstand,x']= 0.0043938714313395, ['map,1,spprtHandMod,handstand,y']= -0.030673089550907, ['map,1,spprtHandMod,handstand,z']= 0.016756593847108, ['map,2,spprtHandMod,handstand,x']= -0.099102755323267, ['map,2,spprtHandMod,handstand,y']= 0.0001763744777647, ['map,2,spprtHandMod,handstand,z']= 0.0072599483172846, })
useCases.unmapControlParam(useCases.handstand)
accumulate({ ['map,0,swingHandMod,handstand,x']= 0.017900495993848, ['map,0,swingHandMod,handstand,y']= 0.093160192787092, ['map,0,swingHandMod,handstand,z']= -0.057653719620107, ['map,1,swingHandMod,handstand,x']= 0.010902561191638, ['map,1,swingHandMod,handstand,y']= 0.011015373699098, ['map,1,swingHandMod,handstand,z']= -0.017847287342079, ['map,2,swingHandMod,handstand,x']= 0.10365866736282, ['map,2,swingHandMod,handstand,y']= -0.029359378166456, ['map,2,swingHandMod,handstand,z']= -0.026520040557445, ['map,0,spprtHandMod,handstand,x']= -0.0018081369095228, ['map,0,spprtHandMod,handstand,y']= 0.013950762780127, ['map,0,spprtHandMod,handstand,z']= 0.050159706029808, ['map,1,spprtHandMod,handstand,x']= 0.00033373524461407, ['map,1,spprtHandMod,handstand,y']= -0.032912248811642, ['map,1,spprtHandMod,handstand,z']= 0.01636905905618, ['map,2,spprtHandMod,handstand,x']= -0.10541581331073, ['map,2,spprtHandMod,handstand,y']= 0.0072547222164503, ['map,2,spprtHandMod,handstand,z']= -0.0020742197906931, })
useCases.unmapControlParam(useCases.handstand)
-- tropt.lua: 	Mon Sep  9 16:14:54 2013
--1.052 --0.788 --0.861 --0.000 --1.114 --0.919 --0.736 --0.968 --0.842 --0.741 
--Mon Sep  9 09:27:37 2013 setStage2 startSeg:1 endSeg:10 2 LRLean [1.5195100426101e-05,141977.46170038] -> oeval:1.2644604367925e-05 dim:12}) -- skipped oeval:1.2644604367925e-05 

accumulate({ ['keyframe,0,footLmod,handstand,R,x']= 0.022926747807389, ['keyframe,0,footLmod,handstand,R,y']= -0.019848757840701, ['keyframe,0,footLmod,handstand,R,z']= 0.0044494905325481, ['keyframe,1,footLmod,handstand,R,x']= 0.030749168682286, ['keyframe,1,footLmod,handstand,R,y']= -0.025203904104776, ['keyframe,1,footLmod,handstand,R,z']= -6.3586108858412e-05, ['keyframe,2,footLmod,handstand,R,x']= 0.01595307462452, ['keyframe,2,footLmod,handstand,R,y']= -0.03176182498147, ['keyframe,2,footLmod,handstand,R,z']= 0.00055805714610733, ['keyframe,0,footRmod,handstand,R,x']= -0.024613295484971, ['keyframe,0,footRmod,handstand,R,y']= -0.04114682380635, ['keyframe,0,footRmod,handstand,R,z']= -0.0164469023962, ['keyframe,1,footRmod,handstand,R,x']= -0.017366236109627, ['keyframe,1,footRmod,handstand,R,y']= -0.015094575799844, ['keyframe,1,footRmod,handstand,R,z']= -0.028382907802175, ['keyframe,2,footRmod,handstand,R,x']= -0.011296916668074, ['keyframe,2,footRmod,handstand,R,y']= -0.02199745455222, ['keyframe,2,footRmod,handstand,R,z']= 0.0047166319639867, ['keyframe,0,desiredLeaning,handstand,R,x']= 0.042552950975261, ['keyframe,0,desiredLeaning,handstand,R,z']= -0.0040912711804343, })
--1.187 --1.028 --1.140 --0.975 --1.004 --0.961 --1.101 --0.971 --0.982 --1.025 
--Mon Sep  9 10:49:33 2013 setStage3 startSeg:1 endSeg:10 4 LRLean [1.1928835912402e-05,2.0407243802003e-05] -> oeval:1.1567775888217e-05 dim:9}) -- skipped oeval:1.1567775888217e-05 

accumulate({ ['keyframe,0,footLmod,handstand,L,x']= 0.011807484301496, ['keyframe,0,footLmod,handstand,L,y']= -0.0091159929797228, ['keyframe,0,footLmod,handstand,L,z']= -0.0033765962555099, ['keyframe,1,footLmod,handstand,L,x']= -0.010091673603281, ['keyframe,1,footLmod,handstand,L,y']= 0.00035142718312898, ['keyframe,1,footLmod,handstand,L,z']= 0.0112709804832, ['keyframe,2,footLmod,handstand,L,x']= -0.00063901189017361, ['keyframe,2,footLmod,handstand,L,y']= -0.0060854262974742, ['keyframe,2,footLmod,handstand,L,z']= -0.010118820623804, ['keyframe,0,footRmod,handstand,L,x']= 0.0017797337661201, ['keyframe,0,footRmod,handstand,L,y']= 0.013005186311573, ['keyframe,0,footRmod,handstand,L,z']= 0.010766645362434, })
--1.117 --1.126 --1.038 --1.060 --1.036 --1.045 --1.181 --1.131 --1.060 --1.025 
--Mon Sep  9 12:12:13 2013 setStage4 startSeg:1 endSeg:10 6 LRLean [1.1395650653112e-05,1.2879464788965e-05] -> oeval:1.1376744036055e-05 dim:12}) -- skipped oeval:1.1376744036055e-05 

accumulate({  ['keyframe,0,footLmod,handstand,B,x']= 0.0024311687166394, ['keyframe,0,footLmod,handstand,B,y']= 0.0048134122069198, ['keyframe,0,footLmod,handstand,B,z']= 0.0017911737043572, ['keyframe,0,handLmod,handstand,B,x']= -0.0021724986443892, ['keyframe,0,handLmod,handstand,B,y']= 0.0023239782864412, ['keyframe,0,handLmod,handstand,B,z']= 0.0026861000505377, ['keyframe,0,handRmod,handstand,B,x']= 0.0013668668668648, ['keyframe,0,handRmod,handstand,B,y']= 0.0004156541593321, ['keyframe,0,handRmod,handstand,B,z']= 0.0033905573455482, })
--1.035 --1.016 --1.056 --1.052 --1.027 --1.006 --1.002 --1.002 --1.002 --1.002 
--Mon Sep  9 13:35:09 2013 setStage5 startSeg:1 endSeg:10 7 LRLean [1.1386676852012e-05,1.2845327155689e-05] -> oeval:1.1380354559168e-05 dim:18}) -- skipped oeval:1.1380354559168e-05 

accumulate({  ['keyframe,0,handLmod,handstand,LH0,x']= 0.056225367319098, ['keyframe,0,handLmod,handstand,LH0,y']= -0.062314427130165, ['keyframe,0,handLmod,handstand,LH0,z']= -0.013619273297372, ['keyframe,1,handLmod,handstand,LH0,x']= 0.046756147123025, ['keyframe,1,handLmod,handstand,LH0,y']= -0.089353657855412, ['keyframe,1,handLmod,handstand,LH0,z']= -0.0096770134972435, ['keyframe,0,handRmod,handstand,LH0,x']= -0.033712712355686, ['keyframe,0,handRmod,handstand,LH0,y']= 0.017284503845343, ['keyframe,0,handRmod,handstand,LH0,z']= 0.014882689714908, ['keyframe,1,handRmod,handstand,LH0,x']= -0.038607457291629, ['keyframe,1,handRmod,handstand,LH0,y']= 0.0089241500004442, ['keyframe,1,handRmod,handstand,LH0,z']= 0.0029013530293684, })
--1.000 --1.000 --0.999 --0.999 --0.999 --0.999 --0.999 --0.999 --0.999 --0.999 
--Mon Sep  9 14:58:21 2013 setStage6 startSeg:1 endSeg:10 9 LRLean [1.1363331024819e-05,1.242550476143e-05] -> oeval:1.1351503535712e-05 dim:18}) -- skipped oeval:1.1351503535712e-05 

accumulate({
		['useCases,handstand,k_d_ID']=25, 
		['useCases,handstand,k_p_ID']=250, 
	['useCases,handstand,contactMargin']=0.0025, 
	})
accumulate({ ['keyframe,0,footLmod,handstand,R,x']= -0.2290037765545, ['keyframe,0,footLmod,handstand,R,y']= -0.28022398335307, ['keyframe,0,footLmod,handstand,R,z']= 0.032967069379076, ['keyframe,1,footLmod,handstand,R,x']= -0.017898411573119, ['keyframe,1,footLmod,handstand,R,y']= -0.34946888859126, ['keyframe,1,footLmod,handstand,R,z']= -0.019904877445851, ['keyframe,2,footLmod,handstand,R,x']= -0.51506795307336, ['keyframe,2,footLmod,handstand,R,y']= 0.085281258125739, ['keyframe,2,footLmod,handstand,R,z']= 0.402414346523, ['keyframe,0,footRmod,handstand,R,x']= 0.10523506998357, ['keyframe,0,footRmod,handstand,R,y']= -0.039017109343216, ['keyframe,0,footRmod,handstand,R,z']= -0.14527120175897, ['keyframe,1,footRmod,handstand,R,x']= 0.15179454226635, ['keyframe,1,footRmod,handstand,R,y']= -0.10515971666765, ['keyframe,1,footRmod,handstand,R,z']= -0.17734177159242, ['keyframe,2,footRmod,handstand,R,x']= -0.39009271779702, ['keyframe,2,footRmod,handstand,R,y']= 0.17969621855845, ['keyframe,2,footRmod,handstand,R,z']= -0.035420370980899, ['keyframe,0,desiredLeaning,handstand,R,x']= -0.060765110412541, ['keyframe,0,desiredLeaning,handstand,R,z']= 0.011569434450444, })
--accumulate({ ['keyframe,0,footLmod,handstand,L,x']= -0.087955774620466, ['keyframe,0,footLmod,handstand,L,y']= -0.095871882409097, ['keyframe,0,footLmod,handstand,L,z']= 0.29069327719339, ['keyframe,1,footLmod,handstand,L,x']= 0.089720521475691, ['keyframe,1,footLmod,handstand,L,y']= -0.010682459469155, ['keyframe,1,footLmod,handstand,L,z']= -0.020410318654735, ['keyframe,2,footLmod,handstand,L,x']= 0.041087100042948, ['keyframe,2,footLmod,handstand,L,y']= 0.069624558163346, ['keyframe,2,footLmod,handstand,L,z']= 0.043625844245319, ['keyframe,0,footRmod,handstand,L,x']= 0.043445235696435, ['keyframe,0,footRmod,handstand,L,y']= 0.064301271016555, ['keyframe,0,footRmod,handstand,L,z']= -0.30191131511503, })
--accumulate({ ['keyframe,0,footLmod,handstand,B,x']= -0.0037048861943052, ['keyframe,0,footLmod,handstand,B,y']= -0.059009950652004, ['keyframe,0,footLmod,handstand,B,z']= -0.59345523880522, ['keyframe,0,handLmod,handstand,B,x']= 0.035843058908659, ['keyframe,0,handLmod,handstand,B,y']= -0.2479494527022, ['keyframe,0,handLmod,handstand,B,z']= -0.2985725981398, ['keyframe,0,handRmod,handstand,B,x']= -1.2240072734568, ['keyframe,0,handRmod,handstand,B,y']= -1.4394153036424, ['keyframe,0,handRmod,handstand,B,z']= 0.50133440277667, })
--accumulate({ ['keyframe,0,handLmod,handstand,LH0,x']= 0.44512487932716, ['keyframe,0,handLmod,handstand,LH0,y']= -0.32260054449611, ['keyframe,0,handLmod,handstand,LH0,z']= 0.6525701190082, ['keyframe,1,handLmod,handstand,LH0,x']= 0.22738171360221, ['keyframe,1,handLmod,handstand,LH0,y']= -0.49920826193272, ['keyframe,1,handLmod,handstand,LH0,z']= -0.7558647256134, ['keyframe,0,handRmod,handstand,LH0,x']= -0.13620816885078, ['keyframe,0,handRmod,handstand,LH0,y']= -0.5383145202266, ['keyframe,0,handRmod,handstand,LH0,z']= 0.8158687458328, ['keyframe,1,handRmod,handstand,LH0,x']= -0.16463308644546, ['keyframe,1,handRmod,handstand,LH0,y']= 0.66802832867622, ['keyframe,1,handRmod,handstand,LH0,z']= -0.23882880061229, })
accumulate({
	['keyframe,0,desiredLeaning,handstand,R,x']= -0.060765110412541,
	['keyframe,0,desiredLeaning,handstand,R,z']= 0.011569434450444,
})
	modify({
		['keyframe,0,footLmod,handstand,R,y']=0.25,
		['keyframe,1,footLmod,handstand,R,y']=0.05,
		['keyframe,0,desiredLeaning,handstand,R,z']=-0.15,
	})
useCases.clearParam(useCases.handstand, 'keyframe,2,foot.mod,handstand,R,')

accumulate({ ['keyframe,0,footLmod,handstand,L,x']= 0.01588174790375, ['keyframe,0,footLmod,handstand,L,y']= -0.0078658957374484, ['keyframe,0,footLmod,handstand,L,z']= -0.013455033198561, ['keyframe,1,footLmod,handstand,L,x']= -0.0099938073208282, ['keyframe,1,footLmod,handstand,L,y']= -0.0036207603859977, ['keyframe,1,footLmod,handstand,L,z']= 0.0071711819847425, ['keyframe,2,footLmod,handstand,L,x']= -0.0010393388907995, ['keyframe,2,footLmod,handstand,L,y']= -0.0067051190136502, ['keyframe,2,footLmod,handstand,L,z']= -0.0070698220711888, ['keyframe,0,footRmod,handstand,L,x']= 0.0014795521874862, ['keyframe,0,footRmod,handstand,L,y']= 0.015554154611054, ['keyframe,0,footRmod,handstand,L,z']= 0.0098364165562571, })
accumulate({ ['keyframe,0,footLmod,handstand,B,x']= 0.15421696991208, ['keyframe,0,footLmod,handstand,B,y']= -0.1311677945353, ['keyframe,0,footLmod,handstand,B,z']= -0.4788883740891, ['keyframe,0,handLmod,handstand,B,x']= -0.58051327555339, ['keyframe,0,handLmod,handstand,B,y']= -0.14637608424475, ['keyframe,0,handLmod,handstand,B,z']= 0.20520894452601, ['keyframe,0,handRmod,handstand,B,x']= -0.21610358704493, ['keyframe,0,handRmod,handstand,B,y']= -0.010086011886951, ['keyframe,0,handRmod,handstand,B,z']= 0.22418772188661, })
accumulate({ ['keyframe,0,footLmod,handstand,L,x']= 0.015851028982285, ['keyframe,0,footLmod,handstand,L,y']= -0.013086193539117, ['keyframe,0,footLmod,handstand,L,z']= -0.01569009779684, ['keyframe,1,footLmod,handstand,L,x']= -0.0076740123499779, ['keyframe,1,footLmod,handstand,L,y']= -0.0038578128029811, ['keyframe,1,footLmod,handstand,L,z']= 0.0055307176322193, ['keyframe,2,footLmod,handstand,L,x']= -0.003274610818408, ['keyframe,2,footLmod,handstand,L,y']= -0.005370448916398, ['keyframe,2,footLmod,handstand,L,z']= -0.0090373491628733, ['keyframe,0,footRmod,handstand,L,x']= 0.0022847686165239, ['keyframe,0,footRmod,handstand,L,y']= 0.0099303014483284, ['keyframe,0,footRmod,handstand,L,z']= 0.0083927281904967, })
accumulate({ ['keyframe,0,footLmod,handstand,B,x']= 0.29307851804818, ['keyframe,0,footLmod,handstand,B,y']= -0.14115112671564, ['keyframe,0,footLmod,handstand,B,z']= -0.76635451285156, ['keyframe,0,handLmod,handstand,B,x']= -0.98321979123314, ['keyframe,0,handLmod,handstand,B,y']= -0.17432854144632, ['keyframe,0,handLmod,handstand,B,z']= 0.19213744253785, ['keyframe,0,handRmod,handstand,B,x']= 0.045333031463339, ['keyframe,0,handRmod,handstand,B,y']= -0.039620796325877, ['keyframe,0,handRmod,handstand,B,z']= -0.084471703507318, })
accumulate({ ['keyframe,0,footLmod,handstand,L,x']= 0.015215504239987, ['keyframe,0,footLmod,handstand,L,y']= -0.0073476809952836, ['keyframe,0,footLmod,handstand,L,z']= -0.021790882437971, ['keyframe,1,footLmod,handstand,L,x']= -0.008011158513237, ['keyframe,1,footLmod,handstand,L,y']= -0.0019589483077347, ['keyframe,1,footLmod,handstand,L,z']= -0.011188075037679, ['keyframe,2,footLmod,handstand,L,x']= -0.00038663980181578, ['keyframe,2,footLmod,handstand,L,y']= 0.016842125978633, ['keyframe,2,footLmod,handstand,L,z']= -0.0068104902411705, ['keyframe,0,footRmod,handstand,L,x']= 0.01669292384302, ['keyframe,0,footRmod,handstand,L,y']= 0.0019331741068501, ['keyframe,0,footRmod,handstand,L,z']= 0.0048122998108206, })
accumulate({ ['keyframe,0,footLmod,handstand,B,x']= 0.24785999403872, ['keyframe,0,footLmod,handstand,B,y']= -0.15992970539098, ['keyframe,0,footLmod,handstand,B,z']= -0.72687011991094, ['keyframe,0,handLmod,handstand,B,x']= -0.99503879491862, ['keyframe,0,handLmod,handstand,B,y']= -0.17315411342655, ['keyframe,0,handLmod,handstand,B,z']= 0.18918212044676, ['keyframe,0,handRmod,handstand,B,x']= 0.0092799694488465, ['keyframe,0,handRmod,handstand,B,y']= -0.036927121800292, ['keyframe,0,handRmod,handstand,B,z']= -0.074516717680552, })
accumulate({ ['map,0,swingHandMod,handstand,x']= 0.040194096698859, ['map,0,swingHandMod,handstand,y']= 0.10331564179204, ['map,0,swingHandMod,handstand,z']= -0.043846449801991, ['map,1,swingHandMod,handstand,x']= 0.0048414504625541, ['map,1,swingHandMod,handstand,y']= 0.016335250529059, ['map,1,swingHandMod,handstand,z']= -0.001909768940064, ['map,2,swingHandMod,handstand,x']= 0.10435614809795, ['map,2,swingHandMod,handstand,y']= -0.035868647789522, ['map,2,swingHandMod,handstand,z']= 0.042913134971476, ['map,0,spprtHandMod,handstand,x']= 0.0039882311744249, ['map,0,spprtHandMod,handstand,y']= -0.004264097747225, ['map,0,spprtHandMod,handstand,z']= 0.10849562086929, ['map,1,spprtHandMod,handstand,x']= -0.033958294544398, ['map,1,spprtHandMod,handstand,y']= -0.045188066963112, ['map,1,spprtHandMod,handstand,z']= 0.013547345980589, ['map,2,spprtHandMod,handstand,x']= -0.08847822053681, ['map,2,spprtHandMod,handstand,y']= -0.0038080905441159, ['map,2,spprtHandMod,handstand,z']= -0.029422133688734, })
useCases.unmapControlParam(useCases.handstand)
accumulate({ ['keyframe,0,footLmod,handstand,R,x']= -0.23172277054029, ['keyframe,0,footLmod,handstand,R,y']= -0.021687232501276, ['keyframe,0,footLmod,handstand,R,z']= 0.039543207120703, ['keyframe,1,footLmod,handstand,R,x']= -0.017862462181191, ['keyframe,1,footLmod,handstand,R,y']= -0.30711369075448, ['keyframe,1,footLmod,handstand,R,z']= -0.014273842882986, ['keyframe,0,footRmod,handstand,R,x']= 0.10126695908207, ['keyframe,0,footRmod,handstand,R,y']= -0.032590340729247, ['keyframe,0,footRmod,handstand,R,z']= -0.13637052762147, ['keyframe,1,footRmod,handstand,R,x']= 0.15202315060065, ['keyframe,1,footRmod,handstand,R,y']= -0.11104695034197, ['keyframe,1,footRmod,handstand,R,z']= -0.17994974931027, ['keyframe,0,desiredLeaning,handstand,R,x']= -0.061275349709976, ['keyframe,0,desiredLeaning,handstand,R,z']= -0.13780180646469, })
accumulate({['keyframe,0,footLmod,handstand,R,x']= -0.23556919761728, ['keyframe,0,footLmod,handstand,R,y']= -0.016488094124834, ['keyframe,0,footLmod,handstand,R,z']= 0.038056796567457, ['keyframe,1,footLmod,handstand,R,x']= -0.017940766100541, ['keyframe,1,footLmod,handstand,R,y']= -0.30525168905285, ['keyframe,1,footLmod,handstand,R,z']= -0.0098118520957191, ['keyframe,0,footRmod,handstand,R,x']= 0.10220513780633, ['keyframe,0,footRmod,handstand,R,y']= -0.038848588077318, ['keyframe,0,footRmod,handstand,R,z']= -0.1360947308357, ['keyframe,1,footRmod,handstand,R,x']= 0.15910555145559, ['keyframe,1,footRmod,handstand,R,y']= -0.11504988199498, ['keyframe,1,footRmod,handstand,R,z']= -0.17716156598654, ['keyframe,0,desiredLeaning,handstand,R,x']= -0.061398026526215, ['keyframe,0,desiredLeaning,handstand,R,z']= -0.13788841166812, })
accumulate({  ['keyframe,0,footLmod,handstand,R,x']= -0.23879465562035, ['keyframe,0,footLmod,handstand,R,y']= 0.00042734765391341, ['keyframe,0,footLmod,handstand,R,z']= 0.045082545214169, ['keyframe,1,footLmod,handstand,R,x']= -0.031448604802085, ['keyframe,1,footLmod,handstand,R,y']= -0.31009470785018, ['keyframe,1,footLmod,handstand,R,z']= -0.011468767241049, ['keyframe,0,footRmod,handstand,R,x']= 0.10666111162555, ['keyframe,0,footRmod,handstand,R,y']= -0.044458979791939, ['keyframe,0,footRmod,handstand,R,z']= -0.14096122493396, ['keyframe,1,footRmod,handstand,R,x']= 0.1628533695744, ['keyframe,1,footRmod,handstand,R,y']= -0.11700320852958, ['keyframe,1,footRmod,handstand,R,z']= -0.1710379477355, ['keyframe,0,desiredLeaning,handstand,R,x']= -0.060479414888334, ['keyframe,0,desiredLeaning,handstand,R,z']= -0.13746000895772, })

accumulate({  ['keyframe,0,footLmod,handstand,R,x']= -0.23673754283705, ['keyframe,0,footLmod,handstand,R,y']= 0.0032218180740146, ['keyframe,0,footLmod,handstand,R,z']= 0.042871646650397, ['keyframe,1,footLmod,handstand,R,x']= -0.03213039976947, ['keyframe,1,footLmod,handstand,R,y']= -0.30754981601192, ['keyframe,1,footLmod,handstand,R,z']= -0.013350769962499, ['keyframe,0,footRmod,handstand,R,x']= 0.10749820733029, ['keyframe,0,footRmod,handstand,R,y']= -0.045858471624626, ['keyframe,0,footRmod,handstand,R,z']= -0.14517559203628, ['keyframe,1,footRmod,handstand,R,x']= 0.15748857966143, ['keyframe,1,footRmod,handstand,R,y']= -0.11382455897184, ['keyframe,1,footRmod,handstand,R,z']= -0.17332377234916, ['keyframe,0,desiredLeaning,handstand,R,x']= -0.06019455432697, ['keyframe,0,desiredLeaning,handstand,R,z']= -0.13774030835395, })
accumulate({ ['keyframe,0,footLmod,handstand,L,x']= 0.023781156591621, ['keyframe,0,footLmod,handstand,L,y']= -0.014275014764708, ['keyframe,0,footLmod,handstand,L,z']= -0.025726946901742, ['keyframe,1,footLmod,handstand,L,x']= -0.011054025774897, ['keyframe,1,footLmod,handstand,L,y']= -0.0062128086570601, ['keyframe,1,footLmod,handstand,L,z']= -0.014170781248954, ['keyframe,2,footLmod,handstand,L,x']= 0.0047234047646773, ['keyframe,2,footLmod,handstand,L,y']= 0.014095970657844, ['keyframe,2,footLmod,handstand,L,z']= -0.0076792748202603, ['keyframe,0,footRmod,handstand,L,x']= 0.011118176272001, ['keyframe,0,footRmod,handstand,L,y']= 0.0080020498972937, ['keyframe,0,footRmod,handstand,L,z']= 0.00236355667689, ['keyframe,2,handLmod,handstand,L,x']= -0.0021831951677153, ['keyframe,2,handLmod,handstand,L,y']= 0.0049289009464703, ['keyframe,2,handLmod,handstand,L,z']= -0.0016661794273429, ['keyframe,2,handRmod,handstand,L,x']= -0.0046608502565622, ['keyframe,2,handRmod,handstand,L,y']= 0.0072190278726362, ['keyframe,2,handRmod,handstand,L,z']= 0.0024600269328293, })
accumulate({ ['keyframe,0,footLmod,handstand,B,x']= 0.24254637453261, ['keyframe,0,footLmod,handstand,B,y']= -0.16337282099465, ['keyframe,0,footLmod,handstand,B,z']= -0.72069547527185, ['keyframe,0,handLmod,handstand,B,x']= -0.9954791911279, ['keyframe,0,handLmod,handstand,B,y']= -0.17289136858481, ['keyframe,0,handLmod,handstand,B,z']= 0.18828354525975, ['keyframe,0,handRmod,handstand,B,x']= 0.013117612997837, ['keyframe,0,handRmod,handstand,B,y']= -0.036264949638219, ['keyframe,0,handRmod,handstand,B,z']= -0.071203345787251, })
accumulate({ ['keyframe,0,handLmod,handstand,LH0,x']= 0.070060998308857, ['keyframe,0,handLmod,handstand,LH0,y']= -0.067112609743233, ['keyframe,0,handLmod,handstand,LH0,z']= -0.017206530459046, ['keyframe,1,handLmod,handstand,LH0,x']= 0.041063139500616, ['keyframe,1,handLmod,handstand,LH0,y']= -0.090472564773144, ['keyframe,1,handLmod,handstand,LH0,z']= -0.0084717881237074, ['keyframe,0,handRmod,handstand,LH0,x']= -0.031680919866084, ['keyframe,0,handRmod,handstand,LH0,y']= 0.024102639115683, ['keyframe,0,handRmod,handstand,LH0,z']= 0.01648135638325, ['keyframe,1,handRmod,handstand,LH0,x']= -0.035924194444037, ['keyframe,1,handRmod,handstand,LH0,y']= 0.0040212599427436, ['keyframe,1,handRmod,handstand,LH0,z']= -0.0040074508042109, })
--accumulate({ ['keyframe,0,handLmod,handstand,RH,x']= -0.010043800606523, ['keyframe,0,handLmod,handstand,RH,y']= 0.11199854565394, ['keyframe,0,handLmod,handstand,RH,z']= -0.019570268830934, ['keyframe,1,handLmod,handstand,RH,x']= 0.0149365313722, ['keyframe,1,handLmod,handstand,RH,y']= 0.014713041991759, ['keyframe,1,handLmod,handstand,RH,z']= 0.05807820493308, ['keyframe,2,handLmod,handstand,RH,x']= 0.11168870540731, ['keyframe,2,handLmod,handstand,RH,y']= -0.026653078526211, ['keyframe,2,handLmod,handstand,RH,z']= 0.080154423997595, ['keyframe,0,handRmod,handstand,RH,x']= -0.0019033107847467, ['keyframe,0,handRmod,handstand,RH,y']= -0.010770488966278, ['keyframe,0,handRmod,handstand,RH,z']= 0.097937169309358, ['keyframe,1,handRmod,handstand,RH,x']= -0.078973411617695, ['keyframe,1,handRmod,handstand,RH,y']= -0.059047506927998, ['keyframe,1,handRmod,handstand,RH,z']= -0.024090997360384, ['keyframe,2,handRmod,handstand,RH,x']= -0.083486460796659, ['keyframe,2,handRmod,handstand,RH,y']= -0.016736252530615, ['keyframe,2,handRmod,handstand,RH,z']= -0.056987839272547, })
--accumulate({ }) -- skipped iter :-1 ['keyframe,0,handLmod,handstand,LH,x']= -0.0042646449921668, ['keyframe,0,handLmod,handstand,LH,y']= -0.0065975745555604, ['keyframe,0,handLmod,handstand,LH,z']= 0.11016821135456, ['keyframe,1,handLmod,handstand,LH,x']= 0.035766035831735, ['keyframe,1,handLmod,handstand,LH,y']= -0.050525916657154, ['keyframe,1,handLmod,handstand,LH,z']= 0.01295364952521, ['keyframe,2,handLmod,handstand,LH,x']= 0.088548057902045, ['keyframe,2,handLmod,handstand,LH,y']= -6.2884311535791e-05, ['keyframe,2,handLmod,handstand,LH,z']= -0.03073303089973, ['keyframe,0,handRmod,handstand,LH,x']= -0.041001660965692, ['keyframe,0,handRmod,handstand,LH,y']= 0.09566218586201, ['keyframe,0,handRmod,handstand,LH,z']= -0.047340659152109, ['keyframe,1,handRmod,handstand,LH,x']= -0.012423278599655, ['keyframe,1,handRmod,handstand,LH,y']= 0.014123888712144, ['keyframe,1,handRmod,handstand,LH,z']= -0.0036538224665645, ['keyframe,2,handRmod,handstand,LH,x']= -0.10847944937879, ['keyframe,2,handRmod,handstand,LH,y']= -0.033234554431023, ['keyframe,2,handRmod,handstand,LH,z']= 0.043795353689978, 
--accumulate({ }) -- skipped iter :-1 ['keyframe,0,footLmod,handstand,R,x']= -0.2361874471494, ['keyframe,0,footLmod,handstand,R,y']= 0.00082834133788586, ['keyframe,0,footLmod,handstand,R,z']= 0.044282046858752, ['keyframe,1,footLmod,handstand,R,x']= -0.034939492985653, ['keyframe,1,footLmod,handstand,R,y']= -0.30604676697502, ['keyframe,1,footLmod,handstand,R,z']= -0.013389416470428, ['keyframe,0,footRmod,handstand,R,x']= 0.10844239362942, ['keyframe,0,footRmod,handstand,R,y']= -0.046659113958408, ['keyframe,0,footRmod,handstand,R,z']= -0.14466195582936, ['keyframe,1,footRmod,handstand,R,x']= 0.15761572593866, ['keyframe,1,footRmod,handstand,R,y']= -0.11805209018344, ['keyframe,1,footRmod,handstand,R,z']= -0.17447889827888, ['keyframe,0,desiredLeaning,handstand,R,x']= -0.060318046492728, ['keyframe,0,desiredLeaning,handstand,R,z']= -0.13753699353231, 
--accumulate({ ['keyframe,0,footLmod,handstand,L,x']= 0.025438559052782, ['keyframe,0,footLmod,handstand,L,y']= -0.01054091828897, ['keyframe,0,footLmod,handstand,L,z']= -0.023265512401389, ['keyframe,1,footLmod,handstand,L,x']= -0.0096200152095043, ['keyframe,1,footLmod,handstand,L,y']= -0.0021655200858975, ['keyframe,1,footLmod,handstand,L,z']= -0.014108236346898, ['keyframe,2,footLmod,handstand,L,x']= 0.00084049012427445, ['keyframe,2,footLmod,handstand,L,y']= 0.010519038525161, ['keyframe,2,footLmod,handstand,L,z']= -0.0020924865732221, ['keyframe,0,footRmod,handstand,L,x']= 0.0020811599257171, ['keyframe,0,footRmod,handstand,L,y']= 0.017204768824243, ['keyframe,0,footRmod,handstand,L,z']= -0.0017478143936687, ['keyframe,2,handLmod,handstand,L,x']= 0.0035955317089716, ['keyframe,2,handLmod,handstand,L,y']= 0.004247147481495, ['keyframe,2,handLmod,handstand,L,z']= -0.0025097167615828, ['keyframe,2,handRmod,handstand,L,x']= -0.0039516838589024, ['keyframe,2,handRmod,handstand,L,y']= 0.0023293690894907, ['keyframe,2,handRmod,handstand,L,z']= 0.0017693571096699, })
--accumulate({ ['keyframe,0,footLmod,handstand,B,x']= 0.24475603473101, ['keyframe,0,footLmod,handstand,B,y']= -0.17172822115046, ['keyframe,0,footLmod,handstand,B,z']= -0.72506279644548, ['keyframe,0,handLmod,handstand,B,x']= -1.0082875367884, ['keyframe,0,handLmod,handstand,B,y']= -0.16708876724732, ['keyframe,0,handLmod,handstand,B,z']= 0.18909107080791, ['keyframe,0,handRmod,handstand,B,x']= 0.017179004352618, ['keyframe,0,handRmod,handstand,B,y']= -0.042033817368588, ['keyframe,0,handRmod,handstand,B,z']= -0.075594209282627, })
accumulate({ ['map,0,swingHandMod,handstand,x']= 0.04079890145647, ['map,0,swingHandMod,handstand,y']= 0.10271914853328, ['map,0,swingHandMod,handstand,z']= -0.041076831863122, ['map,1,swingHandMod,handstand,x']= 0.0047307740281621, ['map,1,swingHandMod,handstand,y']= 0.015779073939777, ['map,1,swingHandMod,handstand,z']= -0.0077209759834255, ['map,2,swingHandMod,handstand,x']= 0.10585914284852, ['map,2,swingHandMod,handstand,y']= -0.041320098367661, ['map,2,swingHandMod,handstand,z']= 0.037947210207036, ['map,0,spprtHandMod,handstand,x']= 0.00048606809796819, ['map,0,spprtHandMod,handstand,y']= -0.0036432103829962, ['map,0,spprtHandMod,handstand,z']= 0.1110653564028, ['map,1,spprtHandMod,handstand,x']= -0.026304869336726, ['map,1,spprtHandMod,handstand,y']= -0.041068778220821, ['map,1,spprtHandMod,handstand,z']= 0.011923579322746, ['map,2,spprtHandMod,handstand,x']= -0.085186447934494, ['map,2,spprtHandMod,handstand,y']= -0.0087626577394464, ['map,2,spprtHandMod,handstand,z']= -0.038560581235887, })
useCases.unmapControlParam(useCases.handstand)
accumulate({ ['keyframe,0,footLmod,handstand,R,x']= -0.23889667799045, ['keyframe,0,footLmod,handstand,R,y']= 0.003170381063265, ['keyframe,0,footLmod,handstand,R,z']= 0.042900832220954, ['keyframe,1,footLmod,handstand,R,x']= -0.033723336682298, ['keyframe,1,footLmod,handstand,R,y']= -0.30798089099402, ['keyframe,1,footLmod,handstand,R,z']= -0.013936249828244, ['keyframe,0,footRmod,handstand,R,x']= 0.1069861894344, ['keyframe,0,footRmod,handstand,R,y']= -0.048005296286084, ['keyframe,0,footRmod,handstand,R,z']= -0.14507586514652, ['keyframe,1,footRmod,handstand,R,x']= 0.16145808221889, ['keyframe,1,footRmod,handstand,R,y']= -0.11407043068917, ['keyframe,1,footRmod,handstand,R,z']= -0.17404926066691, ['keyframe,0,desiredLeaning,handstand,R,x']= -0.060881890969064, ['keyframe,0,desiredLeaning,handstand,R,z']= -0.14044115760859, })
accumulate({ ['keyframe,0,footLmod,handstand,L,x']= 0.031795068684919, ['keyframe,0,footLmod,handstand,L,y']= -0.011398696179998, ['keyframe,0,footLmod,handstand,L,z']= -0.023106091057863, ['keyframe,1,footLmod,handstand,L,x']= -0.011083769938925, ['keyframe,1,footLmod,handstand,L,y']= -0.0067920080101279, ['keyframe,1,footLmod,handstand,L,z']= -0.014993301989531, ['keyframe,2,footLmod,handstand,L,x']= 0.0031902537340054, ['keyframe,2,footLmod,handstand,L,y']= 0.011946515599993, ['keyframe,2,footLmod,handstand,L,z']= -0.0049527373054322, ['keyframe,0,footRmod,handstand,L,x']= 0.019540058677945, ['keyframe,0,footRmod,handstand,L,y']= -0.00038941554488109, ['keyframe,0,footRmod,handstand,L,z']= 0.0090727914508159, ['keyframe,2,handLmod,handstand,L,x']= -0.0011167089224679, ['keyframe,2,handLmod,handstand,L,y']= 0.0068341988870583, ['keyframe,2,handLmod,handstand,L,z']= 0.0050150526356162, ['keyframe,2,handRmod,handstand,L,x']= -0.0075561629155918, ['keyframe,2,handRmod,handstand,L,y']= 0.0037517691481575, ['keyframe,2,handRmod,handstand,L,z']= -0.0052153173430977, })
accumulate({ ['keyframe,0,footLmod,handstand,B,x']= 0.24806976420907, ['keyframe,0,footLmod,handstand,B,y']= -0.16054227974311, ['keyframe,0,footLmod,handstand,B,z']= -0.7192374443249, ['keyframe,0,handLmod,handstand,B,x']= -0.99656245711649, ['keyframe,0,handLmod,handstand,B,y']= -0.16921848504123, ['keyframe,0,handLmod,handstand,B,z']= 0.18702003818067, ['keyframe,0,handRmod,handstand,B,x']= 0.016773365831382, ['keyframe,0,handRmod,handstand,B,y']= -0.03302240602522, ['keyframe,0,handRmod,handstand,B,z']= -0.073843657235211, })
--accumulate({ ['keyframe,0,handLmod,handstand,LH0,x']= 0.073484463752535, ['keyframe,0,handLmod,handstand,LH0,y']= -0.070786238113329, ['keyframe,0,handLmod,handstand,LH0,z']= -0.020627998268526, ['keyframe,1,handLmod,handstand,LH0,x']= 0.044229974793589, ['keyframe,1,handLmod,handstand,LH0,y']= -0.092263355922785, ['keyframe,1,handLmod,handstand,LH0,z']= -0.0072908475532133, ['keyframe,0,handRmod,handstand,LH0,x']= -0.024631788267876, ['keyframe,0,handRmod,handstand,LH0,y']= 0.030509160162525, ['keyframe,0,handRmod,handstand,LH0,z']= 0.028889656171277, ['keyframe,1,handRmod,handstand,LH0,x']= -0.040030570464026, ['keyframe,1,handRmod,handstand,LH0,y']= 0.0031229738389642, ['keyframe,1,handRmod,handstand,LH0,z']= -0.022261122713536, })
accumulate({ ['map,0,swingHandMod,handstand,x']= 0.044048880076696, ['map,0,swingHandMod,handstand,y']= 0.11200097200547, ['map,0,swingHandMod,handstand,z']= -0.043985712372221, ['map,1,swingHandMod,handstand,x']= 0.0086574681071239, ['map,1,swingHandMod,handstand,y']= 0.017477167900593, ['map,1,swingHandMod,handstand,z']= -0.0073864718517612, ['map,2,swingHandMod,handstand,x']= 0.10412145526748, ['map,2,swingHandMod,handstand,y']= -0.039483193327076, ['map,2,swingHandMod,handstand,z']= 0.040276382717323, ['map,0,spprtHandMod,handstand,x']= -0.00013638651284187, ['map,0,spprtHandMod,handstand,y']= -0.0036497623454469, ['map,0,spprtHandMod,handstand,z']= 0.11540705204517, ['map,1,spprtHandMod,handstand,x']= -0.026098105087314, ['map,1,spprtHandMod,handstand,y']= -0.038329600791398, ['map,1,spprtHandMod,handstand,z']= 0.0061219288810139, ['map,2,spprtHandMod,handstand,x']= -0.085835225590862, ['map,2,spprtHandMod,handstand,y']= -0.010287405390454, ['map,2,spprtHandMod,handstand,z']= -0.042816374086243, })
accumulate({  ['map,0,swingHandMod,handstand,z']= -0.04145735262827, ['map,1,swingHandMod,handstand,z']= -0.0095291422995317, ['map,2,swingHandMod,handstand,z']= 0.04090445423447, ['map,0,spprtHandMod,handstand,z']= 0.11523615773941, ['map,1,spprtHandMod,handstand,z']= 0.0065186495410634, ['map,2,spprtHandMod,handstand,z']= -0.040369096832146, })
useCases.unmapControlParam(useCases.handstand)
useCases.clearParam(useCases.handstand, 'keyframe,.,hand.mod,handstand,B,')
accumulate({
 ['keyframe,0,desiredLeaning,handstand,R,x']= -0.030881890969064, ['keyframe,0,desiredLeaning,handstand,R,z']= -0.09044115760859, })
accumulate({ ['keyframe,0,desiredLeaning,handstand,R,x']= -0.032461366763087, ['keyframe,0,desiredLeaning,handstand,R,z']= -0.092132113059488, })
	--accumulate({['useCases,handstand,lambdaObjWeight2']=1, })
	accumulate({
		['useCases,handstand,lambdaObjWeight2']=0.0001, 
--	['useCases,handstand,contactMargin']=0.0025, 
	})
	local h=0.1
	local down=0.3
accumulate({ 
	['map,0,swingHandMod,handstand,y']= down*h,
	['map,1,swingHandMod,handstand,y']= h,
	['map,2,swingHandMod,handstand,y']= down*h,
	['map,0,spprtHandMod,handstand,y']= -down*h,
	['map,1,spprtHandMod,handstand,y']= -h,    
	['map,2,spprtHandMod,handstand,y']= -down*h,
	['useCases,handstand,momentumThr']=50, 
	['useCases,handstand,momentumWeight']=8000, 
	['useCases,handstand,dotMomentumScale']=1, 
	['useCases,handstand,k_d_EE']=24, 
	['useCases,handstand,k_p_ID']=150, 
	['useCases,handstand,contactMargin']=0.0025, 
	['useCases,handstand,noComvelDependentFootAdjustment']=true, 
	['useCases,handstand,noIntersectionPrevenction']=true, 
})

useCases.clampParam(useCases.handstand, '.Mod,handstand,x',0)
useCases.clampParam(useCases.handstand, '.Mod,handstand,z',0)
	--useCases.handstand.maxContactForceY=880
	--useCases.handstand.useLimbIKsolver2={1,2,3}
	useCases.handstand.useLimbIKsolver2={1,2}
accumulate({ ['map,0,swingHandMod,handstand,x']= 0.040242987016246, ['map,0,swingHandMod,handstand,y']= 0.038078227453015, ['map,0,swingHandMod,handstand,z']= -0.044550729278331, ['map,1,swingHandMod,handstand,x']= 0.015258045744803, ['map,1,swingHandMod,handstand,y']= 0.0991071816161, ['map,1,swingHandMod,handstand,z']= -0.015192925503276, ['map,2,swingHandMod,handstand,x']= 0.097382827871871, ['map,2,swingHandMod,handstand,y']= 0.033607515415206, ['map,2,swingHandMod,handstand,z']= 0.036199842515093, ['map,0,spprtHandMod,handstand,x']= -0.0060353315409353, ['map,0,spprtHandMod,handstand,y']= -0.037582687595966, ['map,0,spprtHandMod,handstand,z']= 0.11662797729159, ['map,1,spprtHandMod,handstand,x']= -0.025621412502572, ['map,1,spprtHandMod,handstand,y']= -0.10126644347354, ['map,1,spprtHandMod,handstand,z']= 0.010421832847475, ['map,2,spprtHandMod,handstand,x']= -0.090601288443636, ['map,2,spprtHandMod,handstand,y']= -0.031806051360957, ['map,2,spprtHandMod,handstand,z']= -0.045770785521666, })
useCases.unmapControlParam(useCases.handstand)
accumulate({
	['keyframe,2,footLmod,handstand,LH0,x']=0, 
	['keyframe,2,footLmod,handstand,LH0,y']=0, 
	['keyframe,2,footLmod,handstand,LH0,z']=0, 
	['keyframe,2,footRmod,handstand,LH0,x']=0, 
	['keyframe,2,footRmod,handstand,LH0,y']=0, 
	['keyframe,2,footRmod,handstand,LH0,z']=0, 
})
accumulate({ ['map,0,swingHandMod,handstand,x']= -0.046862948997536, ['map,0,swingHandMod,handstand,y']= -0.023592032027894, ['map,0,swingHandMod,handstand,z']= 0.053556745509903, ['map,1,swingHandMod,handstand,x']= 0.033876671934311, ['map,1,swingHandMod,handstand,y']= 0.17488693288345, ['map,1,swingHandMod,handstand,z']= 0.064824137723141, ['map,2,swingHandMod,handstand,x']= 0.13942024743555, ['map,2,swingHandMod,handstand,y']= -0.0099482621206539, ['map,2,swingHandMod,handstand,z']= 0.0051132461949248, ['map,0,spprtHandMod,handstand,x']= 0.098750157304711, ['map,0,spprtHandMod,handstand,y']= -0.14020534121461, ['map,0,spprtHandMod,handstand,z']= 0.056395874375026, ['map,1,spprtHandMod,handstand,x']= -0.043606733059152, ['map,1,spprtHandMod,handstand,y']= -0.21916470648836, ['map,1,spprtHandMod,handstand,z']= -0.10906477175433, ['map,2,spprtHandMod,handstand,x']= -0.12517813095365, ['map,2,spprtHandMod,handstand,y']= -0.07685262230104, ['map,2,spprtHandMod,handstand,z']= -0.04037261934765, })
accumulate({ ['map,0,swingHandMod,handstand,z']= 0.052960411382176, ['map,1,swingHandMod,handstand,z']= 0.066390621938835, ['map,2,swingHandMod,handstand,z']= 0.0050244745382897, ['map,0,spprtHandMod,handstand,z']= 0.056534853558082, ['map,1,spprtHandMod,handstand,z']= -0.10780235587132, ['map,2,spprtHandMod,handstand,z']= -0.04127029395292, })
accumulate({ }) -- skipped iter :-1 ['map,0,swingHandMod,handstand,x']= -0.044076707020124, ['map,1,swingHandMod,handstand,x']= 0.033674761458338, ['map,2,swingHandMod,handstand,x']= 0.13812672056502, ['map,0,spprtHandMod,handstand,x']= 0.096346538464105, ['map,1,spprtHandMod,handstand,x']= -0.043762853457621, ['map,2,spprtHandMod,handstand,x']= -0.1267686224562, 
accumulate({ }) -- skipped iter :-1 ['map,0,swingHandMod,handstand,y']= -0.023871246886904, ['map,1,swingHandMod,handstand,y']= 0.17437636712374, ['map,2,swingHandMod,handstand,y']= -0.0078458312454747, ['map,0,spprtHandMod,handstand,y']= -0.14159157919004, ['map,1,spprtHandMod,handstand,y']= -0.22082070119557, ['map,2,spprtHandMod,handstand,y']= -0.076349151509301, 
accumulate({ }) -- skipped iter :-1 ['map,0,swingHandMod,handstand,x']= -0.041827755248994, ['map,0,swingHandMod,handstand,y']= -0.021034438762085, ['map,0,swingHandMod,handstand,z']= 0.058057006961145, ['map,1,swingHandMod,handstand,x']= 0.031908196241618, ['map,1,swingHandMod,handstand,y']= 0.18009534992139, ['map,1,swingHandMod,handstand,z']= 0.076661133565833, ['map,2,swingHandMod,handstand,x']= 0.14135296324544, ['map,2,swingHandMod,handstand,y']= -0.0033115486271832, ['map,2,swingHandMod,handstand,z']= 0.00088354043127611, ['map,0,spprtHandMod,handstand,x']= 0.09156798773269, ['map,0,spprtHandMod,handstand,y']= -0.14985283074291, ['map,0,spprtHandMod,handstand,z']= 0.059495494129905, ['map,1,spprtHandMod,handstand,x']= -0.045067532354122, ['map,1,spprtHandMod,handstand,y']= -0.22086800990958, ['map,1,spprtHandMod,handstand,z']= -0.10689870244286, ['map,2,spprtHandMod,handstand,x']= -0.12773714232656, ['map,2,spprtHandMod,handstand,y']= -0.073800136047543, ['map,2,spprtHandMod,handstand,z']= -0.038721509374952, 
accumulate({ }) -- skipped iter :-1 ['map,0,swingHandMod,handstand,z']= 0.057817555915573, ['map,1,swingHandMod,handstand,z']= 0.079567777932815, ['map,2,swingHandMod,handstand,z']= -0.00016836523465127, ['map,0,spprtHandMod,handstand,z']= 0.062667406583241, ['map,1,spprtHandMod,handstand,z']= -0.1060378107228, ['map,2,spprtHandMod,handstand,z']= -0.038086368955386, 
useCases.unmapControlParam(useCases.handstand)
--accumulate({ ['keyframe,0,footLmod,handstand,R,x']= -0.18802838292699, ['keyframe,0,footLmod,handstand,R,y']= -0.081957326403221, ['keyframe,0,footLmod,handstand,R,z']= 0.081474424570292, ['keyframe,1,footLmod,handstand,R,x']= 0.12326555709786, ['keyframe,1,footLmod,handstand,R,y']= -0.29628943734498, ['keyframe,1,footLmod,handstand,R,z']= 0.33938542953082, ['keyframe,0,footRmod,handstand,R,x']= 0.22631067026052, ['keyframe,0,footRmod,handstand,R,y']= -0.054522029570285, ['keyframe,0,footRmod,handstand,R,z']= -0.25503194132376, ['keyframe,1,footRmod,handstand,R,x']= 0.42607774850091, ['keyframe,1,footRmod,handstand,R,y']= -0.29713379995833, ['keyframe,1,footRmod,handstand,R,z']= -0.085580814162852, ['keyframe,0,desiredLeaning,handstand,R,x']= 0.23148121174359, ['keyframe,0,desiredLeaning,handstand,R,z']= -0.47811814298471, })
--accumulate({ ['keyframe,0,footLmod,handstand,L,x']= 0.030783178758693, ['keyframe,0,footLmod,handstand,L,y']= -0.011014059899322, ['keyframe,0,footLmod,handstand,L,z']= -0.024225881516227, ['keyframe,1,footLmod,handstand,L,x']= -0.012144046454164, ['keyframe,1,footLmod,handstand,L,y']= -0.0076581975175714, ['keyframe,1,footLmod,handstand,L,z']= -0.016634408230436, ['keyframe,2,footLmod,handstand,L,x']= 0.003299326143538, ['keyframe,2,footLmod,handstand,L,y']= 0.0087079554426216, ['keyframe,2,footLmod,handstand,L,z']= -0.0043458742696003, ['keyframe,0,footRmod,handstand,L,x']= 0.018101453790649, ['keyframe,0,footRmod,handstand,L,y']= 0.00038146349777067, ['keyframe,0,footRmod,handstand,L,z']= 0.0083783264109549, ['keyframe,2,handLmod,handstand,L,x']= -0.0027909685121966, ['keyframe,2,handLmod,handstand,L,y']= 0.0057733437196035, ['keyframe,2,handLmod,handstand,L,z']= 0.0071824059952227, ['keyframe,2,handRmod,handstand,L,x']= -0.0080936238735021, ['keyframe,2,handRmod,handstand,L,y']= 0.0066518102069543, ['keyframe,2,handRmod,handstand,L,z']= -0.0050700182204375, })
--accumulate({ ['keyframe,0,footLmod,handstand,B,x']= 0.24834306508728, ['keyframe,0,footLmod,handstand,B,y']= -0.16574194404662, ['keyframe,0,footLmod,handstand,B,z']= -0.72317879410704, ['keyframe,0,handLmod,handstand,B,x']= -0.001786038090109, ['keyframe,0,handLmod,handstand,B,y']= 0.0041093551141683, ['keyframe,0,handLmod,handstand,B,z']= 0.0048168977839618, ['keyframe,0,handRmod,handstand,B,x']= 0.0051699099261094, ['keyframe,0,handRmod,handstand,B,y']= 0.00173836017472, ['keyframe,0,handRmod,handstand,B,z']= 0.0011795547204731, })
--accumulate({ ['keyframe,0,handLmod,handstand,LH0,x']= 0.067478988543621, ['keyframe,0,handLmod,handstand,LH0,y']= -0.066970983147178, ['keyframe,0,handLmod,handstand,LH0,z']= -0.016857611080951, ['keyframe,1,handLmod,handstand,LH0,x']= 0.040316096924345, ['keyframe,1,handLmod,handstand,LH0,y']= -0.091124296662768, ['keyframe,1,handLmod,handstand,LH0,z']= -0.0086113418505756, ['keyframe,2,handLmod,handstand,LH0,x']= 0.048234764288184, ['keyframe,2,handLmod,handstand,LH0,y']= -0.085795314138268, ['keyframe,2,handLmod,handstand,LH0,z']= -0.018359562215444, ['keyframe,0,handRmod,handstand,LH0,x']= -0.029543467183367, ['keyframe,0,handRmod,handstand,LH0,y']= 0.02167626427672, ['keyframe,0,handRmod,handstand,LH0,z']= 0.016134119139059, ['keyframe,1,handRmod,handstand,LH0,x']= -0.033684921240748, ['keyframe,1,handRmod,handstand,LH0,y']= 0.0028556060911265, ['keyframe,1,handRmod,handstand,LH0,z']= -0.006082056482336, ['keyframe,2,handRmod,handstand,LH0,x']= -0.0022508099779405, ['keyframe,2,handRmod,handstand,LH0,y']= 0.00168861652145, ['keyframe,2,handRmod,handstand,LH0,z']= 0.024920875919462, })
accumulate({
	['keyframe,0,desiredLeaning,handstand,R,x']=0.0115976717226, 
	['keyframe,0,desiredLeaning,handstand,R,z']=-0.006649614049112, 
	['keyframe,0,footLmod,handstand,R,x']=0, 
	['keyframe,0,footLmod,handstand,R,y']=0, 
	['keyframe,0,footLmod,handstand,R,z']=0, 
	['keyframe,0,footRmod,handstand,R,x']=0, 
	['keyframe,0,footRmod,handstand,R,y']=0, 
	['keyframe,0,footRmod,handstand,R,z']=0, 
	['keyframe,0,footLmod,handstand,R,x']=0, 
	['keyframe,0,footLmod,handstand,R,y']=0, 
	['keyframe,0,footLmod,handstand,R,z']=0, 
	['keyframe,0,footRmod,handstand,R,x']=0, 
	['keyframe,0,footRmod,handstand,R,y']=0, 
	['keyframe,0,footRmod,handstand,R,z']=0, 
})
useCases.clearParam(useCases.handstand, 'keyframe,.,hand.mod,handstand,R,')
accumulate({
	['keyframe,0,footLmod,handstand,L,y']=0, 
})
--accumulate({ ['keyframe,0,footLmod,handstand,R,x']= 0.00099772820007473, ['keyframe,0,footLmod,handstand,R,y']= 0.0028084066970252, ['keyframe,0,footLmod,handstand,R,z']= -0.00020233015811438, ['keyframe,1,footLmod,handstand,R,x']= -0.032692933088805, ['keyframe,1,footLmod,handstand,R,y']= -0.30830789830273, ['keyframe,1,footLmod,handstand,R,z']= -0.018003958049815, ['keyframe,0,footRmod,handstand,R,x']= -0.00054046949474532, ['keyframe,0,footRmod,handstand,R,y']= 0.0026225593010524, ['keyframe,0,footRmod,handstand,R,z']= 0.0013175343296918, ['keyframe,1,footRmod,handstand,R,x']= 0.16342159352511, ['keyframe,1,footRmod,handstand,R,y']= -0.11656848058828, ['keyframe,1,footRmod,handstand,R,z']= -0.17323330811273, ['keyframe,0,desiredLeaning,handstand,R,x']= 0.043917861301149, ['keyframe,0,desiredLeaning,handstand,R,z']= -0.020014637849997, })
--accumulate({ ['keyframe,0,footLmod,handstand,L,x']= 0.050689833824907, ['keyframe,0,footLmod,handstand,L,y']= -0.031543961689846, ['keyframe,0,footLmod,handstand,L,z']= 0.0075294481567402, ['keyframe,1,footLmod,handstand,L,x']= 0.012059374880784, ['keyframe,1,footLmod,handstand,L,y']= -0.026999965059128, ['keyframe,1,footLmod,handstand,L,z']= -0.034641511123917, ['keyframe,2,footLmod,handstand,L,x']= -0.0040045117497564, ['keyframe,2,footLmod,handstand,L,y']= -0.0010899662411082, ['keyframe,2,footLmod,handstand,L,z']= 0.0061601449822235, ['keyframe,0,footRmod,handstand,L,x']= 0.014374299711195, ['keyframe,0,footRmod,handstand,L,y']= 0.0075845672717959, ['keyframe,0,footRmod,handstand,L,z']= -0.010937691519326, ['keyframe,2,handLmod,handstand,L,x']= 0.037999379006386, ['keyframe,2,handLmod,handstand,L,y']= 0.010459819452987, ['keyframe,2,handLmod,handstand,L,z']= -0.0062318088899519, ['keyframe,2,handRmod,handstand,L,x']= 0.016146828830177, ['keyframe,2,handRmod,handstand,L,y']= -0.014950649440729, ['keyframe,2,handRmod,handstand,L,z']= -0.00019434179016168, })
--accumulate({ ['keyframe,0,footLmod,handstand,B,x']= 0.34267249247509, ['keyframe,0,footLmod,handstand,B,y']= -0.63834008944992, ['keyframe,0,footLmod,handstand,B,z']= 0.10595692531851, ['keyframe,0,handLmod,handstand,B,x']= 0.11494629901487, ['keyframe,0,handLmod,handstand,B,y']= 0.0064294901010586, ['keyframe,0,handLmod,handstand,B,z']= 0.1364404398698, ['keyframe,0,handRmod,handstand,B,x']= -1.0149172823469, ['keyframe,0,handRmod,handstand,B,y']= -0.39734899778029, ['keyframe,0,handRmod,handstand,B,z']= -0.26799477145069, })
--accumulate({ ['keyframe,0,footLmod,handstand,R,x']= 0.001280240503735, ['keyframe,0,footLmod,handstand,R,y']= 0.0013457398960968, ['keyframe,0,footLmod,handstand,R,z']= -1.5390747459474e-05, ['keyframe,1,footLmod,handstand,R,x']= -0.031717002950223, ['keyframe,1,footLmod,handstand,R,y']= -0.30813548493959, ['keyframe,1,footLmod,handstand,R,z']= -0.018505312012612, ['keyframe,0,footRmod,handstand,R,x']= 0.00049343945007203, ['keyframe,0,footRmod,handstand,R,y']= 0.003789659546355, ['keyframe,0,footRmod,handstand,R,z']= 7.1880601777801e-05, ['keyframe,1,footRmod,handstand,R,x']= 0.16417728946272, ['keyframe,1,footRmod,handstand,R,y']= -0.11852385598711, ['keyframe,1,footRmod,handstand,R,z']= -0.17250899708342, ['keyframe,0,desiredLeaning,handstand,R,x']= 0.042964441379798, ['keyframe,0,desiredLeaning,handstand,R,z']= -0.017209596511161, })
----accumulate({ ['keyframe,0,desiredLeaning,handstand,R,x']= 0.063058380952747, ['keyframe,0,desiredLeaning,handstand,R,z']= -0.017036680988548, })
----accumulate({	['keyframe,0,desiredLeaning,handstand,R,x']=0.04115976717226, ['keyframe,0,desiredLeaning,handstand,R,z']=-0.0206649614049112, })
accumulate({
	['keyframe,0,desiredLeaning,handstand,R,x']=0.0,
	['keyframe,0,desiredLeaning,handstand,R,z']=-0.0,
})
accumulate({ ['keyframe,0,footLmod,handstand,R,x']= -0.00010806784852267, ['keyframe,0,footLmod,handstand,R,y']= -0.00082647129860026, ['keyframe,0,footLmod,handstand,R,z']= -6.663509353627e-05, ['keyframe,1,footLmod,handstand,R,x']= -0.031867794343858, ['keyframe,1,footLmod,handstand,R,y']= -0.30827822823389, ['keyframe,1,footLmod,handstand,R,z']= -0.013909202490269, ['keyframe,0,footRmod,handstand,R,x']= 0.00068334710675423, ['keyframe,0,footRmod,handstand,R,y']= -0.00016173956685153, ['keyframe,0,footRmod,handstand,R,z']= -0.0011686772955544, ['keyframe,1,footRmod,handstand,R,x']= 0.16040213070503, ['keyframe,1,footRmod,handstand,R,y']= -0.1126693898802, ['keyframe,1,footRmod,handstand,R,z']= -0.17606815587145, ['keyframe,0,desiredLeaning,handstand,R,x']= 0.0023154657086066, ['keyframe,0,desiredLeaning,handstand,R,z']= 0.0020434440083301, })
accumulate({ ['keyframe,0,footLmod,handstand,L,x']= 0.13956593384244, ['keyframe,0,footLmod,handstand,L,y']= 0.031479089473281, ['keyframe,0,footLmod,handstand,L,z']= -0.019773990051217, ['keyframe,1,footLmod,handstand,L,x']= -0.10629949027614, ['keyframe,1,footLmod,handstand,L,y']= -0.01796628299971, ['keyframe,1,footLmod,handstand,L,z']= -0.026866671294005, ['keyframe,2,footLmod,handstand,L,x']= -0.21104384786063, ['keyframe,2,footLmod,handstand,L,y']= -0.2834670240901, ['keyframe,2,footLmod,handstand,L,z']= 0.26136321683754, ['keyframe,0,footRmod,handstand,L,x']= 0.10615310390291, ['keyframe,0,footRmod,handstand,L,y']= -0.019852221176383, ['keyframe,0,footRmod,handstand,L,z']= -0.10181497403941, ['keyframe,2,handLmod,handstand,L,x']= -0.16666827363852, ['keyframe,2,handLmod,handstand,L,y']= -0.13252669177994, ['keyframe,2,handLmod,handstand,L,z']= -0.032709116405713, ['keyframe,2,handRmod,handstand,L,x']= 0.039142782510494, ['keyframe,2,handRmod,handstand,L,y']= -0.29811722999881, ['keyframe,2,handRmod,handstand,L,z']= -0.61357677273171, })
accumulate({ ['keyframe,0,footLmod,handstand,R,x']= 0.0040554202780934, ['keyframe,0,footLmod,handstand,R,y']= 0.0013827498955202, ['keyframe,0,footLmod,handstand,R,z']= 0.0028976618158368, ['keyframe,1,footLmod,handstand,R,x']= -0.034091048388232, ['keyframe,1,footLmod,handstand,R,y']= -0.3101648001927, ['keyframe,1,footLmod,handstand,R,z']= -0.011139720293224, ['keyframe,0,footRmod,handstand,R,x']= 0.00082985057353191, ['keyframe,0,footRmod,handstand,R,y']= -0.0024072368844078, ['keyframe,0,footRmod,handstand,R,z']= -0.004495154269486, ['keyframe,1,footRmod,handstand,R,x']= 0.1617376473708, ['keyframe,1,footRmod,handstand,R,y']= -0.11002704750003, ['keyframe,1,footRmod,handstand,R,z']= -0.17979076701169, ['keyframe,0,desiredLeaning,handstand,R,x']= 0.0029262631570288, ['keyframe,0,desiredLeaning,handstand,R,z']= 0.00054046035335137, })
accumulate({ ['keyframe,0,footLmod,handstand,L,x']= 0.16109903731479, ['keyframe,0,footLmod,handstand,L,y']= -0.0054223923880281, ['keyframe,0,footLmod,handstand,L,z']= 0.027424109525816, ['keyframe,1,footLmod,handstand,L,x']= -0.14541107635122, ['keyframe,1,footLmod,handstand,L,y']= -0.0099700035623955, ['keyframe,1,footLmod,handstand,L,z']= 0.023334978298467, ['keyframe,2,footLmod,handstand,L,x']= -0.23265277499938, ['keyframe,2,footLmod,handstand,L,y']= -0.36757737249005, ['keyframe,2,footLmod,handstand,L,z']= 0.20137612079513, ['keyframe,0,footRmod,handstand,L,x']= 0.10058369034701, ['keyframe,0,footRmod,handstand,L,y']= -0.0088377528129387, ['keyframe,0,footRmod,handstand,L,z']= -0.083542547586733, ['keyframe,2,handLmod,handstand,L,x']= -0.14434305621854, ['keyframe,2,handLmod,handstand,L,y']= -0.17569582588072, ['keyframe,2,handLmod,handstand,L,z']= 0.075111491941511, ['keyframe,2,handRmod,handstand,L,x']= -0.032507092807723, ['keyframe,2,handRmod,handstand,L,y']= -0.24550992645082, ['keyframe,2,handRmod,handstand,L,z']= -0.55449683921729, })
accumulate({ ['keyframe,0,footLmod,handstand,R,x']= 0.0041895970300392, ['keyframe,0,footLmod,handstand,R,y']= 0.00095506992327221, ['keyframe,0,footLmod,handstand,R,z']= 0.0027715395823245, ['keyframe,1,footLmod,handstand,R,x']= -0.034779981805051, ['keyframe,1,footLmod,handstand,R,y']= -0.30953603291458, ['keyframe,1,footLmod,handstand,R,z']= -0.012161045900917, ['keyframe,0,footRmod,handstand,R,x']= -0.0003486626403408, ['keyframe,0,footRmod,handstand,R,y']= -0.0013382394611827, ['keyframe,0,footRmod,handstand,R,z']= -0.0028986464788394, ['keyframe,1,footRmod,handstand,R,x']= 0.16109778402398, ['keyframe,1,footRmod,handstand,R,y']= -0.11066327448752, ['keyframe,1,footRmod,handstand,R,z']= -0.17886195715936, ['keyframe,0,desiredLeaning,handstand,R,x']= 0.0026872003592527, ['keyframe,0,desiredLeaning,handstand,R,z']= -0.00082651647117233, })
accumulate({ ['keyframe,0,footLmod,handstand,L,x']= 0.15773882021346, ['keyframe,0,footLmod,handstand,L,y']= 0.00015236401011411, ['keyframe,0,footLmod,handstand,L,z']= 0.01090509445167, ['keyframe,1,footLmod,handstand,L,x']= -0.15025077991333, ['keyframe,1,footLmod,handstand,L,y']= -0.013604492945372, ['keyframe,1,footLmod,handstand,L,z']= 0.066563301965997, ['keyframe,2,footLmod,handstand,L,x']= -0.22222805553127, ['keyframe,2,footLmod,handstand,L,y']= -0.40735357023757, ['keyframe,2,footLmod,handstand,L,z']= 0.19698110929162, ['keyframe,0,footRmod,handstand,L,x']= 0.097389631536383, ['keyframe,0,footRmod,handstand,L,y']= -0.010387500653693, ['keyframe,0,footRmod,handstand,L,z']= -0.076640991002515, ['keyframe,2,handLmod,handstand,L,x']= -0.14258895864894, ['keyframe,2,handLmod,handstand,L,y']= -0.17770245638685, ['keyframe,2,handLmod,handstand,L,z']= 0.12284492459138, ['keyframe,2,handRmod,handstand,L,x']= -0.073933047558494, ['keyframe,2,handRmod,handstand,L,y']= -0.19191918068663, ['keyframe,2,handRmod,handstand,L,z']= -0.53208269242521, })
accumulate({ ['keyframe,0,footLmod,handstand,R,x']= 0.0032383584449227, ['keyframe,0,footLmod,handstand,R,y']= 0.00099381211307372, ['keyframe,0,footLmod,handstand,R,z']= 0.0032782840377454, ['keyframe,1,footLmod,handstand,R,x']= -0.034148662505459, ['keyframe,1,footLmod,handstand,R,y']= -0.30881683823387, ['keyframe,1,footLmod,handstand,R,z']= -0.01258187632644, ['keyframe,0,footRmod,handstand,R,x']= -0.00056064433732827, ['keyframe,0,footRmod,handstand,R,y']= -0.0035731803140065, ['keyframe,0,footRmod,handstand,R,z']= -0.0029338324283983, ['keyframe,1,footRmod,handstand,R,x']= 0.15641285090778, ['keyframe,1,footRmod,handstand,R,y']= -0.10679624010153, ['keyframe,1,footRmod,handstand,R,z']= -0.17825430351435, ['keyframe,0,desiredLeaning,handstand,R,x']= 0.0030207538290277, ['keyframe,0,desiredLeaning,handstand,R,z']= 0.00020800728215575,}) 
accumulate({ ['keyframe,0,footLmod,handstand,L,x']= 0.16070688166305, ['keyframe,0,footLmod,handstand,L,y']= -0.0067613350414367, ['keyframe,0,footLmod,handstand,L,z']= 0.004236466141245, ['keyframe,1,footLmod,handstand,L,x']= -0.1579064464379, ['keyframe,1,footLmod,handstand,L,y']= -0.0088181569521591, ['keyframe,1,footLmod,handstand,L,z']= 0.07444592030785, ['keyframe,2,footLmod,handstand,L,x']= -0.24774733489356, ['keyframe,2,footLmod,handstand,L,y']= -0.43540550178607, ['keyframe,2,footLmod,handstand,L,z']= 0.18413343558696, ['keyframe,0,footRmod,handstand,L,x']= 0.096046312172738, ['keyframe,0,footRmod,handstand,L,y']= -0.0073807529601544, ['keyframe,0,footRmod,handstand,L,z']= -0.076785249177622, ['keyframe,2,handLmod,handstand,L,x']= -0.15572542454516, ['keyframe,2,handLmod,handstand,L,y']= -0.21164516295523, ['keyframe,2,handLmod,handstand,L,z']= 0.12126884068502, ['keyframe,2,handRmod,handstand,L,x']= -0.087847688322388, ['keyframe,2,handRmod,handstand,L,y']= -0.20772308153448, ['keyframe,2,handRmod,handstand,L,z']= -0.53725984514978, })
accumulate({ ['keyframe,0,footLmod,handstand,L,x']= 0.15485947606767, ['keyframe,0,footLmod,handstand,L,y']= -0.003020891194602, ['keyframe,0,footLmod,handstand,L,z']= 0.0097406504382561, ['keyframe,1,footLmod,handstand,L,x']= -0.14819653698309, ['keyframe,1,footLmod,handstand,L,y']= -0.0026393715110806, ['keyframe,1,footLmod,handstand,L,z']= 0.078145663851317, ['keyframe,2,footLmod,handstand,L,x']= -0.25784007493138, ['keyframe,2,footLmod,handstand,L,y']= -0.43859168195705, ['keyframe,2,footLmod,handstand,L,z']= 0.16711973254665, ['keyframe,0,footRmod,handstand,L,x']= 0.094491266084935, ['keyframe,0,footRmod,handstand,L,y']= -0.0092002768617323, ['keyframe,0,footRmod,handstand,L,z']= -0.076537139660749, ['keyframe,2,handLmod,handstand,L,x']= -0.18625214463933, ['keyframe,2,handLmod,handstand,L,y']= -0.23037931529494, ['keyframe,2,handLmod,handstand,L,z']= 0.14991664723634, ['keyframe,2,handRmod,handstand,L,x']= -0.11093055492879, ['keyframe,2,handRmod,handstand,L,y']= -0.20415211310097, ['keyframe,2,handRmod,handstand,L,z']= -0.54146609841074, })
accumulate({ ['keyframe,0,footLmod,handstand,R,x']= 0.0017413978490634, ['keyframe,0,footLmod,handstand,R,y']= 0.0003733893040655, ['keyframe,0,footLmod,handstand,R,z']= -0.00098867799844076, ['keyframe,1,footLmod,handstand,R,x']= -0.035536096173887, ['keyframe,1,footLmod,handstand,R,y']= -0.30767178580496, ['keyframe,1,footLmod,handstand,R,z']= -0.010975113703647, ['keyframe,0,footRmod,handstand,R,x']= 0.002025845538569, ['keyframe,0,footRmod,handstand,R,y']= -0.00354306016812, ['keyframe,0,footRmod,handstand,R,z']= -0.00020410917242337, ['keyframe,1,footRmod,handstand,R,x']= 0.15680326663028, ['keyframe,1,footRmod,handstand,R,y']= -0.10918724784839, ['keyframe,1,footRmod,handstand,R,z']= -0.17514849961051, ['keyframe,0,desiredLeaning,handstand,R,x']= 0.0024707671839377, ['keyframe,0,desiredLeaning,handstand,R,z']= 0.0015041796441047, })
accumulate({ ['keyframe,0,footLmod,handstand,L,x']= 0.15778119123302, ['keyframe,0,footLmod,handstand,L,y']= -0.0055217806531155, ['keyframe,0,footLmod,handstand,L,z']= 0.01044721657037, ['keyframe,1,footLmod,handstand,L,x']= -0.15221977731643, ['keyframe,1,footLmod,handstand,L,y']= 0.00087626519465941, ['keyframe,1,footLmod,handstand,L,z']= 0.079143031793079, ['keyframe,2,footLmod,handstand,L,x']= -0.25776620021526, ['keyframe,2,footLmod,handstand,L,y']= -0.43639721170407, ['keyframe,2,footLmod,handstand,L,z']= 0.1573691511492, ['keyframe,0,footRmod,handstand,L,x']= 0.095181663751379, ['keyframe,0,footRmod,handstand,L,y']= -0.0095314933167821, ['keyframe,0,footRmod,handstand,L,z']= -0.077750600959811, ['keyframe,2,handLmod,handstand,L,x']= -0.18927656346038, ['keyframe,2,handLmod,handstand,L,y']= -0.22274014617526, ['keyframe,2,handLmod,handstand,L,z']= 0.16233831404436, ['keyframe,2,handRmod,handstand,L,x']= -0.10423735393031, ['keyframe,2,handRmod,handstand,L,y']= -0.19682373834, ['keyframe,2,handRmod,handstand,L,z']= -0.54374227917202, })
accumulate({ ['keyframe,0,footLmod,handstand,R,x']= 0.0022129436180348, ['keyframe,0,footLmod,handstand,R,y']= -0.00062239845309933, ['keyframe,0,footLmod,handstand,R,z']= -5.7728988394106e-05, ['keyframe,1,footLmod,handstand,R,x']= -0.032659145330453, ['keyframe,1,footLmod,handstand,R,y']= -0.30890417862501, ['keyframe,1,footLmod,handstand,R,z']= -0.008427631306471, ['keyframe,0,footRmod,handstand,R,x']= 0.0060966204442904, ['keyframe,0,footRmod,handstand,R,y']= -0.006256123730261, ['keyframe,0,footRmod,handstand,R,z']= -0.0033158894484397, ['keyframe,1,footRmod,handstand,R,x']= 0.15463609942784, ['keyframe,1,footRmod,handstand,R,y']= -0.10802089520321, ['keyframe,1,footRmod,handstand,R,z']= -0.17763169892938, ['keyframe,0,desiredLeaning,handstand,R,x']= 0.0028099551758914, ['keyframe,0,desiredLeaning,handstand,R,z']= 0.003408865151187, })
accumulate({ ['keyframe,0,footLmod,handstand,L,x']= 0.1585438080566, ['keyframe,0,footLmod,handstand,L,y']= -0.0042453410977044, ['keyframe,0,footLmod,handstand,L,z']= 0.0090345166820979, ['keyframe,1,footLmod,handstand,L,x']= -0.14476268328969, ['keyframe,1,footLmod,handstand,L,y']= 0.0014210276921423, ['keyframe,1,footLmod,handstand,L,z']= 0.085462005364786, ['keyframe,2,footLmod,handstand,L,x']= -0.27384505406639, ['keyframe,2,footLmod,handstand,L,y']= -0.43149078874469, ['keyframe,2,footLmod,handstand,L,z']= 0.14818039350171, ['keyframe,0,footRmod,handstand,L,x']= 0.096714886620212, ['keyframe,0,footRmod,handstand,L,y']= -0.0071773835409903, ['keyframe,0,footRmod,handstand,L,z']= -0.080982734173377, ['keyframe,2,handLmod,handstand,L,x']= -0.19995599954295, ['keyframe,2,handLmod,handstand,L,y']= -0.23314076136916, ['keyframe,2,handLmod,handstand,L,z']= 0.1648667186011, ['keyframe,2,handRmod,handstand,L,x']= -0.10085548651731, ['keyframe,2,handRmod,handstand,L,y']= -0.18003199206571, ['keyframe,2,handRmod,handstand,L,z']= -0.55842527476639, })
accumulate({ ['keyframe,0,footLmod,handstand,R,x']= 0.0021441091678323, ['keyframe,0,footLmod,handstand,R,y']= -0.0014071478331034, ['keyframe,0,footLmod,handstand,R,z']= 0.00087079694367822, ['keyframe,1,footLmod,handstand,R,x']= -0.033441741580449, ['keyframe,1,footLmod,handstand,R,y']= -0.30874051925578, ['keyframe,1,footLmod,handstand,R,z']= -0.0089143332305397, ['keyframe,0,footRmod,handstand,R,x']= 0.005524970071023, ['keyframe,0,footRmod,handstand,R,y']= -0.0060026142778642, ['keyframe,0,footRmod,handstand,R,z']= 0.00011584205240568, ['keyframe,1,footRmod,handstand,R,x']= 0.15530590895562, ['keyframe,1,footRmod,handstand,R,y']= -0.10888184618647, ['keyframe,1,footRmod,handstand,R,z']= -0.17633218772145, ['keyframe,0,desiredLeaning,handstand,R,x']= 0.0022995791235022, ['keyframe,0,desiredLeaning,handstand,R,z']= 0.0032802601225506,}) 
accumulate({ ['keyframe,0,footLmod,handstand,L,x']= 0.15731429135961, ['keyframe,0,footLmod,handstand,L,y']= -0.010320469687783, ['keyframe,0,footLmod,handstand,L,z']= 0.0057738912227953, ['keyframe,1,footLmod,handstand,L,x']= -0.14871820191329, ['keyframe,1,footLmod,handstand,L,y']= 0.0063991044618283, ['keyframe,1,footLmod,handstand,L,z']= 0.087706198500973, ['keyframe,2,footLmod,handstand,L,x']= -0.27619928542015, ['keyframe,2,footLmod,handstand,L,y']= -0.43055956472062, ['keyframe,2,footLmod,handstand,L,z']= 0.14422839646981, ['keyframe,0,footRmod,handstand,L,x']= 0.097428820038912, ['keyframe,0,footRmod,handstand,L,y']= -0.0052736047444325, ['keyframe,0,footRmod,handstand,L,z']= -0.085387289017109, ['keyframe,2,handLmod,handstand,L,x']= -0.203481286322, ['keyframe,2,handLmod,handstand,L,y']= -0.23476021833343, ['keyframe,2,handLmod,handstand,L,z']= 0.1690119485859, ['keyframe,2,handRmod,handstand,L,x']= -0.094713940185171, ['keyframe,2,handRmod,handstand,L,y']= -0.18843344147167, ['keyframe,2,handRmod,handstand,L,z']= -0.56462413388773, })
accumulate({ ['keyframe,0,footLmod,handstand,R,x']= 0.00056741780410883, ['keyframe,0,footLmod,handstand,R,y']= -0.0022055723558487, ['keyframe,0,footLmod,handstand,R,z']= 0.0017291655083796, ['keyframe,1,footLmod,handstand,R,x']= -0.035156208368853, ['keyframe,1,footLmod,handstand,R,y']= -0.30828222681803, ['keyframe,1,footLmod,handstand,R,z']= -0.010924847214321, ['keyframe,0,footRmod,handstand,R,x']= 0.0067359794389948, ['keyframe,0,footRmod,handstand,R,y']= -0.0071308891368112, ['keyframe,0,footRmod,handstand,R,z']= 0.00222583966541, ['keyframe,1,footRmod,handstand,R,x']= 0.15337833550849, ['keyframe,1,footRmod,handstand,R,y']= -0.10900630501944, ['keyframe,1,footRmod,handstand,R,z']= -0.17908555085776, ['keyframe,0,desiredLeaning,handstand,R,x']= 0.0013588047668895, ['keyframe,0,desiredLeaning,handstand,R,z']= 0.0044311574545489, })
accumulate({ ['keyframe,0,footLmod,handstand,L,x']= 0.1325620700427, ['keyframe,0,footLmod,handstand,L,y']= -0.0060176168287218, ['keyframe,0,footLmod,handstand,L,z']= 0.015422554780966, ['keyframe,1,footLmod,handstand,L,x']= -0.15830819061296, ['keyframe,1,footLmod,handstand,L,y']= 0.010032676726774, ['keyframe,1,footLmod,handstand,L,z']= 0.098177803499585, ['keyframe,2,footLmod,handstand,L,x']= -0.26908657383546, ['keyframe,2,footLmod,handstand,L,y']= -0.44958237393757, ['keyframe,2,footLmod,handstand,L,z']= 0.13983705259632, ['keyframe,0,footRmod,handstand,L,x']= 0.10305942607182, ['keyframe,0,footRmod,handstand,L,y']= -0.0068500577656765, ['keyframe,0,footRmod,handstand,L,z']= -0.082198376580346, ['keyframe,2,handLmod,handstand,L,x']= -0.21730408494557, ['keyframe,2,handLmod,handstand,L,y']= -0.22759686881054, ['keyframe,2,handLmod,handstand,L,z']= 0.19611654743252, ['keyframe,2,handRmod,handstand,L,x']= -0.048710298353202, ['keyframe,2,handRmod,handstand,L,y']= -0.19946565363928, ['keyframe,2,handRmod,handstand,L,z']= -0.59955320084717, })
accumulate({ ['keyframe,0,footLmod,handstand,L,x']= 0.1325620700427, ['keyframe,0,footLmod,handstand,L,y']= -0.0060176168287218, ['keyframe,0,footLmod,handstand,L,z']= 0.015422554780966, ['keyframe,1,footLmod,handstand,L,x']= -0.15830819061296, ['keyframe,1,footLmod,handstand,L,y']= 0.010032676726774, ['keyframe,1,footLmod,handstand,L,z']= 0.098177803499585, ['keyframe,2,footLmod,handstand,L,x']= -0.26908657383546, ['keyframe,2,footLmod,handstand,L,y']= -0.44958237393757, ['keyframe,2,footLmod,handstand,L,z']= 0.13983705259632, ['keyframe,0,footRmod,handstand,L,x']= 0.10305942607182, ['keyframe,0,footRmod,handstand,L,y']= -0.0068500577656765, ['keyframe,0,footRmod,handstand,L,z']= -0.082198376580346, ['keyframe,2,handLmod,handstand,L,x']= -0.21730408494557, ['keyframe,2,handLmod,handstand,L,y']= -0.22759686881054, ['keyframe,2,handLmod,handstand,L,z']= 0.19611654743252, ['keyframe,2,handRmod,handstand,L,x']= -0.048710298353202, ['keyframe,2,handRmod,handstand,L,y']= -0.19946565363928, ['keyframe,2,handRmod,handstand,L,z']= -0.59955320084717, })
accumulate({ ['keyframe,0,footLmod,handstand,R,x']= 0.0012325382479268, ['keyframe,0,footLmod,handstand,R,y']= 0.0034525817989516, ['keyframe,0,footLmod,handstand,R,z']= -0.0016839765042715, ['keyframe,1,footLmod,handstand,R,x']= -0.029164706872514, ['keyframe,1,footLmod,handstand,R,y']= -0.30899495291976, ['keyframe,1,footLmod,handstand,R,z']= -0.0064237853484352, ['keyframe,0,footRmod,handstand,R,x']= 0.0054018114476535, ['keyframe,0,footRmod,handstand,R,y']= -0.0041235981454334, ['keyframe,0,footRmod,handstand,R,z']= 0.0038230181537334, ['keyframe,1,footRmod,handstand,R,x']= 0.15990304190493, ['keyframe,1,footRmod,handstand,R,y']= -0.10279143179906, ['keyframe,1,footRmod,handstand,R,z']= -0.17886407794813, ['keyframe,0,desiredLeaning,handstand,R,x']= 1.4049181869863e-05, ['keyframe,0,desiredLeaning,handstand,R,z']= 0.012042534215845, })
accumulate({ ['keyframe,0,footLmod,handstand,L,x']= 0.13135244196529, ['keyframe,0,footLmod,handstand,L,y']= 0.0094850252266894, ['keyframe,0,footLmod,handstand,L,z']= 0.028358165629766, ['keyframe,1,footLmod,handstand,L,x']= -0.17043816259346, ['keyframe,1,footLmod,handstand,L,y']= 0.025626762258035, ['keyframe,1,footLmod,handstand,L,z']= 0.08770053310809, ['keyframe,2,footLmod,handstand,L,x']= -0.2814807512131, ['keyframe,2,footLmod,handstand,L,y']= -0.43714720997872, ['keyframe,2,footLmod,handstand,L,z']= 0.13898234111077, ['keyframe,0,footRmod,handstand,L,x']= 0.11242212440543, ['keyframe,0,footRmod,handstand,L,y']= -0.0088885352508606, ['keyframe,0,footRmod,handstand,L,z']= -0.092466970328097, ['keyframe,2,handLmod,handstand,L,x']= -0.22596258885168, ['keyframe,2,handLmod,handstand,L,y']= -0.23164701596033, ['keyframe,2,handLmod,handstand,L,z']= 0.2131590384259, ['keyframe,2,handRmod,handstand,L,x']= -0.058757942179275, ['keyframe,2,handRmod,handstand,L,y']= -0.21024422787153, ['keyframe,2,handRmod,handstand,L,z']= -0.60544407577814, })
accumulate({ ['keyframe,0,footLmod,handstand,B,x']= 0.2963163814339, ['keyframe,0,footLmod,handstand,B,y']= -0.2500590027801, ['keyframe,0,footLmod,handstand,B,z']= -0.41647891427974, ['keyframe,0,handLmod,handstand,B,x']= -0.17070854986316, ['keyframe,0,handLmod,handstand,B,y']= -0.076237769736851, ['keyframe,0,handLmod,handstand,B,z']= -0.2756222996625, ['keyframe,0,handRmod,handstand,B,x']= -0.62597071596372, ['keyframe,0,handRmod,handstand,B,y']= -1.088892903291, ['keyframe,0,handRmod,handstand,B,z']= 0.63523362418719, })

--accumulate({ ['keyframe,0,handLmod,handstand,RH,y']= -0.019106164272055, ['keyframe,1,handLmod,handstand,RH,y']= 0.17111596261182, ['keyframe,2,handLmod,handstand,RH,y']= -0.013671944432739, ['keyframe,0,handRmod,handstand,RH,y']= -0.14274251768692, ['keyframe,1,handRmod,handstand,RH,y']= -0.22028893984747, ['keyframe,2,handRmod,handstand,RH,y']= -0.069366804040761, ['keyframe,0,handLmod,handstand,LH,y']= -0.13931051339467, ['keyframe,1,handLmod,handstand,LH,y']= -0.22220032309976, ['keyframe,2,handLmod,handstand,LH,y']= -0.07266753445838, ['keyframe,0,handRmod,handstand,LH,y']= -0.023322020424505, ['keyframe,1,handRmod,handstand,LH,y']= 0.16939758986339, ['keyframe,2,handRmod,handstand,LH,y']= -0.0014500177799648, })
accumulate({['useCase,QPservoDScaleCoef']=1.0})
accumulate({['useCase,QPservoDDScaleCoef']=0})
accumulate({['useCase,DScaleStart']=7})
	useCases.clampParam(useCases.handstand, 'handRmod,handstand,.H', 0.2)
	useCases.clampParam(useCases.handstand, 'handRmod,handstand,.H,y', -0.05, 0.2)
	useCases.clampParam(useCases.handstand, 'handLmod,handstand,.H', 0.2)
	useCases.clampParam(useCases.handstand, 'handRmod,handstand,.H,y', -0.05, 0.2)
accumulate({ ['keyframe,0,handLmod,handstand,RH,y']= -0.022706677893454, ['keyframe,1,handLmod,handstand,RH,y']= 0.16854148149086, ['keyframe,2,handLmod,handstand,RH,y']= -0.055703292022499, ['keyframe,0,handRmod,handstand,RH,y']= -0.0524295723171, ['keyframe,1,handRmod,handstand,RH,y']= -0.083188827804981, ['keyframe,2,handRmod,handstand,RH,y']= -0.061737920766613, ['keyframe,0,handLmod,handstand,LH,y']= -0.11944651472763, ['keyframe,1,handLmod,handstand,LH,y']= -0.19211401373808, ['keyframe,2,handLmod,handstand,LH,y']= -0.067696657290231, ['keyframe,0,handRmod,handstand,LH,y']= -0.053687174252144, ['keyframe,1,handRmod,handstand,LH,y']= 0.22464815153831, ['keyframe,2,handRmod,handstand,LH,y']= -0.029647529806912, })
if false then
	accumulate({ ['map,0,swingHandMod,handstand,x']= -0.15984967719755, ['map,0,swingHandMod,handstand,y']= -0.035586540835516, ['map,0,swingHandMod,handstand,z']= 0.088448184228718, ['map,1,swingHandMod,handstand,x']= 0.00078600137620745, ['map,1,swingHandMod,handstand,y']= 0.21163970162135, ['map,1,swingHandMod,handstand,z']= 0.028544109563379, ['map,2,swingHandMod,handstand,x']= 0.14141240926752, ['map,2,swingHandMod,handstand,y']= -0.093386652295765, ['map,2,swingHandMod,handstand,z']= -0.037876133887261, ['map,0,spprtHandMod,handstand,x']= 0.062451788102068, ['map,0,spprtHandMod,handstand,y']= -0.18411653944801, ['map,0,spprtHandMod,handstand,z']= 0.12858189051129, ['map,1,spprtHandMod,handstand,x']= -0.061840167503037, ['map,1,spprtHandMod,handstand,y']= -0.090668344726787, ['map,1,spprtHandMod,handstand,z']= -0.1300307201984, ['map,2,spprtHandMod,handstand,x']= -0.063237835218094, ['map,2,spprtHandMod,handstand,y']= -0.011850596453484, ['map,2,spprtHandMod,handstand,z']= -0.025392381656165, })
	accumulate({ ['map,0,swingHandMod,handstand,z']= 0.093166914381561, ['map,1,swingHandMod,handstand,z']= 0.028163352116533, ['map,2,swingHandMod,handstand,z']= -0.038725609411532, ['map,0,spprtHandMod,handstand,z']= 0.13351297951839, ['map,1,spprtHandMod,handstand,z']= -0.13504815941371, ['map,2,spprtHandMod,handstand,z']= -0.022246325690144, })
	useCases.unmapControlParam(useCases.handstand)
end
end
