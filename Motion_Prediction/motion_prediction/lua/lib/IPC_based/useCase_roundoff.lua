do

	local window=10
	local function segments(...)
		local frames={...}
		local firstFrame=frames[1]
		local lastFrame=array.back(frames)
		return table.ijoin({firstFrame-window}, frames, {lastFrame+window})
	end
--	local function footRef(...)
--		local tbl={}
--		local input={...}
--		local str=""
--		for i=1, table.getn(input)-1 do
--			array.pushBack(tbl, {input[i]-i, input[i+1]-i})
--			str=str..'{'..(input[i]-i)..","..(input[i+1]-i).."},"
--		end
--		--print(str)
--		return tbl
--	end
	local function createTransition(from, to, tgt)
		local f1,f2=unpack(string.tokenize(from, '%.'))
		local t1,t2=unpack(string.tokenize(to, '%.'))
		local g1,g2=unpack(string.tokenize(tgt, '%.'))
		return {"addInterpolatedSegment", grpName=g1, name=g2, seg0={f1,f2}, seg1={t1,t2}, startWeight=0, endWeight=1} 
	end
	useCases.roundoff_common={
		noCOMjoint=false,
		initialHeight=0.07,
		useAnalyticIK=true, 
		useMomentumPreservingIK=false,
		invAccConeCoef=0.1,
		contactMargin=0.01,
		velMarginOffset=0,
		maxPenetratingVel=0,
		useFootEncodingMethod2=
		{
			true,
			geom={
				L={ 
					{"lfoot", vector3(-0.01058, -0.087766, -0.021657)},
					{"ltoes", vector3(-0.038955, 0.016017-0.015, 0.057309)*0.5+vector3(-0.038955, 0.016017-0.015, 0.057309)*0.5},
				},
				R={
					{"rfoot",vector3( 0.01058+0.02, -0.097766, -0.021657)*0.5+ vector3( 0.01058-0.06, -0.097766, -0.021657)*0.5},
					{"rtoes",vector3( 0.058955, 0.016017-0.022, 0.057309)*0.5+vector3( -0.028955, 0.016017-0.022, 0.076)*0.5},
				},
				LH={
					{"lhand", vector3( 0.048955+ 0.00, 0.12+0.006017-0.085-0.18, 0.157309-0.08) },
					{"lhand", vector3( -0.038955+0.01, 0.12+0.016017-0.13, 0.057309-0.05) },
				},
				RH={
					{"rhand", vector3( 0.048955- 0.08, 0.12+0.006017-0.085-0.20, 0.157309-0.14) },
					{"rhand", vector3( -0.038955+0.07, 0.12+0.016017-0.13, 0.057309-0.07) },
				},
			}
		},
		lfootpos=vector3(0,-0.06,0.11),
		rfootpos=vector3(0.01,-0.08,0.11),
		lhandpos=vector3(0,-0.08,0.02),
		rhandpos=vector3(0,-0.09,0.01),
		scenario=scenarios.ETC,
		modelName='gymnist',
		mot_file=package.projectPath.."lua/IPC_based/gymnist_roundoff.dof",
		--model_file_name='../Resource/scripts/ui/RigidBodyWin/gymnist_old.wrl', -- override
		desiredPos=vector3(0,0,0),
		segmentation={fillLeft=0},
		segmentationCorrection =0,
		segmentations={
			--popa={
			--	names=          {"stand","rl","lr","COMdown","jump_up","jump_down", "jump_after1", "jump_after2","RL"},
			--	firstFrames={1324,1333, 1343,1393,1455,   1539,     1571,       1622,         1642,          1742, 1904,1933},
			--	--                  1     2    3     4    5           6            7             8              9   10  
			--	footRefL=footRef(   1,    2,   4,    4,   5,          6,           7,            8,             8,  10),
			--	footRefR=footRef(   1,    2,   2,    4,   5,          6,           7,            8,             9,  9 ),
			--	importanceL={   1,      1,   1,   1,        1,        0,         1,             1,             1,    1,},
			--	importanceR={   1,      1,   1,   1,        1,        0,         1,             1,             1,    1,},
			--	pendRotRefTime={ 0,     0,  -1,  -2,       -3,      -4,          -5,            -6,             0,  0},
			--	swingFoot={           "N", "L", "R",  "N",     "N",         "B",         "N",           "R" ,    "L"},
			--	usePositionControl=false,
			--},
			--walk2={ 
			--	firstFrames={636, 653,670,730,754 ,812,833, 890,  913,  964, 987, 1033, 1055, 1105, 1127, 1181, 1202,1257 },
			--	names={       "stand", "r","rl","l1","lr","r2","rl2","l2","lr2","R", "RL", "L", "LR1", "R2","RL2"},
			--	-- COPcontroller_dampingCoef ={5,15,5,15,5,   15,   5,   15,   5,   15,   5,    15,   5,   15,   5}, -- 
			--	swingFoot={      "N",  "L", "N","R","N",  "L","N",   "R","N",  "L",  "N",   "R",  "N", "L",  "N"},
			--	swingL={          0 ,   1 ,  0 , 0 , 0 ,   1 , 0 ,    0 , 0 ,   1 ,   0 ,    0 ,   0 ,  1 ,   0 ,0,0},
			--	swingR={          0 ,   0 ,  0 , 1 , 0 ,   0 , 0 ,    1 , 0 ,   0 ,   0 ,    1 ,   0 ,  0 ,   0 },
			--	--                 1   2   3    4    5    6    7     8
			--	footRefL= "convertFromSwingL",
			--	footRefR= "convertFromSwingR",
			--	usePositionControl=false,
			--	zmpCoordMethod=0,
			--},
			walk3={
				firstFrames={3745,3766,3786,3853,3877,3928,3948, 4000,4020, 4067,4088,4136, 4157, 4204, 4226, 4275, 4296,4345},--,4369},
				names={          "stand", "l","lr","r1","rl","l2","lr2","r2","rl2","L1", "LR", "R", "RL", "L2","LR2"},--,"R1", "RL2"},
				-- COPcontroller_dampingCoef ={5,15,5,15,5,   15,   5,   15,   5,   15,   5,    15,   5,   15,   5}, -- 
				swingFoot={         "N",  "R", "N","L","N",  "R","N",   "L","N",  "R",  "N",   "L",  "N", "R",  "N"},
				swingL={             0 ,   0 ,  0 , 1 , 0 ,   0 , 0 ,    1 , 0 ,   0 ,   0 ,    1 ,   0 ,  0 ,   0 },
				swingR={             0 ,   1 ,  0 , 0 , 0 ,   1 , 0 ,    0 , 0 ,   1 ,   0 ,    0 ,   0 ,  1 ,   0 ,0,0},
				--                    1   2   3    4    5    6    7     8
				footRefL= "convertFromSwingL",
				footRefR= "convertFromSwingR",
				usePositionControl=false,
			},
			roundoff2={
				--               1    2    3    4     5    6   7    8    9     10    11
				names={       'stand','l','lr','r', 'rl','l2','lL','L','LR',  'F', 'lr2','r2','rl2','l3','b', 'l4', 'lr3','r3', },
				firstFrames={10,20, 48, 129,157,  213, 221,241, 250, 263 , 290,   314, 339, 375,  392, 445, 467, 519, 548, 610, 624},
				-- 0: support foot 1: swing foot -1: transition to ignore -2: ignore
				--swingL={      0,    0 , 0 ,  1 ,   0 , 0 ,  0 ,  -1, -2 ,  -1 ,  0 ,   1 ,  0 ,    0 ,  0   , 0 ,   0 ,  1 , 0,0,0},
				swingL={        0,    0 , 0 ,  1 ,   0 , 0 ,  0 ,  -1, -1 ,   0 ,  0 ,   1 ,  0 ,    0 ,  0   , 0 ,   0 ,  1 , 0,0,0},
				swingRefL={     20,  -1, -1 ,  -1,  231, -1, -1,   -1, -1,   -1,   330,  -1,  481,    -1,   -1,  -1,   -1 ,-1, -1  },
				swingR={        0,    1 , 0 ,  0 ,   0 , -1, -2 ,  -2, -2 ,  -1 ,  0 ,  0 ,  0 ,    1 ,  0   , 1 ,   0 ,  0 , 0,},
				swingRefR={     20,   -1, 152, -1,   -1, -1,  -1,  -1 , -1,  -1,   345, -1,  -1,   -1,  -1 ,  -1,   -1,  -1, -1}, -- -1 means default
				swingLH={		-2,   -2, -2,  -2 ,  -2 , -1,  0 ,  0 ,  0 , -1 ,  -2,  -2,  -2,   -2 , -2   ,-2 ,  -2 , -2 ,},
				swingRefLH={     -1,  -1, -1 ,  -1,  -1,  -1, 273, -1, -1,   -1,   -1,  -1,  -1,   -1,  -1,   -1,   -1 ,-1, -1  },
				swingRH={		-2,   -2, -2,  -2 ,  -2 , -2,  -1,  -1,  0 , -1 ,  -2,  -2,  -2,   -2 , -2   ,-2 ,  -2 , -2 ,},
				swingRefRH={     -1,  -1, -1 ,  -1,  -1,  -1, -1,   -1, 275,  -1,   -1,  -1,  -1,   -1,  -1,   -1,   -1 ,-1, -1  },
				--pendRotRefTime={ 0,    0,  0 ,  0 ,  -1 , -2,  -3,  -4, -5 , -6 ,  0 ,  0 ,  0 ,    0 ,  0   , 0 ,   0 ,  0 ,},
				usePositionControl={ true,0,0,  0,   0,  0,  1,    1,  0,      0,    1,    0,   0,   0,   0,false,false,false},
				--usePositionControl={ true,0,0,0,   0,  0,  1,    1,  0,      0,    1,    0,   0,   0,   0,false,false,false},
				footRefL= "convertFromSwingL",
				footRefR= "convertFromSwingR",
				footRefLH= "convertFromSwingLH",
				footRefRH= "convertFromSwingRH",
			},
			backflip={
				firstFrames={2796,2831,   3040, 3082,    3112,   3157,     3167,3187,3207,3263,3292,3312,3333,3411,3470,3527,3555,3617,3632},
				names={           'lowerUB','throw','flight','landL','handoff','l','B', 'F', 'l2', 'B2', 'r','B3','SL','L','LR','R', },
				swingL={              0    ,   0   ,   -1   ,   -1  ,   0     , 0 ,  0,  1  ,  0  , 0   ,   1,  0,  0 ,  0 ,  0,  1 },
				swingRefL={          -1,      -1 ,     -1   ,   -1,    3188,     -1,  -1, -1,   -1,  -1,   -1, -1, -1 , -1,  -1, -1  },
				swingR={              0    ,   0   ,   -1   ,   -2  ,   -2    ,-1 ,  0,  1  ,  1  , 0   ,   0,  0,  0 ,  1 ,  0,  0 },
				swingRefR={          -1,      -1 ,     -1   ,   -1,    -1,     -1,  3188, -1,   -1,  -1,     -1, -1, -1 , -1,  -1, -1  },
				swingRefLH={          -1,      -1 ,     -1   ,   -1,    -1,     -1,  -1, -1,   -1,  -1,     -1, -1, -1 , -1,  -1, -1  },
				swingRefRH={          -1,      -1 ,     -1   ,   -1,    -1,     -1,  -1, -1,   -1,  -1,     -1, -1, -1 , -1,  -1, -1  },
				swingLH={            -2,      -2,      -1,      0,       0,    -1,  -2,  -2,  -2,   -2,   -2, -2,  -2,  -2, -2, -2,},
				swingRH={            -2,      -2,      -1,      0,       0,    -1,  -2,  -2,  -2,   -2,   -2, -2,  -2,  -2, -2, -2,},
				usePositionControl={true,   true,  false,      false,false, false,  0,   0,   0 ,  true,true,true,true,false,false,false},
				footRefL= "convertFromSwingL",
				footRefR= "convertFromSwingR",
				footRefLH= "convertFromSwingLH",
				footRefRH= "convertFromSwingRH",
			},
			popa2={ -- 1690, 1724
				firstFrames={1324,1333,1413,1453, 1495, 1563,   1595,       1627, 1639, 1647, 1680,1690,1724, 1754, 1888,1928},
				names={              'rl','l', 'lf','ff','jump_up','jump_down','l2', 'lrl', 'l3','r4', 'r5', 'st', 'r', } ,
				swingL={              0,   0 ,  0 ,  0 ,     1 ,       1 ,      0 ,   0,     0 ,  0,    1,    0 ,   0 ,  0 ,},
				swingR={              0,   1 ,  0 ,  0 ,     1 ,       1,       1 ,   0,     1 ,  0,    0,    0 ,   1 ,  0 ,},
				--firstFrames={1324,1333,1413,1453, 1495, 1563,   1595,       1627, 1639, 1647, 1680, 1754, 1888,1932},
				--names={              'rl','l', 'lf','ff','jump_up','jump_down','l2', 'lrl', 'l3',  'st', 'r', } ,
				--swingL={              0,   0 ,  0 ,  0 ,     1 ,       1 ,      0 ,   0,    0 ,   0 ,   1 ,  0 ,},
				--swingR={              0,   1 ,  0 ,  0 ,     1 ,       1,       1 ,   0,    1 ,   0 ,   0 ,  0 ,},
				usePositionControl={    0,   0,   0,   0,      0,        0,       0,    1,    1,    1,   1,   1,  0,    0},
				footRefL= "convertFromSwingL",
				footRefR= "convertFromSwingR",
			},
			handstand={
				--firstFrames={  4392, 4410, 4540, 4565, 4594,4604, 4689, 4715, 4735, 4761, 4777, 4811, 4829, 4862, 4912,   4939,  4957, 4987, 4998, 5037,  5058,  5085, 5103,5132, 5165, 5191, 5333, 5345, 5396, 5543, 5604, 5636},
				firstFrames={  4392, 4496, 4540, 4565, 4594,4604, 4689, 4715, 4735, 4761, 4777, 4811, 4829, 4862, 4912,   4939,  4957, 4987, 4998, 5037,  5058,  5085, 5103,5132, 5165, 5191, 5333, 5345, 5396, 5543, 5604, 5636},
				names={                'R',   'RL', 'L',  'LB','B',  'LH0','LRH0','RH','RLH', 'LH','LRH1','RH2','RLH2','LH2','LRH3','RH3','RLH3','LH3','LRH4','RH4','RLH4','LH4','B2','RH5', 'BL','BLL','L2','LR2',  'R2'},
				swingL={                0 ,    0  ,  0 ,   0  , -1,   -2 ,  -2  ,  -2, -2,    -2,   -2 ,   -2,   -2,   -2,     -2,   -2,    -2,   -2,   -2,    -2,    -2,   -2,   -2,  -2,    -1,   0,   0,   0,    1  },
				swingR={                0 ,    0  ,  -1,   -2 , -2,   -2 ,  -2  ,  -2, -2,    -2,   -2 ,   -2,   -2,   -2,     -2,   -2,    -2,   -2,   -2,    -2,    -2,   -2,   -2,  -2,    -2,  -2,   -1,  0,    0  },
				swingLH={              -2 ,   -2  , -1 ,   -0 ,  0,    0,    0  ,  1,   0,     0 ,   0,     1,   0 ,    0,     0,     1,     0,    0,    0,     1,     0,    0,    0,   1,     0,   0,   -1,   -2,   -2 },
				swingRH={              -2 ,   -2  , -1 ,   -0 ,  0,    1,    0  ,  0,   0,     1,    0,     0,   0,     1,     0,     0,     0,    1,    0,     0,     0,    1,    0,   0,     0,   -1,   -2,  -2,  -2 },
				usePositionControl={ true,   false, false,false,false,false,false,false,false,false,false,false,false,false,false,false,   false, false,false,false,false, false,false,false,false, false,false,false,false},
				footRefL= "convertFromSwingL",
				footRefR= "convertFromSwingR",
				footRefLH= "convertFromSwingLH",
				footRefRH= "convertFromSwingRH",
			},
			straddle={
				firstFrames={ 5647, 5664,5718, 5734, 5774, 5820, 5883, 5919, 5966, 5991, 6051, 6116, 6124},
				names =     {         'L',   'LR', 'R', 'LJ', 'F' , 'B' , 'L2', 'B2',   'R2',  'L3',},
				swingL=     {          0,     0,    1,    0,   1,    0,     0,   0,      1,    0 },
				swingR=     {          1,     0,    0,    1,   1,    0,     1,   0,      0,    0},
				usePositionControl={ true,   false, false,false,false,true,false,false,false,false,},
				footRefL= "convertFromSwingL",
				footRefR= "convertFromSwingR",
			}

		},
		ignoreRotY={{157,392},{2990,3199},{3212,3356},{4463,4680}},
	graphParam={
		walk3={
			seg=   {'stand', 'l', 'lr', 'r', 'RL', 'L', 'LR', 'R'},
			num_key= { 2   ,  3 ,   2 ,  3 ,  2  ,  3 ,   2 ,  3 },
			key_first={0   ,  0 ,  'l',  0 ,  'R',  0 ,  'L' , 0},  
			key_last= {'l' ,  0 ,  'r', 'R:2', 'L',  0 ,  'R' ,0}, 
		},
		roundoff2={
			--         1    2    3   4    5    6    7    8   9      10   11    12    13
			seg={  'stand','l','lr','r', 'rl','l2','lL','L','LR',  'F', 'lr2','r2','rl2','l3','b', 'l4', 'lr3','r4', },
			num_key= { 2   , 3 ,  2 , 3 ,  2  ,  3 , 2 ,  3,  2  ,   2,    2,    3,   2,   3,   2,    3,   2,    3  },
			key_first={0,'stand','l' ,0,  'r',   0 ,'l2',0,  'L',   0 ,  'F',   0,  'r2', 0,  'l3',  0,  'l4',   0},
			key_last= {0 ,  0, 'r',   0,  'l2',  0 ,'L' ,0,  'F',   0 ,  'r2',  0,  'l3', 0,  'l4',  0,  'r4',   0}, 
		},
		straddle={
				seg =     {  'L',   'LR', 'R', 'LJ', 'F' , 'B' , 'L2', 'B2',   'R2',  'L3',},
				num_key=    {   3,      2,   3,    3,   2,    2,    3,    2,     3,      3},
				key_first=  {    0,    'L',  0,   'R',  0,   0,    'B',   'L2',     'B2',   0},
				key_last=   {   0,     'R',  0,   'F',  'B:1',0,    0,     0,     'L3',   0},
			},
		popa2={
			--           1   2     3    4     5         6         7     8      9     10   11    12    13
			seg={      'rl','l', 'lf','ff','jump_up','jump_down','l2', 'lrl', 'l3', 'r4','r5', 'st', 'r2', } ,
			num_key={    2 , 3,   2 ,  2,     2,          2,       3,    2,    3,    2,   2,    2,   3 ,},
			key_first={  0 , 0,  'l',  0,   'ff',         0,       0,   'l2',  0,  'l3',  0,   'r5',   0},
			key_last={  'l', 0,  'ff', 0,   'jump_down',  'l2',    0,   'l3',  0,  'r5',  0,   'r2',  0},
		},
		backflip={
			--        1      2         3       4        5      6
			seg={'lowerUB','throw','flight','landL','handoff','l','B', 'F', 'l2', 'B2', 'r','B3','SL','L','LR','R2', },
			num_key={    3,   3,       2,      2,       2,     3,  2,   3,   3,    3,    2,   2,  2,   3,   2,   3},
			key_first={  0,    0,   'throw',   0,      'landL',0,  'l', 0,   0,   0,   'B2', 0,  'B3', 0,  'L', 0},
			key_last={ 'throw',0,   'landL',   0,       'l',   0,  'F','l2', 'B2',  0,   'B3', 0,   'L', 0,  'R2', 0},
		},
		handstand={
			--seg={       'R',   'RL', 'L',  'LB','B',  'LH0', 'LRH','RH','RLH', 'LH1','LRH2','RH2','RLH2','LH'},
			--num_key={    3 ,     2,   3,     2,  2,     3,     2,    3,   2,     3,     2,    3,     2,    3,},
			--key_first={  0,     'R',  0,    'L', 0,     0,   'LH0',  0,   'RH',  0,    'LH',  0,   'RH2',  0,},
			--key_last={   0,     'L',  0,    'B', 'LH0', 0,    'RH',  0,  'LH1','LH:2', 'RH2', 0,   'LH',   0}, 
			seg={       'R',   'RL', 'L',  'LB','B',  'LH0',  'LRH0','RH','RLH','LH','LRH', },
			num_key={    2 ,     2,   3,     2,  2,     3,      2,    3,   2,    3,   2,   },
			key_first={  0,     'R',  0,    'L', 0,     0,    'LH0',  0,  'RH',  0,  'LH', },
			key_last={   0,     'L',  0,    'B', 'LH0', 0,    'RH',   0,  'LH',  0,  'RH', }, 
		}
	},
	keyframes={
		pendDesiredVel={numKey=1, default=vector3(0,0,0)},
		virtualForceCorrection={numKey=1, default=vector3(0,0,0)},
		-- x= accY_compensation (corresponds to roundoffing height, and compliance when landing), 
		-- y= COMacc_compensation (corresponds to roundoffing distance)
		mocapCompensationCoef={numKey=3, default=vector3(1, 1, 1)},
		virtualForceScale={numKey=3, default=1}, 
		importanceLH={numKey=2, default=1},
		importanceRH={numKey=2, default=1},
		spprtImpL={numKey=2, default=1},
		spprtImpR={numKey=2, default=1},
		spprtImpLH={numKey=2, default=1},
		spprtImpRH={numKey=2, default=1},
		mocapOriModL={numKey=3, numKeyFrom='key', isContinuous=false,default=vector3(0,0,0)},
		mocapOriModR={numKey=3, numKeyFrom='key', isContinuous=false,default=vector3(0,0,0)},
		footLmod={numKey=3, numKeyFrom='key', default=vector3(0,0,0)},
		footRmod={numKey=3, numKeyFrom='key', default=vector3(0,0,0)},
		footLmocapMod={numKey=3, numKeyFrom='key', default=vector3(0,0,0)},
		footRmocapMod={numKey=3, numKeyFrom='key', default=vector3(0,0,0)},
		handLmod={numKey=3, numKeyFrom='key', default=vector3(0,0,0)},
		handRmod={numKey=3, numKeyFrom='key', default=vector3(0,0,0)},
		desiredHeadAcc={numKey=3, numKeyFrom='key', default=vector3(0,0,0)},
		desiredMomentum={numKey=3, numKeyFrom='key', default=vector3(0,0,0)},
		desiredDotMomentum={numKey=3, numKeyFrom='key', isContinuous=false,default=vector3(0,0,0)},
		desiredLeaning={numKey=1, isContinuous=false,default=vector3(0,0,0)},
	},
	segProperties={
		swingFoot={ default="N"},
		swingHand={ default="N"},
	},
	controlParam={}
}
useCases.roundoff_common.registerContactPair=function(model, loader, floor, simulator)
	param=vectorn ()
	param:setValues(0.5,0.5, model.penaltyForceStiffness, model.penaltyForceDamp)
	for i=1,loader:numBone()-1 do

		local bone_i=loader:VRMLbone(i)
		if string.find(bone_i:name(), "foot")
			or string.find(bone_i:name(), "toe")
			or string.find(bone_i:name(), "hand")
			then
				simulator:registerCollisionCheckPair(loader:name(),bone_i.NameId, floor:name(), floor:bone(1):name(), param)
			end
		end
	end
	useCases.roundoff_common.init_globals = function ()
		outputSuperSample=1
		PDservoLatency=5-- for debugging. set 5 for normal use.
		IDservoLatency=0
		COPlatency=10
		PredictionLatency=0
		disableTouchDownFeedback=false
		model.simulationFrameRate=120
		model.penaltyForceStiffness=20000
		model.penaltyForceDamp=2000
		model.clampTorqueID=400
		model.clampTorque =400
		model.k_p_ID=250
		model.k_d_ID=35
		model.k_p_PD=100
		model.k_d_PD=15
		model.k_scale_active_pd.hip={3,1,1}
		model.k_scale_active_pd.knee={1,1,1}
		model.k_scale_active_pd.chest={3,1,1}
		model.k_scale_active_pd.ankle={5,1,1}
		model.k_scale_id.knee={1,1,1}
		model.k_scale_id.ankle={1,1,1}
		model.k_scale_id.toes={1,1,1}
		model.k_scale_id.shoulder={1,1,1}
		--require('RigidBodyWin/subRoutines/fastMode') -- not much faster
	end
	--Feb 15 3:44PM
	local function makeOptParam(numKey,...)
		local gtarget=array:new()
		local prec=0.005
		for iid,id in ipairs({...}) do
			if string.sub(id,1,3)=="MAP" then
				for i=0, numKey-1 do
					gtarget:pushBack({"map,"..i..","..string.sub(id,4)..",x",prec})
					gtarget:pushBack({"map,"..i..","..string.sub(id,4)..",y",prec})
					gtarget:pushBack({"map,"..i..","..string.sub(id,4)..",z",prec})
				end
			else
				local fkey=0
				local lkey=numKey-1
				-- id: key,grpName,segName
				local tokens=string.tokenize(id,',')
				local grpName=tokens[2]

				if useCase then
					fkey, lkey=useCases.getNumKey(useCase, tokens[1], grpName, tokens[3])

					for i=fkey,lkey do
						gtarget:pushBack({"keyframe,"..i..","..id..",x",prec})
						gtarget:pushBack({"keyframe,"..i..","..id..",y",prec})
						gtarget:pushBack({"keyframe,"..i..","..id..",z",prec})
					end
				end
			end
		end
		return gtarget
	end
	local function makeOptParamAxis(numKey,axis,...)
		local gtarget=array:new()
		local prec=0.005
		for iid,id in ipairs({...}) do
			for i=0, numKey-1 do
				if string.sub(id,1,3)=="MAP" then
					gtarget:pushBack({"map,"..i..","..string.sub(id,4)..","..axis,prec})
				else
					gtarget:pushBack({"keyframe,"..i..","..id..","..axis,prec})
				end
			end
		end
		return gtarget
	end
	local function paramLR(segName, grpName)
		return "footLmod,"..grpName..","..segName, "footRmod,"..grpName..","..segName
	end
	local function paramLRh(segName, grpName)
		return "handLmod,"..grpName..","..segName, "handRmod,"..grpName..","..segName
	end
	local function genOptAll(starts, ends,ps,pe,name,axis, nohand)
		local tbl=0.01
		tbl={}

		ps=ps or starts
		pe=pe or ends
		name=name or 'walk3'
		path={}
		-- path.roundoff={"Stand", "L_handsup", "R_handsdown", "L_LH", "LH_BH", "BH_BF", "BH_R", "RL", "LR", "RL2", "LR"}
		-- path.walk={       "stand", "SR","rl","lr","RL","LR"}
		--path.walk2={  "stand", "r","rl","l","LR","R", "RL", "L", "LR"}
		path.walk3={  "stand", "l","lr","r","RL","R", "LR", "R", "RL"}
		--path.popa=          {"stand","rl","lr","COMdown","jump_up","jump_down", "jump_after1", "jump_after2","RL"}

		for i=ps,math.ceil(pe) do
			if useCases.roundoff_common.segmentations[name].swingFoot[i]~="B" then
				array.pushBack(tbl, paramLR(path[name][i],name))
			end
			if nohand==nil and useCases.roundoff_common.segmentations[name].swingHand and useCases.roundoff_common.segmentations[name].swingHand[i]~="B" then
				array.pushBack(tbl, paramLRh(path[name][i],name))
			end
		end

		if math.ceil(ends)==ends then
			local nkey=3
			--if name=="walk2" or name=="walk3" then
			--	nkey=2
			--end
			if axis then
				return { startSeg=starts,endSeg=ends, setStageFunc='setStage_param', param=makeOptParamAxis(nkey, axis,unpack(tbl)), baseStage=1, }
			else
				return { startSeg=starts,endSeg=ends, setStageFunc='setStage_param', param=makeOptParam(nkey, unpack(tbl)), baseStage=1, }
			end
		else
			if axis then
				return { startSeg=starts,endSeg=math.ceil(ends), endFrac=math.mod(ends,1), setStageFunc='setStage_param', param=makeOptParamAxis(3, axis,unpack(tbl)), baseStage=1, }
			else
				return { startSeg=starts,endSeg=math.ceil(ends), endFrac=math.mod(ends,1), setStageFunc='setStage_param', param=makeOptParam(3, unpack(tbl)), baseStage=1, }
			end
		end
	end


	useCases.roundoff_common.predictLandingPos=function(remainingFlightTime, comVel, com , roundoff_after_time)
		-- landing prediction model
		local COMpos=remainingFlightTime*comVel+com
		COMpos.y=com.y
		-- assuming constant acceleration
		-- x=x0+vot+0.5*at^2
		-- vo+at=0 --> a=-vo/t
		-- --> x=x0+0.5vot
		local vo=comVel:copy()
		vo.y=0
		local COMpos2=COMpos+0.5*vo*roundoff_after_time
		COMpos2.y=0
		return COMpos2, COMpos
	end

	useCases.roundoff_common.isFlightPhase=function(iframe, graph)
		--local f=graph.roundoff.roundoff_up.first
		--local l=graph.roundoff.roundoff_down.last
		--if f<= iframe and iframe<=l then
		--return true, (iframe-f)/120
		--end
		return false
	end

	if false then
		useCases.roundoff_common.hookAfterGraphConstruction=function(locosyn)
			local graph=locosyn.graph
			local f=graph.popa.jump_up.first
			local l=graph.popa.jump_after1.first
			local locomot=graph.popa.jump_up.locomot
			local zc=locomot.ZMPcalculator
			local ff=useCase.segmentations.popa.firstFrames
			for i=f, l do 
				local com=zc.com(i)
				local comVel=zc.comvel(i)
				local targetHeight=zc.com(l).y + model.initialHeight

				local elapsedFlightTime=(i-f)/model.frame_rate
				-- vo*t-0.5*9.8*t^2+y-y'=0
				local a=-0.5*9.8
				local b=comVel.y
				local c=com.y-targetHeight

				local remainingFlightTime=(-b-math.sqrt(b*b-4*a*c))/(2*a)

				--print((i-f)/model.frame_rate,":", (l-i)/model.frame_rate, " should be similar to ", remainingFlightTime)
				if remainingFlightTime==remainingFlightTime then --NaN check
					local jump_after_time=ff[table.getn(ff)-1]-ff[table.getn(ff)-2]
					jump_after_time=jump_after_time/model.frame_rate
					-- 1.4 is the optimized scaling factor.
					local COMpos2=useCase.predictLandingPos(remainingFlightTime, comVel, com, jump_after_time*0.5)

					if false then
						dbg.draw('Sphere', COMpos2*100, "predictedCOM2")
						locosyn.skin2:setPoseDOF(locomot.mot(i))
						RE.renderOneFrame(true)
					end
				end
			end
		end

		useCases.roundoff_common.calcDesiredFootPosFlightPhase=function(locosyn)
			local simulator=locosyn.simulator
			local graph=locosyn.graph
			local com=simulator:calculateCOM(0)
			local comVel=simulator:calculateCOMvel(0)
			local targetHeight
			local jump_after_time
			local remainingFrames
			do -- find time scale
				if locosyn.outputGlobal then
					local refTime=locosyn.outputGlobal:getRefTime()
					local currSegIndex=math.floor(refTime)
					local currSeg=locosyn.outputLocal.segInfo(currSegIndex).seg
					local up=graph.popa.jump_up
					local down=graph.popa.jump_down
					local after=graph.popa.jump_after1
					local segT=refTime-currSegIndex
					local segmentationCorrection=useCase.segmentationCorrection  -- hand-tuned for precise landing: positive means faster playback of desired motion 
					if currSeg==up then
						remainingFrames=down:length()*(1-segT)+up:length() + segmentationCorrection
					elseif currSeg==down then
						remainingFrames=up:length()*(1-segT) + segmentationCorrection
					elseif currSeg==after then
						remainingFrames=0
					else
						return 'N'
					end

					--RE.output2('currSegFlight', currSeg.name, refTime, remainingFrames)
				else -- ipcno
					remainingFrames=0
				end
			end
			do -- find targetHeight
				local tgtSeg=graph.popa.jump_after1
				local pelvis=tgtSeg:mot():rootTransformation(0)
				pelvis.rotation:assign(pelvis.rotation:rotationY())
				local tgtcom=pelvis:toGlobalPos(tgtSeg.lcom(0))
				targetHeight=tgtcom.y+ model.initialHeight
				jump_after_time=(tgtSeg.last-tgtSeg.first)/model.frame_rate
			end

			local elapsedFlightTime=locosyn.pdservo.state.flightPhaseElapsed/model.simulationFrameRate
			-- vo*t-0.5*9.8*t^2+y-y'=0
			local a=-0.5*9.8
			local b=comVel.y
			local c=com.y-targetHeight

			local remainingFlightTime=(-b-math.sqrt(b*b-4*a*c))/(2*a)

			if remainingFlightTime~=remainingFlightTime then --NaN check
				remainingFlightTime=0
			end
			remainingFlightTime=math.max(remainingFlightTime,0)


			if remainingFlightTime > 0.05 and remainingFrames/120 > 0.05 then
				locosyn.timeScaleOverride=remainingFlightTime/(remainingFrames/120)
				if locosyn.timeScaleOverride~=locosyn.timeScaleOverride then locosyn.timeScaleOverride=1 end
				locosyn.timeScaleOverride=math.clamp(locosyn.timeScaleOverride, 0.1, 10)
			end
			RE.output2("flight", "elapsed:remainingT,remainingF,ts", elapsedFlightTime, remainingFlightTime*120, remainingFrames, locosyn.timeScaleOverride)
			local COMpos2, COMpos=useCase.predictLandingPos(remainingFlightTime, comVel, com, jump_after_time*0.5)
			dbg.namedDraw('Sphere', COMpos2*100, "predictedCOM2", "green")
			return 'B', (COMpos2-COMpos), vector3(0,0,0), sop.clampMap(elapsedFlightTime/(elapsedFlightTime+remainingFlightTime), 0,1, 0,1)
		end
	end

	-- useCases.roundoff_common.pendControlParam_roundoff={
	-- 	--roundoff
	-- 	--['keyframe,0,pendDesiredVel,roundoff,Stand,x']= -0.017924098241482, ['keyframe,0,pendDesiredVel,roundoff,L_LH,x']= 0.73017775881997, ['keyframe,0,pendDesiredVel,roundoff,LH_BH,x']= 0.39409588950881, ['keyframe,0,pendDesiredVel,roundoff,R_handsdown,z']= 2.2225884083999, ['keyframe,0,pendDesiredVel,ignore,1,z']= -0.003014948137975, ['keyframe,0,pendDesiredVel,roundoff,RL2,x']= -0.066348879557405, ['keyframe,0,pendDesiredVel,roundoff,L_LH,z']= 0.7925952586825, ['keyframe,0,pendDesiredVel,roundoff,LH_BH,z']= -0.68463354273353, ['keyframe,0,pendDesiredVel,roundoff,BH_BF,z']= -0.68480511669387, ['keyframe,0,pendDesiredVel,roundoff,BH_R,x']= -0.014506224018965, ['keyframe,0,pendDesiredVel,ignore,0,x']= -0.010941720448091, ['keyframe,0,pendDesiredVel,roundoff,LR,z']= 1.135818301836, ['keyframe,0,pendDesiredVel,ignore,0,z']= 0.093155724049966, ['keyframe,0,pendDesiredVel,roundoff,BH_R,z']= -0.70668093366039, ['keyframe,0,pendDesiredVel,roundoff,Stand,z']= 0.1828349969161, ['keyframe,0,pendDesiredVel,ignore,1,x']= -0.00025004488957223, ['keyframe,0,pendDesiredVel,roundoff,RL,z']= -0.29171373580893, ['keyframe,0,pendDesiredVel,roundoff,R_handsdown,x']= 0.36031957168489, ['keyframe,0,pendDesiredVel,roundoff,BH_BF,x']= 0.38973222946986, ['keyframe,0,pendDesiredVel,roundoff,RL,x']= 0.02329932409275, ['keyframe,0,pendDesiredVel,roundoff,L_handsup,z']= 1.7431746591013, ['keyframe,0,pendDesiredVel,roundoff,LR,x']= -0.39136624996401, ['keyframe,0,pendDesiredVel,roundoff,L_handsup,x']= -0.077810241502735, ['keyframe,0,pendDesiredVel,roundoff,RL2,z']= 0.37625089395602,
	-- 	-- modified rotRefTime
	-- 	['keyframe,0,pendDesiredVel,roundoff,BH_R,z']= 0.24381078843414, ['keyframe,0,pendDesiredVel,roundoff,L_LH,x']= 0.42455841411344, ['keyframe,0,pendDesiredVel,roundoff,R_handsdown,z']= 2.1967779496301, ['keyframe,0,pendDesiredVel,roundoff,R_handsdown,x']= 0.05138880007771, ['keyframe,0,pendDesiredVel,ignore,1,z']= -0.0067264496380499, ['keyframe,0,pendDesiredVel,ignore,0,z']= 0.1980755167618, ['keyframe,0,pendDesiredVel,roundoff,BH_BF,z']= 1.117541521135, ['keyframe,0,pendDesiredVel,roundoff,RL2,x']= 0.093230248501637, ['keyframe,0,pendDesiredVel,roundoff,L_LH,z']= 1.1800996036524, ['keyframe,0,pendDesiredVel,roundoff,Stand,x']= 0.034614851402045, ['keyframe,0,pendDesiredVel,roundoff,L_handsup,x']= -0.00065935982741941, ['keyframe,0,pendDesiredVel,roundoff,BH_R,x']= -0.19215759597255, ['keyframe,0,pendDesiredVel,ignore,0,x']= 0.019249307087473, ['keyframe,0,pendDesiredVel,roundoff,Stand,z']= 0.32435272101303, ['keyframe,0,pendDesiredVel,roundoff,LH_BH,x']= 0.089337697284476, ['keyframe,0,pendDesiredVel,roundoff,LR,x']= -0.46470217075211, ['keyframe,0,pendDesiredVel,roundoff,BH_BF,x']= -0.09953559275175, ['keyframe,0,pendDesiredVel,roundoff,RL,z']= -0.19785885092066, ['keyframe,0,pendDesiredVel,roundoff,LR,z']= 1.053355642853, ['keyframe,0,pendDesiredVel,ignore,1,x']= -0.0020238566617935, ['keyframe,0,pendDesiredVel,roundoff,RL,x']= -0.013758835808963, ['keyframe,0,pendDesiredVel,roundoff,L_handsup,z']= 1.667183223296, ['keyframe,0,pendDesiredVel,roundoff,LH_BH,z']= 0.20751285371746, ['keyframe,0,pendDesiredVel,roundoff,RL2,z']= 0.43689163525396, 
	-- }
	-- useCases.roundoff_common.pendControlParam_walk={
	-- 	--walk
	-- 	--['keyframe,0,pendDesiredVel,walk,lr1,x']= 0.0049563996305455, ['keyframe,0,pendDesiredVel,walk,LR,z']= 1.0076499375993, ['keyframe,0,pendDesiredVel,walk,LR,x']= -0.062679432675146, ['keyframe,0,pendDesiredVel,walk,SR,z']= 0.90250535195942, ['keyframe,0,pendDesiredVel,walk,rl,z']= 1.0086377793126, ['keyframe,0,pendDesiredVel,walk,lr1,z']= 1.0583230417596, ['keyframe,0,pendDesiredVel,walk,rl,x']= 0.02287830144644, ['keyframe,0,pendDesiredVel,ignore,1,x']= -0.17162406480978, ['keyframe,0,pendDesiredVel,walk,RL1,z']= 1.0453279720228, ['keyframe,0,pendDesiredVel,ignore,1,z']= 0.89222817764431, ['keyframe,0,pendDesiredVel,walk,SR,x']= 0.19711904156404, ['keyframe,0,pendDesiredVel,walk,RL2,z']= 0.94809118144618, ['keyframe,0,pendDesiredVel,walk,RL2,x']= -0.17930609072405, ['keyframe,0,pendDesiredVel,ignore,0,z']= 0.86139639303677, ['keyframe,0,pendDesiredVel,ignore,0,x']= 0.32555186764679, ['keyframe,0,pendDesiredVel,walk,RL1,x']= 0.023835706539321, 
	-- 	['keyframe,0,pendDesiredVel,walk,RL1,x']= 0.035175907004405, ['keyframe,0,pendDesiredVel,walk,RL1,z']= 1.0255293369337, ['keyframe,0,pendDesiredVel,ignore,0,x']= 0.36986719016437, ['keyframe,0,pendDesiredVel,walk,lr1,x']= 0.033633174765315, ['keyframe,0,pendDesiredVel,walk,rl,z']= 1.0403641253872, ['keyframe,0,pendDesiredVel,ignore,0,z']= 0.903590282845, ['keyframe,0,pendDesiredVel,walk,rl,x']= 0.081691436019534, ['keyframe,0,pendDesiredVel,ignore,1,x']= -0.17221036257797, ['keyframe,0,pendDesiredVel,walk,RL2,x']= -0.19078967144578, ['keyframe,0,pendDesiredVel,ignore,1,z']= 0.89106326818419, ['keyframe,0,pendDesiredVel,walk,RL2,z']= 0.93992002777303, ['keyframe,0,pendDesiredVel,walk,SR,x']= 0.25260196991805, ['keyframe,0,pendDesiredVel,walk,lr1,z']= 1.0594194774254, ['keyframe,0,pendDesiredVel,walk,LR,x']= -0.061331713832689, ['keyframe,0,pendDesiredVel,walk,SR,z']= 0.947983850464, ['keyframe,0,pendDesiredVel,walk,LR,z']= 0.99199827634891, 
	-- }
	useCases.roundoff_common.pendControlParam_walk2={
		--	['keyframe,0,pendDesiredVel,walk2,stand,z']=0,['keyframe,0,pendDesiredVel,walk2,stand,x']=0,['keyframe,0,pendDesiredVel,walk2,r,z']=0,['keyframe,0,pendDesiredVel,walk2,r,x']=0,['keyframe,0,pendDesiredVel,walk2,rl,z']=0,['keyframe,0,pendDesiredVel,walk2,rl,x']=0,['keyframe,0,pendDesiredVel,walk2,l1,z']=0,['keyframe,0,pendDesiredVel,walk2,l1,x']=0,['keyframe,0,pendDesiredVel,walk2,lr,z']=0,['keyframe,0,pendDesiredVel,walk2,lr,x']=0,['keyframe,0,pendDesiredVel,walk2,r2,z']=0,['keyframe,0,pendDesiredVel,walk2,r2,x']=0,['keyframe,0,pendDesiredVel,walk2,rl2,z']=0,['keyframe,0,pendDesiredVel,walk2,rl2,x']=0,['keyframe,0,pendDesiredVel,walk2,l2,z']=0,['keyframe,0,pendDesiredVel,walk2,l2,x']=0,['keyframe,0,pendDesiredVel,walk2,lr2,z']=0,['keyframe,0,pendDesiredVel,walk2,lr2,x']=0,['keyframe,0,pendDesiredVel,walk2,R,z']=0,['keyframe,0,pendDesiredVel,walk2,R,x']=0,['keyframe,0,pendDesiredVel,walk2,RL,z']=0,['keyframe,0,pendDesiredVel,walk2,RL,x']=0,['keyframe,0,pendDesiredVel,walk2,L,z']=0,['keyframe,0,pendDesiredVel,walk2,L,x']=0,['keyframe,0,pendDesiredVel,walk2,LR,z']=0,['keyframe,0,pendDesiredVel,walk2,LR,x']=0,['keyframe,0,pendDesiredVel,walk2,R2,z']=0,['keyframe,0,pendDesiredVel,walk2,R2,x']=0,['keyframe,0,pendDesiredVel,walk2,RL2,z']=0,['keyframe,0,pendDesiredVel,walk2,RL2,x']=0,['keyframe,0,pendDesiredVel,ignore,0,z']=0,['keyframe,0,pendDesiredVel,ignore,0,x']=0,['keyframe,0,pendDesiredVel,ignore,1,z']=0,['keyframe,0,pendDesiredVel,ignore,1,x']=0,
		--['keyframe,0,pendDesiredVel,walk2,l1,x']= 0.034887004851175, ['keyframe,0,pendDesiredVel,walk2,RL,z']= 0.57707464266085, ['keyframe,0,pendDesiredVel,walk2,L,z']= 1.2957187785397, ['keyframe,0,pendDesiredVel,walk2,LR,x']= -0.062160906664201, ['keyframe,0,pendDesiredVel,walk2,rl2,z']= 0.53212486293336, ['keyframe,0,pendDesiredVel,ignore,0,z']= 0.54825630760322, ['keyframe,0,pendDesiredVel,walk2,RL2,x']= 0.00060241212307287, ['keyframe,0,pendDesiredVel,walk2,l1,z']= 1.1457846697549, ['keyframe,0,pendDesiredVel,walk2,r2,z']= 1.273645951259, ['keyframe,0,pendDesiredVel,walk2,LR,z']= 0.52168215656717, ['keyframe,0,pendDesiredVel,walk2,stand,x']= 0.19970381201559, ['keyframe,0,pendDesiredVel,ignore,1,x']= -0.0040075791900539, ['keyframe,0,pendDesiredVel,walk2,r2,x']= -0.082492080837262, ['keyframe,0,pendDesiredVel,walk2,r,x']= 0.4375057798222, ['keyframe,0,pendDesiredVel,walk2,stand,z']= 0.48941329163192, ['keyframe,0,pendDesiredVel,walk2,lr2,z']= 0.56634605866266, ['keyframe,0,pendDesiredVel,walk2,r,z']= 1.3096443373652, ['keyframe,0,pendDesiredVel,ignore,0,x']= 0.22261654520476, ['keyframe,0,pendDesiredVel,walk2,rl,z']= 0.46977300207944, ['keyframe,0,pendDesiredVel,walk2,rl,x']= 0.062020854044663, ['keyframe,0,pendDesiredVel,walk2,L,x']= -0.2394647798651, ['keyframe,0,pendDesiredVel,ignore,1,z']= -0.017728626553901, ['keyframe,0,pendDesiredVel,walk2,lr,x']= -0.04969810442052, ['keyframe,0,pendDesiredVel,walk2,RL2,z']= 0.1287717020328, ['keyframe,0,pendDesiredVel,walk2,R2,x']= -0.073561895573054, ['keyframe,0,pendDesiredVel,walk2,R2,z']= 0.88408561132959, ['keyframe,0,pendDesiredVel,walk2,RL,x']= -0.099066518463607, ['keyframe,0,pendDesiredVel,walk2,R,x']= -0.170542135283, ['keyframe,0,pendDesiredVel,walk2,R,z']= 1.15624093863, ['keyframe,0,pendDesiredVel,walk2,lr2,x']= 0.0034419116120002, ['keyframe,0,pendDesiredVel,walk2,l2,x']= 0.23003217204426, ['keyframe,0,pendDesiredVel,walk2,rl2,x']= 0.069038215168691, ['keyframe,0,pendDesiredVel,walk2,l2,z']= 1.1886221406373, ['keyframe,0,pendDesiredVel,walk2,lr,z']= 0.44682564709507, 
		-- using smaller number of paramters
		--['keyframe,0,pendDesiredVel,walk2,l1,x']= 0.00048289778084423, ['keyframe,0,pendDesiredVel,walk2,L,z']= 1.0203091969606, ['keyframe,0,pendDesiredVel,ignore,0,z']= 0.62722413388847, ['keyframe,0,pendDesiredVel,walk2,l1,z']= 0.87867495331462, ['keyframe,0,pendDesiredVel,walk2,r2,z']= 1.0689317695409, ['keyframe,0,pendDesiredVel,walk2,stand,x']= 0.20307015238055, ['keyframe,0,pendDesiredVel,ignore,1,x']= -0.0059686258789939, ['keyframe,0,pendDesiredVel,walk2,r2,x']= -0.051395185321055, ['keyframe,0,pendDesiredVel,walk2,r,x']= 0.35166834644458, ['keyframe,0,pendDesiredVel,walk2,stand,z']= 0.53933846102338, ['keyframe,0,pendDesiredVel,walk2,r,z']= 1.1717835233237, ['keyframe,0,pendDesiredVel,ignore,0,x']= 0.23155128481633, ['keyframe,0,pendDesiredVel,walk2,L,x']= -0.18543142544805, ['keyframe,0,pendDesiredVel,walk2,l2,z']= 1.0306897873614, ['keyframe,0,pendDesiredVel,walk2,l2,x']= 0.16808738739457, ['keyframe,0,pendDesiredVel,walk2,R2,x']= 0.011353020606681, ['keyframe,0,pendDesiredVel,walk2,R,z']= 0.94623055876425, ['keyframe,0,pendDesiredVel,walk2,R,x']= -0.18693286352875, ['keyframe,0,pendDesiredVel,walk2,R2,z']= 0.75971603376472, ['keyframe,0,pendDesiredVel,ignore,1,z']= -0.027175562880686, 
		-- ['keyframe,0,pendDesiredVel,ignore,0,x']= 0.23152895945323, ['keyframe,0,pendDesiredVel,ignore,0,z']= 0.62720971242695, ['keyframe,0,pendDesiredVel,walk2,stand,x']= 0.20304601561734, ['keyframe,0,pendDesiredVel,walk2,stand,z']= 0.53931524550595, ['keyframe,0,pendDesiredVel,walk2,r,x']= 0.35118082954977, ['keyframe,0,pendDesiredVel,walk2,r,z']= 1.1712353061516, ['keyframe,0,pendDesiredVel,walk2,l1,x']= 0.00014487076240165, ['keyframe,0,pendDesiredVel,walk2,l1,z']= 0.87834892289652, ['keyframe,0,pendDesiredVel,walk2,r2,x']= -0.051637309537735, ['keyframe,0,pendDesiredVel,walk2,r2,z']= 1.0687264261072, ['keyframe,0,pendDesiredVel,walk2,l2,x']= 0.16794500565306, ['keyframe,0,pendDesiredVel,walk2,l2,z']= 1.0305755976637, ['keyframe,0,pendDesiredVel,walk2,R,x']= -0.18700229639562, ['keyframe,0,pendDesiredVel,walk2,R,z']= 0.94616096766916, ['keyframe,0,pendDesiredVel,walk2,L,x']= -0.18545331216482, ['keyframe,0,pendDesiredVel,walk2,L,z']= 1.0202498434736, ['keyframe,0,pendDesiredVel,walk2,R2,x']= 0.01136917448452, ['keyframe,0,pendDesiredVel,walk2,R2,z']= 0.75970135617564, ['keyframe,0,pendDesiredVel,ignore,1,x']= -0.0059697869669312, ['keyframe,0,pendDesiredVel,ignore,1,z']= -0.027180293765198, 
		['keyframe,0,pendDesiredVel,ignore,0,x']= 0.27944185173277, ['keyframe,0,pendDesiredVel,ignore,0,z']= 0.70128998538803, ['keyframe,0,pendDesiredVel,walk2,stand,x']= 0.25075624499394, ['keyframe,0,pendDesiredVel,walk2,stand,z']= 0.60400799232964, ['keyframe,0,pendDesiredVel,walk2,r,x']= 0.29559839991182, ['keyframe,0,pendDesiredVel,walk2,r,z']= 1.1053785893606, ['keyframe,0,pendDesiredVel,walk2,l1,x']= 0.015947571611017, ['keyframe,0,pendDesiredVel,walk2,l1,z']= 0.89274104920284, ['keyframe,0,pendDesiredVel,walk2,r2,x']= -0.033446205837372, ['keyframe,0,pendDesiredVel,walk2,r2,z']= 1.1145264436703, ['keyframe,0,pendDesiredVel,walk2,l2,x']= 0.13109489807506, ['keyframe,0,pendDesiredVel,walk2,l2,z']= 1.0361544834998, ['keyframe,0,pendDesiredVel,walk2,R,x']= -0.15731685086947, ['keyframe,0,pendDesiredVel,walk2,R,z']= 0.9390807422194, ['keyframe,0,pendDesiredVel,walk2,L,x']= -0.1677654125001, ['keyframe,0,pendDesiredVel,walk2,L,z']= 1.0015064549765, ['keyframe,0,pendDesiredVel,walk2,R2,x']= -0.038155358381657, ['keyframe,0,pendDesiredVel,walk2,R2,z']= 0.75517615696964, ['keyframe,0,pendDesiredVel,ignore,1,x']= -0.0066683560651763, ['keyframe,0,pendDesiredVel,ignore,1,z']= -0.031125074353287, 
	}
	useCases.roundoff_common.pendControlParam_walk3= {
['keyframe,0,pendDesiredVel,walk3,stand,z']=0,['keyframe,0,pendDesiredVel,walk3,stand,x']=0,['keyframe,0,pendDesiredVel,walk3,l,z']=0,['keyframe,0,pendDesiredVel,walk3,l,x']=0,['keyframe,0,pendDesiredVel,walk3,lr,z']=0,['keyframe,0,pendDesiredVel,walk3,lr,x']=0,['keyframe,0,pendDesiredVel,walk3,r1,z']=0,['keyframe,0,pendDesiredVel,walk3,r1,x']=0,['keyframe,0,pendDesiredVel,walk3,rl,z']=0,['keyframe,0,pendDesiredVel,walk3,rl,x']=0,['keyframe,0,pendDesiredVel,walk3,l2,z']=0,['keyframe,0,pendDesiredVel,walk3,l2,x']=0,['keyframe,0,pendDesiredVel,walk3,lr2,z']=0,['keyframe,0,pendDesiredVel,walk3,lr2,x']=0,['keyframe,0,pendDesiredVel,walk3,r2,z']=0,['keyframe,0,pendDesiredVel,walk3,r2,x']=0,['keyframe,0,pendDesiredVel,walk3,rl2,z']=0,['keyframe,0,pendDesiredVel,walk3,rl2,x']=0,['keyframe,0,pendDesiredVel,walk3,L1,z']=0,['keyframe,0,pendDesiredVel,walk3,L1,x']=0,['keyframe,0,pendDesiredVel,walk3,LR,z']=0,['keyframe,0,pendDesiredVel,walk3,LR,x']=0,['keyframe,0,pendDesiredVel,walk3,R,z']=0,['keyframe,0,pendDesiredVel,walk3,R,x']=0,['keyframe,0,pendDesiredVel,walk3,RL,z']=0,['keyframe,0,pendDesiredVel,walk3,RL,x']=0,['keyframe,0,pendDesiredVel,walk3,L2,z']=0,['keyframe,0,pendDesiredVel,walk3,L2,x']=0,['keyframe,0,pendDesiredVel,walk3,LR2,z']=0,['keyframe,0,pendDesiredVel,walk3,LR2,x']=0,['keyframe,0,pendDesiredVel,ignore,0,z']=0,['keyframe,0,pendDesiredVel,ignore,0,x']=0,['keyframe,0,pendDesiredVel,ignore,1,z']=0,['keyframe,0,pendDesiredVel,ignore,1,x']=0,
['keyframe,0,pendDesiredVel,ignore,0,x']= -0.027624787294181, ['keyframe,0,pendDesiredVel,ignore,0,z']= 0.60171579151647, ['keyframe,0,pendDesiredVel,walk3,stand,x']= -0.15878169836645, ['keyframe,0,pendDesiredVel,walk3,stand,z']= 1.4065900871264, ['keyframe,0,pendDesiredVel,walk3,l,x']= -0.18877258881465, ['keyframe,0,pendDesiredVel,walk3,l,z']= 1.2360229246807, ['keyframe,0,pendDesiredVel,walk3,r1,x']= 0.096844658809505, ['keyframe,0,pendDesiredVel,walk3,r1,z']= 1.0138552286928, ['keyframe,0,pendDesiredVel,walk3,l2,x']= -0.079539322259611, ['keyframe,0,pendDesiredVel,walk3,l2,z']= 1.1330686127316, ['keyframe,0,pendDesiredVel,walk3,r2,x']= -0.017711050573028, ['keyframe,0,pendDesiredVel,walk3,r2,z']= 1.2086320229889, ['keyframe,0,pendDesiredVel,walk3,L1,x']= -0.082462919477909, ['keyframe,0,pendDesiredVel,walk3,L1,z']= 1.2729114718038, ['keyframe,0,pendDesiredVel,walk3,R,x']= 0.020749453777447, ['keyframe,0,pendDesiredVel,walk3,R,z']= 0.82677071969039, ['keyframe,0,pendDesiredVel,walk3,L2,x']= -0.014439022187404, ['keyframe,0,pendDesiredVel,walk3,L2,z']= 0.2148777378251, ['keyframe,0,pendDesiredVel,ignore,1,x']= -0.00077758462218707, ['keyframe,0,pendDesiredVel,ignore,1,z']= -0.011583421106249, 
['keyframe,0,pendDesiredVel,ignore,0,x']= -0.020574296868451, ['keyframe,0,pendDesiredVel,ignore,0,z']= 0.63117383391384, ['keyframe,0,pendDesiredVel,walk3,stand,x']= -0.16306219586232, ['keyframe,0,pendDesiredVel,walk3,stand,z']= 1.4226716861766, ['keyframe,0,pendDesiredVel,walk3,l,x']= -0.18957126525173, ['keyframe,0,pendDesiredVel,walk3,l,z']= 1.2339516146208, ['keyframe,0,pendDesiredVel,walk3,r1,x']= 0.10472586516917, ['keyframe,0,pendDesiredVel,walk3,r1,z']= 1.0203362462437, ['keyframe,0,pendDesiredVel,walk3,l2,x']= -0.0808383443044, ['keyframe,0,pendDesiredVel,walk3,l2,z']= 1.128204287493, ['keyframe,0,pendDesiredVel,walk3,r2,x']= -0.022220527367067, ['keyframe,0,pendDesiredVel,walk3,r2,z']= 1.1744456370029, ['keyframe,0,pendDesiredVel,walk3,L1,x']= -0.075429333435939, ['keyframe,0,pendDesiredVel,walk3,L1,z']= 1.2432475498409, ['keyframe,0,pendDesiredVel,walk3,R,x']= 0.023575247890682, ['keyframe,0,pendDesiredVel,walk3,R,z']= 0.832007573933, ['keyframe,0,pendDesiredVel,walk3,L2,x']= -0.012792970994629, ['keyframe,0,pendDesiredVel,walk3,L2,z']= 0.2229312725696, ['keyframe,0,pendDesiredVel,ignore,1,x']= -0.00094523384617939, ['keyframe,0,pendDesiredVel,ignore,1,z']= -0.012542690686434, 
['keyframe,0,pendDesiredVel,ignore,0,x']= -0.02058342670937, ['keyframe,0,pendDesiredVel,ignore,0,z']= 0.63137613744737, ['keyframe,0,pendDesiredVel,walk3,stand,x']= -0.16339074599584, ['keyframe,0,pendDesiredVel,walk3,stand,z']= 1.4225415108882, ['keyframe,0,pendDesiredVel,walk3,l,x']= -0.18990404081925, ['keyframe,0,pendDesiredVel,walk3,l,z']= 1.2338161071479, ['keyframe,0,pendDesiredVel,walk3,r1,x']= 0.10456274429402, ['keyframe,0,pendDesiredVel,walk3,r1,z']= 1.0206207239483, ['keyframe,0,pendDesiredVel,walk3,l2,x']= -0.081104794518271, ['keyframe,0,pendDesiredVel,walk3,l2,z']= 1.1285592044853, ['keyframe,0,pendDesiredVel,walk3,r2,x']= -0.022439610097411, ['keyframe,0,pendDesiredVel,walk3,r2,z']= 1.174615378689, ['keyframe,0,pendDesiredVel,walk3,L1,x']= -0.075520456412003, ['keyframe,0,pendDesiredVel,walk3,L1,z']= 1.2434144462279, ['keyframe,0,pendDesiredVel,walk3,R,x']= 0.023547354508402, ['keyframe,0,pendDesiredVel,walk3,R,z']= 0.83230302207945, ['keyframe,0,pendDesiredVel,walk3,L2,x']= -0.012792556677954, ['keyframe,0,pendDesiredVel,walk3,L2,z']= 0.22306263266905, ['keyframe,0,pendDesiredVel,ignore,1,x']= -0.00094656387667612, ['keyframe,0,pendDesiredVel,ignore,1,z']= -0.012555010642574, 
['keyframe,0,pendDesiredVel,ignore,0,x']= -0.14109262846511, ['keyframe,0,pendDesiredVel,ignore,0,z']= 1.2384178393512, ['keyframe,0,pendDesiredVel,walk3,stand,x']= -0.13699223335441, ['keyframe,0,pendDesiredVel,walk3,stand,z']= 1.1837627163352, ['keyframe,0,pendDesiredVel,walk3,l,x']= -0.089300962489037, ['keyframe,0,pendDesiredVel,walk3,l,z']= 1.1822606226057, ['keyframe,0,pendDesiredVel,walk3,r1,x']= -0.055835955551702, ['keyframe,0,pendDesiredVel,walk3,r1,z']= 1.1825087857771, ['keyframe,0,pendDesiredVel,walk3,l2,x']= -0.025507725011573, ['keyframe,0,pendDesiredVel,walk3,l2,z']= 1.1870340597833, ['keyframe,0,pendDesiredVel,walk3,r2,x']= -0.031420972840028, ['keyframe,0,pendDesiredVel,walk3,r2,z']= 1.1334063166196, ['keyframe,0,pendDesiredVel,walk3,L1,x']= -0.030035465179584, ['keyframe,0,pendDesiredVel,walk3,L1,z']= 1.0682217075222, ['keyframe,0,pendDesiredVel,walk3,R,x']= -0.053704316078666, ['keyframe,0,pendDesiredVel,walk3,R,z']= 0.96760484174071, ['keyframe,0,pendDesiredVel,walk3,L2,x']= -0.06089411413738, ['keyframe,0,pendDesiredVel,walk3,L2,z']= 0.85614906303648, ['keyframe,0,pendDesiredVel,ignore,1,x']= -0.071412587248757, ['keyframe,0,pendDesiredVel,ignore,1,z']= 0.81115697184156, --
['keyframe,0,pendDesiredVel,ignore,0,x']= -0.13916023729863, ['keyframe,0,pendDesiredVel,ignore,0,z']= 1.224070867184, ['keyframe,0,pendDesiredVel,walk3,stand,x']= -0.13463249572574, ['keyframe,0,pendDesiredVel,walk3,stand,z']= 1.2094690922624, ['keyframe,0,pendDesiredVel,walk3,l,x']= -0.099573378353027, ['keyframe,0,pendDesiredVel,walk3,l,z']= 1.1874484603836, ['keyframe,0,pendDesiredVel,walk3,r1,x']= -0.043552098417746, ['keyframe,0,pendDesiredVel,walk3,r1,z']= 1.1787259971897, ['keyframe,0,pendDesiredVel,walk3,l2,x']= -0.030703225709563, ['keyframe,0,pendDesiredVel,walk3,l2,z']= 1.1663250901457, ['keyframe,0,pendDesiredVel,walk3,r2,x']= -0.026433567513806, ['keyframe,0,pendDesiredVel,walk3,r2,z']= 1.1330227488649, ['keyframe,0,pendDesiredVel,walk3,L1,x']= -0.034877999950707, ['keyframe,0,pendDesiredVel,walk3,L1,z']= 1.0615023151327, ['keyframe,0,pendDesiredVel,walk3,R,x']= -0.047118683796602, ['keyframe,0,pendDesiredVel,walk3,R,z']= 0.96134666883946, ['keyframe,0,pendDesiredVel,walk3,L2,x']= -0.06712137484509, ['keyframe,0,pendDesiredVel,walk3,L2,z']= 0.88001405409083, ['keyframe,0,pendDesiredVel,ignore,1,x']= -0.072455660666938, ['keyframe,0,pendDesiredVel,ignore,1,z']= 0.83433934466322, 
['keyframe,0,pendDesiredVel,ignore,0,x']= -0.14044695724588, ['keyframe,0,pendDesiredVel,ignore,0,z']= 1.2229705059262, ['keyframe,0,pendDesiredVel,walk3,stand,x']= -0.138555988668, ['keyframe,0,pendDesiredVel,walk3,stand,z']= 1.2058658013444, ['keyframe,0,pendDesiredVel,walk3,l,x']= -0.10299343196814, ['keyframe,0,pendDesiredVel,walk3,l,z']= 1.18367017749, ['keyframe,0,pendDesiredVel,walk3,r1,x']= -0.046469681630227, ['keyframe,0,pendDesiredVel,walk3,r1,z']= 1.1763573169021, ['keyframe,0,pendDesiredVel,walk3,l2,x']= -0.032484039782161, ['keyframe,0,pendDesiredVel,walk3,l2,z']= 1.1643961048262, ['keyframe,0,pendDesiredVel,walk3,r2,x']= -0.028016610261361, ['keyframe,0,pendDesiredVel,walk3,r2,z']= 1.1315927433101, ['keyframe,0,pendDesiredVel,walk3,L1,x']= -0.035691276853665, ['keyframe,0,pendDesiredVel,walk3,L1,z']= 1.0606374493608, ['keyframe,0,pendDesiredVel,walk3,R,x']= -0.047617186955353, ['keyframe,0,pendDesiredVel,walk3,R,z']= 0.96126198845717, ['keyframe,0,pendDesiredVel,walk3,L2,x']= -0.067149822864115, ['keyframe,0,pendDesiredVel,walk3,L2,z']= 0.88021003161023, ['keyframe,0,pendDesiredVel,ignore,1,x']= -0.072532052666371, ['keyframe,0,pendDesiredVel,ignore,1,z']= 0.83489864546808, 
['keyframe,0,pendDesiredVel,ignore,0,x']= -0.096612651152437, ['keyframe,0,pendDesiredVel,ignore,0,z']= 1.0123143500277, ['keyframe,0,pendDesiredVel,walk3,stand,x']= -0.099692568020366, ['keyframe,0,pendDesiredVel,walk3,stand,z']= 1.0314054430555, ['keyframe,0,pendDesiredVel,walk3,l,x']= -0.093634229808585, ['keyframe,0,pendDesiredVel,walk3,l,z']= 1.0757886790618, ['keyframe,0,pendDesiredVel,walk3,r1,x']= -0.072281630400291, ['keyframe,0,pendDesiredVel,walk3,r1,z']= 1.1066359064401, 
['keyframe,0,pendDesiredVel,walk3,r1,x']= -0.039276534605731, ['keyframe,0,pendDesiredVel,walk3,r1,z']= 1.2209543649843, ['keyframe,0,pendDesiredVel,walk3,l2,x']= -0.031687841343506, ['keyframe,0,pendDesiredVel,walk3,l2,z']= 1.2157703189643, ['keyframe,0,pendDesiredVel,walk3,r2,x']= -0.028275562467192, ['keyframe,0,pendDesiredVel,walk3,r2,z']= 1.1920292143361, ['keyframe,0,pendDesiredVel,walk3,L1,x']= -0.039583257094422, ['keyframe,0,pendDesiredVel,walk3,L1,z']= 1.1204830989254, ['keyframe,0,pendDesiredVel,walk3,R,x']= -0.050675655664572, ['keyframe,0,pendDesiredVel,walk3,R,z']= 1.0297226658224, ['keyframe,0,pendDesiredVel,walk3,L2,x']= -0.06555872172676, ['keyframe,0,pendDesiredVel,walk3,L2,z']= 0.92177618405414, ['keyframe,0,pendDesiredVel,ignore,1,x']= -0.074787889602399, ['keyframe,0,pendDesiredVel,ignore,1,z']= 0.86449490982107, 
['keyframe,0,pendDesiredVel,ignore,0,x']= -0.087179446800004, ['keyframe,0,pendDesiredVel,ignore,0,z']= 1.0348303345441, ['keyframe,0,pendDesiredVel,walk3,stand,x']= -0.090409065052156, ['keyframe,0,pendDesiredVel,walk3,stand,z']= 1.0543276178, ['keyframe,0,pendDesiredVel,walk3,l,x']= -0.082872373150538, ['keyframe,0,pendDesiredVel,walk3,l,z']= 1.133674446991, ['keyframe,0,pendDesiredVel,walk3,r1,x']= -0.0549159982745, ['keyframe,0,pendDesiredVel,walk3,r1,z']= 1.1804148649246, ['keyframe,0,pendDesiredVel,walk3,l2,x']= -0.029513030795877, ['keyframe,0,pendDesiredVel,walk3,l2,z']= 1.1965872231362, ['keyframe,0,pendDesiredVel,walk3,r2,x']= -0.032002234067007, ['keyframe,0,pendDesiredVel,walk3,r2,z']= 1.1644531327121, ['keyframe,0,pendDesiredVel,walk3,L1,x']= -0.037368168548628, ['keyframe,0,pendDesiredVel,walk3,L1,z']= 1.1074730159173, ['keyframe,0,pendDesiredVel,walk3,R,x']= -0.048899570410386, ['keyframe,0,pendDesiredVel,walk3,R,z']= 1.0227265217082, ['keyframe,0,pendDesiredVel,walk3,L2,x']= -0.066771276600533, ['keyframe,0,pendDesiredVel,walk3,L2,z']= 0.93403787671459, ['keyframe,0,pendDesiredVel,ignore,1,x']= -0.075180728043144, ['keyframe,0,pendDesiredVel,ignore,1,z']= 0.88191779574904, 
}
function useCases.roundoff_common:genStageParamInitial()
	local stageParam={}
	self:updateStageParam(1,stageParam)
	return stageParam
end
    useCases.roundoff_common.controlParam={
	['keyframe,0,footLmod,walk3,L,x']=0.032551037185664, 
	['keyframe,0,footLmod,walk3,L,y']=-0.11429149453797, 
	['keyframe,0,footLmod,walk3,L,z']=0.11375186711598, 
	['keyframe,0,footLmod,walk3,R,x']=0.22407127068047, 
	['keyframe,0,footLmod,walk3,R,y']=-0.03730665291945, 
	['keyframe,0,footLmod,walk3,R,z']=0.17486710892858, 
	['keyframe,0,footLmod,walk3,l,x']=0.074585284349029, 
	['keyframe,0,footLmod,walk3,l,y']=-0.019630870566434, 
	['keyframe,0,footLmod,walk3,l,z']=0.014904212212173, 
	['keyframe,0,footLmod,walk3,r,x']=-0.088559653215181, 
	['keyframe,0,footLmod,walk3,r,y']=-0.026029571669039, 
	['keyframe,0,footLmod,walk3,r,z']=-0.037708411206066, 
	['keyframe,0,footLmod,walk3,stand,x']=0.084925654950265, 
	['keyframe,0,footLmod,walk3,stand,y']=-0.020622950417573, 
	['keyframe,0,footLmod,walk3,stand,z']=-0.15346583851196, 
	['keyframe,0,footRmod,walk3,L,x']=0.0031398225089695, 
	['keyframe,0,footRmod,walk3,L,y']=-0.19679901698055, 
	['keyframe,0,footRmod,walk3,L,z']=0.18755433784853, 
	['keyframe,0,footRmod,walk3,R,x']=0.0094986279701422, 
	['keyframe,0,footRmod,walk3,R,y']=-0.22893667992108, 
	['keyframe,0,footRmod,walk3,R,z']=-0.029236219077525, 
	['keyframe,0,footRmod,walk3,l,x']=0.1304233073344, 
	['keyframe,0,footRmod,walk3,l,y']=-0.036396859692388, 
	['keyframe,0,footRmod,walk3,l,z']=0.14764598484425, 
	['keyframe,0,footRmod,walk3,lr,x']=-0.15, 
	['keyframe,0,footRmod,walk3,lr,y']=-0.15, 
	['keyframe,0,footRmod,walk3,lr,z']=-0.15, 
	['keyframe,0,footRmod,walk3,r,x']=0.012555786143549, 
	['keyframe,0,footRmod,walk3,r,y']=-0.08350917462778, 
	['keyframe,0,footRmod,walk3,r,z']=-0.15577631354413, 
	['keyframe,0,footRmod,walk3,stand,x']=-0.0014236998080328, 
	['keyframe,0,footRmod,walk3,stand,y']=0.11367283944752, 
	['keyframe,0,footRmod,walk3,stand,z']=0.15391838623564, 
	['keyframe,1,footLmod,walk3,L,x']=-0.12756502352219, 
	['keyframe,1,footLmod,walk3,L,y']=-0.14248219157192, 
	['keyframe,1,footLmod,walk3,L,z']=-0.0033571333272981, 
	['keyframe,1,footLmod,walk3,R,x']=-0.031692877714662, 
	['keyframe,1,footLmod,walk3,R,y']=0.14541460906986, 
	['keyframe,1,footLmod,walk3,R,z']=-0.048482113999042, 
	['keyframe,1,footLmod,walk3,l,x']=0.085684992293451, 
	['keyframe,1,footLmod,walk3,l,y']=-0.00055232495387547, 
	['keyframe,1,footLmod,walk3,l,z']=0.03871066089903, 
	['keyframe,1,footLmod,walk3,r,x']=0.15269498138966, 
	['keyframe,1,footLmod,walk3,r,y']=-0.13317714075658, 
	['keyframe,1,footLmod,walk3,r,z']=0.13285411025755, 
	['keyframe,1,footRmod,walk3,L,x']=-0.0010171541262532, 
	['keyframe,1,footRmod,walk3,L,y']=0.32772666626025, 
	['keyframe,1,footRmod,walk3,L,z']=-0.066586943279459, 
	['keyframe,1,footRmod,walk3,R,x']=-0.010806815038517, 
	['keyframe,1,footRmod,walk3,R,y']=0.0057386003456488, 
	['keyframe,1,footRmod,walk3,R,z']=-0.17228275548088, 
	['keyframe,1,footRmod,walk3,l,x']=-0.17958909520502, 
	['keyframe,1,footRmod,walk3,l,y']=-0.13612065843125, 
	['keyframe,1,footRmod,walk3,l,z']=0.088123642808051, 
	['keyframe,1,footRmod,walk3,lr,x']=-0.15, 
	['keyframe,1,footRmod,walk3,lr,y']=-0.10688003309708, 
	['keyframe,1,footRmod,walk3,lr,z']=-0.15, 
	['keyframe,1,footRmod,walk3,r,x']=-0.17187657181966, 
	['keyframe,1,footRmod,walk3,r,y']=-0.14879026441232, 
	['keyframe,1,footRmod,walk3,r,z']=0.033628161717164, 
	['keyframe,1,footRmod,walk3,stand,x']=0, 
	['keyframe,1,footRmod,walk3,stand,y']=0, 
	['keyframe,1,footRmod,walk3,stand,z']=0, 
	['keyframe,2,footLmod,walk3,L,x']=0.060376761066944, 
	['keyframe,2,footLmod,walk3,L,y']=-0.18299917735403, 
	['keyframe,2,footLmod,walk3,L,z']=0.07943591138283, 
	['keyframe,2,footLmod,walk3,R,x']=0.02028377351073, 
	['keyframe,2,footLmod,walk3,R,y']=-0.23620621620386, 
	['keyframe,2,footLmod,walk3,R,z']=-0.16982909945119, 
	['keyframe,2,footLmod,walk3,l,x']=-0.15183244123531, 
	['keyframe,2,footLmod,walk3,l,y']=0.092746722974357, 
	['keyframe,2,footLmod,walk3,l,z']=-0.17029052317804, 
	['keyframe,2,footRmod,walk3,L,x']=0.00073919993272682, 
	['keyframe,2,footRmod,walk3,L,y']=-0.37386629438907, 
	['keyframe,2,footRmod,walk3,L,z']=0.089086691178637, 
	['keyframe,2,footRmod,walk3,R,x']=-0.02503793081151, 
	['keyframe,2,footRmod,walk3,R,y']=-0.37996162721607, 
	['keyframe,2,footRmod,walk3,R,z']=0.054430693147304, 
	['keyframe,2,footRmod,walk3,l,x']=0.0034602363864332, 
	['keyframe,2,footRmod,walk3,l,y']=-0.07949714699394, 
	['keyframe,2,footRmod,walk3,l,z']=-0.13158972862933, 
	['sequentialControlParam']=	{ }, 
	['useCases,walk3,COMobjWeight']=0, 
	['useCases,walk3,EEobjWeight']=60000, 
	['useCases,walk3,EEobjWeightAngular']=60000, 
	['useCases,walk3,conservativeW']=1, 
	['useCases,walk3,contactMargin']=0.01, 
	['useCases,walk3,ddqObjWeight']=10000, 
	['useCases,walk3,dotMomentumScale']=0.3, 
	['useCases,walk3,excludeRoot']=true, 
	['useCases,walk3,headControlWeight']=0, 
	['useCases,walk3,k_d_HEAD']=14, 
	['useCases,walk3,k_p_HEAD']=0, 
	['useCases,walk3,initialHeight']=0.1, 
	['useCases,walk3,k_d_EE']=24, 
	['useCases,walk3,k_d_ID']=20, 
	['useCases,walk3,k_d_momentum']=10, 
	['useCases,walk3,k_p_EE']=120, 
	['useCases,walk3,k_p_ID']=120, 
	['useCases,walk3,lambdaObjWeight']=0.0001, 
	['useCases,walk3,lambdaObjWeight2']=5, 
	['useCases,walk3,maxPenetratingVel']=0, 
	['useCases,walk3,momentumThr']=50, 
	['useCases,walk3,momentumWeight']=8000, 
	['useCases,walk3,noComvelDependentFootAdjustment']=true, 
	['useCases,walk3,noIntersectionPrevenction']=true, 
	['useCases,walk3,numericalDerivDmot']=true, 
	['useCases,walk3,perClassContactMargin']=1, 
	['useCases,walk3,tauObjWeight']=0.0001, 
	['useCases,walk3,turnGain']=10, 
	['useCases,walk3,velMarginOffset']=0, 
}

end
