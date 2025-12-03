require("config")
require("common")


rendering_step=1/30

model={}
model.file_name="gym_mp/hyunwoo_lowdof_T_boxfoot.wrl"

random_action_environment=false
random_action_max=1
gui = false
-- RL environment functions (which can be used in python):
-- reset, init_env, step, python_render, get_dim
--
-- luaEnv.py에서
-- state=-100,100범위,
-- action=-1,1범위로 초기화되니, luaEnv.py는 고치지 말고, 루아파일에서 적당히 스케일해서 쓸 것.

-- 루아에서 난 에러는 파이썬에서 디버깅이 쉽지 않으니, 에러가 날 수 있는 루아코드는 
-- pcall (c++의 try, catch와 비슷)을 이용해서 루아안에서 에러를 체크할 것!!!
-- 아래 코드에서 pcall검색.

function ctor()
	this:create("Button", "Reset", "Reset")
	this:create("Button", "capture", "capture")
	this:create("Button", "start random action", "start random action")
	this:create("Button","stop","stop")
	this:updateLayout()
	this:redraw()
	if isMainloopInLua then 
		gui = true
	end
end

function dtor()
end

function onCallback(w, userData)
	if w:id()=='Reset' then
		reset()
	elseif w:id()=='capture' then
		RE.renderer():screenshot(true)
	elseif w:id()=="start random action" then
		random_action_environment=true
		random_action_max=1
		g_stepCount=0
		init_env()
		reset()
	elseif w:id() == "stop" then 
	end
end

-- 맨 처음 한번 호출
function init_env()
	print('lua initenv')
	mLoader=MainLib.VRMLloader(model.file_name)
	mEnv=mvaeEnv(mLoader)
end

-- return state_space dimension, action_space dimension
function get_dim()
	return CT.vec(33+2,10)
end

function frameMove(fElapsedTime)
	if isMainloopInLua then
		-- using python RL environment
		python.F('play_gym_luaenv', 'envStep')
	elseif random_action_environment then
		-- python에서 초출 되는 순서로 호출함

		local dims=get_dim()
		local adim=dims(1)
		local action=CT.rand(adim)

		local step_state, episode_done, step_reward=step(g_stepCount, action)

		-- ctrl+alt+o to see outputs
		RE.output2('RL step', episode_done, step_reward, step_state, action)

		g_stepCount=g_stepCount+1
		if episode_done==1 then
			reset()
			g_stepCount=0
		end
	end
end

-- returns state (vectorn)
function step(step_count_after_reset, action)
	local res
	local state, done, reward
	--action = action*0.3
	--print(action)
	if false then
		-- no error check here.
		-- 아래랑 동일한 의미이지만 에러 발생시 디버깅 어려움
		state, done, reward=mEnv:step(step_count_after_reset, action)
	else
		res, state,done,reward= pcall(mvaeEnv.step, mEnv, step_count_after_reset, action)
		if type(state)=='string' then
			print(state) -- lua error!
			dbg.console()
		end
	end
	-- print("state , done , reward : ",state,done,reward)
	if done then		
		return state,1 , reward
	else
		return state,0 , reward
	end
end

function python_render(renderBool)
	RE.renderOneFrame(true)
end

function reset()
	local return_initial_state= mEnv:reset()
	return return_initial_state 
end

mvaeEnv = LUAclass()
-- state = 타겟 위치(방향,거리)
function mvaeEnv:__init(loader)
	if gui then 
		local osm = RE.ogreSceneManager()
		osm:setFogNone()
	end
	local drawSkeleton=false
	self.DOF_size = 33
	self.en_data_size = 35
    self.observation_sapce = 2 + self.DOF_size
    self.action_space=10
	self.skin=RE.createVRMLskin(loader, drawSkeleton)
    local DOFcontainer = MotionDOFcontainer(loader.dofInfo,"../Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T_locomotion_hl.dof")
	local mMotionDOF = DOFcontainer.mot
	self.start_DOF = vectorn(mMotionDOF:numDOF())
	self.second_DOF = vectorn(mMotionDOF:numDOF())
	self.third_DOF = vectorn(mMotionDOF:numDOF())
	self.start_DOF:assign(mMotionDOF:row(108))
	self.second_DOF:assign(mMotionDOF:row(109))
	self.third_DOF:assign(mMotionDOF:row(110))
	self.skin:scale(100,100,100)
    self.skin:setPoseDOF(self.start_DOF)
	self.mLoader = loader:copy()
	self.mLoader:setPoseDOF(self.start_DOF)
    self.matrix = matrixn(2,self.en_data_size)
	self:pushBack(self.matrix,self:poseEncode(self.start_DOF,self.second_DOF))
	self:pushBack(self.matrix,self:poseEncode(self.second_DOF,self.third_DOF))
    self.prev_pose = vectorn(self.DOF_size)
	self.prev_pose:assign(self.start_DOF)
	--target_x = {-1000,1000}
	--self.target_position = vector3(random.choice(target_x),100,random.choice(target_x))
	self.angle = 0
	self.target_position = self.prev_pose:toVector3(0)*100 + quater(math.rad(self.angle),vector3(0,1,0))*vector3(0,0,100)
	--dbg.namedDraw("Sphere",self.target_position, "target", "blue",100)
end 

function mvaeEnv:step(steps,action)
    local reward = 0.0
    local done = false 
	local output = vectorn(self.en_data_size)
	python.F('play_mp','test_vae',action,self.matrix:row(1),output)
	if output:isnan() then 
		print("VAE output is nan")
		dbg.console()
	end
	self:pushBack(self.matrix,output)
    local pose_output = vectorn(self.DOF_size)
    pose_output:assign(self:poseDecode(self.matrix:row(1),self.prev_pose))
	reward = reward+self:calc_reward(self.prev_pose,pose_output,self.target_position)
	reward  = reward - action:length()*0.01
	reward = reward*0.1
	self.target_position = self.prev_pose:toVector3(0)*100 + quater(math.rad(self.angle),vector3(0,1,0))*vector3(0,0,100)
	dbg.namedDraw("Sphere",self.target_position, "target", "blue",10)
    if self:calc_distance(pose_output:toVector3(0),self.target_position/100) < 1 and false then 
        reward = reward+100.0
		print("target position",self.target_position)
		print("success!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
		local new_target_position = vector3(random.choice(target_x),100,random.choice(target_x))
		while new_target_position == self.prev_pose do
			new_target_position = vector3(random.choice(target_x),100,random.choice(target_x))
		end 
		self.target_position = new_target_position
		dbg.namedDraw("Sphere",self.target_position, "target", "blue",10)
    end 

	self.skin:setPoseDOF(pose_output)
	self.mLoader:setPoseDOF(pose_output)
    self.prev_pose:assign(pose_output)
	local observation_state = vectorn(self.observation_sapce)
	observation_state:assign(self:getEnvState())
	local COM=self.prev_pose:toVector3(0)
	local gridposz=math.floor(COM.z/4)
	local gridposx=math.floor(COM.x/4)
	if gui then 
		local bgnode=RE.ogreSceneManager():getSceneNode("BackgroundNode")
		bgnode:setPosition(vector3(gridposx*4*100,0,gridposz*4*100))
	end
	RE.viewpoint().vat:set(COM.x*100,COM.y*100,COM.z*100)
	RE.viewpoint():update()
	if observation_state:isnan() or reward ~= reward then 
		print("observation_state or reward is nan")
		dbg.console()
	end
	if steps > 1000 then 
		done = true
	end
	return observation_state, done, reward
end 

function mvaeEnv:calc_reward(prev_pose,curr_pose,target_pos)
	local before_root = prev_pose:toVector3(0)
	local after_root = curr_pose:toVector3(0)
	local target = target_pos/100
	local progress=self:calc_progress(after_root,before_root,target)
	
	local alignment = self:calc_alignment(curr_pose,target)

	--print("progress:",progress)
	--print("alignement:",alignment)
	--return progress + alignment
	return  alignment*0.1
end

function mvaeEnv:getEnvState()
	local observation_state = vectorn(self.observation_sapce)
	local rotY = self.prev_pose:toQuater(3):rotationY()
	local target_dir = rotY:inverse()*((self.target_position/100)-self.prev_pose:toVector3(0))
	observation_state:set(0,target_dir.x)
	observation_state:set(1,target_dir.z)
	observation_state:range(2,35):assign(self.prev_pose)
	return observation_state
end


function mvaeEnv:reset()
	-- added slight random perturbations
	self.prev_pose:assign(self.start_DOF) 	
	self.angle = (math.random()-0.5)*180
	self.target_position = self.prev_pose:toVector3(0)*100 + quater(math.rad(self.angle),vector3(0,1,0))*vector3(0,0,100)
	-- print("new target position : ",self.target_position/100)
	dbg.namedDraw("Sphere",self.target_position, "target", "blue",10)
	self:pushBack(self.matrix,self:poseEncode(self.start_DOF,self.second_DOF))
	self:pushBack(self.matrix,self:poseEncode(self.second_DOF,self.third_DOF))
	local character_pos = self.prev_pose:toVector3(0)
	local initial_state = vectorn(35)
	local target_dir = ((self.target_position/100) - character_pos)
	local rotY = self.prev_pose:toQuater(3):rotationY()
	local refcoord = rotY:inverse()*target_dir
	refcoord.y = 0
	initial_state:set(0,refcoord.x)
	initial_state:set(1,refcoord.z)
	initial_state:range(2,35):assign(self.prev_pose)
	self.skin:setPoseDOF(self.start_DOF)
	self.mLoader:setPoseDOF(self.start_DOF)
	if initial_state:isnan() then 
		print("reset is nan")
		dbg.console()
	end
	return initial_state
end

function mvaeEnv:rotationMatrix(q)
	local mat=matrix4()
	mat:setRotation(q)
	return mat
end

function mvaeEnv:poseEncode(pose1,pose2)
    local vec = vectorn(self.en_data_size)
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

function mvaeEnv:calc_alignment(pose,target_position)
	local rotY = pose:toQuater(3):rotationY()
	local character_dir = (rotY*vector3(0,0,1)):Normalize()
	local target_dir = (target_position-pose:toVector3(0)):Normalize()
	dbg.draw('Arrow',pose:toVector3(0)*100,pose:toVector3(0)*100+rotY*vector3(0,0,50),'character_dir',10)
	dbg.draw('Arrow',pose:toVector3(0)*100,pose:toVector3(0)*100+target_dir*50,'target_dir',10,"red")
	local diff = self:calc_dot(target_dir,character_dir)
	local angle = math.acos(diff)
	local alignment = math.exp(math.cos(angle)-1)
	if alignment ~= alignment then 
		print("calc_alignment is nan")
		dbg.console()
	end
	return alignment
end

function mvaeEnv:calc_progress(after_root , before_root , target)
	local after = self:calc_distance(after_root,target)
	local before = self:calc_distance(before_root,target)
	local progress = before - after
	if progress ~= progress then 
		print("calc_progress is nan")
		dbg.console()
	end
	return progress
end

function mvaeEnv:calc_dot(a,b)
	local output = (a.x*b.x)+(a.y*b.y)+(a.z*b.z)
	if output ~= output then 
		print("calc_dat is nan")
		dbg.console()
	end
	return output
end 

function mvaeEnv:raySensor(T,distance,range)
    for i = 1,2*range+1 do 
		dbg.erase("Line","ray"..tostring(i))
	end
	local raypoints = {}
	local angle = T.rotation:rotationAngleAboutAxis(vector3(0,1,0))
	local pos  = T.translation*100

	for i = -range,range,5 do 
		ray_angle = math.deg(angle) + i
		ray_angle = math.rad(ray_angle)
		x = pos.x + distance*math.sin(ray_angle)
		z = pos.z + distance*math.cos(ray_angle)
		table.insert(raypoints,vector3(x,100,z))
	end

	local rays = vectorn(#raypoints)
	rays:setAllValue(distance)

	for i = 1 , #raypoints do 
		local res, position, normal = mChecker:checkRayIntersection(pos,raypoints[i])
		
		if res then 
			position = position*100
			dbg.draw("Line",pos,position,"ray"..tostring(i))
			rays:set(i-1,calc_distance(pos,position))

		else
			dbg.draw("Line",pos,raypoints[i] ,"ray"..tostring(i),"green")
		end
	end	
	return rays
end
	
function mvaeEnv:calc_distance(a,b)
	local output = math.sqrt((a.x-b.x)^2 +(a.z-b.z)^2 )
	if output ~= output then 
		print("calc_distance is nan")
	end
	return output
end

function mvaeEnv:pushBack(matrix,data)
	for i=0,matrix:rows()-2 do
		matrix:row(i):assign(matrix:row(i+1))
	end
	matrix:row(matrix:rows()-1):assign(data)

end

function mvaeEnv:poseDecode(encodeData,prev_pose)
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
