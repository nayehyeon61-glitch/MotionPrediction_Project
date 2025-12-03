
require("config")

require("common")
require("subRoutines/RagdollSim")
require("subRoutines/Timeline")
require("subRoutines/WRLloader")
--[[ parameters ]]--

g_env={
	env_name=spec_id or 'walk-v1',
    scriptFile=script_file or "gym_deepmimic_physX/configmotions.lua", -- loaded before loading deepmimic.lua
}
spec_id=g_env.env_name

require('gym_cdm/torchModule')

dofile(g_env.scriptFile)

ctor_original=ctor -- defined in configmotions

function ctor()
	isMainloopInLua =true

	if isMainloopInLua and RE.ogreSceneManager() then
		hasGUI=true
	end
	--ctor_original()
	prepareEnv(spec_id) -- loads deepmimic.lua

	-- rendering step is defined in prepareEnv
	mTimeline=Timeline('timeline', 10000, rendering_step)

	if not param then
		param={}
	end
	
	param.globalTime=0
	
	-- override frameMove defined in deepmimic.lua
	frameMove=function(fElapsedTime)
		--param.globalTime=param.globalTime+rendering_step
		--if not RL_step then
		--	RL_step=rendering_step
		--end
		--local niter=math.round(rendering_step/RL_step)
		--for i=1, niter do
		--	envStep()
		--end
		--if param then
		--	local target=param.exitAt 
		--	if target and target<param.globalTime then this('exit!',0) end
		--end
		return 0
	end
	onFrameChanged=function(iframe)
		param.globalTime=param.globalTime+rendering_step
		if not RL_step then
			RL_step=rendering_step
		end
		local niter=math.round(rendering_step/RL_step)
		for i=1, niter do
			envStep()
		end
	end
	init_env()

	local filename="trained_models/ppo/"..spec_id..".pt"
	if not random_walk then
		g_agent=TorchAgent(filename, spec_id)
	end
    print('reset started')
	g_env.obs=reset(0) -- to compare with pytorch's obs, use g_agent.vec_norm:process(obs)
    print('reset finished')

    --만약 루프를 frameMove가 아닌 여기서 돌려면 아래 uncomment
    --while True:
    --    envStep() 
    --    if render_func is not None:
    --        render_func()
    --pdb.set_trace()
end
function envStep()
	local action
	if random_walk then
		action=CT.zeros(get_dim()(1))
	else
		action =g_agent:getAction_advanced(g_env.obs)
	end
	local _obs, done, reward=step(0, action)  
	g_env.obs=_obs

	if done==1 then
		g_env.obs=reset(0)
	end
end
