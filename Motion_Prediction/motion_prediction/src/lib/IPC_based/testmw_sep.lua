require("config")
package.path=package.path..";../Resource/scripts/ui/RigidBodyWin/IPC_based/?.lua"

g_config={ path='IPC_based'}

require('RigidBodyWin/testRoutines/testmw')

function ctor()

	gTracker=Tracker:new()
	gTracker.stageParam= g_stageParam
	gTracker.numStage=table.getn(gTracker.stageParam)

	g_startG=0
	n_repetition=0
	gTracker:setStage(1,0)

	for i=1,1+n_repetition do

		if n_repetition>0 and i==1 and test_restoreStates then
			g_bRestoreStates=false
			local func=Tracker[gTracker.stageParam[i].setStageFunc]
			func(gTracker, 1, 0)
		else
			g_bRestoreStates = true
		end

		local pos=vectorn()
		local N_opt_dimension=table.getn(opt_dimension)
		pos:resize(N_opt_dimension)

		for i=1, N_opt_dimension do
			local optvar=opt_dimension[i]
			set1(pos, i, optvar.curval)
		end

		if g_set2 then
			g_i=i+2
		else
			g_i=i
		end

		obj=gTracker:objectiveFunction(pos, 4)
		print('objectivefunction: ',obj)
		if gTracker.synthesis.numFrames~=gTracker.synthesis.outputGlobal.numFrames then
			-- fall down case. just to avoid assertion failure.
			print("fall down")
			local nnn=math.min(gTracker.synthesis.numFrames, gTracker.synthesis.outputGlobal.numFrames)
			gTracker.synthesis.numFrames=nnn
			gTracker.synthesis.outputGlobal.numFrames=nnn
		end
		if g_set2 then
			gTracker.synthesis:saveStates("debugStates"..(i+2)..".tbl")
		else
			gTracker.synthesis:saveStates("debugStates"..i..".tbl")
		end
	end

	tbl1=util.loadTable('debugStates1.tbl')

	if n_repetition>0 then
		tbl2=util.loadTable('debugStates2.tbl')
		util.compareTable(tbl1, tbl2)
	end
	this:updateLayout()

	this("exit",{})
end
