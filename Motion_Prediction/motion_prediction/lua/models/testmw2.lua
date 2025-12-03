require("config")
package.projectPath='../Samples/QP_controller/'
package.path=package.path..";../Samples/QP_controller/lua/?.lua" --;"..package.path
package.path=package.path..";../Samples/classification/lua/?.lua" --;"..package.path
package.path=package.path..";../Resource/scripts/ui/RigidBodyWin/IPC_based/?.lua"

g_config={ path='IPC_based'}

require("module")
preTest=false  -- sometimes preTest can fail when program is actually working well.
testStage34=false 

function loadTable(prefix, i)
	if util.isFileExist(prefix..i..".tbl") then
		local tbl
		tbl=util.loadTable(prefix..i..".tbl")

		if util.isFileExist('debugstates_comp/'..prefix..i..".tbl") then
			local tbl2
			tbl2=util.loadTable('debugstates_comp/'..prefix..i..".tbl") 
			compTable(tbl, tbl2, 'comparision with debugstates_comp/* '..prefix..i..".tbl")
		end
		return tbl
	end
	return nil
end

function compTable(a,b, msg)
	if a and b then
		print(msg)
		util.compareTableIgnore='pendTrajectory'
		util.compareTable(a,b)
	end
end
function diffVec(a,b)
	for i=0, a:size()-1 do
		if a(i)~=b(i) then
			print(i)
		end
	end
end
-- debug print vec3 or quater (full precision print)
function dp(t)
	local str=""
	str=str..string.format("%.90f,\n",t.x)
	str=str..string.format("%.90f,\n",t.y)
	str=str..string.format("%.90f,\n",t.z)
	if t.w then
		str=str..string.format("%.90f,\n",t.w)
	end
	print(str)
end

function dr(t)
	print(string.format("%.90f,\n",t))
end

--dbg.startTrace()
function ctor()
	print("test1")

	local numFrames=420

	local lastStage=3
	local tbl1=loadTable("debugStates_init","")
	local tbl2=loadTable("debugStates_init_f","")
	if tbl2 then
		-- testipcf
		print('testipcf')
		compTable(tbl1,tbl2)
		local a=loadTable("debugInfo_nocomp",1)
		local b=loadTable("debugInfo_nocomp_f",1)
		print(a,b,'tt')
		compTable(a,b)
	end
	if preTest then
		local function compareRestoreStates(a,b)

			do
				local tbl1=loadTable("debugStates_arbc1",a)
				local tbl2=loadTable("debugStates_arbc1",b)

				if tbl1 and tbl2 then
					if tbl1.numFrames~=tbl2.numFrames then
						print('Warning! stage 1 and 2 start at different frames, so the remaining tests in this set will be skip!')
						print('type "cont", to start from the next set of the tests.')
						dbg.console()
						return 
					end
					print("-----------------------------------")
					compTable(tbl1,tbl2, "after restoreState 2")
				end
			end
			do
				local tbl1=loadTable("debugStates_arbc2",a)
				local tbl2=loadTable("debugStates_arbc2",b)

				if tbl1 and tbl2 then
					if tbl1.numFrames~=tbl2.numFrames then
						print('Warning! stage 1 and 2 start at different frames, so the remaining tests in this set will be skip!')
						print('type "cont", to start from the next set of the tests.')
						dbg.console()
						return 
					end
					print("-----------------------------------")
					compTable(tbl1,tbl2, "after restoreState 2")
				end
			end
			do
				local tbl1=loadTable("debugStates_ar",a)
				local tbl2=loadTable("debugStates_ar",b)

				if tbl1 and tbl2 then
					if tbl1.numFrames~=tbl2.numFrames then
						print('Warning! stage 1 and 2 start at different frames, so the remaining tests in this set will be skip!')
						print('type "cont", to start from the next set of the tests.')
						dbg.console()
						return 
					end
					print("-----------------------------------")
					compTable(tbl1,tbl2, "after restoreState 2")
				end
			end
			g_tbl1=loadTable("debugInfo_restoreState",a)
			g_tbl1o=loadTable("debugInfo_oneStep",a)
			g_tbl1os=loadTable("debugInfo_oneStepSimul",a)

			g_tbl2=loadTable("debugInfo_restoreState",b)
			g_tbl2o=loadTable("debugInfo_oneStep",b)
			g_tbl2os=loadTable("debugInfo_oneStepSimul",b)

			print("-----------------------------------")
			compTable(g_tbl1,g_tbl2, "after restoreState")

			print("-----------------------------------")
			compTable(g_tbl1o, g_tbl2o, "after oneStep")
			print("-----------------------------------")
			compTable(g_tbl1os,g_tbl2os, "after oneStepSimul")

			g_tblp1=loadTable("debugStatesPN_restoreState",a)
			g_tblp1o=loadTable("debugInfoPN_oneStep",a)
			g_tblp1os=loadTable("debugInfoPN_oneStepSimul",a)

			compTable(g_tbl1,g_tblp1, "restoreState")
			if g_tbl1 and g_tbl1o then
				compTable(table.isubset(g_tbl1, 4),table.isubset(g_tbl1o,4), "restoreState1")
			end
			compTable(g_tbl1o,g_tblp1o, "restoreState2")
			--   compTable(tbl1os,tblp1os, "restoreState3")
		end
		loadTable("debugInfo_nocomp",2)
		loadTable("debugInfo_nocomp",0)
		loadTable("debugInfo_nocomp",1)
		print('comparing 1,2')
		compareRestoreStates(1,2)
		print('comparing 3,4')
		compareRestoreStates(3,4)
		print('comparing 1,3')
		compareRestoreStates(1,3)

	end
	print('tbl1: stage 1, repetition 1')
	print('tbl2: stage 1, repetition 2')
	for i=2,lastStage do
		print('tbl3: stage '..i..', repetition 1')
		print('tbl4: stage '..i..', repetition 2')
	end
	for i=0, numFrames do


		do 
			print("(execution trace) frame "..i)
			local tbl1, tbl2, tbl3, tbl4, tblp1

			tbl1=loadTable("debugStates_oneStep1-", i)
			tbl2=loadTable("debugStates_oneStep2-", i)
			compTable(tbl1, tbl2, "tbl1 vs tbl2")
			if testStage34 then
				for j=3,lastStage*2,2 do
					tbl3=loadTable("debugStates_oneStep"..j.."-", i)
					tbl4=loadTable("debugStates_oneStep"..(j+1).."-", i)
					compTable(tbl3, tbl4, "tbl3 vs tbl4")
					compTable(tbl1, tbl3, "tbl1 vs tbl3")
				end
			end
			tblp1=loadTable("debugStatesPN_oneStep1", i)
			compTable(tbl1, tblp1, "tbl1 vs tblp1")
		end

		do 
			print("-----------------------------------")
			print("after oneStep "..i)
			local tbl1, tbl2, tbl3, tbl4

			tbl1=loadTable("debugStates1__", i)
			tbl2=loadTable("debugStates2__", i)
			compTable(tbl1, tbl2, "tbl1 vs tbl2")
			if testStage34 then
				for j=3,lastStage*2,2 do
					tbl3=loadTable("debugStates"..j.."__", i)
					tbl4=loadTable("debugStates"..(j+1).."__", i)
					compTable(tbl3, tbl4, "tbl3 vs tbl4")
					compTable(tbl1, tbl3, "tbl1 vs tbl3")
				end
			end
			tblp1=loadTable("debugStatesPN1__", i)


			compTable(tbl1, tblp1, "tbl1 vs tblp1")
		end

		do 
			print("-----------------------------------")
			print("after oneStep simul (execution trace)"..i)
			local tbl1, tbl2, tbl3, tbl4, tblp1

			tbl1=loadTable("debugStates_oneStepSimul1_", i)
			tbl2=loadTable("debugStates_oneStepSimul2_", i)
			compTable(tbl1, tbl2, "tbl1 vs tbl2")
			if testStage34 then
				for j=3,lastStage*2,2 do
					tbl3=loadTable("debugStates_oneStepSimul"..j.."_", i)
					tbl4=loadTable("debugStates_oneStepSimul"..(j+1).."_", i)
					compTable(tbl3, tbl4, "tbl3 vs tbl4")
					compTable(tbl1, tbl3, "tbl1 vs tbl3")
				end
			end
			tblp1=loadTable("debugStatesPN_oneStepSimul1", i)

			compTable(tbl1, tblp1, "tbl1 vs tblp1")
		end

		do 
			print("-----------------------------------")
			print("after oneStepSimul "..i)
			-- if things are different here, then try to catch the error by adding more execution trace,
			-- because it is really difficult to debug errors after all the executions are finished.
			local tbl1, tbl2, tbl3, tbl4

			tbl1=loadTable("debugStates1_", i)
			tbl2=loadTable("debugStates2_", i)
			compTable(tbl1, tbl2, "tbl1 vs tbl2")
			if testStage34 then
				for j=3,lastStage*2,2 do
					tbl3=loadTable("debugStates"..j.."_", i)
					tbl4=loadTable("debugStates"..(j+1).."_", i)
					compTable(tbl3, tbl4, "tbl3 vs tbl4")
					compTable(tbl1, tbl3, "tbl1 vs tbl3")
				end
			end
			tblp1=loadTable("debugStatesPN1_", i)
			compTable(tbl1, tblp1, "tbl1 vs tblp1")
		end

		do 
			print("-----------------------------------")
			print("after prepareNextStep "..i)
			local tbl1, tbl2, tbl3, tbl4,tblp1

			tbl1=loadTable("debugStates1___", i)
			tbl2=loadTable("debugStates2___", i)
			compTable(tbl1, tbl2, "tbl1 vs tbl2")
			if testStage34 then
				for j=3,lastStage*2,2 do
					tbl3=loadTable("debugStates"..j.."___", i)
					tbl4=loadTable("debugStates"..(j+1).."___", i)
					compTable(tbl3, tbl4, "tbl3 vs tbl4")
					compTable(tbl1, tbl3, "tbl1 vs tbl3")
				end
			end
			tblp1=loadTable("debugStatesPN1___", i)
			compTable(tbl1, tblp1, "tbl1 vs tblp1")
		end

		do 
			local tbl1, tbl2, tbl3, tbl4
			print("-----------------------------------")
			print("compareChain "..i)
			tbl1=loadTable("debugStates_compareChain1_", i)
			tbl2=loadTable("debugStates_compareChain2_", i)
			compTable(tbl1, tbl2, "tbl1 vs tbl2")
			if testStage34 then
				for j=3,lastStage*2,2 do
					tbl3=loadTable("debugStates_compareChain"..j.."_", i)
					tbl4=loadTable("debugStates_compareChain"..(j+1).."_", i)
					compTable(tbl3, tbl4, "tbl3 vs tbl4")
					--compTable(tbl1, tbl3, "tbl1 vs tbl3") -- cannot pass
				end
			end
		end
	end

	this("exit",0)
end

function dtor()
end

function onCallback(w, userData)
end

function frameMove(fElapsedTime)
end
