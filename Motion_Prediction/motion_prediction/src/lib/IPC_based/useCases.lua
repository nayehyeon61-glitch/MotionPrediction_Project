scenarios={STANDING=1, JUMP=2, WALK=3, RUN=4, RUN2=11, RUNF2=12, RUNF3=14, JUSTIN_RUN=13,JUSTIN_RUNF3=15,
STAND1=5, STAND2=6, STAND3=7, STAND4=10, STRAIGHT_RUN=16, STRAIGHT_RUN_NOCART=17,ETC=18 }

scenarios.param={}
scenarios.param.stand="NO_SLIDING_JOINTS"

function scenarios.toModel(scenario)

   local model
   if scenario==scenarios.JUMP then
	   model=model_files.justin_jump
   elseif scenario>=scenarios.STAND1 and scenario<=scenarios.STAND4 then
	   if scenarios.param.stand=="NO_SLIDING_JOINTS" then
		   model=model_files.justin_jump
	   else
		   model=model_files.justin_jump_cart
	   end
   elseif scenario==scenarios.JUSTIN_RUN then
      model=model_files.justin_run_cart
   elseif scenario==scenarios.RUNF2 then
      model=model_files.hyunwoo_full_cart
   elseif scenario==scenarios.RUNF3 then
      model=model_files.hyunwoo_full_cart
   elseif scenario==scenarios.JUSTIN_RUNF3 then
      model=model_files.justin_runf3_cart
   elseif scenario==scenarios.STRAIGHT_RUN then
      model=model_files.justin_straight_run_cart
   elseif scenario==scenarios.STRAIGHT_RUN_NOCART then
      model=model_files.justin_straight_run
  elseif scenario==scenarios.ETC then
	  model=model_files[useCase.modelName]
   else   
      model=model_files.hyunwoo_real_cart
      --model=model_files.hyunwoo_real
   end

   return model
end

useCases={}
useCases.prec=0.005
function useCases.clearParam(useCase, pattern)
	 for k,v in pairs(useCase.controlParam) do
		if select(1, string.find(k,pattern)) then
			--print('clearing', k)
			useCase.controlParam[k]=nil
		end
	 end
end
function useCases.clampParam(useCase, pattern, thr, thr2)
	if thr2==nil then
		assert(thr)
		thr2=thr
		thr=-thr
	end
	for k,v in pairs(useCase.controlParam) do
		if select(1, string.find(k, pattern)) then
			if v<thr then
				v=thr
			elseif v>thr2 then
				v=thr2
			end
			useCase.controlParam[k]=v
		end
	end
end
function useCases.scaleParam(useCase, pattern, scale)
	for k,v in pairs(useCase.controlParam) do
		if select(1, string.find(k, pattern)) then
			useCase.controlParam[k]=useCase.controlParam[k]*scale
		end
	end
end
function useCases.adjustParam(useCase, pattern, val)
	for k,v in pairs(useCase.controlParam) do
		if select(1, string.find(k, pattern)) then
			useCase.controlParam[k]=useCase.controlParam[k]+val
		end
	end
end

function useCases.clearParamMulti(useCase, start, endseg, pattern)
	local segn=useCase.segNames
	endseg=math.min(endseg, #segn)
	for iseg=start, endseg do
		for k,v in pairs(useCase.controlParam) do
			if select(1, string.find(k,pattern..','..useCase.grpName..','..segn[iseg])) then
				print('removing key '..k)
				useCase.controlParam[k]=nil
			end
		end
	end
end

function useCases.convertKeyframeall(k,useCase)
	local tokens=string.tokenize(k,',')
	local keyname=tokens[2]
	local grpName=tokens[3]
	local segName=tokens[4]
	local axis=tokens[5]
	local skey,ekey=useCases.getNumKey(useCase, keyname, grpName, segName)
	local out={}
	for i=skey,ekey do
		local key='keyframe,'..i..','..keyname..','..grpName..','..segName..','..axis
		out[#out+1]=key
	end
	return out
end
function useCases.getNumKey(useCase,keyname, grpName, segName)
	local kf=useCase.keyframes[keyname]
	local fkey=0
	local lkey=kf.numKey-1
	assert(grpName)
	if kf and kf.numKeyFrom and useCase.graphParam[grpName] then
		local numKey=useCases._getGraphParam(useCase,grpName,segName, 'num_'..kf.numKeyFrom )
		assert(numKey)
		lkey=numKey-1
		if useCases._getGraphParam(useCase,grpName,segName, 'key_first')~=0 then
			fkey=1
		end
		if useCases._getGraphParam(useCase,grpName,segName, 'key_last')~=0 then
			lkey=lkey-1
		end
	end
	return fkey, lkey
end
function useCases.cleanupStageParam(tbl)
	local max_optdim=20
	local function filtersub(x,pattern) 
		if select(1,string.find(x[1],pattern)) then 
			return true 
		end 
		return false
	end
	local function filter(str, input_param)
		local param=array.filter(function (x) return filtersub(x,str) end, input_param)
		while #param>=max_optdim do
			table.remove(param, math.random(1,#param))
		end
		return param
	end
	for i=1,#tbl do
		assert(tbl[i])
		assert(tbl[i].param)

		if(#(tbl[i].param)==0) then
		else
			if(#(tbl[i].param)>max_optdim) then
				local cdim=#(tbl[i].param)
				print(cdim)
				local param=tbl[i].param

				if cdim<=max_optdim*1.5 then
					local tblo=deepCopyTable(tbl[i])
					table.remove(tbl,i)
					local tempt=deepCopyTable(tblo)
					tempt.param=filter(",[xz]$", param)
					table.insert(tbl,i, tempt)
					local tempt=deepCopyTable(tblo)
					tempt.param=filter(",[xy]$", param)
					table.insert(tbl,i, tempt)
					local tempt=deepCopyTable(tblo)
					tempt.param=filter(",[yz]$", param)
					table.insert(tbl,i, tempt)
					i=i+2
				else
					local tblo=deepCopyTable(tbl[i])
					table.remove(tbl,i)
					local tempt=deepCopyTable(tblo)
					tempt.param=filter(",x$", param)
					table.insert(tbl,i, tempt)
					local tempt=deepCopyTable(tblo)
					tempt.param=filter(",y$", param)
					table.insert(tbl,i, tempt)
					local tempt=deepCopyTable(tblo)
					tempt.param=filter(",z$", param)
					table.insert(tbl,i, tempt)
					i=i+2
				end
				--assert(#param<=max_optdim)
				--printTable(param)
			end
		end
	end
	for i=1,#tbl do
		if tbl[i]==nil then break end
		assert(tbl[i].param)

		if(#(tbl[i].param)==0) then
			table.remove(tbl,i)
		end
	end
	if false  then -- check
		for i=1,#tbl do
			print('stage '..i)
			printTable(tbl[i].param)
			dbg.console()
		end
	end
end
function useCases.setBases(useCase, title, param1)
	local tbl=string.tokenize(title, ',')
	local setIdx=tonumber(tbl[2])
	local basisIdx=tonumber(tbl[3])
	local dim=tonumber(tbl[4])
	useCase.basesSet.set[setIdx]:setBases(basisIdx, dim, param1)	
end
function useCases.accumulate(useCase, cp_mod)
	local cp_mod2={}
	for k,v in pairs(cp_mod) do
		if string.sub(k,1,8)=='useCase,' then
			cp_mod2['useCases,'..useCase.grpName..','..string.sub(k,9)]=v
		elseif string.sub(k,1,12)=="keyframeall," then
			local out=useCases.convertKeyframeall(k,useCase)
			for i,key in ipairs(out) do
				local cval=useCase.controlParam[key] or 0
				cp_mod2[key]=v+cval
			end
			useCase.controlParam[k]=nil -- do not apply multiple times!
		elseif string.sub(k, 1,9)=='setBases,' then
			useCases.setBases(useCase, k,v)
		else
			cp_mod2[k]=v
		end
	end
	useCase.controlParam=useCases.mergeCP(useCase.controlParam, cp_mod2)
end
function useCases.saveOptResult(useCase, trackOpt, afterOpt, filename)
	local bestEval=trackOpt.best_eval
	local optResult=trackOpt:getOptResult()
	if afterOpt==nil then
		afterOpt={{"accumulateOptResult"}, }
	end
	afterOptFunctions={}

	afterOptFunctions.accumulateOptResult=function (output, input, bestEval)
		table.mergeInPlace(output, input)
	end

	local output={}
	for iopt, opt in ipairs(afterOpt) do
		local fcn=afterOptFunctions[opt[1]]
		assert(fcn)
		fcn(output, optResult, bestEval, unpack(array.sub(opt,2)))
	end

	local outputString=table.tostring(output)
	local msg=util.writeFile(filename, ' cp_mod='..outputString..'\n') 
	if msg then
		util.outputToFile(Optimizer.outFile, msg)
	end
end

function useCases.updateStageParam(self, stage, stageParam, working, genOpt, best_eval, best_eval_unscaled, startParam, noHand)
	local optMomentum='desiredMomentum'
	local tgt_all={'footLmod', 'footRmod'}--, optMomentum}
	local tgt_all2={'footLmod', 'footRmod','desiredMomentum'}--, optMomentum}
	local tgt_all3={'desiredMomentum'}--, optMomentum}
	local tgt_all4={'footLmod', 'footRmod', 'desiredLeaning', 'handLmod', 'handRmod'}
	local tgt_all5={'footLmod_all', 'footRmod_all', 'handLmod_all', 'handRmod_all'}
	local tgt_subspace={'subspace'}

	if noHand then
		tgt_all4={'footLmod', 'footRmod', 'desiredLeaning'}
		tgt_all5={'footLmod_all', 'footRmod_all'}
	end
	local thr=7e-6
	local modes=         {     LRLean=1,  LRLean2=2, Subspace=3, Test=4}
	local initialMode=modes.LRLean
	local modeTransition={modes.Test, modes.Subspace, modes.Test, modes.LRLean}
	--local initialMode=modes.LRLean
	--local modeTransition={modes.LRLean, modes.Unused, modes.Unused, modes.Test, modes.LandR, modes.optX, modes.Unused, modes.optZ, modes.Momentum}
	--local modeTransition={modes.Test, modes.Unused, modes.Unused, modes.Test, modes.LandR, modes.LandR, modes.Unused, modes.optZ, modes.Momentum}

	local function filtersub(x,pattern) 
		if select(1,string.find(x[1],pattern)) then 
			return true 
		end 
		return false
	end
	local function filter(str, input_param)
		local param=array.filter(function (x) return filtersub(x,str) end, input_param)
		local max_optdim=20
		while #param>=max_optdim do
			table.remove(param, math.random(1,#param))
		end
		return param
	end
	local defaultAfterOpt={{"accumulateOptResult"}, }
	local function setNextMode(nextMode, s,e,ps, pe)
		if nextMode==modes.LandR then -- optimize L and R
			stageParam[stage]=genOpt(s, e, ps, ps, nil, nil, tgt_all)
			stageParam[stage].mode=nextMode
			stageParam[stage].info=tostring(ps)..' L and R'
		elseif nextMode==modes.Subspace then
			stageParam[stage]=genOpt(s, e, math.max(pe-5,1), pe, nil, nil, tgt_subspace)
			stageParam[stage].mode=nextMode
			stageParam[stage].info=tostring(ps)..' subspace'
		elseif nextMode==modes.LRLean then -- optimize L,R, leaning
			stageParam[stage]=genOpt(s, e, pe, pe, nil, nil, tgt_all4)
			stageParam[stage].mode=nextMode
			stageParam[stage].info=tostring(ps)..' LRLean'
		elseif nextMode==modes.LRLean1 then -- optimize L,R, leaning
			stageParam[stage]=genOpt(s, e, pe, pe, nil, nil, tgt_all4)
			stageParam[stage].mode=nextMode
			stageParam[stage].info=tostring(ps)..' LRLean1'
			-- do not accumulate the last result so that LRLean2 can be run independently to LRLean1.
			stageParam[stage].afterOpt={{"accumulateInitBasis", 0, pe, pe}, }  
		elseif nextMode==modes.LRLean2 then -- optimize L,R, leaning
			stageParam[stage]=genOpt(s, e, pe, pe, nil, nil, tgt_all4)
			stageParam[stage].mode=nextMode
			stageParam[stage].info=tostring(ps)..' LRLean2'
			stageParam[stage].afterOpt={{"accumulateInitBasis",1, pe, pe}, {"accumulateOptResult"}, }  
		elseif nextMode==modes.LRall then
			stageParam[stage]=genOpt(s, e+1, ps, ps+2, nil, nil, tgt_all5)
			stageParam[stage].mode=nextMode
			stageParam[stage].info=tostring(ps)..' LR all'
		elseif nextMode==modes.optX then
			stageParam[stage]=genOpt(s, e, ps, ps+1, nil, nil, tgt_all4)
			stageParam[stage].param=filter(",[xy]$", stageParam[stage].param)
			stageParam[stage].mode=nextMode
			stageParam[stage].info=tostring(ps)..' optX'
		elseif nextMode==modes.optZ then
			stageParam[stage]=genOpt(s, e, ps, ps+1, nil, nil, tgt_all4)
			stageParam[stage].param=filter(",[yz]$", stageParam[stage].param)
			stageParam[stage].mode=nextMode
			stageParam[stage].info=tostring(ps)..' optZ'
		elseif nextMode==modes.L then -- optimize L
			stageParam[stage]=genOpt(s, e, ps, ps, nil, nil, {'footLmod'})
			stageParam[stage].mode=nextMode
			stageParam[stage].info=tostring(ps)..' L'
		elseif nextMode==modes.R then -- optimize R
			stageParam[stage]=genOpt(s, e, ps, ps, nil, nil, {'footRmod'})
			stageParam[stage].mode=nextMode
			stageParam[stage].info=tostring(ps)..' R'
		elseif nextMode==modes.Momentum then -- optimize momentum
			stageParam[stage]=genOpt(s, e, ps, ps, nil, nil, tgt_all2)
			stageParam[stage].info=tostring(ps)..' LR and mmt'
			local numParam=#stageParam[stage].param
			if numParam>18 then
				stageParam[stage]=genOpt(s, e, ps, ps, nil, nil, tgt_all3)
				stageParam[stage].info=tostring(ps)..' mmt'
			end
			stageParam[stage].mode=nextMode
		end
		stageParam[stage].endSeg_save=e
		if stageParam[stage].afterOpt==nil then
			stageParam[stage].afterOpt=defaultAfterOpt
		end
	end
	local startSeg=math.max(startParam-3, 1)
	if stage==1 then -- optimize L and R
		setNextMode(initialMode,startSeg, working+stage, startParam, startParam)
	else
		local prevEnd=stageParam[stage-1].endSeg_save
		local prevParamStart=stageParam[stage-1].ps
		local prevParamEnd=stageParam[stage-1].pe
		local prevMode=stageParam[stage-1].mode
		local nextMode=modeTransition[prevMode]
		--if coarseLog then coarseLog('update stage: ', prevEnd, prevParamStart, prevMode, nextMode) end
		if nextMode~=modes.Test then
			setNextMode(nextMode, startSeg,prevEnd, prevParamStart,prevParamEnd)
		else
			assert(nextMode==modes.Test)
			assert(best_eval_unscaled)
			nextMode=modeTransition[nextMode]
			if fineLog then fineLog('best eval: ', stage, best_eval, best_eval_unscaled) end
			local startParam=prevParamEnd+1
			local startSeg=math.max(startParam-3, 1)

			setNextMode(nextMode, startSeg, prevEnd+1, startParam, startParam)
		end
	end
end
function useCases.genOpt(useCase, starts, ends,ps,pe,name,axis, others, grpName, path)
	local tbl={}

	ps=ps or starts
	pe=pe or ends
	ends=math.max(ends,ps)
	ends=math.max(ends,pe)
	name=name or useCase.grpName
	path=path or useCase.segNames
	grpName=grpName or useCase.grpName
	local excludePattern=useCase.pendOptExcludePattern

	--starts=math.max(1, ps-2)
	--starts=math.max(1, ps-1)
	--starts=math.max(1, ps-10)

	local keyrange={}
	for i,segn in pairs(path) do
		local fkey,lkey=useCases.getNumKey(useCase, 'footLmod', grpName, segn)
		keyrange[i]={ fkey, lkey }
	end

	if axis==nil then
		axis={'x','y','z'}
	elseif type(axis)~='table' then
		axis={axis}
	end

	local gtarget=array:new()

	while true do
	for i=ps,math.ceil(pe) do
		local segn=path[i]
		if not segn then break end
		if not others then
			others={'footLmod', 'footRmod', 'handLmod', 'handRmod'}
		else
			--printTable(others)
		end


		local limb={footLmod='L', footRmod='R', handLmod='LH', handRmod='RH'}
		local precs={desiredMomentum=0.1, desiredDotMomentum=0.5, default=useCases.prec, desiredLeaning=0.005}
		local noY={desiredLeaning=true, pendDesiredVel=true}
		
		for ioptName, optName in ipairs(others) do
			local caxis=axis

			if #caxis==3 and noY[optName] then
				caxis={'x', 'z'}
			end
			local isAll=false
			if string.sub(optName, -4)=='_all' then
				isAll=true
				optName=string.sub(optName, 1, -5)
			end

			if select(1,string.find(optName, ',')) then
				local tokens=string.tokenize(optName, ',')
				optName=tokens[1]
				caxis=string.tokenize(tokens[2],':')
			end
			local isL=limb[optName]
			local skey,ekey
--			if keyrange[i] then
--				skey,ekey=unpack(keyrange[i])
--			end
			skey,ekey=useCases.getNumKey(useCase, optName, grpName, segn)

			--print(skey,ekey,i,useCase.segNames[i])
			local prec=precs[optName] or precs.default
			if isL then
				-- limb position optimization
				local segmentation=useCase.segmentation or useCase.segmentations[grpName]
				if segmentation['swing'..isL][i]~=-2 and keyrange[i] then
					local i1=segmentation['importance'..isL][i]
					local i2=segmentation['importance'..isL][i+1]
					if i==1 then
						print(i,isL, i1,i2)
					end
					if i1==1 and i2==1 then
						useCases.addOptParam(gtarget, skey, ekey, optName..','..name..','..path[i], prec, caxis, isAll)
					elseif i1==1 then
						useCases.addOptParam(gtarget, skey, skey, optName..','..name..','..path[i], prec, caxis, isAll)
					elseif i2==1 then
						if ekey>0 and skey<=ekey then
							useCases.addOptParam(gtarget, ekey, ekey, optName..','..name..','..path[i], prec, caxis, isAll)
						end
					end
				end
			elseif optName=='desiredLeaning' then
				if keyrange[i] then
					local segmentation=useCase.segmentation or useCase.segmentations[grpName]
					if segmentation.usePositionControl and segmentation.usePositionControl[i]==true then
						useCases.addOptParam(gtarget, skey, ekey, optName..','..name..','..path[i], prec, caxis)
					end
				end
			else
				-- others
				if keyrange[i] then
					useCases.addOptParam(gtarget, skey, ekey, optName..','..name..','..path[i], prec, caxis)
				end
			end
		end
	end
	if #gtarget>0 then 
		break
	else
		ps=ps+1
		pe=pe+1
	end
	end

	-- printTable(gtarget)
	-- local dbgParam=useCase.segmentation
	if math.ceil(ends)==ends then
		return { startSeg=starts,endSeg=ends,ps=ps, pe=pe, setStageFunc='setStage_param', param=gtarget, baseStage=0, }
	else
		return { startSeg=starts,endSeg=math.ceil(ends),ps=ps,pe=pe, endFrac=math.mod(ends,1), setStageFunc='setStage_param', param=gtarget, baseStage=0, }
	end
end

function useCases.getNumSeg(firstFrames)
	return table.getn(firstFrames)-3
end

function useCases.getGraphParam(grpName, segName, key)
	return useCases._getGraphParam(useCase,grpName, segName,key)
end
function useCases._getGraphParam(useCase, grpName, segName, key)
	assert(useCase.rectified)
	local gp=useCase.graphParam[grpName]
	if not gp then dbg.console('useCases.getGraphParam') end
	return gp[key][gp.segToIndex[segName]]
end
function useCases.rectifyGraph(useCase)

	if useCase.rectified then return end

	useCase.rectified=true
	if useCase.graphParam then
		local gp=useCase.graphParam
		for k, v in pairs(gp) do
			v.segToIndex={}
			for iseg, segname in ipairs(v.seg) do
				v.segToIndex[segname]=iseg
			end
		end
	end
	useCase.slope=useCase.slope or 0
	-- fill in unspecified default settings in useCase.segmentations and useCase.graph.
	if useCase.segProperties==nil then
		useCase.segProperties={}
	end
	if useCase.grpNameToUseCase==nil then
		useCase.grpNameToUseCase=useCases
	end

	local segmentations=useCase.segmentations
	if segmentations==nil then
		useCase.segmentations={segmentation=useCase.segmentation}
		segmentations=useCase.segmentations
	end
	local function copyProperty(tgt, src)
		useCase.segProperties[tgt]=deepCopyTable(useCase.segProperties[src])
		for segmentationName, segmentation in pairs(segmentations) do
			local firstFrames=segmentation.firstFrames
			local nseg=useCases.getNumSeg(firstFrames)
			segmentation[tgt]=util.copy(segmentation[src])
		end
	end
	do -- default segProperties
		local sp=useCase.segProperties
		local function fillDefault(tbl)
			for key, value in pairs(tbl) do
				if sp[key]==nil then
					sp[key]=value
				end
			end	
		end
		local default={
			usePositionControl ={default=useCase.usePositionControl},
			swingFoot={default="N"}, -- no swing foot
			swingL={default=-2}, -- means ignore
			swingR={default=-2},
			swingLH={default=-2},
			swingRH={default=-2},
		}
		fillDefault(default)
	end
	if useCase.lfootpos==nil then
		useCase.lfootpos=vector3(0,-0.03,0.11) 
		useCase.rfootpos=vector3(0,-0.03,0.11) 
	end

	if useCase.keyframes==nil then
		useCase.keyframes={}
	end
	if not useCase.keyframes.virtualForceCorrection then
		useCase.keyframes.virtualForceCorrection={numKey=1, default=vector3(0,0,0)}
	end
	if not useCase.keyframes.importanceL then
		useCase.keyframes.importanceL={numKey=2, default=1}
		useCase.keyframes.importanceR={numKey=2, default=1}
	end
	if not useCase.keyframes.spprtImpL then
		useCase.keyframes.spprtImpL={numKey=2, default=1}
		useCase.keyframes.spprtImpR={numKey=2, default=1}
	end
--	if not useCase.keyframes.mocapOriModL then
--		useCase.keyframes.mocapOriModL={numKey=3, numKeyFrom='key', isContinuous=false,default=vector3(0,0,0)}
--	end
	for keyname, keyproperty in pairs(useCase.keyframes) do
		if keyproperty.keytime then
			keyproperty.keytimeCurve=math.PiecewiseLinearCurve:new(keyproperty.keytime,CT.linspace(0,keyproperty.numKey-1, keyproperty.numKey))
		end
	end
	for segmentationName, segmentation in pairs(segmentations) do
		local firstFrames=segmentation.firstFrames
		local nseg=useCases.getNumSeg(firstFrames)
		if not segmentation.names then
			segmentation.names={}
			if nseg==1 then
				segmentation.names={segmentationName}
			else
				for i=1, nseg do
					segmentation.names[i]="s"..i
				end
			end
		end
		do 
			-- check name uniqueness
			local usedNames={}
			for i=1, nseg do
				assert(usedNames[segmentation.names[i]]==nil)
				usedNames[segmentation.names[i]]=true
			end
		end

		if not segmentation.grpName then
			segmentation.grpName=segmentationName
		end
		if not segmentation.importanceL then
			segmentation.importanceL={}
			segmentation.importanceR={}
			for i=1, table.getn(segmentation.firstFrames)-1 do
				segmentation.importanceL[i]=1
				segmentation.importanceR[i]=1
			end
		end
		if segmentation.usePositionControl then
			local pc=segmentation.usePositionControl
			if type(pc)~='table'  then
				local opc=pc
				pc={}
				for i=1, #segmentation.names do
					pc[i]=opc
				end
				segmentation.usePositionControl=pc
			end
			for i=1,#pc do
				if pc[i]==0 then
					pc[i]=false
				elseif pc[i]==1 then
					pc[i]=true
				end
			end
		end
		if useCase.keyframes.importanceLH and not segmentation.importanceLH then
			segmentation.importanceLH={}
			segmentation.importanceRH={}
			for i=1, table.getn(segmentation.firstFrames)-1 do
				segmentation.importanceLH[i]=0
				segmentation.importanceRH[i]=0
			end
		end

		
		local function initializeRefTime(key)
			if segmentation[key]==nil then
				segmentation[key]={}
				for i=1,nseg do
					segmentation[key][i]={0,1}
				end
			elseif type(segmentation[key])=='string' and string.startsWith(segmentation[key],"convertFromSwing") then
				local isL=string.sub(segmentation[key], 17)
				if segmentation['importance'..isL]==nil then
					segmentation['importance'..isL]={}
				end
				if segmentation['importanceFadeInOutInterval'..isL]==nil then
					segmentation['importanceFadeInOutInterval'..isL]={}
				end
				local sw=segmentation['swing'..isL]
				segmentation[key]={}
				local function findInterval(i,val)
					local ss=i
					local se=i
					while ss-1>=1 and sw[ss-1]==val do ss=ss-1 end
					while se+1<=#sw and sw[se+1]==val do se=se+1 end
					return ss,se+1
				end
				local function timeToFrame(refTime)
					local firstFrames=segmentation.firstFrames
					local idx=math.max(math.floor(refTime)+1,1)
					return sop.map(refTime, idx-1, idx, firstFrames[math.min(idx,#firstFrames)], firstFrames[math.min(idx+1,#firstFrames)])
				end
				local function frameToTime(refFrame)
					--print(refFrame)
					local firstFrames=segmentation.firstFrames
					local i
					for i=1,#firstFrames do
						assert(refFrame)
						if firstFrames[i]>refFrame then 
							assert(firstFrames[i-1])
							assert(firstFrames[i-1]<=refFrame and refFrame <firstFrames[i])
							return sop.map(refFrame, firstFrames[i-1], firstFrames[i], i-2, i-1)
						end
					end
					assert(false)
					return nil
				end
				local function calcSpprtRef(sps,spe)
					local mp=(sps+spe)*0.5
					local swingRef=segmentation['swingRef'..isL] 
					if swingRef  and swingRef[sps]~=-1 and swingRef[sps]~=nil then
						mp=frameToTime(swingRef[sps])
					end
					return mp
				end
				local excludePattern=useCase.pendOptExcludePattern
				for i=1,nseg do
					-- set importance
					local segn=segmentation.names[i]
					--if string.isMatched(segn, excludePattern)  then
					--	segmentation.numFootModKeys[i]=2
					--else
					--	segmentation.numFootModKeys[i]=3
					--end
					segmentation[key][i]={0,1,0,1}
					if sw[i]==0 then
						-- support foot
						local ss,se=findInterval(i,0)
						--    0 0 0 0 0
						--   1 2 3 4 5
						-- ss=1
						--         se=6
						local mid=calcSpprtRef(ss,se)
						segmentation[key][i]={mid-i, mid-i,0,1}
						segmentation['importance'..isL][i]=1
						segmentation['importance'..isL][i+1]=1
					elseif sw[i]==1 then
						local ss,se=findInterval(i,1)
						--assert(ss~=1)
						
						local sps,spe=findInterval(ss-1,0)
						local mp=calcSpprtRef(sps,spe)
						local sns,sne=findInterval(se,0)
						local me=calcSpprtRef(sns,sne)
						local startSW=timeToFrame(spe)
						local endSW=timeToFrame(sns)
						segmentation[key][i]={ mp-i, me-i,
						sop.map(timeToFrame(i), startSW, endSW, 0, 1) ,
						sop.map(timeToFrame(i+1), startSW, endSW, 0, 1) 
						} -- 
						segmentation['importance'..isL][i]=1
						segmentation['importance'..isL][i+1]=1
					elseif sw[i]==-1 then
						local ss,se=findInterval(i,-1)
						assert(ss~=1)
						if sw[ss-1]==0 then -- support to ignore transition
							local sps,spe=findInterval(ss-1,0)
							local mp=calcSpprtRef(sps,spe)
							local startSW=timeToFrame(spe)
							local endSW=timeToFrame(se)
							local w1= sop.map(timeToFrame(i), startSW, endSW, 0, 1) 
							local w2=sop.map(timeToFrame(i+1), startSW, endSW, 0, 1) 
							segmentation[key][i]={ 
								mp-i, 1,w1, w2
							} -- 
							segmentation['importance'..isL][i]=1-w1
							segmentation['importance'..isL][i+1]=1-w2
							segmentation['importanceFadeInOutInterval'..isL][i]={spe, se}
						else -- ignore to support transition
							local sns,sne=findInterval(se,0)
							local me=calcSpprtRef(sns,sne)
							local startSW=timeToFrame(ss)
							local endSW=timeToFrame(sns)
							local w1= sop.map(timeToFrame(i), startSW, endSW, 0, 1) 
							local w2=sop.map(timeToFrame(i+1), startSW, endSW, 0, 1) 
							segmentation[key][i]={ 
								0, me-i, w1, w2
							} -- 
							segmentation['importance'..isL][i]=w1
							segmentation['importance'..isL][i+1]=w2
							segmentation['importanceFadeInOutInterval'..isL][i]={ss,sns}
						end
					elseif sw[i]==-2 then
						segmentation['importance'..isL][i]=0
						segmentation['importance'..isL][i+1]=0
					end
				end
			end
		end
		initializeRefTime("footRefL")
		initializeRefTime("footRefR")
		if useCase.keyframes.importanceLH then
			initializeRefTime("footRefLH")
			initializeRefTime("footRefRH")
		end
		initializeRefTime("pelvisRef")

		do -- initializeSpprtRef

			local countSpprt={}
			for i=1, nseg do countSpprt[i]=0 end
			for iIsL, isL in ipairs({'L','R','LH', 'RH'}) do
				
				local sw=segmentation['swing'..isL]
				if sw then
					if segmentation['spprtImp'..isL]==nil then
						segmentation['spprtImp'..isL]={}
					end
					for i=1,nseg do
						if sw[i]==0 then
							countSpprt[i]=countSpprt[i]+1
						end
					end
				end
			end

			for iIsL, isL in ipairs({'L','R','LH', 'RH'}) do
				local sw=segmentation['swing'..isL]
				if sw then
					local spprtCount=countSpprt
					for i=1,nseg do
						--  state:      L D R D L D R ---
						--  countSpprt: 1 2 1 2 1 2 1
						--  spprtImpL :  1 0 0 1 1
						if sw[i]~=0 then
							segmentation['spprtImp'..isL][i]=0
							segmentation['spprtImp'..isL][i+1]=0
						-- else
						-- 	segmentation['spprtImp'..isL][i]=1
						-- 	segmentation['spprtImp'..isL][i+1]=1
						-- end
						elseif countSpprt[i]==1 then
							segmentation['spprtImp'..isL][i]=1
							segmentation['spprtImp'..isL][i+1]=1
						else
							if i==1 or sw[i-1]==0 then
								if i~=1 and spprtCount[i-1]==1 then
									segmentation['spprtImp'..isL][i]=1
								else
									assert(spprtCount[i]>0 and (spprtCount[i-1] or 1)>0)
									segmentation['spprtImp'..isL][i]=0.5/spprtCount[i]+0.5/(spprtCount[i-1] or 1)
								end
							else
								segmentation['spprtImp'..isL][i]=0
							end
							if i==nseg or sw[i+1]==0 then
								if i~=nseg and spprtCount[i+1]==1 then
									segmentation['spprtImp'..isL][i+1]=1
								else
									assert(spprtCount[i]>0 and (spprtCount[i+1] or 1)>0)
									segmentation['spprtImp'..isL][i+1]=0.5/spprtCount[i]+0.5/(spprtCount[i+1] or 1)
								end
							else
								segmentation['spprtImp'..isL][i+1]=0
							end
						end
					end
				end
			end

		end
		for k,v in pairs(useCase.segProperties) do
			if segmentation[k]==nil then
				segmentation[k]=v.default
			end

			if type(segmentation[k])~="table" then
				local v=segmentation[k]
				segmentation[k]={}
				for i=1, nseg do
					segmentation[k][i]=v
				end
			else
				assert(#segmentation[k]>=nseg)
			end
		end
		local key2targets={} --	{"pendRotRefTime"}
		local function setUseCase(key, gname, val, i)
			if val then
				if useCase.controlParam['keyframe,0,'..key..','..gname]==nil then
					useCase.controlParam['keyframe,0,'..key..','..gname]=val[i]
					useCase.controlParam['keyframe,1,'..key..','..gname]=val[i+1]
				end
			end
		end
		for i, name in ipairs(segmentation.names) do
			local grpName=segmentation.grpName
			local gname=grpName..','..name
			setUseCase('importanceL', gname, segmentation.importanceL, i)
			setUseCase('importanceR', gname, segmentation.importanceR, i)
			setUseCase('importanceLH', gname, segmentation.importanceLH, i)
			setUseCase('importanceRH', gname, segmentation.importanceRH, i)
			setUseCase('spprtImpL', gname, segmentation.spprtImpL, i)
			setUseCase('spprtImpR', gname, segmentation.spprtImpR, i)
			setUseCase('spprtImpLH', gname, segmentation.spprtImpLH, i)
			setUseCase('spprtImpRH', gname, segmentation.spprtImpRH, i)

			for ikey, keyname in ipairs(key2targets) do
				local key_value_list=segmentation[keyname]
				local keyframe_id='keyframe,0,'..keyname..','..gname
				local keyframe2_id='keyframe,1,'..keyname..','..gname
				if key_value_list and useCase.controlParam[keyframe_id]==nil then
					useCase.controlParam[keyframe_id]=key_value_list[i]
					useCase.controlParam[keyframe2_id]=key_value_list[i+1]
				end
			end
		end
	end
end
function useCases.summarizeControlParam(useCase)
	util.writeFile('cp_summary.lua', table.toHumanReadableString(useCase.controlParam))
end

function useCases.summarizeControlParam2(tbl, filename, cleanTable)
	-- print useCase.controlParam
	local out

	if cleanTable then
		out={}
		for k,v in pairs(tbl) do
			if type(v)~='table' then
				out[k]=v
			end
		end
	else
		out=deepCopyTable(tbl)
	end
	local function cleanZero(out)
		for k,v in pairs(out) do
			if type(v)=='table' then
				cleanZero(v)
			elseif type(v)=='function' then
				out[k]=nil
			elseif select(1,string.find(k,',spprtImp'))or
				select(1,string.find(k,',importance')) then
				out[k]=nil
			end
		end
	end

	cleanZero(out)

	local str= table.toIndentedString(out)
	print(str)
	util.writeFile(filename, 	str)
end

function useCases.calcKeyframeStat(pattern, controlParam)
	local min=1e10
	local max=-1e10
	for k,v in pairs(controlParam) do
		local s,e=string.find(k, pattern)
		if s then
			min=math.min(min, v)
			max=math.max(max, v)
		end
	end
	return min,max
end
function useCases.makeOptParam(numKey,...)
	local gtarget=array:new()
	local prec=useCases.prec
	for iid,id in ipairs({...}) do
		useCases.addOptParam(gtarget,0,numkey-1, id, prec, {'x','y','z'})
	end
	return gtarget
end
function useCases.makeOptParamAxis(numKey,axis,...)
	local gtarget=array:new()
	local prec=useCases.prec
	for iid,id in ipairs({...}) do
		useCases.addOptParam(gtarget,0,numkey-1,id, prec, {axis})
	end
	return gtarget
end
function useCases.addOptParam(gtarget,startKey, endKey, id, prec, axis, isAll)
	print('prec: ', prec)
	assert(not isAll)
	for i=startKey, endKey do
		for iax, ax in ipairs(axis) do
			if string.sub(id,1,3)=="MAP" then
				gtarget:pushBack({"map,"..i..","..string.sub(id,4)..','..axprec})
			else
				gtarget:pushBack({"keyframe,"..i..","..id..","..ax,prec})
			end
		end
	end
end
function useCases.measureOptCost(pos, graph)
	local table={}

	local seq=useCase.pendOptimizationPath.segments
	for i=0, pos:size()-1 do
		local title=opt_dimension[i+1].title
		table[title]=pos(i)
	end

	for i=1,#seq do
		local axes={'x','z'}
		for j=1,2 do
			local s='keyframe,0,pendDesiredVel,'..seq[i]..','..axes[j]
			--if not select(1,string.find(s,'ignore')) then
			do
				local v=graph:getControlParameterFromGraph(s)
				assert(table[s]==nil or table[s]==v)
				if table[s]==nil then
					table[s]=v
				end
			end
		end
	end

	local cost=0
	local function sqDist(ida,idb)
		local a=table["keyframe,0,pendDesiredVel,"..ida..",x"]
		local b=table["keyframe,0,pendDesiredVel,"..ida..",z"]
		local c=table["keyframe,0,pendDesiredVel,"..idb..",x"]
		local d=table["keyframe,0,pendDesiredVel,"..idb..",z"]

		return (a-c)*(a-c)+(b-d)*(b-d)
	end

	for i=1, #seq-1 do
		cost=cost+sqDist(seq[i], seq[i+1])
	end

	return cost
end
function useCases.isOneOf(useCase, ...)
	local grp={...}
	for i, rr in ipairs(grp) do
		if useCase.scenario==scenarios[rr] then
			return true
		end
	end
	return false
end
function useCases.isStanding(useCase)
	return useCases.isOneOf(useCase,"STAND1","STAND2", "STAND3", "STAND4")
end

function useCases.isRun(useCase) 
	return useCases.isOneOf(useCase, "JUSTIN_RUN", "JUSTIN_RUNF3", "RUN", "RUN2", "RUNF2", "RUNF3", "STRAIGHT_RUN", "STRAIGHT_RUN_NOCART")
end

function useCases.derive(u)
   local uu=deepCopyTable(u)
   if u.unoptimized==nil then
      uu.unoptimized=u
   else
      uu.unoptimized=u.unoptimized
   end
   return uu
end

function useCases.mergeCP(cp1, cp2)
   local cp=deepCopyTable(cp1)
   for k,v in pairs(cp2) do
      cp[k]=v
   end

   return cp
end

function useCases.unmapControlParam(useCase)
	while true do
		local bBreak=true
		for title,val in pairs(useCase.controlParam) do
			if string.sub(title,1,4)=="map," then
				local tbl=useCase.mapControlParam(self, title, val)
				for i,v in ipairs(tbl) do
					useCase.controlParam[v[1]]=v[2]
				end
				useCase.controlParam[title]=nil
				bBreak=false
				break
			end
		end
		if bBreak then break end
	end
end
--require('IPC_based/subspaceControl')
--
useCases.justinRun={
   scenario=scenarios.JUSTIN_RUN,
   controlParam={ 
velz_StR_SR= 3.2577998812174, velx_StR_LF= 0.1688270091674, velx_run_FL= -0.05666762243691, velx_StR_SR= -0.27125305907818, velx_StR_RD= 0.038923022007489, velx_StR_FR= 0.1736318073928, velz_run_FR= 2.0169888147239, velz_StR_DL= 1.4523188594197, velx_run_LF= -0.010145766777009, velz_StR_LF= 2.392332038851, velz_StR_RD= 0.68373577849242, velz_run_LF= 1.5871834271308, velz_StR_FR= 2.038983910804, velx_run_FR= 0.17339745415969, velz_run_FL= 1.0041759346847, velz_run_RF= 1.6125973785284, velx_run_RF= 0.068936497702274, velx_StR_DL= -0.027937292222662, }

}


useCases.justinRun3={
   scenario=scenarios.JUSTIN_RUN,
   controlParam={
['velx_run_LR']= -0.071087654600016, ['velx_StR_RL']= -0.11247794616802, ['velz_StR_RL']= 3.3055513748982, ['velz_StR_LR']= -0.23050817017128, ['velz_StR_SR']= 2.3179466227147, ['velz_STP_st']= 0.95243960060073, ['velz_run_LR']= 1.1273010534811, ['velx_StR_SR']= -0.21242888877646, ['velx_StR_LR']= -0.097742452623895, ['velx_STP_st']= -0.10377021426573, ['velz_run_RL']= 2.6467914327285, ['velx_run_RL']= -0.00046316978836039,
}

}




--[[
useCases.justinRun_opt=useCases.derive(useCases.justinRun)
useCases.justinRun_opt.mot_file="../Resource/scripts/ui/RigidBodyWin/justin_run_cart_opt.dof"
require("RigidBodyWin/opt_history/runheight_map")

do 
   local cp_mod={runheight_18= 0.02050303532761, runheight_28= 0.096736506232204, runheight_38= -0.21357123801536, runheight_48= 0.34864135995395,runheight_68= 0.095579482472643, runheight_58= -0.0005267924136226, runheight_88= -0.22549466703763,  runheight_78= 0.11015161197706, }

   useCases.justinRun_opt.controlParam=
      useCases.mergeCP(useCases.justinRun.controlParam, runheight_map(cp_mod))
end
]]--


--[[
useCases.jumpForward2_opt=useCases.derive(useCases.jumpForward2)
useCases.jumpForward2_opt.mot_file="../Resource/scripts/ui/RigidBodyWin/justin_jump_cart_smooth_forward2.dof"

 useCases.jumpForward2_opt.controlParam.velx_up= 0.5
-- useCases.jumpForward2_opt.controlParam.velz_up= 2.165
 useCases.jumpForward2_opt.controlParam.velz_up= 1.88
 useCases.jumpForward2_opt.controlParam.velx_after= 0
 useCases.jumpForward2_opt.controlParam.velz_after= 0


useCases.jumpUp_opt=useCases.derive(useCases.jumpUp)
useCases.jumpUp_opt.mot_file="../Resource/scripts/ui/RigidBodyWin/justin_jump_cart_smooth_up.dof"

--useCases.jumpUp_opt.controlParam.velx_up= -0.000801324649881
--useCases.jumpUp_opt.controlParam.velz_up= 0.010108280399894
--useCases.jumpUp_opt.controlParam.velx_after= 0.0039829523772489
--useCases.jumpUp_opt.controlParam.velz_after= 0.0014937752347826
]]--
function useCases.setInitialSolution(opt_dimension)
   for i,v in ipairs(opt_dimension) do
      if useCase.controlParam[v.title]~=nil then
	 v.curval=useCase.controlParam[v.title]
	 assert(v.curval~=nil, v.title)
      end
   end
end
function useCases.defaultFuncUpdateConstraints(useCase, graph, keyframes)
	keyframes=keyframes or useCase.keyframes
	--useCases.defaultFuncUpdateConstraintsSub(useCase,graph,'pendDesiredVel',1,useCase.pendOptExcludePattern)

	local function setParam(pf, val)
		local axes={'x','y','z'}
		for a,axis in ipairs(axes) do
			graph:changeControlParameter('keyframe,'..pf..','..axis,val[axis])
		end
	end
	local function getParam(pf)
		local axes={'x','y','z'}
		local val=vector3()
		for a,axis in ipairs(axes) do
			val[axis]=graph:getControlParameterFromGraph('keyframe,'..pf..','..axis)
		end
		return val
	end
	if useCases.subspaceUpdateConstraints then
		useCases.subspaceUpdateConstraints (useCase, graph, keyframes)
	end
	-- then do low-priority jobs (such as continuously connecting the keyframes).
	for k,v in pairs(keyframes) do
		if k=='subspace' then
		elseif v.numKeyFrom then
			for grpName, grpParam in pairs(useCase.graphParam) do
				for iseg=1, #grpParam.seg do
					local segn=grpParam.seg[iseg]
					if grpParam.key_first[iseg]~=0 then
						local kf=grpParam.key_first[iseg]
						local tokens=string.tokenize(kf, ':')
						local copyFrom
						if #tokens==2 then
							copyFrom=tonumber(tokens[2])
						else
							-- copy from the last key
							copyFrom=useCases.getGraphParam(grpName, kf, 'num_'..v.numKeyFrom) -1
						end
						setParam('0,'..k..','..grpName..','..segn, 
						getParam(tostring(copyFrom)..','..k..','..grpName..','..tokens[1]))
					end
					if grpParam.key_last[iseg]~=0 then
						local kf=grpParam.key_last[iseg]
						local tokens=string.tokenize(kf, ':')
						local copyFrom
						local copyFromGrp=grpName
						local copyFromSeg=tokens[1]
						if #tokens==2 then
							copyFrom=tonumber(tokens[2])
						elseif #tokens==3 then
							copyFromGrp=tokens[1]
							copyFromSeg=tokens[2]
							copyFrom=tonumber(tokens[3])
						else
							copyFrom=0
						end
						local numKey=useCases.getGraphParam(grpName, segn, 'num_'..v.numKeyFrom) 
						setParam(tostring(numKey-1)..','..k..','..grpName..','..segn, 
						getParam(tostring(copyFrom)..','..k..','..copyFromGrp..','..copyFromSeg))
					end
				end
			end
		end
	end
end


function useCases.defaultFuncUpdateConstraintsSub(useCase, graph, prefix,numKey, excludePattern)

	local function setParam(out,keyout, val)
		local pf=','..prefix..','..useCase.grpName..','
		local axes={'x','y','z'}
		for a,axis in ipairs(axes) do
			graph:changeControlParameter('keyframe,'..keyout..pf..out..','..axis,val[axis])
		end
	end
	local function getParam(out,keyout)
		local pf=','..prefix..','..useCase.grpName..','
		local axes={'x','y','z'}
		local val=vector3()
		for a,axis in ipairs(axes) do
			val[axis]=graph:getControlParameterFromGraph('keyframe,'..keyout..pf..out..','..axis)
		end
		return val
	end
	local function clearParam(out,keyout)
		setParam(out, keyout, vector3(0,0,0))
	end
	local function copyParam(a,ka,b,kb)
		setParam(a,ka, getParam(b,kb))
	end
	local function avgParam(out,keyout,b,kb,c,kc)
		setParam(a,ka, getParam(b,kb)*0.5+getParam(c,kc)*0.5)
	end

	local segn=useCase.segNames
	if numKey==1 then
		local excluded={}
		for i=1, #segn do
			if string.isMatched(segn[i], excludePattern) then
				excluded[i]=true
			else
				excluded[i]=false
			end
		end
		for i=1,#segn do
			if excluded[i] then
				if i>1 and not excluded[i-1] then
					copyParam(segn[i],0, segn[i-1],0)
				else
					assert(i<#segn)
					assert(not excluded[i+1])
					copyParam(segn[i],0, segn[i+1],0)
				end
			end
		end
	else
		assert(false)
	end
end

function useCases.set_opt_dimension(startSeg, endSeg, opt_dimension, max_step, grad_step, pattern, paramTableName, exclude_pattern)
	local i=1

	local seq=useCase.pendOptimizationPath.segments
	endSeg=math.min(#seq, endSeg)

	local cp=useCase[paramTableName or 'controlParam']

	if cp==nil then
		local msg='Error! pendControlParam doesn\'t exist. try l ipco_prep'
		print(msg)
		util.msgBox(msg)
	end


	for iseg =startSeg, endSeg do
		local axes={'x','z'}
		for axis=1,2 do
			local k='keyframe,0,pendDesiredVel,'..seq[iseg]..','..axes[axis]
			if (pattern==nil or str_include(k, pattern)) then

				if (exclude_pattern==nil or not string.isMatched(k, exclude_pattern)) then

					local od={}
					od.title=k
					od.curval=cp[k] or 0
					od.max_step=max_step
					od.grad_step=grad_step
					opt_dimension[i]=od
					i=i+1
				else
					print(k.." excluded from the set of optimization variables")
				end
			end
		end
	end
	printTable(opt_dimension)
	opt_dimension[i]=nil
	return useCase.pendOptimizationPath.firstFrames[endSeg+1]
end


function useCases.add_opt_dimension(opt_dimension, max_step, grad_step, pattern, paramTableName)
   local i=table.getn(opt_dimension)+1
   
   local cp=useCase[paramTableName or 'controlParam']
   for k,v in pairs(cp) do

      if pattern==nil or str_include(k, pattern) then

	 local od={}
	 od.title=k
	 od.curval=cp[k]
	 od.max_step=max_step
	 od.grad_step=grad_step
	 opt_dimension[i]=od
	 i=i+1
      end
   end
   opt_dimension[i]=nil
end

-- unused
--require("IPC_based/useCase_hopping")
--require("IPC_based/useCase_straightRun")
--require("IPC_based/useCase_skipping")
--require("IPC_based/useCase_cartwheel")
--useCase=useCases.jump_up
--useCase=useCases.jump_forward2
--useCase=useCases.cartwheel -- actually round off
--
--
-- stand3, jump_up, jump_forward2, testJump, 
-- stand_opt, run_opt, run2_opt, runf2_opt, runf3_srcGenarator, runf3, justinStraightRun , justinStraightRun_nocart, justinRunf3, skipping, justinRunf3_part2, justinRunf3_part3, justinRunf4, justinRun3, justinRun_opt
--

----------------------------------------------------------------
--
-- useCase can be set from the commandline too
--require("IPC_based/useCase_straightRun_nocart") useCase=useCases.justinStraightRun_nocart
--require("IPC_based/useCase_standing_no_cart") useCase=useCases.stand3
--require("IPC_based/useCase_standing_no_cart") useCase=useCases.stand2
--require("IPC_based/useCase_roundoff2") useCase=useCases.roundoff2
--require("IPC_based/useCase_straddle") useCase=useCases.straddle
--require("IPC_based/useCase_walk3") useCase=useCases.walk3 
--require("IPC_based/useCase_walk3_hwangpil") useCase=useCases.walk3 
--require("IPC_based/useCase_walk4_newIK") useCase=useCases.walk3 
--require("IPC_based/useCase_popa") useCase=useCases.popa2
--require("IPC_based/useCase_backflip") useCase=useCases.backflip
--require("IPC_based/useCase_handstand") useCase=useCases.handstand
--require("IPC_based/useCase_turn") useCase=useCases.turn 
require("IPC_based/useCase_turn2") useCase=useCases.turn  -- hwangpil
--
--	useCases.summarizeControlParam2(useCase.controlParam,'temp4')
--	useCases.summarizeControlParam2(useCase,'temp4_all', true)
if false then
	useCases.summarizeControlParam(useCase)
	-- print useCase.controlParam
	local out={}
	table.foreach(useCase.controlParam, function (k,v) if v~=0 then table.insert(out, {k,v}) end end)
	table.sort(out, function (v1, v2) return v1[1]<v2[1] end)
	table.foreachi(out, function (k,v) print(v[1], v[2]) end)
end
if false then
	print('bound,footmod'..useCase.grpName, useCases.calcKeyframeStat('foot..od,'..useCase.grpName, useCase.controlParam))
	print('bound,desiredLeaning', useCases.calcKeyframeStat('desiredLeaning,'..useCase.grpName, useCase.controlParam))
	print('boudn,desiredMomentum', useCases.calcKeyframeStat('desiredMomentum,'..useCase.grpName, useCase.controlParam))
end

