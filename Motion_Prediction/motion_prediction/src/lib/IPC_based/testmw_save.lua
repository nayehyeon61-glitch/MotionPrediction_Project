
function saveDebugInfo(simulator, filename, additional_info)
	print('saving '..filename)
	local debugInfo=array:new()
	local w=simulator:getWorldState(0)
	local tbl={}
	for i=1,simulator:skeleton(0):numBone()-1 do
		tbl[#tbl+1]={simulator:skeleton(0):bone(i):name(), w:globalFrame(i).translation:copy(), w:globalFrame(i).rotation:copy()}
	end
	if additional_info then
		debugInfo:pushBack(additional_info)
	else
		debugInfo:pushBack({1})
	end
	debugInfo:pushBack(tbl)

	if simulator._debugInfo and simulator._debugInfo:length()~=0 then 
		debugInfo:pushBack(tostring(simulator._debugInfo)) 
		simulator._debugInfo:assign("")
	else
		debugInfo:pushBack(" ")
	end

	local cqInfo=simulator:queryContactAll()
	debugInfo:pushBack({"cqInfo", cqInfo:size()})
	for i=0, cqInfo:size()-1 do
		if cqInfo(i).chara==0 then
			debugInfo:pushBack({cqInfo(i).bone:name(), cqInfo(i).p:copy(), cqInfo(i).tau:copy(), cqInfo(i).f:copy(), })
		end
	end
	if filename then
		util.saveTable(debugInfo, filename) 
	else
		return debugInfo
	end
end
