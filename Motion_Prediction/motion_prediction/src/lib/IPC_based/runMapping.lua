 function convertSymmetric(title, param1)
    local out={}
    local domain
    if string.sub(title,1,5)=="spprt" or string.sub(title,1,5)=="swing" then
       if string.sub(title,6,6)=="_" then
	  -- not supported format
       else
	  local tgts
	  if string.sub(title,1,5)=="swing" then
	     tgts={"f2L_LR_RL", "ftL_RL", "f1L_RL_LR"}
	  else
	     tgts={"f2R_LR_RL", "ftR_RL", "f1R_RL_LR"}
	  end

	  local grp=string.sub(title, 8,10)

	  domain=string.sub(title, 12)
	    

	  local subIndex=tonumber(string.sub(title,7,7))
	  if subIndex then
	     tgts={tgts[subIndex]}
	  end

	  for t,tgt in ipairs(tgts) do
	     local axis=string.sub(title,6,6)
	     local target1=string.sub(tgt,1,3)..axis.."_"..grp..string.sub(tgt,4)
	     
	     out[target1]=param1

	     -- swap L and R
	     tgt2=string.gsub(tgt, "L", "9")
	     tgt2=string.gsub(tgt2, "R", "L")
	     tgt2=string.gsub(tgt2, "9", "R")

	     local target2=string.sub(tgt2,1,3)..axis.."_"..grp..string.sub(tgt2,4)

	     if axis=="x" then
		out[target2]=param1*-1
	     else
		out[target2]=param1
	     end
	  end
       end
    end
    return out, domain
 end


 function expandUseCase(tbl)
    local expanded={}
    for title,param1 in pairs(tbl) do
       local out=convertSymmetric(title, param1)
       expanded=table.join(expanded, out)
    end
    
    -- duplicate entries
    for k,v in pairs(expanded) do
       tbl[k]=v
    end
 end