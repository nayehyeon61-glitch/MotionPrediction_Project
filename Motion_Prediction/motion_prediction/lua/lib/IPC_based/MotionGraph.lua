require("subRoutines/CompareChain")

-- scenarios definitions has moved to useCases.lua


function scenarios.loadAnimation(skel)
   local motion=skel.mMotion

   local container=MotionDOFcontainer(skel.dofInfo, model.mot_file)
   local leftFoot=container.conL
   local rightFoot=container.conR
   local motionDOF=container.mot
   
   skel.dofInfo:setFrameRate(model.frame_rate)   


   local dmotionDOF=calcDerivative(motionDOF, container.discontinuity)

   return motionDOF, dmotionDOF, leftFoot, rightFoot, container.discontinuity
end

function scenarios.loadAndExportAnimation(skel, file)
   local motion=skel.mMotion

   local container=MotionDOFcontainer(skel.dofInfo, model.mot_file)
   local leftFoot=container.conL
   local rightFoot=container.conR
   local motionDOF=container.mot
   
   skel.dofInfo:setFrameRate(model.frame_rate)   


   local dmotionDOF=calcDerivative(motionDOF)


   file:packInt(motionDOF:numFrames())
   file:pack(motionDOF:matView())
   file:pack(dmotionDOF)
   file:pack(leftFoot)
   file:pack(rightFoot)

   return motionDOF, dmotionDOF, leftFoot, rightFoot
end

function scenarios.importAnimation(skel, file)
   skel.dofInfo:setFrameRate(model.frame_rate)

   local motionDOF=MotionDOF(skel.dofInfo)
   motionDOF:resize(file:unpackInt())
   file:unpack(motionDOF:matView())
   local dmotionDOF=matrixn()
   file:unpack(dmotionDOF)

   print("Create graph")

   local leftFoot=boolN(motionDOF:numFrames())
   local rightFoot=boolN(motionDOF:numFrames())

   file:unpack(leftFoot)
   file:unpack(rightFoot)

   return motionDOF, dmotionDOF, leftFoot, rightFoot
end
  
function math.findNearest(v, pos, start, endFrame)
   local argMin=start
   local minDist=v(start):distance(pos)
   for i=start, endFrame-1 do
      local dist=v(i):distance(pos)
      
      if dist<minDist then
	 minDist=dist
	 argMin=i
      end
   end
   return argMin, minDist
end

--class 'stransfN'
stransfN=LUAclass()
function stransfN:__init(N)
   self.scale=vectorn(N)
   self.rotation=quaterN(N)
   self.translation=vector3N(N)
end

function stransfN:__call(i)
   return stransf(self.scale(i), self.rotation(i), self.translation(i))
end

function stransfN:set(i, b)
   self.scale:set(i, b.scale)
   self.rotation(i):assign(b.rotation)
   self.translation(i):assign(b.translation)
end

function InterframeDifference.recoverPose(poseInout)
   -- 0,1: dv
   -- 2: dq
   -- 3: offset_y
   -- 4,5,6: offset_q
   local offset=poseInout:toVector3(4)
   local pelvis=transf()
   pelvis.rotation:setRotation(offset)
   pelvis.translation:setValue(0,poseInout(3),0)

   MotionDOF.setRootTransformation(poseInout,pelvis)
end

function InterframeDifferenceC1.recoverPose(poseInout)
   -- 0,1: dv
   -- 2: dq
   -- 3: offset_y
   -- 4,5: offsetQ ("ZX")
   -- 6: offset_qy

   local offset=poseInout:toVector3(4)
   local pelvis=transf()
   pelvis.rotation:setRotation("ZX", offset)
   pelvis.translation:setValue(0, poseInout(3),0)

   MotionDOF.setRootTransformation(pelvis, poseInout)
end

--class 'MotionGraph'
MotionGraph=LUAclass()

-- a motion graph is a table containing segments.
-- e.g. graph={ jump={ mot=mot1, dmot=dmot1}, walk1={ mot=mot2, dmot=dmot2}, ...}

function MotionGraph:createSegment(first, last, name, grpName)
   local seg= {}
   seg.graph=self
   seg.first=first
   seg.last=last
   seg.name=name
   seg.grpName=grpName
   return seg
end


function MotionGraph:createSegmentsFromDiscontinuity(discontinuity)
   self:setBones()

    local segf=SegmentFinder(discontinuity)

   for i=0, segf:numSegment()-1 do
      first=segf:startFrame(i)
      last=segf:endFrame(i)-1
   

       local name=tostring(first).."_"..tostring(last)
       print("seg", first, last, name)
	self:setSegment("motions", name, first,last)
   end

end

function MotionGraph:__finalize()
   self.skel=nil
   self.mot=nil
   self.dmot=nil
   self.id=nil
   self.lmot=nil
end

function MotionGraph:__init(skel, mot, dmot, useID_C1, binaryFile)

   self.skel=skel
   self.mot=mot
   self.dmot=dmot
   self.useID_C1=useID_C1
   self.id=InterframeDifference()
   self.groups=vector()
   if useID_C1==true then
      self.id=InterframeDifferenceC1(self.mot.dofInfo:frameRate())
   end

   self.lmot=MotionDOF(self.mot)
   
   if binaryFile~=nil then
      self:unpack(binaryFile)
      return
   end

   if useID_C1==true then
      id=InterframeDifferenceC1(self.lmot)
      id:exportToDeltaRep(self.lmot)
   else
      self.lmot:convertToDeltaRep()			
   end
   
   
   local offset=quater()
   local nf=self.mot:numFrames()
   self.rotY=quaterN(nf)
   
   -- calc rot_y		
   for i=0,nf-1 do 
      local tf=self.mot:rootTransformation(i)
      local rot_y=self.rotY(i)
      tf.rotation:decompose(rot_y, offset)
   end
end

function MotionGraph:pack(binaryFile)
   -- pack root trajectory.
   binaryFile:pack(self.lmot:matView():range(0, self.lmot:numFrames(), 0, 7))
   binaryFile:pack(self.rotY)
end

function MotionGraph:unpack(binaryFile)
   binaryFile:unpack(self.lmot:matView():range(0, self.lmot:numFrames(), 0, 7))
   self.rotY=quaterN()
   binaryFile:unpack(self.rotY)
end

function MotionGraph:loadSegment(file)
   local first=file:unpackInt()
   local last=file:unpackInt()
   local name=file:unpackStr()
   local grpName=file:unpackStr()
   
   if self[grpName]==nil then
      self[grpName]={}
      self.groups:pushBack(grpName)
   end
   local seg
   seg=self:createSegment(first, last, name, grpName, false)
   seg:unpack(file)

   self[grpName][name]=seg
end
   
function MotionGraph:setSegment(grpName, name,first, last)
   if self[grpName]==nil then
      self[grpName]={}
      self.groups:pushBack(grpName)
   end

   local seg
   if self.fromFile==true then
      seg=self[grpName][name]

      assert(seg.first==first)
      assert(seg.last==last)
      assert(seg.name==name)
      assert(seg.grpName==grpName)
   else
      seg=self:createSegment(first, last, name, grpName)
   end

   self[grpName][name]=seg
   return seg
end

function MotionGraph:setBones()
   MotionLoader.setVoca(self.skel, model.bones)
end

function MotionGraph:createHoppingGraph()
   local mot=self.mot
   local dmot=self.dmot
   local motions=self.motions
   
   dbg.console()
   self.lfoot=self.skel:getBoneByName("lfoot")
   self.rfoot=self.skel:getBoneByName("rfoot")
   self.lknee=self.skel:getBoneByName("ltibia")
   self.rknee=self.skel:getBoneByName("rtibia")
   self.lfootpos=vector3(0,-0.03,0.11)
   self.rfootpos=vector3(0,-0.03,0.11)
   self.chest=self.skel:getBoneByName("lowerback")

   self:setSegment("motions", "stand", 405,482)
   self:setSegment("motions", "stand2", 405,482)
   self:setSegment("motions", "stand_short", 455,482)
   self:setSegment("motions", "stand_to_jump", 1, 75)
   self:setSegment("motions", "COM_up", 75, 107)
   self:setSegment("motions", "jump_up", 107, 142)
   self:setSegment("motions", "jump_down", 142, 179)
   self:setSegment("motions", "jump_after", 179, 274)

   local motions=self.motions
   
   -- graph construction
   motions.stand.next=motions.stand_short -- jump
--   motions.stand.next=motions.stand -- standing test
   motions.stand_short.next=motions.stand_to_jump
	
   motions.stand_to_jump.next=motions.COM_up
   motions.COM_up.next=motions.jump_up
   motions.jump_up.next=motions.jump_down
   motions.jump_down.next=motions.jump_after
--   motions.jump_after.next=motions.stand
   motions.jump_after.next=motions.stand2
   motions.stand2.next=motions.stand

   assert(self.rightFoot:size()>0)

   -- manual constraint marking
   self.leftFoot:range(motions.jump_after.first, 
			motions.jump_after.last+1):setAllValue(true)
   self.rightFoot:range(motions.jump_after.first, 
			motions.jump_after.last+1):setAllValue(true)

 
   self.types={}
   self.types.JUMP=1

   self.initialSegment=motions.stand

   self.transitionFunction=function (graph, desiredType, currSegment)
			      if desiredType~=nil then
				 if currSegment==graph.motions.stand and desiredType==graph.types.JUMP then
				    currSegment=graph.motions.stand_to_jump
				    desiredType=nil
				 else
				    currSegment=currSegment.next
				 end
			      else
				 currSegment=currSegment.next
			      end

			      return desiredType, currSegment
			   end
end

function MotionGraph:createSegmentsFromFile()

   if useCase.mot_file~=nil and
      util.isFileExist(str.left(useCase.mot_file,-3).."graph") then
      local graphFn=str.left(useCase.mot_file,-3).."graph"
      local graphFile=util.BinaryFile()
      graphFile:openRead(graphFn)

      while graphFile:unpackInt()==1  do
	 local f=graphFile:unpackInt()
	 local l=graphFile:unpackInt()
	 local name=graphFile:unpackStr()
	 local grpName=graphFile:unpackStr()
	 self:setSegment(grpName, name, f, l)
	 print(grpName, name, f, l)
      end
      return true
   end

   return false
end

function MotionGraph:beginGroup(grp1, grp2)
   self._temporary={}
   self._temporary.grp1=self[grp1]

   if grp2==nil then
      self._temporary.grp2=self._temporary.grp1
   else
      self._temporary.grp2=self[grp2]
   end
end

function MotionGraph:dupSegment(grp, newseg, _refseg)
   return self:setSegment(grp, newseg, _refseg.first, _refseg.last)
end



function MotionGraph:connect(seg1, seg2)
   local grp1=self._temporary.grp1
   local grp2=self._temporary.grp2

   if grp1~=nil and grp2~= nil then
      local s1=grp1[seg1]
      local s2=grp2[seg2]
      if s1~=nil and s2~=nil then
	 s1.next=s2
      end
   end   
end
function MotionGraph:Hopping2_createLinks()
   local create2DownSegments=self.create2DownSegments
   local stand=self.stand

   for k,v in pairs(stand) do
      v.next=v
   end

   local hopping1=self.hopping1
   local hopping2=self.hopping2

   if scenario==scenarios.JUMP_UP then
      stand.stand_short.next=hopping1.stand_to_jump
   elseif scenario==scenarios.JUMP_FORWARD2 then
      stand.stand_short.next=hopping2.stand_to_jump
   else
      -- modify stand segment.
      for k,sstand in pairs(stand) do

	 local len=sstand:length()
	 local offset=vectorn()
	 offset:linspace(0, len, len+1)

	 local footForego=0 -- didn't work
	 local map1=math.linearCurve({0,len},
				     {footForego,len+footForego})

	 if str.right(sstand.name,4)=="tail" then
	    map1=math.linearCurve({0,len}, {0,len})
	 end
	 sstand.footRefL:assign(map1-offset)
	 sstand.footRefR:assign(map1-offset)
      end
   end
   
   local hopping
   for ihop=1,2 do 
      if ihop==1 then
	 hopping=hopping1
      else
	 hopping=hopping2
      end
      if hopping~=nil then 
	 hopping.stand_to_jump.next=hopping.COM_down
	 hopping.COM_down.next=hopping.jump_up

	 if create2DownSegments then
	    hopping.jump_up.next=hopping.jump_down1
	    hopping.jump_down1.next=hopping.jump_down2
	    hopping.jump_down2.next=hopping.jump_after
	 else
	    hopping.jump_up.next=hopping.jump_down
	    hopping.jump_down.next=hopping.jump_after
	 end
	 hopping.jump_after.next=stand.stand_short

	 
	 
	 local jump_after=hopping.jump_after

	 do
	    if create2DownSegments then
	       local jump_down1=hopping.jump_down1
	       local jump_down2=hopping.jump_down2
	       
	       -- modify jump_down segment.
	       local len=jump_down1:length()
	       local len2=jump_down2:length()
	       local len3=jump_after:length()

--	       local map1=math.linearCurve({0,len+len2, len+len2+len3},
--					   {0,len+len2+len3/2, len+len2+len3})

	       local key2=len+len2
	       local key3=key2+len3
	       local map1=math.linearCurve({0,key2, key2, key2+len3/3,key3},
	       				   {0,key2+len3/2, key2, key3, key3})

	       local function setFootRef(seg, map, offsett)
		  local offset=vectorn()
		  local slen=seg:length()
		  offset:linspace(offsett, slen+offsett, slen+1)
		  local ref=map:range(offsett, slen+offsett+1)-offset
		  seg.footRefL:assign(ref)
		  seg.footRefR:assign(ref)
	       end

	       setFootRef(jump_down1, map1, 0)
	       setFootRef(jump_down2, map1, len)
	       setFootRef(jump_after, map1, len+len2)

	    else
	       local jump_down=hopping.jump_down

	       -- modify jump_down segment.
	       local len=jump_down:length()
	       local len2=jump_down.next:length()
	       local offset=vectorn()
	       offset:linspace(0, len, len+1)

	       local map1=math.linearCurve({0,len},
					   {0,len+len2/2})
	       jump_down.footRefL:assign(map1-offset)
	       jump_down.footRefR:assign(map1-offset)
	       do
		  -- modify jump_after segment
		  local seg=jump_after
		  len=seg:length()
		  local offset=vectorn()
		  offset:linspace(0,len,len+1)
		  local map2=math.simpleSpline({0,len}, {len/2, len+len/2})
		  seg.footRefL:assign(map2-offset)
		  seg.footRefR:assign(map2-offset)
	       end

	    end
	 end

      end
   end
self.types={}
   self.types.STAND=1

   if scenario==scenarios.STAND1 then
      self.initialSegment=stand.stand1
   elseif scenario==scenarios.STAND2 then
      self.initialSegment=stand.stand2
   elseif scenario==scenarios.STAND3 then
      self.initialSegment=stand.stand3
   elseif scenario==scenarios.STAND4 then
      self.initialSegment=stand.stand4
   else
      self.initialSegment=stand.stand_short
   end

   self.transitionFunction=function (graph, desiredType, currSegment)
			      currSegment=currSegment.next
			      return desiredType, currSegment
			   end

end
function MotionGraph:Hopping2_createSegments()
   local create2DownSegments=self.create2DownSegments
   self:setSegment("stand", "stand1", 1,4594)
   self:setSegment("stand", "stand2", 4693,7243)
   self:setSegment("stand", "stand3", 7288, 11562)
   self:setSegment("stand", "stand3_tail", 11562, 11622)
   self:setSegment("stand", "stand4", 11664,15780)
   self:setSegment("stand", "stand4_tail", 15780, 15840)
   self:setSegment("hopping1", "stand_to_jump", 16313, 16415)
   self:setSegment("hopping1", "COM_down", 16415, 16482)
   self:setSegment("hopping1", "jump_up",16482, 16520)
   if create2DownSegments then
      self:setSegment("hopping1", "jump_down1",16520, (16520+16556)/2)
      self:setSegment("hopping1", "jump_down2",(16520+16556)/2, 16556)
   else
      self:setSegment("hopping1", "jump_down",16520, 16556)
   end

   self:setSegment("hopping1", "jump_after", 16556, 16697)

   --    hopping 1               hopping 2
   -- touch-up -  touchdown  touch-up   touch-down
   -- 16515:16520:16556 == 19026:19033:19077
   -- hopping2 segments are the same as the forward2 segment when merged.
   self:setSegment("hopping2", "stand_to_jump",16373, 16415)
   self:setSegment("hopping2", "COM_down",18917, 18992)
   self:setSegment("hopping2", "jump_up",18992, 19033)

   if create2DownSegments then
      self:setSegment("hopping2", "jump_down1",19033, (19033+19077)/2)
      self:setSegment("hopping2", "jump_down2",(19033+19077)/2, 19077)
   else
      self:setSegment("hopping2", "jump_down",19033, 19077)
   end
   self:setSegment("hopping2", "jump_after",19077, 19185) 

   self:setSegment("hopping", "forward",18201, 18477)
   self:setSegment("hopping", "backward", 18515, 18808)
   self:setSegment("hopping", "forward2",18870, 19174)
   self:setSegment("hopping", "backward2",19199, 19441)
   self:setSegment("stand", "stand_short", 16652,16789)

end
function MotionGraph:createHoppingGraph2()
   local mot=self.mot
   local dmot=self.dmot
   local motions=self.motions
   
   self.create2DownSegments=true
   self.lfoot=self.skel:getBoneByName("lfoot")
   self.rfoot=self.skel:getBoneByName("rfoot")
   self.lknee=self.skel:getBoneByName("ltibia")
   self.rknee=self.skel:getBoneByName("rtibia")
   self.lfootpos=vector3(0,-0.03,0.11)
   self.rfootpos=vector3(0,-0.03,0.11)
   self.chest=self.skel:getBoneByName("lowerback")

   if not self:createSegmentsFromFile() then 
      self:Hopping2_createSegments()
   end      


   self:Hopping2_createLinks()
   
end

function MotionGraph:projectJointAngles(jointname)

   local bone=self.skel:getBoneByName(jointname):treeIndex()
   local startR=self.skel.dofInfo:startR(bone)
   local endR=self.skel.dofInfo:endR(bone)
   for k,v in pairs(self.motions) do
--      local plot=math.gnuPlotQueue(jointname.."___"..k, 2, jointname.."_"..k)


      for i=startR, endR-1 do
--	 plot:plotSignal(v:mot():column(i), "before"..i-startR)
	 math.alignAngles(v:mot():column(i),0)
	 math.alignAngles(v:lmot():column(i),0)
--	 plot:plotSignal(v:mot():column(i), "after"..i-startR)
      end
   end
end

function MotionGraph:plotJointAngles(jointname)

   local bone=self.skel:getBoneByName(jointname):treeIndex()
   local startR=self.skel.dofInfo:startR(bone)
   local endR=self.skel.dofInfo:endR(bone)
   for k,v in pairs(self.motions) do

      local plot=math.gnuPlotQueue(jointname.."_"..k, 2, jointname.."_"..k)

      for i=startR, endR-1 do
	 plot:plotSignal(v:mot():column(i), "dof"..i-startR)
      end
   end

end


function MotionGraph:createWalkingGraph()
   
   
   local mot=self.mot
   local dmot=self.dmot

   self.lfootpos=vector3(0,-0.07,0.09)
   self.rfootpos=vector3(0,-0.07,0.09)
   self.lheelpos=vector3(0,-0.07,0.02)
   self.rheelpos=vector3(0,-0.07,0.02)
   
   if self:createSegmentsFromFile() then 
      if self.motions.stand==nil then
	 
	 if self.motions.stand_short~= nil then
	    self:dupSegment("motions",  "stand",self.motions.stand_short)
	 else
	    self:dupSegment("motions", "stand", self.motions.SR)
	 end
      end
   else

      --   self:setSegment("motions", "stand",60,107) --modified from 60,77 
      --   self:setSegment("motions", "stand", 11343,11416)
      self:setSegment("motions", "stand", 11370*4, 11416*4)

      self:setSegment("motions", "stand_short", 90*4, 107*4)
      self:setSegment("motions", "SRDL",107*4, 151*4)		
      self:setSegment("motions", "RDL",352*4, 370*4)		
      self:setSegment("motions", "LDR",370*4, 388*4)		
      self:setSegment("motions", "RDLF",646*4, 666*4)		

      self:setSegment("motions", "FRF",716*4, 728*4)		
      --self:setSegment("motions", "FRDL",980, 1008)
      self:setSegment("motions", "FR",980*4, 991*4)
      self:setSegment("motions", "FLF",728*4, 739*4)
      --      self:setSegment("motions", "SRDLF", 12891*4, 12916*4) -- 6927, 

      
      self:setSegment("motions", "SR", 360, 494)
      self:setSegment("motions", "RD", 51575,51610)
      self:setSegment("motions", "DLF", 51610, 51670)
   end

   self:beginGroup("motions")
   local motions=self.motions
   -- graph construction (only default transitions are listed here)
   if scenario==scenarios.WALK then
      self:connect("stand", "stand_short")
      self:connect("stand_short", "SRDL")
   elseif scenario==scenarios.RUN then
      self:connect("stand", "SR")
      self:connect("SR", "RD")
      self:connect("RD", "DLF")
   else
      self:connect("stand", "stand")
   end
   
   self:connect("SRDL","LDR")
   self:connect("LDR","RDL")
   self:connect("RDL","LDR")
   self:connect("RDLF","FRF")
   --motions.FRDL.next=motions.LDR		
   self:connect("FR","RDL")
   self:connect("FLF","FRF")
   self:connect("FRF","FLF")	
   self:connect("DLF","FRF")

   if scenario==scenarios.RUN then
      local len={}
      local mot=self.motions
      len[1]=mot.SR:length()
      len[2]=mot.RD:length()
      len[3]=mot.DLF:length()
      local key={0, len[1], len[1]+len[2], len[1]+len[2]+len[3]}
      key35=(key[3]+key[4])/2
      --                                S     R      D        LF
      local mapL=math.linearCurve(key, {0, key[2], key35, key35})
      local mapR=math.linearCurve(key, {0, key[2], key[2], key[4]})
            
      -- math.plotVec(mapL)
      -- math.plotVec(mapR,true)
      local function setFootRef(seg, isL, map, offsett)
	 local offset=vectorn()
	 local slen=seg:length()
	 offset:linspace(offsett, slen+offsett, slen+1)
	 local ref=map:range(offsett, slen+offsett+1)-offset

	 if isL then
	    seg.footRefL:assign(ref)
	 else
	    seg.footRefR:assign(ref)
	 end
      end

      setFootRef(mot.SR, true, mapL, key[1])
      setFootRef(mot.RD, true, mapL, key[2])
      setFootRef(mot.DLF, true, mapL, key[3])
      setFootRef(mot.SR, false, mapR, key[1])
      setFootRef(mot.RD, false, mapR, key[2])
      setFootRef(mot.DLF, false, mapR, key[3])

   end

   self.types={}
   self.types.NONE=0
   self.types.WALK=1
   self.types.RUN=2
   self.types.STAND=3

   self.initialSegment=motions.stand

   self.transitionFunction=function (graph, desiredType, currSegment)
			      if desiredType~=nil then
				 if currSegment==graph.motions.LDR and desiredType==graph.types.RUN then
				    currSegment=graph.motions.RDLF
				    desiredType=nil
				 elseif currSegment==graph.motions.FLF and graph.types.desiredType==graph.types.WALK then
				    --self.currSegment=self.graph.motions.FRDL
				    currSegment=graph.motions.FR
				    desiredType=nil
				 elseif currSegment==graph.motions.stand and desiredType==graph.types.WALK then
				    currSegment=graph.motions.SRDL
				    desiredType=nil
				 else
				    currSegment=currSegment.next
				 end
			      else
				 currSegment=currSegment.next
			      end

			      return desiredType, currSegment
			   end
end

-- segmented at contact force maximum
function MotionGraph:createWalkingGraph2()
   
   
   local mot=self.mot
   local dmot=self.dmot
   local motions=self.motions
   
   self.lfootpos=vector3(0,-0.07,0.09)
   self.rfootpos=vector3(0,-0.07,0.09)
   self.lheelpos=vector3(0,-0.07,0.02)
   self.rheelpos=vector3(0,-0.07,0.02)

--   self:setSegment("motions", "stand",60,127) --modified from 60,77 
--   self:setSegment("motions", "stand", 11343,11416)
   self:setSegment("motions", "stand", 11370,11416)

--   self:setSegment("motions", "stand", 344,362)
   self:setSegment("motions", "SRD",127, 139)		

   -- manual adjust of foot contact states

   if self.rightFoot:size() >0 then
      print("adjust")
      self.rightFoot:set(344, true)
      self.rightFoot:set(345, true)
      self.leftFoot:set(362, true)

   end


   self:setSegment("motions", "DRD",344, 362)
   self:setSegment("motions", "DLD",362, 380)		
   self:setSegment("motions", "DRL",638, 658)		
   self:setSegment("motions", "LFR",710, 722)		
   self:setSegment("motions", "RFL",722, 734)
   --self:setSegment("motions", "FRDL",980, 1008)
   self:setSegment("motions", "RD",987, 1000)

   local motions=self.motions
   -- graph construction (only default transitions are listed here)

   motions.SRD.next=motions.DLD
   motions.DLD.next=motions.DRD
   motions.DRD.next=motions.DLD
   motions.RD.next=motions.DLD
   motions.DRL.next=motions.LFR
   motions.LFR.next=motions.RFL
   motions.RFL.next=motions.LFR

   if scenario==scenarios.WALK then
      motions.stand.next=motions.SRD
   else
      motions.stand.next=motions.stand
   end


   self.types={}
   self.types.NONE=0
   self.types.WALK=1
   self.types.RUN=2
   self.types.STAND=3

   self.initialSegment=motions.stand

   self.transitionFunction=function (graph, desiredType, currSegment)
			      if desiredType~=nil then
				 if currSegment==graph.motions.DLD and desiredType==graph.types.RUN then
				    currSegment=graph.motions.DRL
				    desiredType=nil
				 elseif currSegment==graph.motions.LFR and graph.types.desiredType==graph.types.WALK then
				    --self.currSegment=self.graph.motions.FRDL
				    currSegment=graph.motions.RD
				    desiredType=nil
				 elseif currSegment==graph.motions.stand and desiredType==graph.types.WALK then
				    currSegment=graph.motions.SRD
				    desiredType=nil
				 else
				    currSegment=currSegment.next
				 end
			      else
				 currSegment=currSegment.next
			      end

			      return desiredType, currSegment
			   end
   
end

-- segmented based on foot contacts
function MotionGraph:createWalkingGraph3()
   local mot=self.mot
   local dmot=self.dmot

   self.lfootpos=vector3(0,-0.03,0.09)
   self.rfootpos=vector3(0,-0.03,0.09)
   self.lheelpos=vector3(0,-0.03,0.02)
   self.rheelpos=vector3(0,-0.03,0.02)

   if self:createSegmentsFromFile() then 
      if self.motions==nil then
	 self:dupSegment("motions", "stand", self.StR.SR)
      end
   else

      self:setSegment("motions", "stand", 11370*4, 11416*4)
      self:setSegment("motions", "stand_short", 90*4, 107*4)
      

      -- S R D L F R F L F R 
      -- D: 51610
      -- F: 51679
      -- F: 51722
      self:setSegment("StR", "SR", 360, 491) 
      self:setSegment("StR", "RD", 51572,51610) 
      self:setSegment("StR", "DL", 51610, 51650) 
      self:setSegment("StR", "LF", 51650, 51679) 
      self:setSegment("StR", "FR", 51679, 51700) 

      self:setSegment("run", "FR",2871,2895)
      self:setSegment("run", "RF",2895,2920)
      self:setSegment("run", "FL",2920,2945)
      self:setSegment("run", "LF",2945,2970)
   end
   self:beginGroup("motions", "StR")
   self:connect("stand", "SR")
   self:beginGroup("StR")
   self:connect("SR", "RD")
   self:connect("RD", "DL")
   self:connect("DL", "LF")
   self:connect("LF", "FR")
   self:beginGroup("StR", "run")
   self:connect("FR", "RF")
   self:beginGroup("run")
   self:connect("RF", "FL")
   self:connect("FL", "LF")
   self:connect("LF", "FR")
   self:connect("FR", "RF")

   self:setRefTiming()

   self.types={}
   self.types.NONE=0
   self.types.WALK=1
   self.types.RUN=2
   self.types.STAND=3

   self.initialSegment=self.motions.stand

   self.transitionFunction=function (graph, desiredType, currSegment)
			      currSegment=currSegment.next
			      return desiredType, currSegment
			   end

end

-- segmented based on foot contacts
function MotionGraph:createWalkingGraph4()
   local mot=self.mot
   local dmot=self.dmot

   self.lfootpos=vector3(0,-0.03,0.09)
   self.rfootpos=vector3(0,-0.03,0.09)
   self.lheelpos=vector3(0,-0.03,0.0)
   self.rheelpos=vector3(0,-0.03,0.0)

   if self:createSegmentsFromFile() then 
      if self.motions==nil then
	 self:dupSegment("motions", "stand", self.StR.SR)
      end
   else

      self:setSegment("motions", "stand", 1, 11)
      self:setSegment("motions", "stand_short", 1, 11)
      

      -- S R D L F R F L F R 
      -- D: 51610
      -- F: 51679
      -- F: 51722
      self:setSegment("StR", "SR", 1, 60) 
      self:setSegment("StR", "RD", 60,81) 
      self:setSegment("StR", "DL", 81, 100) 
      self:setSegment("StR", "LF", 100, 129) 
      self:setSegment("StR", "FR", 129, 154) 

      self:setSegment("run", "FR",230,255)
      self:setSegment("run", "RF",255,281)
      self:setSegment("run", "FL",281,304)
      self:setSegment("run", "LF",304,329)
   end
   self:beginGroup("motions", "StR")
   self:connect("stand", "SR")
   self:beginGroup("StR")
   self:connect("SR", "RD")
   self:connect("RD", "DL")
   self:connect("DL", "LF")
   self:connect("LF", "FR")
   self:beginGroup("StR", "run")
   self:connect("FR", "RF")
   self:beginGroup("run")
   self:connect("RF", "FL")
   self:connect("FL", "LF")
   self:connect("LF", "FR")
   self:connect("FR", "RF")

   self:setRefTiming()

   self.types={}
   self.types.NONE=0
   self.types.WALK=1
   self.types.RUN=2
   self.types.STAND=3

   self.initialSegment=self.motions.stand

   self.transitionFunction=function (graph, desiredType, currSegment)
			      currSegment=currSegment.next
			      return desiredType, currSegment
			   end

end


function MotionGraph:createGraphFromFile(grcFileName, grpName)
   

   local mot=self.mot
   local dmot=self.dmot
   local motions=self.motions
   
   self.lfootpos=vector3(0,-0.07,0.09)
   self.rfootpos=vector3(0,-0.07,0.09)
   
   grc=MotionUtil.ParseGrcFile(grcFileName)			
   
   for i=0, grc:numSegment()-1 do
      
      if grc:class(i, "ignore")=="others" then		
	 
	 local tn=grc:typeName(i)
	 
	 if grpName==nil or grpName==tn then
	    local st=grc:startTime(i)
	    local et=grc:endTime(i)
	    
	    
	    --				print(st, et, tn)
	    
	    if self[tn]==nil then
	       self[tn]={}
	    end
	    
	    --(grpName, name,first, last)			
	    self:setSegment(tn, i, st, et)
	 end
      end
   end
end

-- input: global model, scenario
function MotionGraph:createGraph(filename, fromFile)

   self:setBones()
   self.lfoot=self.skel:getBoneByVoca(MotionLoader.LEFTANKLE)
   self.rfoot=self.skel:getBoneByVoca(MotionLoader.RIGHTANKLE)
   self.lknee=self.skel:getBoneByVoca(MotionLoader.LEFTKNEE)
   self.rknee=self.skel:getBoneByVoca(MotionLoader.RIGHTKNEE)

   local file
   print("createGraph", filename, fromFile)
   if filename~=nil then
      self.fromFile=fromFile
      file=util.BinaryFile()

      if self.fromFile then
	 file:openRead(filename)
      else
	 file:openWrite(filename)
      end
   else
      self.fromFile=nil
   end

   self.name=filename

   if self.fromFile==true then
      while file:unpackInt()==1 do
	 self:loadSegment(file)
      end
   end

   if scenario==scenarios.JUSTIN_RUN then
      self:createWalkingGraph4()
   elseif model==model_files.hyunwoo_real_cart or model==model_files.hyunwoo_full_cart then

      if scenario==scenarios.RUN2 or scenario==scenarios.RUNF2 then
	 self:createWalkingGraph3()
      else
	 self:createWalkingGraph()
	 print("hihihihihi")
	 self:projectJointAngles("RightShoulder") -- there exists singularity so...
	 self:plotJointAngles("RightShoulder")
      end
   elseif model==model_files.justin_jump_cart or
      model==model_files.justin_jump or
      model==model_files.justin_jump2 then
      self:createHoppingGraph2()
   else
      self:createHoppingGraph()
   end

   if self.fromFile==false then
      for i,grpName in ipairs(self.groups.tf) do
	 for k,seg in pairs(self[grpName]) do
	    seg:calculateMembers()
	    file:packInt(1)
	    seg:pack(file)
	 end
      end
      file:packInt(0)
   end


   file:close()
   file=nil
end

function MotionGraph:createIKsolver2()
	require('RigidBodyWin/subRoutines/ikSolver')
	local effectors={}
	effectors[1]={self.lfoot, self.lfootpos}
	effectors[2]={self.rfoot, self.rfootpos}

	local ik={}
	if useCase.keyframes.importanceLH then
		effectors[3]={self.lhand, self.lhandpos}
		effectors[4]={self.rhand, self.rhandpos}
		ik.solver=IKsolver:new(self.skel, effectors, 
		{{self.lknee,1,170}, {self.rknee,1,170}, {self.lelbow, 1,170}, {self.relbow,1,170}})
	else
		ik.solver=IKsolver:new(self.skel, effectors, {{self.lknee,5,170}, {self.rknee,5,170}})
	end

	function ik:solve(rotY, roottf, pose,...) 
		self.solver:solve(rotY,roottf, pose,...)
	end
	return ik
end

function MotionGraph:createIKsolver_COM(useHand, skel)
	skel=skel or self.skel
   local ik={}
   ik.effectors=MotionUtil.Effectors()
   local dim=2
   if useHand then dim=4 end
   ik.effectors:resize(dim)
   ik.axisSign=vectorn(dim)
   
   local lfoot=self.bone2.lfoot
   local rfoot=self.bone2.rfoot
   local lknee=self.bone2.lknee
   local rknee=self.bone2.rknee
   ik.effectors(0):init(lfoot,self.lfootpos)
   ik.effectors(1):init(rfoot,self.rfootpos)
   ik.axisSign:set(0,1)
   ik.axisSign:set(1,1)

	if useHand then
		local lhand=self.bone2.lhand
		local rhand=self.bone2.rhand
		local lelbow=self.bone2.lelbow
		local relbow=self.bone2.relbow

		ik.effectors(2):init(lhand,self.lhandpos)
		ik.effectors(3):init(rhand,self.rhandpos)
		ik.axisSign:set(2,-1)
		ik.axisSign:set(3,-1)
		ik.solver=COM_IKsolver(skel,ik.effectors, CT.ivec(lknee:treeIndex(), rknee:treeIndex(), lelbow:treeIndex(), relbow:treeIndex()), ik.axisSign)
	else
		ik.solver=COM_IKsolver(skel,ik.effectors, CT.ivec(lknee:treeIndex(), rknee:treeIndex() ) ,ik.axisSign)
	end
	ik.conPos=vector3N(dim)
	ik.conDelta=quaterN(dim)
	ik.importance=vectorn(dim)
	
	--if iterativeIK~=nil then  
	ik.fk=BoneForwardKinematics(skel)
	ik.fk:init()
	ik.compareChain=CompareChain(skel, ik.fk, skel:fkSolver())
	ik.skel=skel
	--end
	
	-- rotY is used for defining foot global orientations.
	function ik:solve(rotY, roottf, lfoot, rfoot, lhand, rhand, pose, desiredCOM)
		assert(pose~=nil)
		self.conPos(0):assign(lfoot[1])
		self.conPos(1):assign(rfoot[1])
		self.conDelta(0):assign(quater(1,0,0,0))
		self.conDelta(1):assign(quater(1,0,0,0))
		self.importance:set(0, lfoot[3])
		self.importance:set(1, rfoot[3])
		if lhand then
			self.conPos(2):assign(lhand[1])
			self.conPos(3):assign(rhand[1])
			-- conori
			self.conDelta(2):assign(quater(1,0,0,0))
			self.conDelta(3):assign(quater(1,0,0,0))
			self.importance:set(2, lhand[3])
			self.importance:set(3, rhand[3])
		end
		self.solver:IKsolve(pose, rotY, roottf, self.conDelta, self.conPos, self.importance, desiredCOM)
	end
	function ik:solve2(rotY, roottf, lfoot, rfoot, lhand, rhand, pose, desiredCOM)
		-- reuse recent conDelta and importance
		assert(pose~=nil)
		self.conPos(0):assign(lfoot)
		self.conPos(1):assign(rfoot)
		if lhand then
			self.conPos(2):assign(lhand)
			self.conPos(3):assign(rhand)
		end
		self.solver:IKsolve(pose, rotY, roottf, self.conDelta, self.conPos, self.importance, desiredCOM)
	end
	return ik
end

function MotionGraph:createIKsolver(useHand, skel)
	skel=skel or self.skel
	local ik={}
	ik.effectors=MotionUtil.Effectors()
	local dim=2
	if useHand then
		ik.effectors:resize(4)
		dim=4
	else
		ik.effectors:resize(2)
	end

	local lfoot=self.bone2.lfoot
	local rfoot=self.bone2.rfoot
	local lknee=self.bone2.lknee
	local rknee=self.bone2.rknee
   assert(lfoot)
   assert(rfoot)
	ik.effectors(0):init(lfoot,self.lfootpos)
	ik.effectors(1):init(rfoot,self.rfootpos)

	--   ik.solver=MotionUtil.createFullbodyIkDOF_limbIK(skel.dofInfo, ik.effectors, lknee, rknee)

	if useHand then

		local lhand=self.bone2.lhand
		local rhand=self.bone2.rhand
		local lelbow=self.bone2.lelbow
		local relbow=self.bone2.relbow

		ik.effectors(2):init(lhand,self.lhandpos)
		ik.effectors(3):init(rhand,self.rhandpos)
		if useCase.useLimbIKsolver2 then
			ik.solver=LimbIKsolver2(skel.dofInfo,ik.effectors, CT.ivec(lknee:treeIndex(), rknee:treeIndex(), lelbow:treeIndex(), relbow:treeIndex()), CT.vec(1,1,-1,-1))
			for i,v in ipairs(useCase.useLimbIKsolver2 ) do
				ik.solver:setOption(v)
			end
			-- 디폴트 값은 바꾸지 마세요. 여러사람이 사용하는 코드니까요.
			-- 모든 개인 설정은 useCase파일에서!
			local value=useCase.limbIKsolver2_value or {1,5,0.3,5}
			ik.solver:setValue(unpack(value))
		else
			ik.solver=LimbIKsolver(skel.dofInfo,ik.effectors, CT.ivec(lknee:treeIndex(), rknee:treeIndex(), lelbow:treeIndex(), relbow:treeIndex()), CT.vec(1,1,-1,-1))
		end
	else
		ik.solver=LimbIKsolver(skel.dofInfo,ik.effectors, CT.ivec(lknee:treeIndex(), rknee:treeIndex()), CT.vec(1,1))
	end
   ik.conPos=vector3N(dim)
   ik.conDelta=quaterN(dim)
   ik.importance=vectorn(dim)
   
   --if iterativeIK~=nil then  
   ik.fk=BoneForwardKinematics(skel)
   ik.fk:init()
   ik.compareChain=CompareChain(skel, ik.fk, skel:fkSolver())
   ik.skel=skel
   --end
   
   -- rotY is used for defining foot global orientations.
   function ik:solve(rotY, roottf,pose, lfoot, rfoot )
	   local skel=self.skel
	   if false then
		   -- use IKsolve : importance are ignored
		   assert(pose~=nil)
		   self.conPos(0):assign(lfoot[1])
		   self.conPos(1):assign(rfoot[1])
		   self.solver:IKsolve(pose, rotY, roottf, self.conPos)
	   else
		   -- use IKsolve3
		   local origRootTF=MotionDOF.rootTransformation(pose)
		   local origRotY=quater()
		   local offset=quater()
		   origRootTF.rotation:decompose(origRotY, offset);
		   origRootTF.rotation:mult(rotY, offset);
		   MotionDOF.setRootTransformation(pose, origRootTF)

		   skel:setPoseDOF(pose)
		   -- conpos
		   self.conPos(0):assign(lfoot[1])
		   self.conPos(1):assign(rfoot[1])
		   -- conori
		   self.conDelta(0):assign(self.effectors(0).bone:getFrame().rotation)
		   self.conDelta(1):assign(self.effectors(1).bone:getFrame().rotation)
		   self.importance:set(0, lfoot[2])
		   self.importance:set(1, rfoot[2])
		   self.solver:IKsolve3(pose, roottf, self.conPos, self.conDelta, self.importance)
	   end
   end
   function ik:solve3(rotY, roottf,pose, lfoot, rfoot, lhand, rhand )
	   local skel=self.skel
	   -- use IKsolve3
	   -- conpos
	   self.conPos(0):assign(lfoot[1])
	   self.conPos(1):assign(rfoot[1])
	   -- conori
	   self.conDelta(0):assign(lfoot[2])
	   self.conDelta(1):assign(rfoot[2])
	   self.importance:set(0, lfoot[3])
	   self.importance:set(1, rfoot[3])
	   if lhand then
		   self.conPos(2):assign(lhand[1])
		   self.conPos(3):assign(rhand[1])
		   -- conori
		   self.conDelta(2):assign(lhand[2])
		   self.conDelta(3):assign(rhand[2])
		   self.importance:set(2, lhand[3])
		   self.importance:set(3, rhand[3])
	   end
	   self.solver:IKsolve3(pose, roottf, self.conPos, self.conDelta, self.importance)
   end

   function ik:solve2(rotY, roottf, lfootdelta, rfootdelta, lfoot, rfoot, pose)
      assert(pose~=nil)
      self.conPos(0):assign(lfoot)
      self.conPos(1):assign(rfoot)
      self.conDelta(0):assign(lfootdelta)
      self.conDelta(1):assign(rfootdelta)
      self.solver:IKsolve2(pose, rotY, roottf, self.conDelta, self.conPos)
   end

   function ik:solveIterative(rotY, pelvis, lfootPos, rfootPos, outputPose)
	   local skel=self.skel
      local refPose=outputPose:copy()
      local pelvisOrig=MotionDOF.rootTransformation(outputPose)
      local pelvisEstimate=pelvis:copy()
      self:solve(rotY, pelvis, lfootPos, rfootPos, outputPose)

      
      local tf
      for i=1, iterativeIK.nIter do
	 self.fk:setPoseDOF(outputPose)
	 self.skel:setPoseDOF(refPose)
	 tf=self.compareChain:compare()
	 

	 -- use only orientation errors
	 --local error=quater()
	 --	 error:difference(tf.rotation*pelvisOrig.rotation, pelvisEstimate.rotation)	
	 --	 pelvis.rotation:leftMult(error)

	 -- use both orientation and position errors
	 local error=transf()
	 error:difference(tf*pelvisOrig, pelvisEstimate)
	 pelvis=error*pelvis
	 outputPose:assign(refPose)
	 
	 self:solve(rotY, pelvis, lfootPos, rfootPos, outputPose)
	 
      end
      
      if iterativeIK.drawDebugPose then
	 if self.skin2==nil then
	    self.skin2=RE.createVRMLskin(self.skel, false)
	    self.skin2:scale(100,100,100)
	    self.skin2:setTranslation(0,200,0)
	 end
	 local outputPose2=refPose:copy()
	 
	 MotionDOF.setRootTransformation(outputPose2, tf*pelvisOrig)
	 self.skin2:setPoseDOF(outputPose2)
      end
   end
   return ik
end

