require("IPC_based/invertedPendulum2")
--require("RigidBodyWin/subRoutines/invertedPendulum3")

__globals.useCartPoleBallCpp=true

if __globals.useCartPoleBallCpp==true then
   __globals.SDFASTcartPole_Used=true
end


--class 'CartPoleBall'
CartPoleBall=LUAclass()

function CartPoleBall:__deinit()
   if self.skin~=nil then
      RE.remove(self.skin)
      self.skin=nil
   end
end

function CartPoleBall:__finalize()
   self.skin=nil
   self.simulator=nil
   self.loader=nil

   if self.SDFASTsimulator == true then
      __globals.SDFASTcartPole_Used=nil
   end
end

function CartPoleBall:__init(filename, b,g,dt,q)
   self.loader=VRMLloader(filename)
--   self.loader:printHierarchy()

   self.skin=RE.createVRMLskin(self.loader, false)
   
   self.skin:scale(100,100,100)
   
--   if __globals.SDFASTcartPole_Used==nil then
   if true then
      self.simulator=DynamicsSimulator_SDFAST()
      __globals.SDFASTcartPole_Used=true
      self.SDFASTsimulator=true
   else
--      self.simulator=DynamicsSimulator_UT_penalty()
      self.simulator=DynamicsSimulator_gmbs_penalty()
--      util.msgBox("Warning! SDFAST cannot be used for CartPoleBall class")
   end

   self.simulator:registerCharacter(self.loader)
   
   self.simulator:init(dt, DynamicsSimulator.EULER)
   
   self.skin:setPose(self.simulator,0)
   
   self.simulator:setGVector(vector3(0,g,0))
   
   self.theta=vectorn(6)
   self.theta:setAllValue(0)
   self.theta:set(2,1) -- q_identity= 1,0,0,0
   self.dtheta=vectorn(6)
   self.dtheta:setAllValue(0)
   self.simulator:setLinkData(0, DynamicsSimulator.JOINT_VALUE, self.theta)
   self.simulator:setLinkData(0, DynamicsSimulator.JOINT_VELOCITY, self.dtheta)
   self.simulator:initSimulation()
   
   local cart=self.loader:VRMLbone(1)
   local pole=self.loader:VRMLbone(2)
   self.pole=pole
   self.cart=cart
--   prin(cart:name(), pole:name())
   local M=cart:mass()
   local m=pole:mass()
   local l=pole:localCOM().y
   local i=pole:inertia().x
   self.friction=b
   
--   prin(M,m,l,i)

	self.pendX=InvertedPendulum(M,m,b,i,g,l,dt,q)
	self.pendZ=InvertedPendulum(M,m,b,i,g,l,dt,q)
	self.q=q
	self.mode=1-- velocity control

   self.dt=dt
--   self.pendPositionControl=InvertedPendulum2(M,m,b,i,g,l,dt,q,0)
--   self.pendAngleControl=InvertedPendulum3(M,m,b,i,g,l,dt,q,0)
   self.ori=quater()	-- cart orientation
   self.ori:setValue(1,0,0,0)
   self.desiredVel=vector3(0,0,0)
   self.desiredVelGlobal=vector3(0,0,0)
   self.controlforce=vectorn(6)
   self.Y=matrixn()
   self.numFrame=0
end

function CartPoleBall:setOrientation2(q)
   self.ori:assign(q)
    self:setDesiredParam(self.desiredVelGlobal)
end
function CartPoleBall:setOrientation(angle)

   self.ori:setRotation(vector3(0,1,0), math.rad(angle))
   self:setDesiredParam(self.desiredVelGlobal)
end
function CartPoleBall:projectState()

   local q=self.theta:toQuater(2)
   local w=self.dtheta:toVector3(3)

   
   -- local vt=vector3(0,1,0)
   -- local vtt=q*vt

   -- local q_cleanup=quater()
   -- q_cleanup:axisToAxis(vt, vtt)

   -- local qv=q_cleanup:rotationVector()

   qv=q:rotationVector()

   local limitA=math.rad(60)

   -- qv.y and w.y can not be generated when pole is near upright position.
   qv.y=0
   w.y=0

   if qv.x>limitA then
      qv.x=limitA
      w.x=0
   elseif qv.x<limitA*-1 then
      qv.x= limitA*-1
      w.x=0
   end

   if qv.y>limitA then
      qv.y=limitA
      w.y=0
   elseif qv.y<limitA*-1 then
      qv.y= limitA*-1
      w.y=0
   end

   -- q_cleanup:setRotation(qv)
   -- self.theta:setQuater(2, q_cleanup)
   q:setRotation(qv)
   self.theta:setQuater(2, q)
   self.dtheta:setVec3(3, w)
   self.simulator:setLinkData(0, DynamicsSimulator.JOINT_VALUE, self.theta)
   self.simulator:setLinkData(0, DynamicsSimulator.JOINT_VELOCITY, self.dtheta)
   self.simulator:initSimulation()
end

function CartPoleBall:setState(zmp_pos, com_pos, zmp_vel, com_vel)
   self.theta:set(0, zmp_pos.z)
   self.theta:set(1, zmp_pos.x)
   
   local q=quater()
   q:axisToAxis(vector3(0,1,0), com_pos-zmp_pos)

   self.theta:setQuater(2, q)
   
   -- comPos=zmpPos+q*localCOM
   
   -- comVel= zmp_vel + w.cross(q*localCOM)
   -- lc*w=zmp_vel-com_vel   where lc=tilde(q*localCOM)
   
   local lc=matrix3()
   lc:setTilde(q*self.pole:localCOM())


   -- number of equations: 3, number of unknowns: 3 (although w.y will be clipped later due to linearization)
   -- minimum norm solution.
   local S=matrixn(3,2)
   S:setValue({lc._11, lc._13, lc._21, lc._23, lc._31, lc._33})
   local b=vectorn()
   b:assign({zmp_vel.x-com_vel.x, zmp_vel.y-com_vel.y, zmp_vel.z-com_vel.z})
   local x=vectorn(2)
   math.PIsolve(S,b,x)

   local w=vector3()
   w.x=x(0)
   w.y=0
   w.z=x(1)

   self.dtheta:set(0, zmp_vel.z)
   self.dtheta:set(1, zmp_vel.x)

   self.dtheta:setVec3(3,w)
  
   self:projectState()
end

function CartPoleBall:setDesiredParam(velOrPos)
	self.desiredVelGlobal:assign(velOrPos)
	self.desiredVel:assign(velOrPos)
	self.desiredVel:rotate(self.ori:inverse())
end

function CartPoleBall:setDesiredVelocity(vel) -- vector3

	if self.mode==0 then
		self.mode=1
		local q=self.q
		self.pendX:setQ(1,q,q)
		self.pendZ:setQ(1,q,q)
	end

	self:setDesiredParam(vel)
end

function CartPoleBall:setDesiredPosition(pos) -- vector3
	if self.mode==1 then
		self.mode=0
		local q=self.q
		self.pendX:setQ(0, q,0)
		self.pendZ:setQ(0, q,0)
	end

	self:setDesiredParam(pos)
end

function CartPoleBall:numFrames()
   --		return self.Y:rows()
   return self.numFrame
end

function CartPoleBall:getPos(iframe)
   
	assert(self.Y:rows()>iframe)
   return vector3(self.Y(iframe,1), 0, self.Y(iframe,0))
end

function CartPoleBall:calcState2D(xx, zx)

   local pos=vector3()
   local d=vector3()
   
   pos.x=self.theta(1)
   pos.y=0
   pos.z=self.theta(0)
   d.x=self.dtheta(1)
   d.y=0
   d.z=self.dtheta(0)

   local q=self.theta:toQuater(2)
   local w=self.dtheta:toVector3(3)

   local vt=vector3(0,1,0)
   local vtt=q*vt

   local q_cleanup=quater()
   q_cleanup:axisToAxis(vt, vtt)
   local theta=q_cleanup:rotationVector()



    local toLocal=self.ori:inverse()

    pos:rotate(toLocal)
    w:rotate(toLocal)
     theta:rotate(toLocal)
     d:rotate(toLocal)


   if pos.x~=pos.x then debug.debug() end

   xx:set(0, 0, pos.x)
   xx:set(1, 0, d.x)
   xx:set(2, 0, theta.z)
   xx:set(3, 0, w.z)
   
--   RE.output("x_v", tostring(posx).." "..tostring(posz) .." "..tostring(dx).." "..tostring(dz))
--   RE.output("theta_w", tostring(theta).." "..tostring(w))
   zx:set(0, 0, pos.z)
   zx:set(1, 0, d.z)
   zx:set(2, 0, -theta.x)
   zx:set(3, 0, -w.x)

   local controlForce=vector3()   
   --print(self.Y:rows())
   -- calc control force
   local K=self.pendX.K


   controlForce.x=(K*xx*-1):get(0,0)+self.pendX.U-self.friction*d.x
   controlForce.z=(K*zx*-1):get(0,0)+self.pendZ.U-self.friction*d.z

   -- RE.output("controlForce", tostring(self.ori).." "..tostring(pos.x).." "..tostring(pos.z) .." " ..tostring(controlForce))

   controlForce.y=0
   --print(self.ori)
   controlForce:rotate(self.ori)


   return controlForce
end
function CartPoleBall:oneStep(ctrlForceScale)
   
   
   local xx=self.pendX.x
   local zx=self.pendZ.x

   self.pendX:setParam(self.desiredVel.x)
   self.pendZ:setParam(self.desiredVel.z)
   local controlForce=self:calcState2D(xx,zx)

   self.controlforce:setAllValue(0)
   self.controlforce:set(0, controlForce.z)
   self.controlforce:set(1, controlForce.x)

   if ctrlForceScale~=nil then
      self.controlforce=self.controlforce*ctrlForceScale
   end

   self.simulator:setLinkData(0, DynamicsSimulator.JOINT_TORQUE, self.controlforce)

   if self.simulator:stepSimulation()== false then
      self.errorOccurred=true
      print("error detected")
      fineLog("error detected")
--      debug.debug()
      return
   end


   self.simulator:getLinkData(0, DynamicsSimulator.JOINT_VALUE, self.theta)
   self.simulator:getLinkData(0, DynamicsSimulator.JOINT_VELOCITY, self.dtheta)

   self:projectState()

--		self.Y:resize(self.Y:rows()+1, 2)
   self.Y:resize(math.max(self.Y:rows(), self.numFrame+1), 6) -- do not delete predicted result
   self.numFrame=self.numFrame+1
   local lastRow=self.numFrame-1
   
   self.Y:row(lastRow):assign(self.theta)

end



function CartPoleBall:fourSteps()

   local xx=self.pendX.x
   local zx=self.pendZ.x

	self.pendX:setParam(self.desiredVel.x)
	self.pendZ:setParam(self.desiredVel.z)
   local controlForce=self:calcState2D(xx,zx)

   self.controlforce:setAllValue(0)
   self.controlforce:set(0, controlForce.z)
   self.controlforce:set(1, controlForce.x)

   self.simulator:setTimestep(self.dt*4)

   self.simulator:setLinkData(0, DynamicsSimulator.JOINT_TORQUE, self.controlforce)

   if self.simulator:stepSimulation()== false then
      self.errorOccurred=true
      print("error detected")
--      debug.debug()
      return
   end

   self.simulator:getLinkData(0, DynamicsSimulator.JOINT_VALUE, self.theta)
   self.simulator:getLinkData(0, DynamicsSimulator.JOINT_VELOCITY, self.dtheta)

   self:projectState()

   self.simulator:setTimestep(self.dt)

   --		self.Y:resize(self.Y:rows()+1, 2)

   assert(self.numFrame~=0)

   self.Y:resize(math.max(self.Y:rows(), self.numFrame+4), 6) -- do not delete predicted result
   self.numFrame=self.numFrame+4
   local prevRow=self.numFrame-5
   local lastRow=self.numFrame-1

   self.Y:row(lastRow):assign(self.theta)
   self.Y:row(lastRow-1):interpolate(0.75, self.Y:row(prevRow), self.theta)
   self.Y:row(lastRow-2):interpolate(0.5, self.Y:row(prevRow), self.theta)
   self.Y:row(lastRow-3):interpolate(0.25, self.Y:row(prevRow), self.theta)

end

function CartPoleBall:predictTrajectory(numSteps)
   local theta=vectorn()
   local dtheta=vectorn()
   local numFrames=self:numFrames()
   
   self:_saveStates(theta, dtheta)
   
   for i=1,numSteps do
      self:oneStep()
   end
   
   return self:_restoreStates(theta, dtheta, numFrames)
end



function CartPoleBall:_saveStates(theta, dtheta)
   self.simulator:getLinkData(0, DynamicsSimulator.JOINT_VALUE, theta)
   self.simulator:getLinkData(0, DynamicsSimulator.JOINT_VELOCITY, dtheta)
--   assert(not self.errorOccurred)
   if self.errorOccurred then print('warning ! cartpoleball:_saveStates')
	   self.errorOccurred=false
   end
end

function CartPoleBall:getStates()
	local states={}
	states.x=vectorn()
	states.dx=vectorn()
	states.numFrames=self:numFrames()
	self:_saveStates(states.x, states.dx)
	states.ori=self.ori:copy()
	return states
end

function CartPoleBall:restoreStates(states)
	self:_restoreStates(states.x, states.dx, states.numFrames)
	self.ori:assign(states.ori)
end

function CartPoleBall:_restoreStates(theta, dtheta, numFrames)
   --		local res=matrixn()
   --		local output=self.Y:range(numFrames, self.Y:rows(), 0, self.Y:cols())
   --
   --		res:setSize(output:rows(), output:cols())
   --		res:assign(output)
   
   -- restore states
   --		self.Y:resize(numFrames, self.Y:cols())
   self.errorOccurred=false
   self.numFrame=numFrames
   self.simulator:setLinkData(0, DynamicsSimulator.JOINT_VALUE, theta)
   self.simulator:setLinkData(0, DynamicsSimulator.JOINT_VELOCITY, dtheta)
   self.simulator:initSimulation()
   
   self.theta:assign(theta)
   self.dtheta:assign(dtheta)
   return res
end

function CartPoleBall._getPosition(cartpole,frame)
   local v1=vector3()
   local q=quater()

   if frame==nil then
      frame=-1
   end
   cartpole:_getPosition2(frame, v1,q)
   return v1, q
end

function CartPoleBall.__getPosition(theta)
   local v1=vector3()
   local q=quater()

   local posz=theta(0)
   local posx=theta(1)

   local vt=vector3(0,1,0)
   local vtt=theta:toQuater(2)*vt

   q:axisToAxis(vt, vtt)
   
   v1:setValue(posx,0,posz);
   return v1,q
end

function CartPoleBall:_getPosition2(frame, v1,q)

   
   
   local theta
   if frame>=0 then
      theta=self.Y:row(frame)
   else
      theta=self.theta
   end


   local posz=theta(0)
   local posx=theta(1)

   local vt=vector3(0,1,0)
   local vtt=theta:toQuater(2)*vt

   q:axisToAxis(vt, vtt)
   
   v1:setValue(posx,0,posz);
   
   
end

function CartPoleBall.__getPosition(theta)
   
   local v1=vector3()
   local q=quater()
   local posz=theta(0)
   local posx=theta(1)

   local vt=vector3(0,1,0)
   local vtt=theta:toQuater(2)*vt

   q:axisToAxis(vt, vtt)
   
   v1:setValue(posx,0,posz);
   
         
   return v1, q
end

function CartPoleBall.__getCOMpos(pend, theta)
   local v1,q=CartPoleBall.__getPosition(theta)
   local frame=transf()
   frame.rotation:assign(q)
   frame.translation:assign(v1)

   return frame:toGlobalPos(pend.pole:localCOM())   
end
function CartPoleBall:calcLeaning()
   
   if self.theta:size()<6 then
      return 0
   end
   
   local q=quater()

   local vt=vector3(0,1,0)
   local vtt=self.theta:toQuater(2)*vt

   q:axisToAxis(vt, vtt)

   return q:rotationAngle()
end

function CartPoleBall:calcCOMvel()
   
   
   --[[ local w=self:calcPoleAngVel()
   
   local v=self:calcCartVel()+w:cross(self.pole:localCOM())
   
   local v2=self.simulator:getWorldVelocity(0,self.pole, self.pole:localCOM())
   print("V==v2", v, v2)
   return v2
]]--

return self.simulator:getWorldVelocity(0,self.pole, self.pole:localCOM())
end

function CartPoleBall:calcPoleAngVel()


   return self.dtheta:toVector3(3)

end

function CartPoleBall:calcCartVel()
   return self.simulator:getWorldVelocity(0, self.cart, self.cart:localCOM())
end

function CartPoleBall:calcCOMpos()
   return self.simulator:getWorldState(0):globalFrame(self.pole):toGlobalPos(self.pole:localCOM())
end
function CartPoleBall:calcCartPos()
   return self.simulator:getWorldState(0):globalFrame(self.cart):toGlobalPos(self.cart:localCOM())
end

function CartPoleBall:addExtForce(f)
   self.simulator:addForceToBone(0,self.pole, self.pole:localCOM(), f)
end

function CartPoleBall:_calcTip(q)
   local qq=vector3()
   qq:rotate(q,vector3(0,self.pendX.l*2,0));
   return qq
end

function CartPoleBall:setVisualTranslation(x,y,z)
	self.vt=vector3(x,y,z)
	self.skin:setTranslation(x,y,z)
end

function CartPoleBall:draw()

	self.skin:setPose(self.simulator,0)

	if self.vt then

		local cf=vector3(0,0,0)
		cf.z=self.controlforce(0)
		cf.x=self.controlforce(1)
		local cp=self:calcCartPos()
		
		local startpos=cp*100+self.vt+vector3(0,1,0)
		dbg.draw('Line', startpos, startpos+cf*0.1, 'cartPoleBall_force', 'solidgreen')
	end
	local drawTest=false

	if drawTest then
		pNode=RE.createEntity("cart_pole_3d", "cube.mesh")

		v1,q=self:_getPosition()
		local scale=100
		v1=v1*scale
		v2=vector3()


		v2:rotate(q,vector3(0,self.pendX.l*2*scale,0));
		v2:radd(v1)

		thick=4;

		pNode:resetToInitialState();
      pNode:scale(0.01*thick, 0.01*(v2-v1):length(), 0.01*thick);
      pNode:rotate(q);
      pNode:translate((v1+v2)/2)
      
      RE.setMaterialName(pNode, "blue")
   end
end

function CartPoleBall:drawFrames(startFrame, objectList)
   local lines=vector3N()
   for i=startFrame, self.numFrame-1 do
      local cart, poleOri=self:_getPosition(i)
      local pole=cart+self:_calcTip(poleOri)

      lines:pushBack(cart)
      lines:pushBack(pole)
   end

   if lines:rows()~=0 then
      objectList:registerObject("zmpPredict2", "LineList", "solidred", 
				lines:matView()*100,0)
   end
end

function CartPoleBall:calcZMPCOM(iframe)
   local cart, poleOri=CartPoleBall._getPosition(self, iframe)
   local pole=cart+self:_calcTip(poleOri)

   return cart, (cart+pole)/2
end
function CartPoleBall:drawFrame(iframe)
   -- todo
end

function CartPoleBallCpp.setVisualTranslation() end
CartPoleBallCpp._getPosition=CartPoleBall._getPosition

function CartPoleBallCpp.__getPosition(theta)
	local v=vector3() local q=quater()
	CartPoleBallCpp.__getPosition2(theta, v,q)
	return v,q
end

