
--class 'InvertedPendulum' -- support both position and velocity control
InvertedPendulum=LUAclass()

function InvertedPendulum:setQ(mode, q, qd)
	if self.mode~=mode then
		self.Q:setAllValue(0)
		if mode==0 then -- position control
			-- pos, posdot, theta, thetadot
			self.Q:diag():assign({q,qd,0,0})
		else -- velocity control
			self.Q:diag():assign({0,qd,q,0})
		end
		self.mode=mode
		self.K=matrixn() 
		LQR(self.K,self.A,self.B,self.Q,self.matR) -- cpp implementation ported from lua implementation
		self.Ac=self.A-self.B*self.K
	end
end

function InvertedPendulum:__init(M,m,b,i,g,l,dt,q)
   self.M=M
   self.m=m
   self.b=b
   self.i=i
   self.g=g
   self.l=l
   
   self.A=matrixn()
   self.A:setSize(4,4)
   
   local t1=1/(i*(M+m)+M*m*l^2)
    --print("t1",t1,i,M,m,l,g);
   self.A:row(0):assign({0,1,0,0})
   self.A:row(1):assign({0, -(i+m*l^2)*b*t1, m^2*g*l^2*t1,0})
   self.A:row(2):assign({0,0,0,1})
   self.A:row(3):assign({0, -m*l*b*t1, m*g*l*(M+m)*t1,0})
   
   self.B=matrixn()
   self.B:setSize(4,1)
   self.B:column(0):assign({0, (i+m*l^2)*t1,0, m*l*t1})
   
   self.C=matrixn()
   self.C:assign({{1,0,0,0},{0,0,1,0}})
   
   self.D=matrixn()
   self.D:assign({{0},{0}})
   
   --print(self.A)
   --print(self.B)
   --print(self.C)
   
   self.Q=matrixn(4,4)
   self.Q:setAllValue(0)
   
   
   self.R=1;
   
   --self.K=octave:call("lqr",1,{self.A,self.B,self.Q,self.R})[1]

   self.matR=matrixn(1,1)
   self.matR:set(0,0,self.R)
   
   self:setQ(1, q,q) -- velocity control
   
   -- prepare euler integration
   self.t=0
   self.prevRenderT=0
   self.dt=dt
   self.x=matrixn(4,1)
   self.x:setAllValue(0)
   self.xdot=matrixn(4,1)
   self.U=0
   
   self.T=vectorn()
   self.Y=matrixn()
   self.y=vectorn(2)
end

function InvertedPendulum:setParam(param)
   local xd=matrixn(4,1)
   xd:column(0):setAllValue(0)
	if self.mode==0 then
		xd:set(0,0,param) -- position
	else
		xd:set(1,0,param) -- speed
	end
   
   self.Uprime_d=self.K*xd;
   --		print(self.Uprime_d)
   self.U=self.Uprime_d:get(0,0)
end

function InvertedPendulum:draw()
   -- taesoo drawing code
   if self.t-self.prevRenderT >= 0.01 then
      self.prevRenderT=self.t
      pNode=RE.createEntity("cart_pole", "cube.mesh");
      v1=vector3()
      v2=vector3()
      q=quater()
      
      local pos=self.y:get(0)
      local theta=self.y:get(1)
      local scale=100
      q:setRotation(vector3(0,0,1), theta);
      v1:setValue(pos*scale,0,0);
      v2:rotate(q,vector3(0,self.l*2*scale,0));
      v2:radd(v1)
      
      thick=4;
      
      pNode:resetToInitialState();
      pNode:scale(0.01*thick, 0.01*(v2-v1):length(), 0.01*thick);
      pNode:rotate(q);
      pNode:translate((v1+v2)/2)
      
      if mv==3 then
	 RE.setMaterialName(pNode, "blue")
      else
	 RE.setMaterialName(pNode, "green")
      end
   end
end

function InvertedPendulum:oneStep()
   
   local pos	=self.x:get(0,0)
   local pos_dot	=self.x:get(1,0)
   -- theta=Pi+pi
   local theta	=self.x:get(2,0)+3.14159265358979323846
   local theta_dot	=self.x:get(3,0)
   
   local costheta=math.cos(theta)
   local sintheta=math.sin(theta)
   local U=self.U
   local m=self.m
   local M=self.M
   local l=self.l
   local i=self.i
   local g=self.g
   local b=self.b
   
   local temp=(U-b*pos_dot+m*l*theta_dot*theta_dot*sintheta)/(M+m)
   local mlcostheta=m*l*costheta
   
   local theta_ddot=-m*l*(g*sintheta+costheta*temp)/(i+m*l*l-mlcostheta*mlcostheta/(m+M))
   
   local pos_ddot=temp-(mlcostheta*theta_ddot)/(m+M)

   self.xdot:set(0,0, pos_dot) -- pos_dot
   self.xdot:set(1,0, pos_ddot) -- pos_dot
   self.xdot:set(2,0, theta_dot) -- theta_dot
   self.xdot:set(3,0, theta_ddot) -- pos_dot
   
   
   
   
   -- add control force
   self.xdot:rsub(self.B*self.K*self.x)
   
   self.x:radd(self.xdot*self.dt)
   self.t=self.t+self.dt
   
   
   self.T:pushBack(self.t)
   self.y:set(0, self.x:get(0,0))
   self.y:set(1, self.x:get(2,0))
   self.Y:pushBack(self.y)
end



function InvertedPendulum:numFrames()
   return self.Y:rows()
end


function InvertedPendulum:plotSimul()
   
   octave:eval([[
		     function plotSimul(Ac,Bc,Cc,Dc,K)
			
			disp(Ac)
			disp(Bc)
			disp(Cc)
			disp(Dc)
			disp(K)
			
			T=0:0.01:5;
			U=0.2*ones(size(T));
			sys=ss(Ac,Bc,Cc,Dc)
			[Y,X]=lsim(sys,U',T);
				   plot(T,Y)
				   
				   print -demf foo.emf
				   
				   
			     ]])
--[[recover lua-mode']]--

octave:call("plotSimul",0, {Ac,self.B,self.C,self.D,self.K})
end


