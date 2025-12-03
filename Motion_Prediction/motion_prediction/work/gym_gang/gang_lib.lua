require("config")
require("common")
require("subRoutines/MatplotLib")

function vectorn:concat_qua(b)
	local start_pos=self:size()
	self:resize(self:size()+4)
	self:setQuater(start_pos,b)
end

function vectorn:concat_quaN(b)
	for i=0,b:size()-1 do
		local a_size=self:size()
		self:resize(a_size+4)
		self:range(a_size, self:size()):assign(b(i))
	end
end
function vectorn:concat_vec3(b)
   local a_size=self:size()
   self:resize(a_size+3)
   self:range(a_size, self:size()):assign(b)
end

function vectorn:concat_value(b)
   local a_size=self:size()
   self:resize(a_size+1)
   self:set(self:size()-1, b)
end
function vectorn:concat_vec3N(b)
	for i=0,b:size()-1 do
		local a_size=self:size()
		self:resize(a_size+3)
		self:range(a_size, self:size()):assign(b(i))
	end
end
function vectorn:concat_matrixn(b)
	for i=0,b:rows()-1 do
		local a_size=self:size()
		self:resize(a_size+b:row(i):size())
		self:range(a_size, self:size()):assign(b:row(i))
	end
end
function quater:printRot(option)
	local x=self:rotationAngleAboutAxis(vector3(1,0,0))	
	local y=self:rotationAngleAboutAxis(upv())	
	local z=self:rotationAngleAboutAxis(fwv())	
	if option==0 or option==nil then
		print('radian value x',x,'y',y,'z',z)
	elseif option==1 then
		print('degree value x',radian2degree(x),'y',radian2degree(y),'z',radian2degree(z))
	end
end

function quater:change_to_6d()
	local vec_=vectorn(6)
	local facing_=fwv()
	local up_=upv()
	vec_:range(0,3):assign(self*facing_)
	vec_:range(3,6):assign(self*up_)
	return vec_
end

function quaterN:change_to_6d()
	local matn_=matrixn(self:rows(),6)
	for i=0,self:rows()-1 do
		local target_qua=self:row(i):copy()
		local result_=target_qua:change_to_6d()
		matn_:row(i):assign(result_)
	end
	return matn_
end
function matrixn:pop()
	local rows=self:size()
		for i=0,rows-2 do
		self:row(i):assign(self:row(i+1))
	end
	self:resize(rows-1,self:cols())
end

function vector3N:pop()
	local rows=self:size()
	for i=0,rows-2 do
		self:row(i):assign(self:row(i+1))
	end
	self:resize(rows-1)
end
--merge two matrixn (condition : same cols)
function matrixn:merge(b)
	assert(self:cols()==b:cols())
	for i=0,b:rows()-1 do
		self:pushBack(b:row(i))
	end
end

function MainLib.VRMLloader:printMass()
	local skel=self
	local mass=0
	for i=1,skel:numBone()-1 do
		mass=mass+skel:VRMLbone(i):mass()
		print(skel:VRMLbone(i):name(),skel:VRMLbone(i):mass())	
	end
	print('---------------------------------------------------')
	print('total model mass', mass)
	return mass
end

function dbg.line()
	print('---------------------------------------------------')
end

function angle_dif_vec(vec1,vec2)
	local numerator=vec1:dotProduct(vec2)
	local denominator=vec1:length()*vec2:length()
	local gap=math.acos(math.min(1,numerator/denominator))
	local gap2=math.pi-math.acos(numerator/denominator)
	local degree_gap=gap*(180/math.pi)
	local degree_gap2=gap2*(180/math.pi)
	return gap
end

function angle_dif_atan2(vec1,vec2)
	local gap_1=math.atan2(vec1.x,vec1.z)
	local gap_2=math.atan2(vec2.x,vec2.z)
	local delta_=gap_1-gap_2
	return delta_
end
function angle_dif_atan2_qua(qua1,qua2)
	local from_=fwv()
	local to_=fwv()
	from_:rotate(qua1)
	to_:rotate(qua2)
	return angle_dif_atan2(from_,to_)
end
function vecAtan2(vec)
	return math.atan2(vec.x,vec.z)
end
----projection vec2 to vec1
function vector3:projection(vec1,vec2)
	local projection_vector=vec1:copy()
	projection_vector:rmult((vec1:dotProduct(vec2))/(vec1:length()*vec1:length()))
	return projection_vector 
end
--projection self to target
function vector3:projection(target)
	target:rmult((self:dotProduct(target))/(target:length()*target:length()))
	self:assign(target)
end

function matrixn:setVec3(vec3_)
	assert(self:row(0):size()==3)
	for row_=0,self:rows()-1 do
		self:row(row_):setVec3(0,vec3_)
	end
end

function matrixn:addVec3(vec3_,start_)
	if start_==nil then start_=0 end
	for row_=0,self:rows()-1 do
		self:row(row_):set(start_+0,self:row(row_):get(start_+0)+vec3_.x)
		self:row(row_):set(start_+1,self:row(row_):get(start_+1)+vec3_.y)
		self:row(row_):set(start_+2,self:row(row_):get(start_+2)+vec3_.z)
	end
end
function getDesiredFinalVel(initialRotY, fRotY, v)
	local vv=fRotY*vector3(0,0,v)
	local pendRotY

	if true then
		-- prevent extreme turning 
		pendRotY=initialRotY
		local initialfront=pendRotY*vector3(0,0,1)
		local finalfront=vv:copy()
		finalfront:normalize()
		local delta=quater()
		delta:setAxisRotation(vector3(0,1,0), initialfront, finalfront)
		delta:align(quater(1,0,0,0))
		local maxangle=
		arc.getLinearSplineABS(v,
		CT.mat(5,2,
		0,math.rad(180),
		1,math.rad(180),
		2.5,math.rad(120),
		5.5,math.rad(50),
		10.5,math.rad(30)))
		local angle=delta:rotationAngleAboutAxis(vector3(0,1,0))
		angle=math.clamp(angle, -maxangle, maxangle)
		finalrotY=pendRotY*quater(angle, vector3(0,1,0))
		vv=finalrotY*vector3(0,0,v)
	end
	return finalrotY, vv, pendRotY
end

function dbg._namedDraw(type, ...)
	local t2=type
	if type=='Arrow2' then
		t2='Arrow'
	end
	dbg.draw(t2, ...)
	local pos
	local nameid
	local color='blue'

	if type=="Sphere" then
		local p={...}
		pos=p[1]
		nameid=p[2]
		if p[3] then color=p[3] end
	elseif type=='Axes' then
		local p={...}
		pos=p[1].translation
		nameid=p[2]
		scale=p[3]
		if scale then
			pos=pos*scale
		end
	elseif type=='Coordinate' then
		local p={...}
		pos=p[1].translation*100 + (p[3] or vector3(0,0,0))*100
		nameid=p[2]
	elseif type=="Line" or type=='Line2' then
		local p={...}
		pos=p[1]
		w_start=0.2
		w_end=0.8
		local midle=(w_start*p[1]:copy())+
					(w_end*p[2]:copy())
		pos=midle
		nameid=p[3]
		if p[4] then color=p[4] end
	elseif type=="Arrow" then
		local p={...}
		pos=p[2]
		nameid=p[3]
		color=p[5]
	elseif type=='Arrow2' then
		local p={...}
		pos=p[2]
		nameid=p[3]
		if p[5]~=nil then
			color=p[5]
		end
		type='Arrow'
	elseif type=="registerObject" then
		local p={...}
		pos=p[4][0]
		nameid=p[1]
	end
	if pos then
	--if false then
		local mat
		if string.find(color,'ed',nil,true)~=nil then --red
			mat=CT.mat(1,4, 0.7,0,0,1)
		elseif string.find(color,'reen',nil,true)~=nil then --green
			mat=CT.mat(1,4, 0,0.5,0,1)
		else
			mat=CT.mat(1,4, 0,0,1,1) -- blue
		end
		local obj=dbg.objectList:registerObject(nameid.."_mt", "MovableText", nameid, mat, 8)
		if obj then obj:setPosition(pos.x, pos.y+20, pos.z) end
	end
	return pos,nameid
end

function printViewpoint()
	if RE~=nil then
		print('view pos',RE.viewpoint().vpos.x,',',RE.viewpoint().vpos.y,',',RE.viewpoint().vpos.z)
		print('view at',RE.viewpoint().vat.x,',',RE.viewpoint().vat.y,',',RE.viewpoint().vat.z)
	end
end
function upv(size_)
	if size_~=nil then
		return vector3(0,size_,0)
	else
		return vector3(0,1,0)
	end
end

function fwv(size_)
	if size_~=nil then
		return vector3(0,0,size_)
	else
		return vector3(0,0,1)
	end
end

function sdv(size_)
	if size_~=nil then
		return vector3(size_,0,0)
	else
		return vector3(1,0,0)
	end
end

function radian2degree(radian_value)
	return radian_value*(180/math.pi)
end

function init_plotter()
	return MatplotLib()
end

Queue=LUAclass()
function Queue:__init(n)
	self.n=n
	self.data={}
	self.front=1
end
function Queue:pushBack(data)
	if #self.data==self.n then
		self.data[self.front]=data
		self.front=self.front+1
		if self.front>self.n then
			self.front=1
		end
	else
		table.insert(self.data, data)
	end
end
function Queue:back()
	local f=self.front
	if f==1 then
		return self.data[#self.data]
	else
		return self.data[f-1]
	end
end
function Queue:front()
	return self.data[self.front]
end
function Queue:get(i)-- i==1 : front, i>= #self.data : back
	assert(i<=self.n)
	local f=self.front
	if f==1 then
		return self.data[math.min(i+f-1, #self.data)]
	else
		assert(#self.data==self.n)
		if f+i-1>self.n then
			return self.data[i-self.n+f-1]
		else
			return self.data[f+i-1]
		end
	end
end

function MatplotLib:axvline(x_value,color,style,linewidth)
	if x_value==nil then print("not valid input") dbg.console() end
	if color==nil then color='r' end
	if style==nil then style=0 end
	if linewidth==nil then linewidth=1 end
	
	if style==0 then
		self:add('axvline(x='..x_value..',color='.."'"..color.."'"..',linewidth='..linewidth..')')
	elseif style==1 then
		self:add('axvline(x='..x_value..',color='.."'"..color.."'"..',linestyle="--",linewidth='..linewidth..')')
	elseif style==2 then
		self:add('axvline(x='..x_value..',color='.."'"..color.."'"..',linestyle=":",linewidth='..linewidth..')')
	end
end


OnlineLTIFilter=LUAclass()
function OnlineLTIFilter:__init(pose, filterSize)
	self.filterSize=filterSize
	self.queue=Queue(filterSize)
end

function OnlineLTIFilter:setCurrPose(pose)
	self.queue:pushBack(pose:copy())
end
function OnlineLTIFilter:getFiltered()
	if #self.queue.data==self.queue.n then
		local sum=vectorn(self.queue:back():size())
		sum:setAllValue(0)

		for i,v in ipairs(self.queue.data) do
			sum:radd(self.queue.data[i])
		end
		sum:rmult(1/self.queue.n)
		return sum
	else
		return self.queue:back()
	end
end
--for gaussian distribution
function erf(x)
    -- constants
    a1 =  0.254829592
    a2 = -0.284496736
    a3 =  1.421413741
    a4 = -1.453152027
    a5 =  1.061405429
    p  =  0.3275911

    -- Save the sign of x
    sign = 1
    if x < 0 then
        sign = -1
    end
    x = math.abs(x)

    -- A&S formula 7.1.26
    t = 1.0/(1.0 + p*x)
    y = 1.0 - (((((a5*t + a4)*t) + a3)*t + a2)*t + a1)*t*math.exp(-x*x)

    return sign*y
end

function gaussian_sample(mean, variance)
    return  math.sqrt(-2 * variance * math.log(math.random())) *
            math.cos(2 * math.pi * math.random()) + mean
end
