
function tbl2vecn(tbl)
	local vn = vectorn(#tbl)
	for i=1,#tbl do
		vn:set(i-1, tbl[i])
	end
	return vn
end
function tbl2ivecn(tbl)
	local vn = intvectorn(#tbl)
	for i=1,#tbl do
		vn:set(i-1, tbl[i])
	end
	return vn
end

function vecn2tbl(vn)
	tbl = {}
	for i=0,vn:size()-1 do
		tbl[i+1] = vn:get(i)
	end
	return tbl
end

function tbl2vec3(tbl)
	return vector3(tbl[1], tbl[2], tbl[3])
end

function vec32vecn(v3)
	local vn = vectorn(3)
	vn:set(0, v3:getX())
	vn:set(1, v3:getY())
	vn:set(2, v3:getZ())
	return vn
end

function round(num)
	return math.floor(num+.5)
end
 
function s_mul_t(sca, tbl)
	local tbln = {}
	for i=1,#tbl do
		tbln[i] = sca*tbl[i]
	end
	return tbln
end

function t_mul_t(tbl1, tbl2)
	local tbln = {}
	for i=1,#tbl1 do
		tbln[i] = tbl1[i]*tbl2[i]
	end
	return tbln
end

function t_div_t(tbl1, tbl2)
	local tbln = {}
	for i=1,#tbl1 do
		tbln[i] = tbl1[i]/tbl2[i]
	end
	return tbln
end

function t_add_t(tbl1, tbl2)
	local tbln = {}
	for i=1,#tbl1 do
		tbln[i] = tbl1[i]+tbl2[i]
	end
	return tbln
end

function t_add_t_update(tbl1, tbl2)
	for i=1,#tbl1 do
		tbl1[i] = tbl1[i]+tbl2[i]
	end
end

function t_sub_t(tbl1, tbl2)
	local tbln = {}
	for i=1,#tbl1 do
		tbln[i] = tbl1[i]-tbl2[i]
	end
	return tbln
end

function t_div_s(tbl, sca)
	local tbln = {}
	for i=1,#tbl do
		tbln[i] = tbl[i]/sca
	end
	return tbln
end

function table_slice (values,i1,i2)
	local res = {}
	local n = #values
	-- default values for range
	i1 = i1 or 1
	i2 = i2 or n
	if i2 < 0 then
	i2 = n + i2 + 1
	elseif i2 > n then
	i2 = n
	end
	if i1 < 1 or i1 > n then
	return {}
	end
	local k = 1
	for i = i1,i2 do
	res[k] = values[i]
	k = k + 1
	end
	return res
end

function extend_array(tbl, tbl_to_extend)
	for i, v in ipairs(tbl_to_extend) do
		table.insert(tbl, v)
	end
	return tbl
end

function concat_array(tbl1, tbl2)
	local tbl = {}
	for i, v in ipairs(tbl1) do
		table.insert(tbl, v)
	end
	for i, v in ipairs(tbl2) do
		table.insert(tbl, v)
	end
	return tbl
end

function merge_array(tbl1, tbl2)
	tbln = {}
	for i, v in ipairs(tbl1) do
		table.insert(tbln, v)
	end
	for i, v in ipairs(tbl2) do
		table.insert(tbln, v)
	end
	return tbln
end

function split_array(tbl)
	return table_slice(tbl,1,#tbl/2), table_slice(tbl,#tbl/2+1,#tbl)
end

function create_array(length, value)
	value = value or 0.
	local array = {}
	for i=1,length do
		array[i] = value
	end
	return array
end

function rk4_step(f, t_curr, y_curr, h, data)
	local data = data or nil
	local k1 = s_mul_t(h, f(t_curr, y_curr, data))
	local k2 = s_mul_t(h, f(t_curr+.5*h, t_add_t(y_curr, s_mul_t(.5, k1)), data))
	local k3 = s_mul_t(h, f(t_curr+.5*h, t_add_t(y_curr, s_mul_t(.5, k2)), data))
	local k4 = s_mul_t(h, f(t_curr+h, t_add_t(y_curr, k3), data))
	local y_next = t_add_t(y_curr, t_div_s(t_add_t(k1, t_add_t(s_mul_t(2., t_add_t(k2, k3)), k4)), 6.))
	return t_curr+h, y_next
	--k1 = h*f(t_curr, y_curr, data)
	--k2 = h*f(t_curr+.5*h, y_curr+.5*k1, data)
	--k3 = h*f(t_curr+.5*h, y_curr+.5*k2, data)
	--k4 = h*f(t_curr+h, y_curr+k3, data)
	--y_next = y_curr + (k1 + 2*(k2+k3) + k4)/ 6.
	--return t_curr+h, y_next
end

function concattwovec3(v1, v2)
	local vn = vectorn(6)
	vn:set(0, v1:getX())
	vn:set(1, v1:getY())
	vn:set(2, v1:getZ())
	vn:set(3, v2:getX())
	vn:set(4, v2:getY())
	vn:set(5, v2:getZ())
	return vn
end

function lerptbl(tbl1, tbl2, t)
	newtbl = {}
	for i=1,#tbl1 do
		newtbl[i] = (1.-t)*tbl1[i] + t*tbl2[i]
	end
	return newtbl
end

function printtblh(t)
	io.write'{'
	for k, v in pairs(t) do
		io.write(k..':'..tostring(v)..', ')
	end
	io.write'}\n'
end

function printtbl(t)
	for k, v in pairs(t) do
		print(k, v)
	end
end

function printtbl2(t1, t2)
	for k, v in pairs(t1) do
		print(k, t1[k], t2[k])
	end
end

function printmatn(m)
	for r=0,m:rows()-1 do
		print('['..r..']', m:row(r))
	end
end

function printmatn_space(m)
	for r=0,m:rows()-1 do
		print('['..r..']', m:row(r))
		print()
	end
end

function tablelength(T)
  local count = 0
  for _ in pairs(T) do count = count + 1 end
  return count
end

function repr(obj)
	if type(obj)=='string' then
		return '\''..tostring(obj)..'\''
	else
		return tostring(obj)
	end
end

function printtbl_pylist(t)
	io.write'['
	for i, v in ipairs(t) do
		io.write(repr(v))
		if i<#t then
			io.write', '
		end
	end
	io.write']\n'
end

function printtbl_pydict(t)
	io.write'{'
	local cnt=1
	for k, v in pairs(t) do
		io.write(repr(k)..':'..repr(v))
		if cnt<#t then
			io.write', '
		end
		cnt = cnt + 1
	end
	io.write'}\n'
end

function ASIN(x)
	if x > 1.0 then			return math.pi/2.
	elseif x < -1.0 then	return -math.pi/2.
	else 					return math.asin(x)
	end
end

function string:split(sSeparator, nMax, bRegexp)
	assert(sSeparator ~= '')
	assert(nMax == nil or nMax >= 1)

	local aRecord = {}

	if self:len() > 0 then
		local bPlain = not bRegexp
		nMax = nMax or -1

		local nField=1 nStart=1
		local nFirst,nLast = self:find(sSeparator, nStart, bPlain)
		while nFirst and nMax ~= 0 do
			aRecord[nField] = self:sub(nStart, nFirst-1)
			nField = nField+1
			nStart = nLast+1
			nFirst,nLast = self:find(sSeparator, nStart, bPlain)
			nMax = nMax-1
		end
		aRecord[nField] = self:sub(nStart)
	end

	return aRecord
end


function brent(ax, bx, cx, func1, ftol, xmin) 
	local iter
	local a, b, d, etemp, fu, fv, fw, fx, p, q, r, tol1, tol2, u, v, w, x, xm;
	local e=0.0
	local ITMAX=200
	local ZEPS=1.0e-10
	local CGOLD=0.3819660
	a = math.min(ax, cx)
	b = math.max(ax, cx)
	v=bx
	w=bx
	x=bx

	fx = func1(x);
	--fx = fb
	fv = fx
	fw = fv

	local function SIGN(a,b)
		if b<0 then
			return -1*math.abs(a)
		end
		return math.abs(a)
	end

	for iter=1,ITMAX do
		xm = 0.5 * (a + b);
		tol1=ftol * math.abs(x) + ZEPS
		tol2 = 2.0 * tol1 ;

		if (math.abs(x - xm) <= (tol2 - 0.5 * (b - a))) then
			xmin = x;
			fret= fx;
			break
		end

		if (math.abs(e) > tol1) then

			r = (x - w) * (fx - fv);
			q = (x - v) * (fx - fw);
			p = (x - v) * q - (x - w) * r;
			q = 2.0 * (q - r);
			if (q > 0.0) then
				p = -p;
			end
			q = math.abs(q);
			etemp = e;
			e = d;
			if (math.abs(p) >= math.abs(0.5*q*etemp) or p <= q*(a-x) or p >= q*(b-x)) then
				if x>=xm then
					e=a-x
				else
					e=b-x
				end
				d = CGOLD * e
			else
				d = p / q;
				u = x + d;
				if (u - a < tol2 or b - u < tol2) then
					d = SIGN(tol1, xm - x);
				end
			end
		else
			if x>= xm then
				e=a-x
			else
				e=b-x
			end

			d = CGOLD * e
		end

		if math.abs(d)>=tol1 then
			u=x+d
		else
			u=x+SIGN(tol1, d);
		end

		fu = func1 (u);

		if (fu <= fx) then
			if (u >= x) then	a = x;
			else b = x; end
			v=w;w=x;x=u
			fv=fw;fw=fx;fx=fu
		else
			if (u < x) then a = u; 
			else b = u; end

			if (fu <= fw or w == x) then
				v = w;
				w = u;
				fv = fw;
				fw = fu;
			elseif (fu <= fv or v == x or v == w) then
				v = u;
				fv = fu;
			end
		end
	end 

	xmin = x;
	fret= fx;
	return fret, xmin
end

function min_brent(func, x0, lb, ub, ftol)
	ftol = ftol or 2e-5
	return brent(lb, x0, ub, func, ftol)
end

function _importBVHys(chosenFile, heightOffset, dof_scale)
	if mMotionDOFcontainer~=nil and chosenFile~=nil then
		local dofScale=0.01 -- millimeters to meters
		local dofRot=quater(math.rad(-90), vector3(0,1,0))
		*quater(math.rad(-90), vector3(1,0,0)) -- change to Y_up
		if dof_scale then
			dofScale=dof_scale
			dofRot=quater(1,0,0,0)
		end

		local bvhloader=RE.createMotionLoader(chosenFile, chosenFile)
		-- rotated skeleton
		local bvhloader1=RE.createMotionLoader(chosenFile, chosenFile)
		rotateSkeleton(bvhloader, bvhloader1,dofRot)

		local mot=bvhloader1.mMotion
		local mot2=Motion(bvhloader)
		-- convert to model space (meter, Y_up)
		mot:scale(dofScale)
		
		if heightOffset then
			print("heightOffset=",heightOffset)
			for f=0, mot:length()-1 do
				for i=0, mot:pose(f).translations:size()-1 do
					mot:pose(f).translations(i):radd(vector3(0,heightOffset,0))
				end
			end
		end
		MotionUtil.upsample(mot2, mot, 2)

	  	local motdof=convertMotionToMotDOF(bvhloader1, mot2, mLoader)

		return motdof
	end
end

function table.val_to_str ( v )
	if "string" == type( v ) then
		v = string.gsub( v, "\n", "\\n" )
		if string.match( string.gsub(v,"[^'\"]",""), '^"+$' ) then
			return "'" .. v .. "'"
		end
		return '"' .. string.gsub(v,'"', '\\"' ) .. '"'
	else
		return "table" == type( v ) and table.tostring( v ) or
		tostring( v )
	end
end

function table.key_to_str ( k )
	if "string" == type( k ) and string.match( k, "^[_%a][_%a%d]*$" ) then
		return k
	else
		return "[" .. table.val_to_str( k ) .. "]"
	end
end

function table.tostring( tbl )
	local result, done = {}, {}
	for k, v in ipairs( tbl ) do
		table.insert( result, table.val_to_str( v ) )
		done[ k ] = true
	end
	for k, v in pairs( tbl ) do
		if not done[ k ] then
			table.insert( result,
			table.key_to_str( k ) .. "=" .. table.val_to_str( v ) )
		end
	end
	return "{" .. table.concat( result, "," ) .. "}"
end

function file_exists(name)
	local f=io.open(name,"r")
	if f~=nil then io.close(f) return true else return false end
end
function tbl2vecd(tbl)
	if type(tbl)~='table' then
		local v=VectorXd(tbl:size())
		v:assign(tbl)
		return v
	end
	local v=VectorXd(#tbl)
	local vn = v:vecView()
	for i=1,#tbl do
		vn:set(i-1, tbl[i])
	end
	return v
end

function vecn2vecd(vec)
	local v=VectorXd()
	v:assign(vec)
	return  v
end

function VectorXd.__tostring(self)
	return self:vecView():__tostring()
end
function VectorXd.__sub(t1, t2)
	return vecn2vecd(t1:vecView()-t2:vecView())
end
function VectorXd.__add(t1, t2)
	return vecn2vecd(t1:vecView()+t2:vecView())
end
function VectorXd.__mul(t1, t2)
	return vecn2vecd(t1:vecView()*t2:vecView())
end
function VectorXd.__div(t1, t2)
	return vecn2vecd(t1:vecView()/t2:vecView())
end
function VectorXd.__call(i)
	return self:vecView()(i)
end


CSVexporter=LUAclass()
function CSVexporter:__init(fn, freq)
	self.filename=fn
	self.elapsedTime=0
	self.freq=freq
	util.writeFile(self.filename,'')
end
function CSVexporter:exportLine(data, duration)
	if duration then
		assert(math.fmod(1/duration, self.freq)<1e-4)
		self.elapsedTime=self.elapsedTime+duration
		if self.elapsedTime<1/self.freq-1e-4 then
			return
		else
			self.elapsedTime=0
		end
	end
	local tn=dbg.lunaType(data)
	if tn=='vectorn' then
		util.appendFile(self.filename, data:__tostring():sub(2, -2)..'\n')
	elseif tn=='TStrings' then
		dbg.console()
	elseif tn=='VectorXd' then
		util.appendFile(self.filename, data:vecView():__tostring():sub(2, -2)..'\n')
	elseif tn=='table' then
		local out=table.concat(data,',')
		util.appendFile(self.filename, out..'\n')
	else
		assert(false)
	end
end

