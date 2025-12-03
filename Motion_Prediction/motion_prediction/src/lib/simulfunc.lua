function calcContactJacobianAll_gmbs(simulator, bases, numActualDOF, giOffset,  J, dotJ, V, dotV, linkPairCount, frictionCoef)
	local V_temp = matrixn()
	local dotV_temp = matrixn()
	local N = numActualDOF

	--local numTorqueVector = w.bases(0).torqueNormal:size()
	--local a = w.bases:size() * (1+w.bases(0).frictionNormal:size()) 
	local a = bases:size() * (1+bases(0).frictionNormal:size()) 

	--if ENABLETORQUE then
		--a = a+ bases:size() * numTorqueVector
	--end
	
	J:setSize(6*linkPairCount,N)
	dotJ:setSize(6*linkPairCount,N)
	V:setSize(6*linkPairCount,a)
	dotV:setSize(6*linkPairCount,a)
	V_temp:setSize(6,a)	--(ys) V for all contact points (6x5 matrix)
	dotV_temp:setSize(6,a)

	local dot_R_dAd=matrixn()
	dot_R_dAd:setSize(6,6)
	dot_R_dAd:setAllValue(0)

	local link_ibodies = intvectorn()
	local link_ibones = intvectorn()
	simulator:getLinkPairBodiesBones(link_ibodies, link_ibones)

	for i=0, linkPairCount-1 do
		local J_i=matrixn()
		local dotJ_i=matrixn()

		J_i:setSize(6,N)
		dotJ_i:setSize(6,N)
		--simulator:calcJacobian(b.ibody, b.ibone, J_i)
		--simulator:calcDotJacobian(b.ibody, b.ibone, dotJ_i)
		local ibody = intvectorn.get(link_ibodies, i)
		local ibone = intvectorn.get(link_ibones, i)
		simulator:calcJacobian(ibody, ibone, J_i)
		simulator:calcDotJacobian(ibody, ibone, dotJ_i)
		J:range( i*6, (i+1)*6, 0, N):assign(J_i)
		dotJ:range( i*6, (i+1)*6, 0, N):assign(dotJ_i)

		V_temp:setAllValue(0)
		dotV_temp:setAllValue(0)

		for c=0, bases:size()-1 do
			local b = bases(c)
			local n = b.normal:copy()
			local p = b.globalpos:copy()
			local pcn = p:cross(n)	--(ys) direction of torque by normal contact force in global frame

			local globalIndex = b.globalIndex
			local vk 
			local dot_vk 
			vk = V_temp:column(globalIndex)	--(ys) 6 dim vector
			dot_vk = dotV_temp:column(globalIndex)
			vk:setVec3(0, vk:toVector3(0)+pcn)	--(ys) angular part
			vk:setVec3(3, vk:toVector3(3)+n)	--(ys) linear part
			local dot_p = vector3(0,0,0)
			dot_p = -1*b.relvel:copy()
			pcn=dot_p:cross(n)
			dot_vk:setVec3(0, dot_vk:toVector3(0)+pcn)
			local n0=n:copy()

			for j=0, b.frictionNormal:size()-1 do
				local vk=V_temp:column(b.globalFrictionIndex+j-giOffset);
				local dot_vk=dotV_temp:column(b.globalFrictionIndex+j-giOffset)
				local n=b.frictionNormal(j)
				--if frictionCoef ~= 0.0 then
					--n=(n+frictionCoef*n0)
					--n:normalize()
				--end
				pcn=p:cross(n)

				-- transform the global generalized force (0, n) to spatial coordinate
				-- local f=liegroup.dse3(vector3(0,0,0), n)
				-- vk+=f:invdAd(transf(quater(1,0,0,0), p)))
				vk:setVec3(0, vk:toVector3(0)+pcn)
				vk:setVec3(3, vk:toVector3(3)+n)
				pcn=dot_p:cross(n)
				dot_vk:setVec3(0,dot_vk:toVector3(0)+pcn)
			end

			--local GT=simulator:getWorldState(0):globalFrame(b.ibone)
			---- spatial coordinate to bone global coordinate
			--local R_dAd= liegroup.invdAd(transf(GT.rotation,vector3(0,0,0))*GT:inverse())
			
			--V:range(i*6, (i+1)*6, 0, a):assign(R_dAd*V_temp)

			--local iboneVel=-1*simulator:getWorldVelocity(0, simulator:skeleton(0):VRMLbone(b.ibone), vector3(0,0,0)):copy()
			--dot_R_dAd:range(0,3,3,6):assign(CT.skew(iboneVel):copy())
			--dotV:range(i*6, (i+1)*6, 0, a):assign(dot_R_dAd*V_temp + R_dAd*dotV_temp)
		end

		local GT=simulator:getWorldState(0):globalFrame(ibone)
		-- spatial coordinate to bone global coordinate
		local R_dAd= liegroup.invdAd(transf(GT.rotation,vector3(0,0,0))*GT:inverse())
		V:range(i*6, (i+1)*6, 0, a):assign(R_dAd*V_temp)

		local iboneVel=-1*simulator:getWorldVelocity(0, simulator:skeleton(0):VRMLbone(ibone), vector3(0,0,0)):copy()
		dot_R_dAd:range(0,3,3,6):assign(CT.skew(iboneVel):copy())
		dotV:range(i*6, (i+1)*6, 0, a):assign(dot_R_dAd*V_temp + R_dAd*dotV_temp)
	end
end

function calcContactJacobianAll_new(simulator, bases, numDof, numContactBodies, J, dJ, V, dV)
	J:setSize(6*numContactBodies, numDof)
	dJ:setSize(6*numContactBodies, numDof)

	local numBasis = bases:size() * (1+bases(0).frictionNormal:size()) 
	V:setSize(6*numContactBodies, numBasis)
	dV:setSize(6*numContactBodies, numBasis)

	local J_i = matrixn(6,numDof)
	local dJ_i = matrixn(6,numDof)
	local V_i = matrixn(6,numBasis)
	local dV_i = matrixn(6,numBasis)

	local link_ibodies = intvectorn()
	local link_ibones = intvectorn()
	simulator:getLinkPairBodiesBones(link_ibodies, link_ibones)

	local dot_R_dAd=matrixn()
	dot_R_dAd:setSize(6,6)
	dot_R_dAd:setAllValue(0)

	for i=0,numContactBodies-1 do

		-- J_i (jacobian of body i): 6 x dof
		local ibody = intvectorn.get(link_ibodies, i)
		local ibone = intvectorn.get(link_ibones, i)
		simulator:calcJacobian(ibody, ibone, J_i)
		simulator:calcDotJacobian(ibody, ibone, dJ_i)

		V_i:setAllValue(0)
		dV_i:setAllValue(0)

		for c=0, bases:size()-1 do
			b = bases(c)

			-- V_i (all bases) : 6 x numBasis
			local basisPos = b.globalpos:copy()	
		
			local normalBasis = b.normal:copy()
			local normalBasisIndex = b.globalIndex

			local V_i_col = V_i:column(normalBasisIndex)
			V_i_col:setVec3(0, basisPos:cross(normalBasis))
			V_i_col:setVec3(3, normalBasis)

			local dot_basisPos = -1*b.relvel:copy()
			local dV_i_col = dV_i:column(normalBasisIndex)
			dV_i_col:setVec3(0, dot_basisPos:cross(normalBasis))

			local frictionBases = b.frictionNormal
			local firstFrictionBasisIndex = b.globalFrictionIndex
			for j=0,frictionBases:size()-1 do
				local V_i_col = V_i:column(firstFrictionBasisIndex+j)
				V_i_col:setVec3(0, basisPos:cross(frictionBases(j)))
				V_i_col:setVec3(3, frictionBases(j))

				local dV_i_col = dV_i:column(firstFrictionBasisIndex+j)
				dV_i_col:setVec3(0, dot_basisPos:cross(frictionBases(j)))
			end
		end
		
		-- append
		J:range(i*6,(i+1)*6,0,numDof):assign(J_i)
		dJ:range(i*6,(i+1)*6,0,numDof):assign(dJ_i)
		
		local GT=simulator:getWorldState(0):globalFrame(ibone)
		-- spatial coordinate to bone global coordinate
		local R_dAd= liegroup.invdAd(transf(GT.rotation,vector3(0,0,0))*GT:inverse())
		V:range(i*6, (i+1)*6, 0, numBasis):assign(R_dAd*V_i)

		local iboneVel=-1*simulator:getWorldVelocity(0, simulator:skeleton(0):VRMLbone(ibone), vector3(0,0,0)):copy()
		dot_R_dAd:range(0,3,3,6):assign(CT.skew(iboneVel):copy())
		dV:range(i*6, (i+1)*6, 0, numBasis):assign(dot_R_dAd*V_i + R_dAd*dV_i)
	end
end

-- contact force in generalized coord 
-- = Jc.T*Vcb*lambda
-- Jc : Jacobian to origins of contact bodies in global frame (#body*6 x #dof)
-- Vcb : coord transform * basis vectors of all contact points (#body*6 x #basis)
-- lambda : coeff. of all basis vectors (#basis x 1)
--
-- muscle force in generalized coord 
-- = Ja.T*Vaf*faf
-- = Ja.T*Vaf*C*ft
-- = Ja.T*Vaf*C*P*A*a + Ja.T*Vaf*C*P*p
--
-- Ja : Jacobian to origins of all bodies in global frame (#body*6 x #dof)
--
-- Vaf : coord transform * unit direction vectors of acting forces at pathpoints
-- 		(not equal to #pathpoints) (#body*6 x #actingforce)
-- (example)
-- Vaf =	   -------------------
--			   | | | | | | | | | |
--			   | | | | | | | | | |
--			   | | | | | | | | | |
--			   | | | | | | | | | |
--			   | | | | | | | | | |
--			   | | | | | | | | | |
--			   -------------------
-- actf idx		0 1 2 3 4 5 6 7 8
-- mscl idx 	0 0 0 1 1 2 3 4 4
--
-- C : converting matrix from muscle-indexing vector to pathpoint-indexing 
-- 		vector (#actingforce x #muscle)
-- (example of above Vaf case)
-- 				C			f
--  		-----------	  ----   ----
--			|1|0|0|0|0|   |f0|   |f0|
--          |1|0|0|0|0|   |f1|   |f0|
--          |1|.|.|.| | x |f2| = |f0|
--          | |1| | | |   |f3|   |f1|
--          | |1| | | |   |f4|   |f1|
--          | | |1| | |   ----   |f2|
--          | | | |1| |          |f3|
--          | | | | |1|          |f4|
--          | | | | |1|          |f4|
-- 			-----------          ----
--
-- faf : acting forces (#actingforce x 1)
-- ft : muscle tendon forces (#muscle x 1)
-- a : muscle activations (#muscle x 1)
--
-- (linearized activation-force relationship)
-- f_m_new = a_new * gal(l_m_prev)*gv(dl_m_prev) + gpl(l_m_prev) + b*dl_m_prev
--
-- P : digonal matrix of pennation angle cosine (#muscle x #muscle)
--
-- A : digonal matrix of active force component (#muscle x #muscle)
-- A = diag(gal(l_m_prev)*gv(dl_m_prev))
--
-- p : passive force and damping force (#muscle x 1)
-- p = gpl(l_m_prev) + b*dl_m_prv
--

function calcMuscleRelatedMatrices(simulator, osim, numDof, Ja, Vaf)
	local numJoint = osim:getNumJoints()
	local numMuscle = osim:getNumMuscles()

	local afnums_joint = {}
	local afpoints = {}
	local afdirecs = {}
	for i_joint=1,numJoint do
		local points = osim:getActingForcePoints(i_joint)
		local directions = osim:getActingForceDirections(i_joint)

		table.insert(afnums_joint, #points)
		extend_array(afpoints, points)
		extend_array(afdirecs, directions)
	end
	local numActForce = #afpoints

	Ja:setSize(numJoint*6, numDof)

	Vaf:setSize(numJoint*6, numActForce)

	local J_i = matrixn(6,numDof)
	local Vaf_i = matrixn(6,numActForce)
	
	local start_j_af_ofjoint = 1

	for i_joint=1,numJoint do
		local i = i_joint-1
		local ibody = 0
		-- same
		--local ibone = osim.mLoader:getTreeIndexByName(osim:getJointNames()[i_joint])
		local ibone = i_joint

		--print(i_joint, osim:getJointNames()[i_joint])

		simulator:calcJacobian(ibody, ibone, J_i)

		Vaf_i:setAllValue(0)

		local numActForce_ofjoint = afnums_joint[i_joint]

		--print('numActForce_ofjoint', numActForce_ofjoint)

		for j_af=start_j_af_ofjoint, start_j_af_ofjoint+numActForce_ofjoint-1 do
			local j = j_af - 1
			local Vaf_i_j = Vaf_i:column(j)

			local p = afpoints[j_af]
			local d = afdirecs[j_af]

			--print('j_af', j_af)
			--print(p:cross(d))
			--print(d)

			Vaf_i_j:setVec3(0, p:cross(d))
			Vaf_i_j:setVec3(3, d)
		end

		--printmatn(Vaf)

		Ja:range(i*6,(i+1)*6,0,numDof):assign(J_i)

		local GT=simulator:getWorldState(ibody):globalFrame(ibone)
		-- spatial coordinate to bone global coordinate
		local R_dAd= liegroup.invdAd(transf(GT.rotation,vector3(0,0,0))*GT:inverse())
		Vaf:range(i*6, (i+1)*6, 0, numActForce):assign(R_dAd*Vaf_i)

		--printmatn(Vaf)

		start_j_af_ofjoint = start_j_af_ofjoint + numActForce_ofjoint
	end
	--printmatn(Vaf)
end
