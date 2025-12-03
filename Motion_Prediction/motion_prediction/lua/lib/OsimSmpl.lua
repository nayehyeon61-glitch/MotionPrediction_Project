local out={}
out.volumes={
	-- todo: build quadratic regression, 1000 samples
	male={
		beta_0={ 0.090836088546582	, boneLength=4.5390683145534},
		beta1={
			{ value=-3, volume=0.12968623860617, 	boneLength=4.466391345974},
			{ value=-1.5, volume=0.1081953356908	, boneLength=4.5015519090294,},
			{ value=1.5, volume=0.076975480561462, boneLength=	4.5790725960948},
			{ value=3, volume=0.066118513946362, boneLength=4.6217196565285},
		},
		beta8={
			-- upperbody muscle (-3)
			{ value=-3, volume=0.092151384786842, boneLength=4.5451478391512},
			{ value=3, volume=0.094037534279216, boneLength=4.5370552779794},
		},
		beta9={
			-- lowerbody muscle
			{ value=-3, volume= 0.095157731268663, boneLength=	4.5463842313532},
			{ value=3, volume= 0.090728537278582, boneLength=	4.532121531237},
		},
	}
}

function out.fitOsimParser(tbl, _scaleInfo)
	local scaleInfo=deepCopyTable(_scaleInfo)
	local default=scaleInfo.default
	scaleInfo.default=nil
	for i, bone in ipairs(tbl.bones) do 
		local s=copyTable(default)
		local sb=scaleInfo[bone.name]
		if sb then table.mergeInPlace(s, sb, true) end
		scaleInfo[bone.name]=s

		bone.localCOM:rmult(s.lengthScale)
		bone.geomTransform=s.geomTransform:copy()
		bone.mass=bone.mass*s.massScale
		bone.inertia=bone.inertia*s.massScale
	end
	for i, bone in ipairs(tbl.bones) do 
		if i>1 then
			local s=scaleInfo[tbl.bones[bone.pid].name]
			bone.offset:rmult(s.lengthScale)
		end
	end

	for i, muscle in ipairs(tbl.muscles) do
		local pps=muscle.GeometryPath.PathPointSet
		if #pps>0  then
			local sm=scaleInfo[pps[1].joint]
			local sm2=scaleInfo[pps[#pps].joint]

			--muscle.FmaxMuscleStrain=muscle.FmaxMuscleStrain*
			--(sm.lengthScale*0.5+sm2.lengthScale*0.5)
			--muscle.FmaxTendonStrain=muscle.FmaxTendonStrain*
			--(sm.lengthScale*0.5+sm2.lengthScale*0.5)
			muscle.max_isometric_force=muscle.max_isometric_force*
			(sm.massScale*0.5+sm2.massScale*0.5)
		end

		for k, pp in ipairs(pps) do
			local v=vector3(pp.location_wrl[1], pp.location_wrl[2], pp.location_wrl[3])
			local sb=scaleInfo[pp.joint]
			v=sb.geomTransform*v
			pp.location_wrl={v.x, v.y, v.z}
		end
	end
end
-- option_ == redundant bones
function out.gotoSMPL_Tpose(osimLoader,option_)
	local pose=osimLoader:pose()
	local height=0

	if not option_ then
		height=parser.loader:getBoneByName('toes_l'):getFrame().translation.y
	else
		height=parser.loader:getBoneByName('talus_l'):getFrame().translation.y
	end
	--pose.translations(0).y=0.98
	pose.translations(0).y=pose.translations(0).y-height

	local forwardTilt1=0
	local forwardTilt2=3
	local jT=osimLoader:getRotJointIndexByName('torso')
	pose.rotations(jT):assign(quater(math.rad(forwardTilt1), vector3(0,0,-1)))
	pose.rotations(0):assign(quater(math.rad(forwardTilt2), vector3(1,0,0))*quater(math.rad(-90), vector3(0,1,0)))
	pose.translations(0).z=forwardTilt2*0.02
	local jL=osimLoader:getRotJointIndexByName('humerus_l')
	local jR=osimLoader:getRotJointIndexByName('humerus_r')
	local jL2=osimLoader:getRotJointIndexByName('femur_l')
	local jL2=osimLoader:getRotJointIndexByName('femur_r')
	pose.rotations(jL):assign(quater(math.rad(-90), vector3(0,0,1))* quater(math.rad(90), vector3(1,0,0)))
	pose.rotations(jR):assign(quater(math.rad(-90), vector3(0,0,1))* quater(math.rad(90), vector3(-1,0,0)))

	osimLoader:setPose(pose)
end

function out.findBetas(dd, osimLoader, option_)
	-- vertices
	--dd.v_shaped=dd.shapedirs:dotProduct(dd.betas)+dd.v_template
	-- joint locations
	--dd.J=dd.J_regressor*dd.v_shaped
	if not dd.J_regress_shape then

		assert(dd.shapedirs:rows()==3)
		dd.J_regress_shape=hypermatrixn(3, dd.J_regressor:rows(), dd.betas:size())


		for i=0, dd.shapedirs:rows()-1 do
			dd.J_regress_shape:page(i):mult(dd.J_regressor, dd.shapedirs:row(i))
		end

		dd.J_regress_template=dd.J_regressor*dd.v_template

		-- dd.J:column(i)==dd.J_regress_shape:page(i)*dd.betas:column()+dd.J_regressor*dd.v_template:column(i)
		--
		local function genPair(n1, n2, offset)
			return { array.locate(SMPL.SMPL_JOINT_NAMES, n1)-1, osimLoader:getTreeIndexByName(n2), offset }
		end
		if not option_ then
		dd.correspondences={
			genPair("left_foot", "toes_l"),
			genPair("left_ankle",'talus_l'),
			genPair("left_knee",'tibia_l'),
			genPair("left_hip",'femur_l'),
			genPair("left_wrist",'hand_l'),
			genPair("left_elbow",'radius_l'),
			genPair("left_shoulder",'humerus_l'),
			genPair("right_foot",'toes_r'),
			genPair("right_ankle",'talus_r'),
			genPair("right_knee",'tibia_r'),
			genPair("right_hip",'femur_r'),
			genPair("right_wrist",'hand_r'),
			genPair("right_elbow",'radius_r'),
			genPair("right_shoulder",'humerus_r'),
			--genPair("neck",'head', vector3(0,0.40,0)),
		}
		else
		dd.correspondences={
			genPair("left_ankle",'talus_l'),
			genPair("left_knee",'tibia_l'),
			genPair("left_hip",'femur_l'),
			genPair("left_elbow",'radius_l'),
			genPair("left_shoulder",'humerus_l'),
			genPair("right_ankle",'talus_r'),
			genPair("right_knee",'tibia_r'),
			genPair("right_hip",'femur_r'),
			genPair("right_elbow",'radius_r'),
			genPair("right_shoulder",'humerus_r'),
		}

		end
	end

	local height=dd.J(dd.correspondences[1][1], 1)*-1 
	--
	local func=QuadraticFunction()
	local selected=boolN(dd.J:rows())
	selected:setAllValue(false)

	local desired=matrixn(dd.J:rows(),3)
	local footPos=osimLoader:bone(dd.correspondences[1][2]):getFrame().translation
	local hipPos=osimLoader:bone(dd.correspondences[4][2]):getFrame().translation
	
	for i, v in ipairs(dd.correspondences) do
		local jindex=v[1]
		selected:set(jindex, true)
		if v[3] then
			desired:row(jindex):setVec3(0, osimLoader:bone(v[2]):getFrame().translation+v[3])
		else
			desired:row(jindex):setVec3(0, osimLoader:bone(v[2]):getFrame().translation)
		end
		--desired:set(jindex,1,desired(jindex,1)-footPos.y)
		desired:set(jindex,1,desired(jindex,1)-hipPos.y)
	end

	do

		local nvar=dd.betas:size()
		
		--- w*(Ax-b)^2 (only for selected rows)
		-- desired x location
		-- dd.J_regress_shape:page(0)*betas +dd.J_regress_template:column(0)
		local b=desired:column(0)
		func:addSystemSelectedRows(1, dd.J_regress_shape:page(0), b-dd.J_regress_template:column(0), selected)


		-- desired z location
		local b=desired:column(2)
		func:addSystemSelectedRows(1, dd.J_regress_shape:page(2), b-dd.J_regress_template:column(2), selected)
		
		-- desired y location 
		-- y= dd.J_regress_shape:page(1)*x +dd.J_regress_template:column(1)
		-- footY= dd.J_regress_shape:page(1):row(dd.correspondences[1][1])*x +dd.J_regress_template:column(1)
		--local footIndex=dd.correspondences[1][1]
		local hipIndex=dd.correspondences[4][1]

		local A=dd.J_regress_shape:page(1):copy()
		local b=desired:column(1):copy()
		for i=0, A:rows()-1 do
			--A:row(i):rsub(dd.J_regress_shape:page(1):row(footIndex))
			A:row(i):rsub(dd.J_regress_shape:page(1):row(hipIndex))
			--b:set(i, b(i)+dd.J_regress_template( footIndex,1))
			b:set(i, b(i)+dd.J_regress_template( hipIndex,1))
		end

		func:addSystemSelectedRows(1, dd.J_regress_shape:page(1), b-dd.J_regress_template:column(1), selected)

		-- regularization
		func:addSystem(0.001, CT.eye(nvar), CT.zeros(nvar))

		local x=func:solve(nvar)
		dd.betas:assign(x)
		SMPL.updateShape(dd, mesh)
		redrawSMPL()

		--out.drawCorrespondences(dd, osimLoader, vector3(1,0,0))
	end
end
function out.drawCorrespondences( dd, osimLoader, offset)
	local height=dd.J(dd.correspondences[1][1], 1)*-1 
	local lines=vector3N()
	local joints=vector3N()
	for i, v in ipairs(dd.correspondences) do
		lines:pushBack( dd.J:row(v[1]):toVector3()+vector3(0, height,0))

		local dpos
		if v[3] then
			dpos= osimLoader:bone(v[2]):getFrame().translation+v[3]
		else
			dpos= osimLoader:bone(v[2]):getFrame().translation
		end
		lines:pushBack(dpos)

		lines:pushBack( dd.J:row(v[1]):toVector3()+vector3(0, height,0)+offset)
		lines:pushBack( dpos+offset)
		joints:pushBack( dpos+offset)
	end

	local thickness=2
	dbg.drawBillboard( lines:matView()*100, 'corr'..offset.x, 'solidgreen', thickness, 'BillboardLineList' )
	dbg.drawBillboard( joints:matView()*100, 'jcorr'..offset.x, 'blueCircle', thickness*2, 'QuadListV' )
end

function out.alignPoses(dd, osimLoader, option_)
	local function genPair(n1, n2, offset)
		return { array.locate(SMPL.SMPL_JOINT_NAMES, n1)-1, osimLoader:getTreeIndexByName(n2), offset }
	end
	local alignInfo
	if not option_ then
	alignInfo={
		{
			genPair("left_shoulder",'humerus_l'),
			genPair("left_elbow",'radius_l'),
			genPair("left_wrist",'hand_l'),
		},
		{
			genPair("right_shoulder",'humerus_r'),
			genPair("right_elbow",'radius_r'),
			genPair("right_wrist",'hand_r'),
		},
		{
			genPair("left_hip",'femur_l'),
			genPair("left_knee",'tibia_l'),
			genPair("left_ankle",'talus_l'),
			genPair("left_foot", "toes_l"),
		},
		{
			genPair("right_hip",'femur_r'),
			genPair("right_knee",'tibia_r'),
			genPair("right_ankle",'talus_r'),
			genPair("right_foot",'toes_r'),
		},
	}

	else
	alignInfo={
		{
			genPair("left_shoulder",'humerus_l'),
			genPair("left_elbow",'radius_l'),
		},
		{
			genPair("right_shoulder",'humerus_r'),
			genPair("right_elbow",'radius_r'),
		},
		{
			genPair("left_hip",'femur_l'),
			genPair("left_knee",'tibia_l'),
			genPair("left_ankle",'talus_l'),
		},
		{
			genPair("right_hip",'femur_r'),
			genPair("right_knee",'tibia_r'),
			genPair("right_ankle",'talus_r'),
		},
	}

	end
	local sh_to_hip1=dd.J:row(alignInfo[1][1][1])-dd.J:row(alignInfo[3][1][1])
	local sh_to_hip2=dd.J:row(alignInfo[2][1][1])-dd.J:row(alignInfo[4][1][1])

	local function getPos(i,j)
		return osimLoader:bone(alignInfo[i][j][2]):getFrame().translation
	end

	local sh_to_hip_smpl=sh_to_hip1:toVector3()*0.5+sh_to_hip2:toVector3()*0.5
	local sh_to_hip_osim=(getPos(1,1)-getPos(3,1))*0.5+(getPos(2,1)-getPos(4,1))*0.5
	local function align(v1, v2, treeIndex)
		local q=quater()
		q:axisToAxis(v2, v1)
		osimLoader:rotateBoneGlobal(osimLoader:bone(treeIndex), q)
	end
	local jT=osimLoader:getTreeIndexByName('torso')
	align(sh_to_hip_smpl, sh_to_hip_osim, jT)

	for i, v in ipairs(alignInfo) do

		for j=2, #v do
			local parent_to_child_smpl=dd.J:row(v[j][1])-dd.J:row(v[j-1][1])
			local parent_to_child_osim=getPos(i, j)-getPos(i, j-1)

			align(parent_to_child_smpl:toVector3(), parent_to_child_osim, v[j-1][2])
		end
	end
	--out.drawCorrespondences(dd, osimLoader, vector3(2,0,0))

end

return out
