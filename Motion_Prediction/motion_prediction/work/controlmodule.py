import luamodule as lua
m=lua.taesooLib()

# ported from PDservo_spd.lua
class PoseMaintainer:
    def __init__(self, skeletonIndex):
        assert(skeletonIndex)
        self.theta=m.vectorn()
        self.dtheta=m.vectorn()
        self.theta_d=m.vectorn() # desired q
        self.dtheta_d=m.vectorn() # desired dq
        self.controlforce=m.vectorn()

        self.skeletonIndex=skeletonIndex or 0
        # followings are temporaries
        self.kp=m.vectorn()
        self.kd=m.vectorn()


    def init(self, skel, simulator, k_p, k_d, k_p_slide, k_d_slide):
        si=self.skeletonIndex
        simulator.getLinkData(si, m.Physics.JOINT_VALUE, self.theta_d)
        simulator.getLinkData(si, m.Physics.JOINT_VELOCITY, self.dtheta_d)

        dofInfo=skel.dofInfo
        self.kp.setSize(dofInfo.numDOF())
        self.kp.setAllValue(k_p)
        self.kd.setSize(dofInfo.numDOF())
        self.kd.setAllValue(k_d)
        
        if skel.VRMLbone(1).HRPjointType(0)==MainLib.VRMLTransform.FREE :
            # exclude free root joint
            self.kp.range(0,7).setAllValue(0)
            self.kd.range(0,7).setAllValue(0)

        
        if k_p_slide==nil :
           k_p_slide=k_p*10


        if k_d_slide==nil :
           k_d_slide=k_d*500


        for i in range(1,skel.numBone()-1 +1) :
            vbone=skel.VRMLbone(i)
            nJoint=vbone.numHRPjoints()
            for j in range(0, nJoint-1 +1) :
                if vbone.HRPjointType(j)==MainLib.VRMLTransform.SLIDE :
                    self.kp.set(vbone.DOFindex(j), k_p_slide)
                    self.kd.set(vbone.DOFindex(j), k_d_slide)





    def generateTorque(self, simulator):
        si=self.skeletonIndex
        simulator.getLinkData(si, m.Physics.JOINT_VALUE, self.theta)
        simulator.getLinkData(si, m.Physics.JOINT_VELOCITY, self.dtheta)

        tau=m.vectorn()
        simulator.calculateStablePDForces(self.skeletonIndex, self.poseToQ(self.theta_d),tau)

        self.controlforce=PDservo.DQtoDpose(tau)

    def resetParam(self, kp, kd, theta_d):
        self.kp.setAllValue(kp)
        self.kd.setAllValue(kd)
        self.theta_d.assign(theta_d)


class PDservo:
    def setCoef(self, dofInfo,kp, kd, tgtVelScale, k_scale, model):
        assert(dofInfo.numSphericalJoint()==1)
        # spherical joint가 있는 경우 PDservo_spherical 사용할 것!
        kp.setSize(dofInfo.numDOF())
        k_p=model.k_p_PD
        k_d=model.k_d_PD
        kp.setAllValue(k_p)
        kd.setSize(dofInfo.numDOF())
        kd.setAllValue(k_d)

        tgtVelScale.setSize(dofInfo.numDOF())
        tgtVelScale.setAllValue(model.k_d_PD)

        # exclude root joint
        kp.range(0,7).setAllValue(0)
        kd.range(0,7).setAllValue(0)
        tgtVelScale.range(0,7).setAllValue(0)

        print("initPDservo."+dofInfo.skeleton().bone(1).name())
        for i in range(2,dofInfo.skeleton().numBone()-1 +1) :
            bone=dofInfo.skeleton().bone(i)
            vbone=bone.treeIndex()
            nJoint=dofInfo.numDOF(vbone)
            #      print("initPDservo."..bone.name())
            for j in range(0, nJoint-1 +1) :

                dofIndex=dofInfo.DOFindex(vbone,j)

                kp.set(dofIndex, k_p*k_scale.default[1])
                kd.set(dofIndex, k_d*k_scale.default[2])
                tgtVelScale.set(dofIndex, k_scale.default[3])

                if bone.voca()==m.Voca.LEFTANKLE or bone.voca()==m.Voca.RIGHTANKLE :
                    if k_scale.ankle :
                        kp.set(dofIndex, k_p*k_scale.ankle[1])
                        kd.set(dofIndex, k_d*k_scale.ankle[2])
                        tgtVelScale.set(dofIndex, k_scale.ankle[3])

                elif bone.voca()==m.Voca.LEFTCOLLAR or bone.voca()==m.Voca.RIGHTCOLLAR :
                    if k_scale.collar :
                        kp.set(dofIndex, k_p*k_scale.collar[1])
                        kd.set(dofIndex, k_d*k_scale.collar[2])
                        tgtVelScale.set(dofIndex, k_scale.collar[3])

                elif bone.voca()==m.Voca.LEFTSHOULDER or bone.voca()==m.Voca.RIGHTSHOULDER :
                    if k_scale.shoulder :
                        kp.set(dofIndex, k_p*k_scale.shoulder[1])
                        kd.set(dofIndex, k_d*k_scale.shoulder[2])
                        tgtVelScale.set(dofIndex, k_scale.shoulder[3])

                elif bone.voca()==m.Voca.LEFTELBOW or bone.voca()==m.Voca.RIGHTELBOW :
                    if k_scale.elbow :
                        kp.set(dofIndex, k_p*k_scale.elbow[1])
                        kd.set(dofIndex, k_d*k_scale.elbow[2])
                        tgtVelScale.set(dofIndex, k_scale.elbow[3])

                elif bone.voca()==m.Voca.LEFTKNEE or bone.voca()==m.Voca.RIGHTKNEE :
                    if k_scale.knee :
                        kp.set(dofIndex, k_p*k_scale.knee[1])
                        kd.set(dofIndex, k_d*k_scale.knee[2])
                        tgtVelScale.set(dofIndex, k_scale.knee[3])

                elif bone.voca()==m.Voca.LEFTHIP or bone.voca()==m.Voca.RIGHTHIP :
                    if k_scale.hip :
                        kp.set(dofIndex, k_p*k_scale.hip[1])
                        kd.set(dofIndex, k_d*k_scale.hip[2])
                        tgtVelScale.set(dofIndex, k_scale.hip[3])

                elif bone.voca()==m.Voca.CHEST :
                    if k_scale.chest :
                        kp.set(dofIndex, k_p*k_scale.chest[1])
                        kd.set(dofIndex, k_d*k_scale.chest[2])
                        tgtVelScale.set(dofIndex, k_scale.chest[3])

                elif bone.voca()==m.Voca.CHEST2 :
                    if k_scale.chest2 :
                        kp.set(dofIndex, k_p*k_scale.chest2[1])
                        kd.set(dofIndex, k_d*k_scale.chest2[2])
                        tgtVelScale.set(dofIndex, k_scale.chest2[3])

                elif bone.voca()==m.Voca.NECK :
                    if k_scale.neck :
                        kp.set(dofIndex, k_p*k_scale.neck[1])
                        kd.set(dofIndex, k_d*k_scale.neck[2])
                        tgtVelScale.set(dofIndex, k_scale.neck[3])

                elif bone.voca()==m.Voca.HEAD :
                    if k_scale.head :
                        kp.set(dofIndex, k_p*k_scale.head[1])
                        kd.set(dofIndex, k_d*k_scale.head[2])
                        tgtVelScale.set(dofIndex, k_scale.head[3])


                if "toes" in bone.name() :
                    dofIndex=dofInfo.DOFindex(vbone,j)
                    if k_scale.toes :
                        kp.set(dofIndex, k_p*k_scale.toes[1])
                        kd.set(dofIndex, k_d*k_scale.toes[2])
                        tgtVelScale.set(dofIndex, k_scale.toes[3])




                if dofInfo.DOFtype(vbone, j)==m.DOFtype.SLIDE :
                    dofIndex=dofInfo.DOFindex(vbone,j)
                    kp.set(dofIndex, model.k_p_slide)
                    kd.set(dofIndex, model.k_d_slide)
                    tgtVelScale.set(dofIndex, 0)




    def __init__(self, dofInfo, model):
        assert(dofInfo.numSphericalJoint()==1) # otherwise, use PDservo_spherical instead.
        self.model=model
        self.theta=m.vectorn()
        self.dtheta=m.vectorn()
        self.theta_d=m.vectorn() # desired q
        self.dtheta_d=m.vectorn() # desired dq
        self.controlforce=m.vectorn()
        self.kp=m.vectorn()
        self.kd=m.vectorn()
        self.tgtVelScale=m.vectorn()
        self.mask_slide=m.vectorn()
        self.muscleActiveness=0.3
        self.mask_slide.setSize(dofInfo.numDOF())
        self.mask_slide.setAllValue(0)
        self.dofInfo=dofInfo
        self.updateCoef(model)
        print ("kp=",self.kp)
        print ("kd=",self.kd)

        clampTorque=800
        clampForce=8000

        if 'clampTorque' in model :
            clampTorque=model.clampTorque


        if 'clampForce' in model :
            clampForce=model.clampForce


        self.clampMax=m.vectorn(dofInfo.numDOF())
        self.clampMax.setAllValue(clampTorque)
        for i in range(2,dofInfo.skeleton().numBone()-1 +1) :
            bone=dofInfo.skeleton().bone(i)
            vbone=bone.treeIndex()
            nJoint=dofInfo.numDOF(vbone)
            for j in range(0, nJoint-1 +1) :
                dofIndex=dofInfo.DOFindex(vbone,j)
                if dofInfo.DOFtype(vbone, j)==m.DOFtype.SLIDE :
                    dofIndex=dofInfo.DOFindex(vbone,j)
                    self.mask_slide.set(dofIndex, 1)
                    self.clampMax.set(dofIndex, clampForce)
                else:
                    self.clampMax.set(dofIndex, clampTorque)




        self.clampMin=self.clampMax*-1

    def updateCoef(self, model):
        dofInfo=self.dofInfo
        k_scale_active=model.k_scale_active_pd

        self.setCoef(dofInfo,self.kp, self.kd, self.tgtVelScale, k_scale_active, model)


    def dposeToDQ(dpose):
        return dpose.slice(0,3)|dpose.slice(7,0)|dpose.slice(4,7)

    def DQtoDpose(dq):
        return dq.slice(0,3)| lua.zeros(1)|dq.slice(-3,0)|dq.slice(3,-3)

    def poseToQ(pose):
        return pose.slice(0,3)|pose.slice(7,0)|pose.slice(3,7)


    def initPDservo(self, startf, endf,motionDOF, dmotionDOF, simulator):
        self.startFrame=startf
        self.endFrame=endf
        self.currFrame=startf
        self.deltaTime=0
        self.motionDOF=motionDOF
        self.dmotionDOF=dmotionDOF

        dofInfo=self.dofInfo
        assert(dofInfo.numSphericalJoint()==1)

        assert(self.kp.size()==dofInfo.numDOF())


        self.skeletonIndex=0


        simulator.setStablePDparam(self.skeletonIndex, PDservo.dposeToDQ(self.kp), PDservo.dposeToDQ(self.kd))


    # generate FBtorque
    def generateTorque(self, simulator):
        model=self.model
       
        self.currFrame=(simulator.currentTime()+self.deltaTime)*model.frame_rate+self.startFrame
        #print(self.currFrame) # extremely slow.
        if self.currFrame>self.endFrame-1 :
            simulator.getLinkData(0, m.Physics.JOINT_VALUE, self.theta)
            simulator.getLinkData(0, m.Physics.JOINT_VELOCITY, self.dtheta)
            return False


        self._generateTorque(simulator, self.currFrame)
        return True


    #gTimer=util.Timer()
    def stepSimul(self, simulator, drawDebugInformation=None):
        simulator.setLinkData(0, m.Physics.JOINT_TORQUE, self.controlforce)
        if drawDebugInformation :
            simulator.drawDebugInformation()

        #gTimer.start()
        simulator.stepSimulation()
        #print(gTimer.stop2())


    def _generateTorque(self, simulator, frame, target_delta=None):
       
        simulator.getLinkData(0, m.Physics.JOINT_VALUE, self.theta)
        simulator.getLinkData(0, m.Physics.JOINT_VELOCITY, self.dtheta)

        #[[ continuous sampling ]]#
        #   print("theta",self.theta)

        # desired (target) pose
        self.motionDOF.samplePose(frame, self.theta_d)
        if target_delta :
            self.theta_d.radd(target_delta) 
        self.dmotionDOF.sampleRow(frame, self.dtheta_d)

        #   self.dtheta_d.setAllValue(0)

        self.dtheta_d.rmult(self.muscleActiveness) # this corresponds to muscle activeness

        self.controlforce.setSize(self.motionDOF.numDOF())

        #delta=self.theta_d-self.theta
        #MainLib.VRMLloader.projectAngles(delta) # [-pi, pi]
        #self.controlforce.assign(self.kp*delta + self.kd*(self.dtheta_d*self.tgtVelScale-self.dtheta))


        tau=m.vectorn()
        simulator.calculateStablePDForces(self.skeletonIndex, PDservo.poseToQ(self.theta_d),tau)

        self.tau=tau
        self.controlforce=PDservo.DQtoDpose(tau)


        self.controlforce.clamp(self.clampMin, self.clampMax)


    def rewindTargetMotion(self, simulator):
        self.deltaTime=-1*simulator.currentTime()




class PDservo_spherical:

    # returns motQ, motDQ which are compatible with loader_spherical
    def convertMotionState(loader_euler, loader_spherical, motionDOF_euler, frame_rate):

        DMotionDOF_euler=lua.M1_matrixn( motionDOF_euler, 'calcDerivative', frame_rate)

        # important!!!
        # convert loader, motionDOF, and its time-derivative to new formats.

        nf=motionDOF_euler.numFrames()
        motQ=m.matrixn(nf, loader_spherical.dofInfo.numDOF())
        motDQ=m.matrixn(nf, loader_spherical.dofInfo.numActualDOF())

        tree=m.LoaderToTree(loader_euler, False,False)

        euler_dofInfo=loader_euler.dofInfo
        spherical_dofInfo=loader_spherical.dofInfo

        for i in range(0, nf-1 +1) :
            tree.setPoseDOF(euler_dofInfo, motionDOF_euler.row(i))
            tree.setVelocity(euler_dofInfo, DMotionDOF_euler.row(i))

            tree.getSphericalState(spherical_dofInfo, motQ.row(i), motDQ.row(i))

        return motQ, motDQ


    def __init__(self, dofInfo, _model):
        self.theta=m.vectorn()
        self.dtheta=m.vectorn()
        self.theta_d=m.vectorn() # desired q
        self.dtheta_d=m.vectorn() # desired dq
        self.controlforce=m.vectorn()
        self.dofInfo=dofInfo
        self.skeletonIndex=0
        self.model=_model or model


    def initPDservo(self, startf, endf,motQ, motDQ, simulator, ichara):
        csize=m.vector3()
        dofInfo=simulator.skeleton(ichara).dofInfo
        model=self.model
        csize.y=simulator.numSphericalJoints(ichara)
        csize.x=dofInfo.numDOF() -csize.y*4

        # for the root joint and other 1-DOF joints
        self.kp=m.vectorn(int(csize.x+csize.y*3))
        self.kd=m.vectorn(int(csize.x+csize.y*3))
        self.kp.setAllValue(self.model.k_p_PD)
        self.kd.setAllValue(self.model.k_d_PD)
        
        # exclude root translation
        self.kp.range(0,3).setAllValue(0)
        self.kp.range(0,3).setAllValue(0)


        clampTorque=800
        clampForce=8000

        if 'clampTorque' in model:
            clampTorque=model.clampTorque


        if 'clampForce' in model:
            clampForce=model.clampForce

        self.clampMax=m.vectorn(int(csize.x+csize.y*3))
        self.clampMax.setAllValue(clampTorque)
        self.clampMin=self.clampMax*-1

        self.startFrame=startf
        self.endFrame=endf
        self.currFrame=startf
        self.deltaTime=0

        q=m.vectorn()
        dq=m.vectorn()
        simulator.initSimulation()
        simulator.getSphericalState(ichara, q, dq)

        self.nonQsize=csize.x
        self.motions=[motQ, motDQ]
        simulator.setStablePDparam(self.skeletonIndex, self.kp, self.kd)


    def generateTorque(self, simulator):

        model=self.model
        self.currFrame=(simulator.currentTime()+self.deltaTime)*model.frame_rate+self.startFrame
        #print(self.currFrame) # extremely slow.
        if self.currFrame>self.endFrame-1 :
            return False


        self._generateTorque(simulator, self.currFrame)
        return True


    def stepSimul(self, simulator, drawDebugInformation=None):
        simulator.setTau(0, self.controlforce)
        if drawDebugInformation :
            simulator.drawDebugInformation()

        simulator.stepSimulation()


    def _generateTorque(self, simulator, frame):

        self.motions[0].sampleRow(frame, self.theta_d)
        self.motions[1].sampleRow(frame, self.dtheta_d)

        simulator.calculateStablePDForces(self.skeletonIndex, self.theta_d, self.controlforce)

        self.controlforce.clamp(self.clampMin, self.clampMax)


    def rewindTargetMotion(self, simulator):
        self.deltaTime=-1*simulator.currentTime()



class PoseMaintainer_spherical:

    def __init__(self, skeletonIndex):
        self.theta=m.vectorn()
        self.dtheta=m.vectorn()
        self.theta_d=m.vectorn() # desired q
        self.dtheta_d=m.vectorn() # desired dq
        self.controlforce=m.vectorn()

        self.skeletonIndex=skeletonIndex or 0




    def init(self, skel, simulator, k_p, k_d):
        csize=m.vector3()
        ichara=self.skeletonIndex
        dofInfo=simulator.skeleton(ichara).dofInfo
        csize.y=simulator.numSphericalJoints(ichara)
        csize.x=dofInfo.numDOF() -csize.y*4

        # for the root joint and other 1-DOF joints
        self.kp=m.vectorn(csize.x+csize.y*3)
        self.kd=m.vectorn(csize.x+csize.y*3)
        self.kp.setAllValue(k_p)
        self.kd.setAllValue(k_d)
        
        if dofInfo.hasTranslation(1) :
            # exclude root translation
            self.kp.range(0,3).setAllValue(0)
            self.kd.range(0,3).setAllValue(0)
            self.freeRoot=True


        print ("kp=",self.kp)
        print ("kd=",self.kd)

        self.startFrame=startf
        self.endFrame=endf
        self.currFrame=startf
        self.deltaTime=0

        simulator.initSimulation()
        q=m.vectorn()
        dq=m.vectorn()
        simulator.getSphericalState(self.skeletonIndex, q, dq)

        self.nonQsize=q.size()-csize.y*4
        assert(q.toQuater(self.nonQsize).length()>0.99)
        self.theta_d=q
        self.dtheta_d=dq
        self.dtheta_d.zero()

        simulator.setStablePDparam(self.skeletonIndex, self.kp, self.kd)


    def stepSimul(self, simulator, drawDebugInformation=None):
        simulator.setTau(self.skeletonIndex, self.controlforce)
        if drawDebugInformation :
            simulator.drawDebugInformation()

        simulator.stepSimulation()


    def generateTorque(self, simulator):

        simulator.getSphericalState(self.skeletonIndex, self.theta, self.dtheta)
        simulator.calculateStablePDForces(self.skeletonIndex, self.theta_d, self.controlforce)

        # self.controlforce.clamp(self.clampMin, self.clampMax)


def registerContactPairAll(model, loader, floor, simulator):
    param=m.vectorn()
    if model and 'penaltyForceStiffness' in model:
        param.assign([0.5,0.5, model.penaltyForceStiffness, model.penaltyForceDamp])
    else:
        param.assign([0.5,0.5, 10000, 1000])
    for i in range(1,loader.numBone()-1 +1) :
        for j in range(1, floor.numBone()-1+1):
            bone_i=loader.VRMLbone(i)
            simulator.registerCollisionCheckPair(loader.name(),bone_i.name(), floor.name(), floor.bone(j).name(), param)
