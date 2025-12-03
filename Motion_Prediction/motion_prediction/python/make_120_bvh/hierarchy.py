def optitrack_hierarchy():
    hirearchy_string = """HIERARCHY
ROOT Hips
{
    OFFSET 0.000000 0.000000 0.000000
    CHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation
    JOINT Spine
    {
        OFFSET -0.000001 9.999228 0.000003
        CHANNELS 3 Zrotation Xrotation Yrotation
        JOINT Spine1
        {
            OFFSET 0.000002 20.755894 -0.000002
            CHANNELS 3 Zrotation Xrotation Yrotation
            JOINT Neck
            {
                OFFSET 0.000006 17.346041 -0.000002
                CHANNELS 3 Zrotation Xrotation Yrotation
                JOINT Head
                {
                    OFFSET 0.000003 11.759543 1.951037
                    CHANNELS 3 Zrotation Xrotation Yrotation
                    End Site
                    {
                        OFFSET 0.000000 19.510290 0.000000
                    }
                }
            }
            JOINT LeftShoulder
            {
                OFFSET 3.901517 16.007399 -0.091854
                CHANNELS 3 Zrotation Xrotation Yrotation
                JOINT LeftArm
                {
                    OFFSET 17.724688 0.000002 0.000004
                    CHANNELS 3 Zrotation Xrotation Yrotation
                    JOINT LeftForeArm
                    {
                        OFFSET 27.839609 0.000004 -0.000003
                        CHANNELS 3 Zrotation Xrotation Yrotation
                        JOINT LeftHand
                        {
                            OFFSET 26.988998 -0.000001 -0.000010
                            CHANNELS 3 Zrotation Xrotation Yrotation
                            JOINT LeftHandThumb1
                            {
                                OFFSET 2.038827 -2.316851 3.243599
                                CHANNELS 3 Zrotation Xrotation Yrotation
                                JOINT LeftHandThumb2
                                {
                                    OFFSET 3.079338 -0.000002 0.000005
                                    CHANNELS 3 Zrotation Xrotation Yrotation
                                    JOINT LeftHandThumb3
                                    {
                                        OFFSET 2.631937 -0.000006 -0.000004
                                        CHANNELS 3 Zrotation Xrotation Yrotation
                                        End Site
                                        {
                                            OFFSET 2.567496 0.000000 0.000000
                                        }
                                    }
                                }
                            }
                            JOINT LeftHandIndex1
                            {
                                OFFSET 8.340647 0.000002 3.243593
                                CHANNELS 3 Zrotation Xrotation Yrotation
                                JOINT LeftHandIndex2
                                {
                                    OFFSET 4.633691 0.000003 0.000002
                                    CHANNELS 3 Zrotation Xrotation Yrotation
                                    JOINT LeftHandIndex3
                                    {
                                        OFFSET 2.316849 0.000001 -0.000006
                                        CHANNELS 3 Zrotation Xrotation Yrotation
                                        End Site
                                        {
                                            OFFSET 2.260121 0.000000 0.000000
                                        }
                                    }
                                }
                            }
                            JOINT LeftHandMiddle1
                            {
                                OFFSET 8.340652 0.000002 1.075026
                                CHANNELS 3 Zrotation Xrotation Yrotation
                                JOINT LeftHandMiddle2
                                {
                                    OFFSET 5.097061 -0.000002 0.000002
                                    CHANNELS 3 Zrotation Xrotation Yrotation
                                    JOINT LeftHandMiddle3
                                    {
                                        OFFSET 2.780223 0.000004 -0.000009
                                        CHANNELS 3 Zrotation Xrotation Yrotation
                                        End Site
                                        {
                                            OFFSET 2.712145 0.000000 0.000000
                                        }
                                    }
                                }
                            }
                            JOINT LeftHandRing1
                            {
                                OFFSET 7.877277 0.000000 -1.075015
                                CHANNELS 3 Zrotation Xrotation Yrotation
                                JOINT LeftHandRing2
                                {
                                    OFFSET 4.633693 0.000004 -0.000009
                                    CHANNELS 3 Zrotation Xrotation Yrotation
                                    JOINT LeftHandRing3
                                    {
                                        OFFSET 2.316844 -0.000003 0.000009
                                        CHANNELS 3 Zrotation Xrotation Yrotation
                                        End Site
                                        {
                                            OFFSET 2.260121 0.000000 0.000000
                                        }
                                    }
                                }
                            }
                            JOINT LeftHandPinky1
                            {
                                OFFSET 7.413911 -0.000000 -3.243588
                                CHANNELS 3 Zrotation Xrotation Yrotation
                                JOINT LeftHandPinky2
                                {
                                    OFFSET 3.706963 0.000005 -0.000007
                                    CHANNELS 3 Zrotation Xrotation Yrotation
                                    JOINT LeftHandPinky3
                                    {
                                        OFFSET 1.853475 -0.000004 0.000005
                                        CHANNELS 3 Zrotation Xrotation Yrotation
                                        End Site
                                        {
                                            OFFSET 1.808097 0.000000 0.000000
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            JOINT RightShoulder
            {
                OFFSET -3.901518 16.007402 -0.091860
                CHANNELS 3 Zrotation Xrotation Yrotation
                JOINT RightArm
                {
                    OFFSET -17.724699 -0.000002 -0.000003
                    CHANNELS 3 Zrotation Xrotation Yrotation
                    JOINT RightForeArm
                    {
                        OFFSET -27.839598 -0.000008 -0.000001
                        CHANNELS 3 Zrotation Xrotation Yrotation
                        JOINT RightHand
                        {
                            OFFSET -26.988995 -0.000001 -0.000004
                            CHANNELS 3 Zrotation Xrotation Yrotation
                            JOINT RightHandThumb1
                            {
                                OFFSET -2.038831 -2.316857 3.243591
                                CHANNELS 3 Zrotation Xrotation Yrotation
                                JOINT RightHandThumb2
                                {
                                    OFFSET -3.079338 0.000001 0.000003
                                    CHANNELS 3 Zrotation Xrotation Yrotation
                                    JOINT RightHandThumb3
                                    {
                                        OFFSET -2.631940 0.000001 0.000003
                                        CHANNELS 3 Zrotation Xrotation Yrotation
                                        End Site
                                        {
                                            OFFSET -2.567496 0.000000 0.000000
                                        }
                                    }
                                }
                            }
                            JOINT RightHandIndex1
                            {
                                OFFSET -8.340650 0.000006 3.243589
                                CHANNELS 3 Zrotation Xrotation Yrotation
                                JOINT RightHandIndex2
                                {
                                    OFFSET -4.633693 0.000002 0.000000
                                    CHANNELS 3 Zrotation Xrotation Yrotation
                                    JOINT RightHandIndex3
                                    {
                                        OFFSET -2.316854 0.000001 0.000005
                                        CHANNELS 3 Zrotation Xrotation Yrotation
                                        End Site
                                        {
                                            OFFSET -2.260121 0.000000 0.000000
                                        }
                                    }
                                }
                            }
                            JOINT RightHandMiddle1
                            {
                                OFFSET -8.340650 0.000001 1.075024
                                CHANNELS 3 Zrotation Xrotation Yrotation
                                JOINT RightHandMiddle2
                                {
                                    OFFSET -5.097059 -0.000006 -0.000002
                                    CHANNELS 3 Zrotation Xrotation Yrotation
                                    JOINT RightHandMiddle3
                                    {
                                        OFFSET -2.780220 0.000005 -0.000000
                                        CHANNELS 3 Zrotation Xrotation Yrotation
                                        End Site
                                        {
                                            OFFSET -2.712145 0.000000 0.000000
                                        }
                                    }
                                }
                            }
                            JOINT RightHandRing1
                            {
                                OFFSET -7.877277 -0.000005 -1.075021
                                CHANNELS 3 Zrotation Xrotation Yrotation
                                JOINT RightHandRing2
                                {
                                    OFFSET -4.633687 -0.000006 0.000003
                                    CHANNELS 3 Zrotation Xrotation Yrotation
                                    JOINT RightHandRing3
                                    {
                                        OFFSET -2.316854 0.000006 -0.000003
                                        CHANNELS 3 Zrotation Xrotation Yrotation
                                        End Site
                                        {
                                            OFFSET -2.260121 0.000000 0.000000
                                        }
                                    }
                                }
                            }
                            JOINT RightHandPinky1
                            {
                                OFFSET -7.413904 0.000001 -3.243593
                                CHANNELS 3 Zrotation Xrotation Yrotation
                                JOINT RightHandPinky2
                                {
                                    OFFSET -3.706960 0.000002 0.000001
                                    CHANNELS 3 Zrotation Xrotation Yrotation
                                    JOINT RightHandPinky3
                                    {
                                        OFFSET -1.853475 0.000004 -0.000002
                                        CHANNELS 3 Zrotation Xrotation Yrotation
                                        End Site
                                        {
                                            OFFSET -1.808097 0.000000 0.000000
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    JOINT LeftUpLeg
    {
        OFFSET 9.755154 -0.000005 0.000000
        CHANNELS 3 Zrotation Xrotation Yrotation
        JOINT LeftLeg
        {
            OFFSET 0.000002 -47.924694 -0.000002
            CHANNELS 3 Zrotation Xrotation Yrotation
            JOINT LeftFoot
            {
                OFFSET -0.000004 -47.266193 0.000006
                CHANNELS 3 Zrotation Xrotation Yrotation
                JOINT LeftToeBase
                {
                    OFFSET 0.000005 -6.340840 14.632693
                    CHANNELS 3 Zrotation Xrotation Yrotation
                    End Site
                    {
                        OFFSET 0.000000 0.000000 3.902058
                    }
                }
            }
        }
    }
    JOINT RightUpLeg
    {
        OFFSET -9.755154 0.000005 -0.000000
        CHANNELS 3 Zrotation Xrotation Yrotation
        JOINT RightLeg
        {
            OFFSET 0.000002 -47.924694 0.000005
            CHANNELS 3 Zrotation Xrotation Yrotation
            JOINT RightFoot
            {
                OFFSET -0.000001 -47.266193 0.000005
                CHANNELS 3 Zrotation Xrotation Yrotation
                JOINT RightToeBase
                {
                    OFFSET 0.000003 -6.340840 14.632710
                    CHANNELS 3 Zrotation Xrotation Yrotation
                    End Site
                    {
                        OFFSET 0.000000 0.000000 3.902058
                    }
                }
            }
        }
    }
}
MOTION
"""
    return hirearchy_string


def optitrack_etri_hierarchy():
    hierarchy_string = """HIERARCHY
ROOT pelvis
{
  OFFSET 0.0 0.0 0.00
  CHANNELS 6 Xposition Yposition Zposition Zrotation Yrotation Xrotation
  JOINT waist
  {
    OFFSET 0 20.377175 -1.500002
    CHANNELS 3 Zrotation Xrotation Yrotation
    JOINT neck
    {
      OFFSET 0.000007 27.723988 1.500003
      CHANNELS 3 Zrotation Xrotation Yrotation
      JOINT forehead
      {
        OFFSET 0.000003 11.759543 1.951037
        CHANNELS 3 Zrotation Xrotation Yrotation
        End Site
        {
          OFFSET 0.000000 19.510290 0.000000
        }
      }
    }
    JOINT r_shoulder
    {
      OFFSET -21.626216 26.385347 -1.408153
      CHANNELS 3 Zrotation Xrotation Yrotation
      JOINT r_elbow
      {
        OFFSET -27.839598 -0.000008 -0.000001
        CHANNELS 3 Zrotation Xrotation Yrotation
        JOINT r_wrist
        {
          OFFSET -26.988995 -0.000001 -0.000004
          CHANNELS 3 Zrotation Xrotation Yrotation
          End Site
          {
            OFFSET 0.0 0.0 0.0
          }
        }
      }
    }
    JOINT l_shoulder
    {
      OFFSET 21.026216 29.385347 -2.408153
      CHANNELS 3 Zrotation Xrotation Yrotation
      JOINT l_elbow
      {
        OFFSET 27.839609 0.000004 -0.000003
        CHANNELS 3 Zrotation Xrotation Yrotation
        JOINT l_wrist
        {
          OFFSET 26.988998 -0.000001 -0.000010
          CHANNELS 3 Zrotation Xrotation Yrotation
          End Site
          {
            OFFSET 0.0 0.0 0.0
          }
        }
      }
    }
  }
  JOINT r_hip
  {
    OFFSET -9.755154 0.000005 -0.000000
    CHANNELS 3 Zrotation Xrotation Yrotation
    JOINT r_knee
    {
      OFFSET 0.000002 -47.924694 0.000005
      CHANNELS 3 Zrotation Xrotation Yrotation
      JOINT r_ankle
      {
        OFFSET -0.000001 -47.266193 0.000005
        CHANNELS 3 Zrotation Xrotation Yrotation
        JOINT r_toe
        {
          OFFSET 0.000003 -6.340840 14.632710
          CHANNELS 3 Zrotation Xrotation Yrotation
          End Site
          {
            OFFSET 0.000000 0.000000 3.902058
          }
        }
      }
    }
  }
  JOINT l_hip
  {
    OFFSET 9.755154 -0.000005 0.000000
    CHANNELS 3 Zrotation Xrotation Yrotation
    JOINT l_knee
    {
      OFFSET 0.000002 -47.924694 -0.000002
      CHANNELS 3 Zrotation Xrotation Yrotation
      JOINT l_ankle
      {
        OFFSET -0.000004 -47.266193 0.000006
        CHANNELS 3 Zrotation Xrotation Yrotation
        JOINT l_toe
        {
          OFFSET 0.000005 -6.340840 14.632693
          CHANNELS 3 Zrotation Xrotation Yrotation
          End Site
          {
            OFFSET 0.000000 0.000000 3.902058
          }
        }
      }
    }
  }
}
MOTION
"""
    return hierarchy_string


def xsens_hierarchy():
    hierarchy_string = """HIERARCHY
ROOT Hips
{
 OFFSET 0.000000 0.000000 0.000000
 CHANNELS 6 Xposition Yposition Zposition Yrotation Xrotation Zrotation
 JOINT Chest
 {
  OFFSET 0.000000 9.817266 -1.090808
  CHANNELS 3 Yrotation Xrotation Zrotation
  JOINT Chest2
  {
   OFFSET 0.000000 10.878341 0.000000
   CHANNELS 3 Yrotation Xrotation Zrotation
   JOINT Chest3
   {
    OFFSET 0.000000 9.932046 -0.002339
    CHANNELS 3 Yrotation Xrotation Zrotation
    JOINT Chest4
    {
     OFFSET 0.000000 9.920956 -0.002341
     CHANNELS 3 Yrotation Xrotation Zrotation
     JOINT Neck
     {
      OFFSET 0.000000 13.897299 0.000000
      CHANNELS 3 Yrotation Xrotation Zrotation
      JOINT Head
      {
       OFFSET 0.000000 9.237018 0.005030
       CHANNELS 3 Yrotation Xrotation Zrotation
       End Site
       {
        OFFSET 0.000000 17.155989 0.000677
       }
      }
     }
     JOINT RightCollar
     {
      OFFSET -3.028356 7.791542 0.000000
      CHANNELS 3 Yrotation Xrotation Zrotation
      JOINT RightShoulder
      {
       OFFSET -14.177729 0.000000 0.000000
       CHANNELS 3 Yrotation Xrotation Zrotation
       JOINT RightElbow
       {
        OFFSET -30.364375 0.000000 0.000000
        CHANNELS 3 Yrotation Xrotation Zrotation
        JOINT RightWrist
        {
         OFFSET -24.820744 0.000000 0.000000
         CHANNELS 3 Yrotation Xrotation Zrotation
         End Site
         {
          OFFSET -18.530853 0.000000 0.000000
         }
        }
       }
      }
     }
     JOINT LeftCollar
     {
      OFFSET 3.028356 7.791542 0.000000
      CHANNELS 3 Yrotation Xrotation Zrotation
      JOINT LeftShoulder
      {
       OFFSET 14.177729 0.000000 0.000000
       CHANNELS 3 Yrotation Xrotation Zrotation
       JOINT LeftElbow
       {
        OFFSET 30.364375 0.000000 0.000000
        CHANNELS 3 Yrotation Xrotation Zrotation
        JOINT LeftWrist
        {
         OFFSET 24.820744 0.000000 0.000000
         CHANNELS 3 Yrotation Xrotation Zrotation
         End Site
         {
          OFFSET 18.530853 0.000000 0.000000
         }
        }
       }
      }
     }
    }
   }
  }
 }
 JOINT RightHip
 {
  OFFSET -8.081064 0.006831 -0.000759
  CHANNELS 3 Yrotation Xrotation Zrotation
  JOINT RightKnee
  {
   OFFSET 0.000000 -41.932394 -0.000553
   CHANNELS 3 Yrotation Xrotation Zrotation
   JOINT RightAnkle
   {
    OFFSET 0.000000 -40.919741 -0.000931
    CHANNELS 3 Yrotation Xrotation Zrotation
    JOINT RightToe
    {
     OFFSET 0.000000 -7.124271 16.180034
     CHANNELS 3 Yrotation Xrotation Zrotation
     End Site
     {
      OFFSET 0.000000 -1.571563 6.815722
     }
    }
   }
  }
 }
 JOINT LeftHip
 {
  OFFSET 8.081064 0.004620 -0.000513
  CHANNELS 3 Yrotation Xrotation Zrotation
  JOINT LeftKnee
  {
   OFFSET 0.000000 -41.932394 -0.000553
   CHANNELS 3 Yrotation Xrotation Zrotation
   JOINT LeftAnkle
   {
    OFFSET 0.000000 -40.919741 -0.000931
    CHANNELS 3 Yrotation Xrotation Zrotation
    JOINT LeftToe
    {
     OFFSET 0.000000 -7.124271 16.180034
     CHANNELS 3 Yrotation Xrotation Zrotation
     End Site
     {
      OFFSET 0.000000 -1.571563 6.815722
     }
    }
   }
  }
 }
}
MOTION
"""
    return hierarchy_string
      

def xsens_etri_hierarchy():
    hierarchy_string = """HIERARCHY
ROOT pelvis
{
  OFFSET 0.0 0.0 0.00
  CHANNELS 6 Xposition Yposition Zposition Zrotation Yrotation Xrotation
  JOINT waist
  {
    OFFSET 0.000000 20.695607 -1.090808
    CHANNELS 3 Zrotation Xrotation Yrotation
    JOINT neck
    {
      OFFSET 0.000000 33.750301 -0.00468
      CHANNELS 3 Zrotation Xrotation Yrotation
      JOINT forehead
      {
        OFFSET 0.000000 9.237018 0.005030
        CHANNELS 3 Zrotation Xrotation Yrotation
        End Site
        {
          OFFSET 0.000000 17.155989 0.000677
        }
      }
    }
    JOINT r_shoulder
    {
      OFFSET -17.206085 27.644544 -0.00468
      CHANNELS 3 Zrotation Xrotation Yrotation
      JOINT r_elbow
      {
        OFFSET -30.364375 0.000000 0.000000
        CHANNELS 3 Zrotation Xrotation Yrotation
        JOINT r_wrist
        {
          OFFSET -24.820744 0.000000 0.000000
          CHANNELS 3 Zrotation Xrotation Yrotation
          End Site
          {
            OFFSET -18.530853 0.000000 0.000000
          }
        }
      }
    }
    JOINT l_shoulder
    {
      OFFSET 17.206085 27.644544 -3.00468
      CHANNELS 3 Zrotation Xrotation Yrotation
      JOINT l_elbow
      {
        OFFSET 30.364375 0.000000 0.000000
        CHANNELS 3 Zrotation Xrotation Yrotation
        JOINT l_wrist
        {
          OFFSET 24.820744 0.000000 0.000000
          CHANNELS 3 Zrotation Xrotation Yrotation
          End Site
          {
            OFFSET 18.530853 0.000000 0.000000
          }
        }
      }
    }
  }
  JOINT r_hip
  {
    OFFSET -8.081064 0.006831 -0.000759
    CHANNELS 3 Zrotation Xrotation Yrotation
    JOINT r_knee
    {
      OFFSET 0.000000 -41.932394 -0.000553
      CHANNELS 3 Zrotation Xrotation Yrotation
      JOINT r_ankle
      {
        OFFSET 0.000000 -40.919741 -0.0009315
        CHANNELS 3 Zrotation Xrotation Yrotation
        JOINT r_toe
        {
          OFFSET 0.000000 -7.124271 16.180034
          CHANNELS 3 Zrotation Xrotation Yrotation
          End Site
          {
            OFFSET 0.000000 -1.571563 6.815722
          }
        }
      }
    }
  }
  JOINT l_hip
  {
    OFFSET 8.081064 0.004620 -0.000513
    CHANNELS 3 Zrotation Xrotation Yrotation
    JOINT l_knee
    {
      OFFSET 0.000000 -41.932394 -0.000553
      CHANNELS 3 Zrotation Xrotation Yrotation
      JOINT l_ankle
      {
        OFFSET 0.000000 -40.919741 -0.000931
        CHANNELS 3 Zrotation Xrotation Yrotation
        JOINT l_toe
        {
          OFFSET 0.000000 -7.124271 16.180034
          CHANNELS 3 Zrotation Xrotation Yrotation
          End Site
          {
            OFFSET 0.000000 -1.571563 6.815722
          }
        }
      }
    }
  }
}
MOTION
"""
    return hierarchy_string