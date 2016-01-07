import numpy
import prpy.tsr

@prpy.tsr.tsrlibrary.TSRFactory('BarrettWAM', 'fuze_bottle', 'grasp')
def fuze_grasp(robot, fuze, manip=None):
    """
    @param robot The robot performing the grasp
    @param fuze The fuze to grasp
    @param manip The manipulator to perform the grasp, if None
       the active manipulator on the robot is used
    """
    return _fuze_grasp(robot, fuze, manip = manip)

@prpy.tsr.tsrlibrary.TSRFactory('BarrettWAM', 'fuze_bottle', 'push_grasp')
def fuze_grasp(robot, fuze, push_distance = 0.1, manip=None):
    """
    @param robot The robot performing the grasp
    @param fuze The fuze to grasp
    @param push_distance The distance to push before grasping
    @param manip The manipulator to perform the grasp, if None
       the active manipulator on the robot is used
    """
    return _fuze_grasp(robot, fuze, push_distance = push_distance, manip = manip)

def _fuze_grasp(robot, fuze, push_distance = 0.0, manip = None):
    """
    @param robot The robot performing the grasp
    @param fuze The fuze to grasp
    @param push_distance The distance to push before grasping
    @param manip The manipulator to perform the grasp, if None
       the active manipulator on the robot is used
    """
    if manip is None:
        manip_idx = robot.GetActiveManipulatorIndex()
    else:
        with manip.GetRobot():
            manip.SetActive()
            manip_idx = manip.GetRobot().GetActiveManipulatorIndex()

    T0_w = fuze.GetTransform()
    ee_to_palm_distance = 0.0
    #default_offset_distance = 0.05 # This is the radius of the fuze plus a little bit
    default_offset_distance = -0.022 # Need a negative offset due to position of palm axis

    total_offset = ee_to_palm_distance + default_offset_distance + push_distance
    print total_offset
    Tw_e = numpy.array([[0., 0., 1., -total_offset], 
                        [1., 0., 0., 0.], 
                        [0., 1., 0., 0.06], # 0.108 is half of fuze bottle height
                        [0., 0., 0., 1.]])

    Bw = numpy.zeros((6,2))
    Bw[2,:] = [0.0, 0.02]  # Allow a little vertical movement
    Bw[5,:] = [-numpy.pi, numpy.pi]  # Allow any orientation
    
    grasp_tsr = prpy.tsr.TSR(T0_w = T0_w, Tw_e = Tw_e, Bw = Bw, manip = manip_idx)
    grasp_chain = prpy.tsr.TSRChain(sample_start=False, sample_goal = True, 
                                    constrain=False, TSR = grasp_tsr)

    return [grasp_chain]
