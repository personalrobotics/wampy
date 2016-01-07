#!/usr/bin/env python

import openravepy
import prpy
from prpy.base import MobileBase
import numpy
import logging

logger = logging.getLogger('wampy')

class WAMBase(MobileBase):
    def __init__(self, sim, robot):
        MobileBase.__init__(self, sim=sim, robot=robot)
        self.controller = robot.AttachController(name=robot.GetName(),
                                                 #args='SegwayController {0:s}'.format('herbpy'),
                                                 args='NavigationController {0:s} {1:s}'.format('wampy', '/navcontroller'),
                                                 dof_indices=[],
                                                 affine_dofs=openravepy.DOFAffine.Transform,
                                                 simulated=sim)

    def CloneBindings(self, parent):
        MobileBase.CloneBindings(self, parent)
        
    """
    def Forward(self, meters, execute=True, timeout=None, **kwargs):

        Drive forward for the desired distance.
        @param distance distance to drive, in meters
        @param timeout time in seconds; pass \p None to block until complete 
        @return base trajectory

        if self.simulated or not execute:
            return MobileBase.Forward(self, meters, execute=execute, timeout=timeout,  **kwargs)
        else:
            with prpy.util.Timer("Drive segway"):
                self.controller.SendCommand("Drive " + str(meters))
                is_done = prpy.util.WaitForControllers([ self.controller ], timeout=timeout)

    def Rotate(self, angle_rad, execute=True, timeout=None, **kwargs):

        Rotate in place by a desired angle
        @param angle angle to turn, in radians
        @param timeout time in seconds; pass \p None to block until complete 
        @return base trajectory

        if self.simulated or not execute:
            return MobileBase.Rotate(self, angle_rad, execute=execute, timeout=timeout, **kwargs)
        else:
            with prpy.util.Timer("Rotate segway"):
                self.controller.SendCommand("Rotate " + str(angle_rad))
                running_controllers = [ self.controller ]
                is_done = prpy.util.WaitForControllers(running_controllers, timeout=timeout)

    def DriveAlongVector(self, direction, goal_pos):

        Rotate to face the given direction, then drive to the goal position.
        @param direction A 2 or 3-element direction vector
        @param goal_pos A 2 or 3-element position vector (world frame)

        import numpy
        direction = numpy.array(direction[:2]) / numpy.linalg.norm(direction[:2])
        robot_pose = self.robot.GetTransform()
        distance = numpy.dot(numpy.array(goal_pos[:2]) - robot_pose[:2, 3], direction)
        cur_angle = numpy.arctan2(robot_pose[1, 0], robot_pose[0, 0])
        des_angle = numpy.arctan2(direction[1], direction[0])
        self.Rotate(des_angle - cur_angle)
        self.Drive(distance)
    """