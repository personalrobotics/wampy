PACKAGE = 'wampy'

import logging
import openravepy
import prpy
import prpy.rave
import prpy.util
from prpy.base.barretthand import BarrettHand
from prpy.base.wam import WAM
from prpy.base.robot import Robot
from prpy.exceptions import PrPyException
from prpy.planning.base import UnsupportedPlanningError
from wambase import WAMBase

logger = logging.getLogger('wampy')


def try_and_warn(fn, exception_type, message, default_value=None):
    try:
        return fn()
    except exception_type:
        logger.warning(message)
        return None


class WAMRobot(Robot):
    def __init__(self, arm_sim,
                       hand_sim,
                       ft_sim,
                       segway_sim,
                       ):
        from prpy.util import FindCatkinResource

        Robot.__init__(self, robot_name='BarrettWAM')

        # Convenience attributes for accessing self components.
        self.arm = self.GetManipulator('arm')

        self.arm.hand = self.arm.GetEndEffector()
        self.hand = self.arm.hand # 'handbase'

        self.manipulators = [ self.arm ]

        # Dynamically switch to self-specific subclasses.
        prpy.bind_subclass(self.arm, WAM, sim=arm_sim, owd_namespace='/arm/owd')

        # namespace must be set, even for sim
        prpy.bind_subclass(self.arm.hand, BarrettHand, sim=hand_sim, manipulator=self.arm,
                           owd_namespace='/arm/owd', bhd_namespace='/arm/bhd', ft_sim=ft_sim)

        self.base = WAMBase(sim=segway_sim, robot=self)
        #self.base = WAMBase(sim=base_sim, robot=self)

        # Set WAM acceleration limits. These are not specified in URDF.
        accel_limits = self.GetDOFAccelerationLimits()
        accel_limits[self.arm.GetArmIndices()] = [ 2. ] * self.arm.GetArmDOF()
        self.SetDOFAccelerationLimits(accel_limits)
        
        # Support for named configurations.
        import os.path
        self.configurations.add_group('arm', self.arm.GetArmIndices())
        self.configurations.add_group('hand', self.hand.GetIndices())
        configurations_path = FindCatkinResource('wampy', 'config/configurations.yaml')
        try:
            self.configurations.load_yaml(configurations_path)
        except IOError as e:
            raise ValueError('Failed laoding named configurations from "{:s}".'.format(
                configurations_path))

        # Initialize a default planning pipeline.
        from prpy.planning import (
            FirstSupported,
            MethodMask,
            Ranked,
            Sequence,
        )
        from prpy.planning import (
            CBiRRTPlanner,
            CHOMPPlanner,
            GreedyIKPlanner,
            IKPlanner,
            NamedPlanner,
            SBPLPlanner,
            SnapPlanner,
            TSRPlanner,
            VectorFieldPlanner
        )

        # TODO: These should be meta-planners.
        self.named_planner = NamedPlanner()
        self.ik_planner = IKPlanner()

        # Special-purpose planners.
        self.snap_planner = SnapPlanner()
        self.vectorfield_planner = VectorFieldPlanner()
        self.greedyik_planner = GreedyIKPlanner()

        # General-purpose planners.
        self.cbirrt_planner = CBiRRTPlanner()

        # Trajectory optimizer.
        try:
            from or_trajopt import TrajoptPlanner

            self.trajopt_planner = TrajoptPlanner()
        except ImportError:
            self.trajopt_planner = None
            logger.warning('Failed creating TrajoptPlanner. Is the or_trajopt'
                           ' package in your workspace and built?')

        try:
            self.chomp_planner = CHOMPPlanner()
        except UnsupportedPlanningError:
            self.chomp_planner = None
            logger.warning('Failed loading the CHOMP module. Is the or_cdchomp'
                           ' package in your workspace and built?')

        if not (self.trajopt_planner or self.chomp_planner):
            raise PrPyException('Unable to load both CHOMP and TrajOpt. At'
                                ' least one of these packages is required.')
        
        actual_planner = Sequence(
            # First, try the straight-line trajectory.
            self.snap_planner,
            # Then, try a few simple (and fast!) heuristics.
            self.vectorfield_planner,
            #self.greedyik_planner,
            # Next, try a trajectory optimizer.
            self.trajopt_planner or self.chomp_planner
        )
        self.planner = FirstSupported(
            Sequence(actual_planner, 
                     TSRPlanner(delegate_planner=actual_planner),
                     self.cbirrt_planner),
            # Special purpose meta-planner.
            NamedPlanner(delegate_planner=actual_planner),
        )
        
        from prpy.planning.retimer import HauserParabolicSmoother
        self.smoother = HauserParabolicSmoother(do_blend=True, do_shortcut=True)
        self.retimer = HauserParabolicSmoother(do_blend=True, do_shortcut=False,
            blend_iterations=5, blend_radius=0.4)
        self.simplifier = None

        # Base planning
        from prpy.util import FindCatkinResource
        planner_parameters_path = FindCatkinResource('wampy', 'config/base_planner_parameters.yaml')

        self.sbpl_planner = SBPLPlanner()
        try:
            with open(planner_parameters_path, 'rb') as config_file:
                import yaml
                params_yaml = yaml.load(config_file)
            self.sbpl_planner.SetPlannerParameters(params_yaml)
        except IOError as e:
            raise ValueError('Failed loading base planner parameters from "{:s}".'.format(
                planner_parameters_path))

        self.base_planner = self.sbpl_planner

        # Create action library
        from prpy.action import ActionLibrary
        self.actions = ActionLibrary()

        # Register default actions and TSRs
        import wampy.action
        import wampy.tsr

        # Setting necessary sim flags
        self.segway_sim = segway_sim

    def CloneBindings(self, parent):
        from prpy import Cloned
        super(WAMRobot, self).CloneBindings(parent)
        self.arm = Cloned(parent.arm)
        self.arm.hand = Cloned(parent.arm.GetEndEffector())
        self.hand = self.arm.hand
        self.manipulators = [ self.arm ]
        self.planner = parent.planner
        self.base_planner = parent.base_planner

    def ExecuteTrajectory(self, traj, *args, **kwargs):
        from prpy.exceptions import TrajectoryAborted

        active_manipulators = self.GetTrajectoryManipulators(traj)

        for manipulator in active_manipulators:
            manipulator.ClearTrajectoryStatus()

        value = super(WAMRobot, self).ExecuteTrajectory(traj, *args, **kwargs)

        for manipulator in active_manipulators:
            status = manipulator.GetTrajectoryStatus()
            if status == 'aborted':
                raise TrajectoryAborted()

        return value

    # Inherit docstring from the parent class.
    ExecuteTrajectory.__doc__ = Robot.ExecuteTrajectory.__doc__
