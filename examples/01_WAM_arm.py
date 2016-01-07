#!/usr/bin/env python

import os
import sys # exit()
import openravepy
import prpy
import prpy.planning
import wampy
import numpy
import roslib
import rospy
from tf import transformations # rotation_matrix(), concatenate_matrices()

# Find the robots directory
from catkin.find_in_workspaces import find_in_workspaces
package_name = 'wampy'
directory = 'robots'
robots_path = find_in_workspaces(
    search_dirs=['share'],
    project=package_name,
    path=directory,
    first_match_only=True)
if len(robots_path) == 0:
    print('Can\'t find directory %s/%s' % (package_name, directory))
    sys.exit()
else:
    robots_path = robots_path[0]

# Use robot model of the Barett WAM arm
robot_file = os.path.join(robots_path, 'barrettwam.robot.xml')

# Initialize OpenRAVE robot of type wampy.wamrobot.WAMRobot:
#env, robot = wampy.initialize(sim=True, attach_viewer=True)
#env, robot = wampy.initialize(sim=True, attach_viewer='qtcoin')
env, robot = wampy.initialize(robot_xml=robot_file, sim=True,
                              attach_viewer='rviz')
#env, robot = wampy.initialize(robot_xml=robot_file, sim=True,
#                              attach_viewer='interactivemarker

manipulator = robot.GetManipulator('arm')

# Set 7 DOF of the arm active
robot.SetActiveDOFs( manipulator.GetArmIndices() )
robot.SetActiveManipulator( manipulator )

# By default, the base is not at the world origin
# so we move the robot to the origin
trans = numpy.array([[1., 0., 0., 0.0],
                     [0., 1., 0., 0.0],
                     [0., 0., 1., 0.0],
                     [0., 0., 0., 1.]])
robot.SetTransform(trans)

# we do this so the viewer doesn't close when the example is done
import IPython;
IPython.embed()
