#!/usr/bin/env python

import os
import sys # exit()
import openravepy
import prpy
import prpy.planning
import wampy
import roslib
import rospy
import numpy
from tf import transformations # rotation_matrix(), concatenate_matrices()

# Initialize OpenRAVE robot of type wampy.wamrobot.WAMRobot:
#env, robot = wampy.initialize(sim=True, attach_viewer=True)
#env, robot = wampy.initialize(sim=True, attach_viewer='qtcoin')
env, robot = wampy.initialize(sim=True, attach_viewer='rviz')
#env, robot = wampy.initialize(robot_xml=robot_file, sim=True,
#                              attach_viewer='interactivemarker

manipulator = robot.GetManipulator('arm')

# Set 7 DOF of the arm active
robot.SetActiveDOFs( manipulator.GetArmIndices() )
robot.SetActiveManipulator( manipulator )

# Find the pr_ordata package, which contains some object models
from catkin.find_in_workspaces import find_in_workspaces
package_name = 'pr_ordata'
directory = 'data/objects'
objects_path = find_in_workspaces(
    search_dirs=['share'],
    project=package_name,
    path=directory,
    first_match_only=True)
if len(objects_path) == 0:
    print('Can\'t find directory %s/%s' % (package_name, directory))
    sys.exit()
else:
    print objects_path
    objects_path = objects_path[0]

# Add floor to the environment
floor = openravepy.RaveCreateKinBody(env,'')
floor.SetName('floor')
# Set geometry as one box of extents 2.0,2.0,0.005
# -0.02 stops collision errors between Segway base and floor.
floor.InitFromBoxes(numpy.array([[0,0,-0.02, 2.0,2.0,0.005]]),True)
floor.GetLinks()[0].SetStatic(True)
for igeom,geom in enumerate(floor.GetLinks()[0].GetGeometries()):
    color = [0.6, 0.6, 0.6]
    #color[igeom] = 1
    geom.SetDiffuseColor(color)
    geom.SetAmbientColor(color)
env.AddKinBody(floor)

# Add a table to the environment
table_file = os.path.join(objects_path, 'table.kinbody.xml')
table = env.ReadKinBodyXMLFile(table_file)
if table == None:
    print('Failed to load table kinbody')
    sys.exit()
env.AddKinBody(table)
# Rotate so table surface is horizontal
Rot_x = transformations.rotation_matrix(numpy.pi/2.0, (1,0,0))
Rot_z = transformations.rotation_matrix(numpy.pi/2.0, (0,0,1))
trans = numpy.array([[1., 0., 0., 0.0], # along base frame y-axis
                     [0., 1., 0., 0.0], # vertical in base frame
                     [0., 0., 1., 0.8], # along base frame x-axis
                     [0., 0., 0., 1.]])
table_pose = transformations.concatenate_matrices(Rot_z, Rot_x, trans)
table.SetTransform(table_pose)

# Add a Fuze drink bottle on top of the table
fuze_path = os.path.join(objects_path, 'fuze_bottle.kinbody.xml')
fuze = env.ReadKinBodyXMLFile(fuze_path)
if fuze == None:
    print('Failed to load fuze bottle kinbody')
    sys.exit()
table_aabb = table.ComputeAABB()
x = table_aabb.pos()[0] + table_aabb.extents()[0]*0 # middle of table in x
#y = table_aabb.pos()[1] + table_aabb.extents()[1]*.6 # closer to one side in y
y = table_aabb.pos()[1] + table_aabb.extents()[1]*0 # middle of table in y
z = table_aabb.pos()[2] + table_aabb.extents()[2] + .01 # above table in z
fuze_pose = fuze.GetTransform()
fuze_pose[:3,3] = numpy.transpose([x, y, z])
fuze.SetTransform(fuze_pose)
env.AddKinBody(fuze)

#import time
#time.sleep(5)

# Grasp the bottle
robot.arm.Grasp(fuze)

robot.arm.Lift(fuze, distance=0.05)

# Move arm back to initial configuration
try:
    robot.arm.PlanToNamedConfiguration('zero', execute=True)
except Exception as e:
    print("Failed to plan to named configuration")

# we do this so the viewer doesn't close when the example is done
import IPython;
IPython.embed()
