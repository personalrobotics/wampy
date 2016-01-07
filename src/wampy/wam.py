PACKAGE = 'wampy'
import logging, prpy, openravepy, prpy.dependency_manager, os
from wambase import WAMBase

logger = logging.getLogger('wampy')

def initialize(robot_xml=None, env_path=None, attach_viewer=False,
               sim=True, **kw_args):
    import prpy, os

    prpy.logger.initialize_logging()

    # Hide TrajOpt logging.
    os.environ.setdefault('TRAJOPT_LOG_THRESH', 'WARN')

    # Load plugins.
    if prpy.dependency_manager.is_catkin():
        prpy.dependency_manager.export()
    else:
        prpy.dependency_manager.export(PACKAGE)

    openravepy.RaveInitialize(True)

    # Create the environment.
    env = openravepy.Environment()
    if env_path is not None:
        if not env.Load(env_path):
            raise Exception('Unable to load environment frompath %s' % env_path)

    # Load the robot
    model_name = None
    share_directories = None
    if robot_xml is None:

        if prpy.dependency_manager.is_catkin():           
            from catkin.find_in_workspaces import find_in_workspaces
            share_directories = find_in_workspaces(search_dirs=['share'],
                                                   project=PACKAGE)
            if not share_directories:
                logger.error('Unable to find share directory in catkin workspace.')
                raise ValueError('Unable to find share directory in catkin workspace.')

            import os.path

            model_name = 'barrettsegway.robot.xml'
            for share_directory in share_directories:
                xml_path = os.path.join(share_directory, 'robots', model_name)
                if os.path.exists(xml_path):
                    robot_xml = xml_path

    if robot_xml is None:
        err_str = 'Unable to find robot model with name: %s in directories %s.' % (model_name, share_directories)
        logger.error(err_str)
        raise ValueError(err_str)
                    
    # Load the robot model
    robot = env.ReadRobotXMLFile(robot_xml)
    env.Add(robot)

    # Default arguments.
    keys = [ 'arm_sim',
             'hand_sim',
             'ft_sim',
             'segway_sim'
             ]
    for key in keys:
        if key not in kw_args:
            kw_args[key] = sim

    from wamrobot import WAMRobot
    prpy.bind_subclass(robot, WAMRobot, **kw_args)

    if sim:
        dof_indices, dof_values \
            = robot.configurations.get_configuration('zero')
        robot.SetDOFValues(dof_values, dof_indices)

    # Start by attempting to load or_rviz.
    if attach_viewer == True:
        attach_viewer = 'rviz'
        env.SetViewer(attach_viewer)

        # Fall back on qtcoin if loading or_rviz failed
        if env.GetViewer() is None:
            logger.warning(
                'Loading the RViz viewer failed. Do you have or_interactive'
                ' marker installed? Falling back on qtcoin.')
            attach_viewer = 'qtcoin'

    if attach_viewer and env.GetViewer() is None:
        env.SetViewer(attach_viewer)
        if env.GetViewer() is None:
            raise Exception('Failed creating viewer of type "{0:s}".'.format(
                            attach_viewer))

    # Remove the ROS logging handler again.
    # It might have been added when we loaded or_rviz.
    prpy.logger.remove_ros_logger()

    return env, robot
