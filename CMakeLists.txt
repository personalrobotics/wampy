cmake_minimum_required(VERSION 2.8.3)
project(wampy)

find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

install(DIRECTORY config/
  DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}/config"
)
install(PROGRAMS scripts/console.py
  DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
)

# Don't have any tests yet
#if (CATKIN_ENABLE_TESTING)
#  catkin_add_nosetests(tests)
#endif()
