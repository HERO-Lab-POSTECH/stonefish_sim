cmake_minimum_required(VERSION 3.0.2)
project(stonefish_description)

find_package(catkin REQUIRED)

catkin_package()

# Install all resources
install(DIRECTORY data robots worlds scenarios launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".git" EXCLUDE
)
