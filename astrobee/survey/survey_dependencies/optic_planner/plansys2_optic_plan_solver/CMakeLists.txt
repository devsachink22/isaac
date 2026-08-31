project(plansys2_optic_plan_solver)

cmake_minimum_required(VERSION 3.5)

find_package(Threads REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(DEPS
  roscpp
  plansys2_core
  plansys2_msgs
  pluginlib
)

find_package(catkin2 REQUIRED COMPONENTS
  ${DEPS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${DEPS}
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/plansys2_optic_plan_solver/optic_plan_solver.cpp
)

#pluginlib_export_plugin_description_file(plansys2_core plansys2_optic_plan_solver_plugin.xml)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES plansys2_optic_plan_solver_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  # Disable auto copyright because no copyright claimed on this work.
#  list(APPEND AMENT_LINT_AUTO_EXCLUDE
#    ament_cmake_copyright
#  )
#  ament_lint_auto_find_test_dependencies()
#
#  find_package(ament_cmake_gtest REQUIRED)
#  add_subdirectory(test)
#endif()
