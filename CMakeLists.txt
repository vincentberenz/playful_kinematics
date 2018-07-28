PROJECT(playful_kinematics)

set(ROBOT "pepper") 

catkin_package(
  LIBRARIES  ${ROBOT}_kinematics 
)

find_package(orocos_kdl)


find_package(catkin REQUIRED COMPONENTS
  kdl_parser
  urdf
)


include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})


get_filename_component(URDF_PATH urdf ABSOLUTE)


if(ROBOT STREQUAL "pepper")

  set(PEPPER_DEFS URDF_PATH="${URDF_PATH}/pepper/pepper.urdf" FIRST_LEFT_LINK="base_footprint" LAST_LEFT_LINK="l_wrist" FIRST_RIGHT_LINK="base_footprint" LAST_RIGHT_LINK="r_wrist" NB_JOINTS=8)

  add_library(pepper_kinematics src/soma.cpp src/fk.cpp src/ik.cpp src/score_functions.cpp src/kinematic_config.cpp)
  target_link_libraries(pepper_kinematics ${catkin_LIBRARIES} orocos-kdl)
  set_target_properties(pepper_kinematics PROPERTIES COMPILE_DEFINITIONS "${PEPPER_DEFS}")

  add_executable(pepper_fk_example src/fk_example.cpp)
  target_link_libraries(pepper_fk_example pepper_kinematics)
  set_target_properties(pepper_fk_example PROPERTIES COMPILE_DEFINITIONS "${PEPPER_DEFS}")
  
endif()


catkin_python_setup()


catkin_add_gtest(soma_unit_tests
  tests/main.cpp
  tests/soma_unit_tests.cpp
  )
target_link_libraries(soma_unit_tests ${ROBOT}_kinematics)

