# - SAI2-URDF
find_package(SAI2-PRIMITIVES REQUIRED)
include_directories(${SAI2-PRIMITIVES_INCLUDE_DIRS})
add_definitions(${SAI2-PRIMITIVES_DEFINITIONS})

# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CS225A_BINARY_DIR}/project_starter_code)
ADD_EXECUTABLE (controller_project controller.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (simviz_project simviz.cpp ${CS225A_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (controller_project ${CS225A_COMMON_LIBRARIES} ${SAI2-PRIMITIVES_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz_project ${CS225A_COMMON_LIBRARIES} ${SAI2-PRIMITIVES_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world_iiwa.urdf world_panda.urdf world_panda_gripper.urdf world_mmp_panda.urdf world_mmp_panda_two_arm.urdf world_quad_1_arm.urdf world_quad_2_arm.urdf world_O2.urdf world_toro.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY kuka_iiwa.urdf panda_arm.urdf panda_arm_hand.urdf mmp_panda.urdf mmp_panda_two_arm.urdf quad_1_arm.urdf quad_2_arm.urdf Divebot_Hybrid.urdf toro.urdf DESTINATION ${APP_RESOURCE_DIR})
