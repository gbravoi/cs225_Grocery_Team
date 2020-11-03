# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CS225A_BINARY_DIR}/grocery_robot)
ADD_EXECUTABLE (simviz_grocery_robot simviz_demo.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (controller_grocery_robot controller_demo.cpp ${CS225A_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (simviz_grocery_robot ${CS225A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (controller_grocery_robot ${CS225A_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY mmp_panda.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY ${PROJECT_SOURCE_DIR}/model/objects/cup.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY ${PROJECT_SOURCE_DIR}/model/objects/jar.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY ${PROJECT_SOURCE_DIR}/model/objects/milk.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY ${PROJECT_SOURCE_DIR}/model/objects/jar.obj DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY ${PROJECT_SOURCE_DIR}/model/objects/milk.obj DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY ${PROJECT_SOURCE_DIR}/model/objects/shelf.obj DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY ${PROJECT_SOURCE_DIR}/model/objects/shelf_collision.obj DESTINATION ${APP_RESOURCE_DIR})
