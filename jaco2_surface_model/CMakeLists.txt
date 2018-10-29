cmake_minimum_required(VERSION 2.8.3)
project(jaco2_surface_model)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

## Compile as C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    jaco2_kin_dyn_lib
    roscpp
    tf
    visualization_msgs
    )

## System dependencies are found with CMake's conventions
find_package(OpenMesh QUIET)

if(${OPENMESH_FOUND})
    message("=====================> OpenMesh found")
    ################################################

    catkin_package(
        INCLUDE_DIRS include
        LIBRARIES jaco2_surface_model
        CATKIN_DEPENDS jaco2_kin_dyn_lib roscpp tf visualization_msgs
        #  DEPENDS system_lib
        )

    ###########
    ## Build ##
    ###########

    ## Specify additional locations of header files
    ## Your package locations should be listed before other locations
    include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OPENMESH_INCLUDE_DIRS}
        )

    ## Declare a C++ library
    #add_library(jaco2_surface_model
    #    src/parse_stl.cpp
    #    )

    #add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

    ## Specify libraries to link a library or executable target against
    #target_link_libraries(${PROJECT_NAME}
    #    ${catkin_LIBRARIES}
    #    )

    add_executable(${PROJECT_NAME}_test_node src/test/jaco2_surface_model_test_node.cpp)

    ## Add cmake target dependencies of the executable
    ## same as for the library above
    add_dependencies(${PROJECT_NAME}_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


    target_link_libraries(${PROJECT_NAME}_test_node
        #    jaco2_surface_model
        ${catkin_LIBRARIES}
        ${OPENMESH_LIBRARIES}
        )

    #############
    ## Install ##
    #############

    # all install targets should use catkin DESTINATION variables
    # See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

    ## Mark executable scripts (Python etc.) for installation
    ## in contrast to setup.py, you can choose the destination
    # install(PROGRAMS
    #   scripts/my_python_script
    #   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    # )

    ## Mark executables and/or libraries for installation
    # install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
    #   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    #   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    #   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    # )

    ## Mark cpp header files for installation
    # install(DIRECTORY include/${PROJECT_NAME}/
    #   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    #   FILES_MATCHING PATTERN "*.h"
    #   PATTERN ".svn" EXCLUDE
    # )

    ## Mark other files for installation (e.g. launch and bag files, etc.)
    # install(FILES
    #   # myfile1
    #   # myfile2
    #   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    # )

    #############
    ## Testing ##
    #############

    ## Add gtest based cpp test target and link libraries
    # catkin_add_gtest(${PROJECT_NAME}-test test/test_jaco2_surface_model.cpp)
    # if(TARGET ${PROJECT_NAME}-test)
    #   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
    # endif()

    ## Add folders to be run by python nosetests
    # catkin_add_nosetests(test)
else()
    message("====================> OpenMesh not found. Skipping this package.")
endif()

