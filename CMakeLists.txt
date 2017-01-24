# CMake minimum version and policies
cmake_minimum_required(VERSION 3.0.0)

# centauro UDP receiver
project(CentauroUDP)

# Add cmake dir and include custom macros
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(MacroOptionalBuild)

# C++11 
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Enable Xenomai flag and if enabled find required packages
set(ENABLE_XENO CACHE BOOL "ON")
if ( ${ENABLE_XENO} )
   find_package(Xenomai)
   find_package(RTnet)
endif()

set(COMPILE_SENDER CACHE BOOL "OFF")
set(COMPILE_RECEIVER CACHE BOOL "OFF")
set(COMPILE_ROS_COMMUNICATION CACHE BOOL "OFF")

# include 
include_directories(include) 

                                      
########################################################################

IF( ${COMPILE_SENDER} )

    add_executable(centauro_udp_sender
        src/centauro_udp_sender.cpp
    )

    set(VARS_PREFIX centauro_udp_sender)

    set(centauro_udp_sender_MAJOR_VERSION 0)
    set(centauro_udp_sender_MINOR_VERSION 0)
    set(centauro_udp_sender_PATCH_VERSION 1)
    set(centauro_udp_sender_VERSION ${centauro_udp_sender_MAJOR_VERSION}.${centauro_udp_sender_MINOR_VERSION}.${centauro_udp_sender_PATCH_VERSION})

    find_package(YCM REQUIRED)
    include(YCMDefaultDirs)
    ycm_default_dirs(${VARS_PREFIX})

    target_include_directories(centauro_udp_sender PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>"
                                                    "$<INSTALL_INTERFACE:${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

    set_target_properties(centauro_udp_sender PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                                SOVERSION ${${VARS_PREFIX}_VERSION})

    install(DIRECTORY include/
            DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}"
            FILES_MATCHING PATTERN "*.h*")

    install(TARGETS centauro_udp_sender
            EXPORT centauro_udp_sender
            ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT lib
            RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
            LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib)



    #enabling it will add all the project  dependencies as dependencies for third eparty users
    set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS centauro_udp_sender)


    include(InstallBasicPackageFiles)
    install_basic_package_files(centauro_udp_sender VARS_PREFIX ${VARS_PREFIX}
                                        VERSION ${${VARS_PREFIX}_VERSION}
                                        COMPATIBILITY SameMajorVersion
                                        TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                        NO_CHECK_REQUIRED_COMPONENTS_MACRO)                                    

ENDIF(${COMPILE_SENDER})
                                    
########################################################################

IF(${COMPILE_RECEIVER})

    # set Xenomain flags if Xenomai is founded
    if ( Xenomai_FOUND )
        # exe
        add_executable(centauro_udp_receiver
            src/centauro_udp_receiver.cpp
            src/rt_ipc.cpp
        )
        set_xeno_flags(centauro_udp_receiver)
    else ()
        # exe
        add_executable(centauro_udp_receiver
            src/centauro_udp_receiver.cpp
        )
    endif()

    set(VARS_PREFIX centauro_udp_receiver)

    set(centauro_udp_receiver_MAJOR_VERSION 0)
    set(centauro_udp_receiver_MINOR_VERSION 0)
    set(centauro_udp_receiver_PATCH_VERSION 1)
    set(centauro_udp_receiver_VERSION ${centauro_udp_receiver_MAJOR_VERSION}.${centauro_udp_receiver_MINOR_VERSION}.${centauro_udp_receiver_PATCH_VERSION})

    find_package(YCM REQUIRED)
    include(YCMDefaultDirs)
    ycm_default_dirs(${VARS_PREFIX})

    target_include_directories(centauro_udp_receiver PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>"
                                                    "$<INSTALL_INTERFACE:${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

    set_target_properties(centauro_udp_receiver PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                                SOVERSION ${${VARS_PREFIX}_VERSION})

    install(DIRECTORY include/
            DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}"
            FILES_MATCHING PATTERN "*.h*")

    install(TARGETS centauro_udp_receiver
            EXPORT centauro_udp_receiver
            ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT lib
            RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
            LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib)



    #enabling it will add all the project  dependencies as dependencies for third eparty users
    set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS centauro_udp_receiver)


    include(InstallBasicPackageFiles)
    install_basic_package_files(centauro_udp_receiver VARS_PREFIX ${VARS_PREFIX}
                                        VERSION ${${VARS_PREFIX}_VERSION}
                                        COMPATIBILITY SameMajorVersion
                                        TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                        NO_CHECK_REQUIRED_COMPONENTS_MACRO)                                       

ENDIF(${COMPILE_RECEIVER})

########################################################################

IF( ${COMPILE_ROS_COMMUNICATION} )

    find_package(catkin REQUIRED COMPONENTS roscpp
                                           geometry_msgs
                                           eigen_conversions
                                           tf2_ros)
                                           
    include_directories(${catkin_INCLUDE_DIRS})                                                

    add_executable(centauro_ros_autonomous_grasping
        src/centauro_ros_autonomous_grasping.cpp
    )
    
    
    # set Xenomain flags if Xenomai is founded
    if ( Xenomai_FOUND )
        # exe
        set_xeno_flags(centauro_ros_autonomous_grasping)
    else ()
        # exe
    endif()
    
    target_link_libraries(centauro_ros_autonomous_grasping ${catkin_LIBRARIES})

    set(VARS_PREFIX centauro_ros_autonomous_grasping)

    set(centauro_ros_autonomous_grasping_MAJOR_VERSION 0)
    set(centauro_ros_autonomous_grasping_MINOR_VERSION 0)
    set(centauro_ros_autonomous_grasping_PATCH_VERSION 1)
    set(centauro_ros_autonomous_grasping_VERSION ${centauro_ros_autonomous_grasping_MAJOR_VERSION}.${centauro_ros_autonomous_grasping_MINOR_VERSION}.${centauro_ros_autonomous_grasping_PATCH_VERSION})

    find_package(YCM REQUIRED)
    include(YCMDefaultDirs)
    ycm_default_dirs(${VARS_PREFIX})

    target_include_directories(centauro_ros_autonomous_grasping PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>"
                                                    "$<INSTALL_INTERFACE:${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

    set_target_properties(centauro_ros_autonomous_grasping PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                                SOVERSION ${${VARS_PREFIX}_VERSION})

    install(DIRECTORY include/
            DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}"
            FILES_MATCHING PATTERN "*.h*")

    install(TARGETS centauro_ros_autonomous_grasping
            EXPORT centauro_ros_autonomous_grasping
            ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT lib
            RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
            LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib)



    #enabling it will add all the project  dependencies as dependencies for third eparty users
    set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS centauro_ros_autonomous_grasping)


    include(InstallBasicPackageFiles)
    install_basic_package_files(centauro_ros_autonomous_grasping VARS_PREFIX ${VARS_PREFIX}
                                        VERSION ${${VARS_PREFIX}_VERSION}
                                        COMPATIBILITY SameMajorVersion
                                        TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                        NO_CHECK_REQUIRED_COMPONENTS_MACRO)                                    

ENDIF(${COMPILE_ROS_COMMUNICATION})

