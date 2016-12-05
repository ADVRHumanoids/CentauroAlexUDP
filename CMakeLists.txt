# CMake minimum version and policies
cmake_minimum_required(VERSION 3.0.0)

# centauro UDP receiver
project(centauro_udp_receiver)

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
#set(ENABLE_XENO CACHE BOOL "ON")
#if ( ${ENABLE_XENO} )
#    find_package(Xenomai)
#    find_package(RTnet)
#endif()

#if ( Xenomai_FOUND )
#    set(LIB_TYPE _rt)
#else ()
#    set(LIB_TYPE _nrt)
#endif()

# find packages 



# include 
include_directories(include) 

# exe
add_executable(${PROJECT_NAME}
    src/centauro_udp_receiver.cpp
)

# set Xenomain flags if Xenomai is founded
#if ( Xenomai_FOUND )
#    set_xeno_flags(${PROJECT_NAME})
#    set_xeno_flags(XBotPluginHandler)
#endif()

#target_link_libraries(${PROJECT_NAME})
                                      
                                      
########################################################################

set(VARS_PREFIX ${PROJECT_NAME})

set(${PROJECT_NAME}_MAJOR_VERSION 0)
set(${PROJECT_NAME}_MINOR_VERSION 0)
set(${PROJECT_NAME}_PATCH_VERSION 1)
set(${PROJECT_NAME}_VERSION ${${PROJECT_NAME}_MAJOR_VERSION}.${${PROJECT_NAME}_MINOR_VERSION}.${${PROJECT_NAME}_PATCH_VERSION})

find_package(YCM REQUIRED)
include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>"
                                                "$<INSTALL_INTERFACE:${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                               SOVERSION ${${VARS_PREFIX}_VERSION})

install(DIRECTORY include/
        DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}"
        FILES_MATCHING PATTERN "*.h*")

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT lib
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib)



#enabling it will add all the project  dependencies as dependencies for third eparty users
set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${PROJECT_NAME})


include(InstallBasicPackageFiles)
install_basic_package_files(${PROJECT_NAME} VARS_PREFIX ${VARS_PREFIX}
                                    VERSION ${${VARS_PREFIX}_VERSION}
                                    COMPATIBILITY SameMajorVersion
                                    TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                    NO_CHECK_REQUIRED_COMPONENTS_MACRO)                                    

include(AddUninstallTarget)



