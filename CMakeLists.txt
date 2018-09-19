# CMake minimum version and policies
cmake_minimum_required(VERSION 3.0.0)

# Centauro - ALEX UDP Communication 
project(CentauroAlexUDP)

# Add cmake dir and include custom macros
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(MacroOptionalBuild)
include(MacroYCMInstallLibrary)

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
endif()

# find packages
find_package(XCM REQUIRED)
find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp)

# include 
include_directories(include ${XCM_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}) 

# plugin
add_library(AlexCommunication SHARED src/ExoskeletonCommunicationPlugin.cpp)

# link
target_link_libraries(AlexCommunication ${XCM_LIBRARIES})

# exe
add_executable(${PROJECT_NAME}
               src/CentauroAlexUDP.cpp)
               
add_executable(CentauroPedalUDP
               src/CentauroPedalUDP.cpp)    
               
               
target_link_libraries(${PROJECT_NAME} ${XCM_LIBRARIES})               
target_link_libraries(CentauroPedalUDP ${catkin_LIBRARIES})



# set Xenomai flags if Xenomai is found
if ( Xenomai_FOUND )
    set_xeno_flags(${PROJECT_NAME})
    set_xeno_flags(AlexCommunication)
endif()

# install

ycm_library_install(AlexCommunication 1 1 0)

ycm_library_install(${PROJECT_NAME} 2 0 0)

ycm_library_install(CentauroPedalUDP 1 0 0)
