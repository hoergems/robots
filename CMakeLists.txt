cmake_minimum_required (VERSION 2.6)

project (robots)

if (NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_DEBUG "-g -std=c++11 -Og")
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++11 -O3 -DEIGEN_NO_DEBUG -flto")
SET(BOOST_MIN_VERSION "1.54.0")
SET(PYTHON_MIN_VERSION "2.7.0")
include(FindPkgConfig)
include(GNUInstallDirs)


set(USE_OPENRAVE True)
set(INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/include")

message("LIBDIR: ${CMAKE_INSTALL_LIBDIR}")
 
############################# LOAD OPENRAVE #############################
if(PKG_CONFIG_FOUND)
    pkg_check_modules(OPENRAVE openrave0.9-core)
    if(NOT OPENRAVE_FOUND)
       message("-- OpenRAVE 0.9 could not be found. Compliling without viewer support")
       set(USE_OPENRAVE False)            
    endif()
    include_directories(${OPENRAVE_INCLUDE_DIRS})
    link_directories(${OPENRAVE_LIBRARY_DIRS})           
    endif() 
    
if(PKG_CONFIG_FOUND)    
	pkg_check_modules(VIEWER viewer_interface)
	if(NOT VIEWER_FOUND)
	   message("-- viewer_interface could not be found. compiling without viewer support")
	   set(USE_OPENRAVE False)	       
	endif()	
	include_directories(${VIEWER_INCLUDE_DIRS})
	link_directories(${VIEWER_LIBRARY_DIRS})
endif()

if(USE_OPENRAVE)
add_definitions(-DUSE_OPENRAVE)
endif() 

############################# LOAD PYTHON #############################
if(PKG_CONFIG_FOUND)    
    pkg_check_modules(PYTHON python-2.7)
    if(NOT PYTHON_FOUND)
       message(FATAL_ERROR "Python could not be found")
    endif()
    include_directories(${PYTHON_INCLUDE_DIRS})
    link_directories(${PYTHON_LIBRARY_DIRS})    
endif()

############################# LOAD BOOST #############################
find_package(Boost ${BOOST_MIN_VERSION} REQUIRED COMPONENTS python system thread timer)
if (NOT Boost_FOUND)
    set(Boost_INCLUDE_DIRS "${BOOST_INCLUDE_DIRS}")
    IF ( NOT Boost_INCLUDE_DIRS )
        message(FATAL_ERROR "Please point the environment variable BOOST_INCLUDE_DIRS to the include directory of Boost")
    ENDIF()
endif ()
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

############################# LOAD FCL #############################
if(PKG_CONFIG_FOUND)
    pkg_check_modules(FCL fcl)
    if(NOT FCL_FOUND)
       message(FATAL_ERROR "FCL could not be found")
    endif()
    include_directories(${FCL_INCLUDE_DIRS})
    link_directories(${FCL_LIBRARY_DIRS}) 
    message("-- FCL LIB DIRS ${FCL_LIBRARY_DIRS}")    
endif()

############################# LOAD RBDL #############################
if(PKG_CONFIG_FOUND)    
    pkg_check_modules(RBDL rbdl)
    if(NOT RBDL_FOUND)
       message(FATAL_ERROR "RBDL could not be found")
    endif()
    include_directories(${RBDL_INCLUDE_DIRS})
    link_directories(${RBDL_LIBRARY_DIRS})
    message("RBDL include: ${RBDL_INCLUDE_DIRS}")   
endif()

############################# LOAD RBDL_INTERFACE #############################
if(PKG_CONFIG_FOUND)    
    pkg_check_modules(RBDL_INTERFACE rbdl_interface)
    if(NOT RBDL_FOUND)
       message(FATAL_ERROR "RBDL_INTERFACE could not be found")
    endif()
    include_directories(${RBDL_INTERFACE_INCLUDE_DIRS})
    link_directories(${RBDL_INTERFACE_LIBRARY_DIRS})  
    message("RBDL include: ${RBDL_INTERFACE_INCLUDE_DIRS}")   
endif()

############################# LOAD TINYXML #############################
if(PKG_CONFIG_FOUND)    
    pkg_check_modules(TINYXML tinyxml)
    if(NOT TINYXML_FOUND)
       message(FATAL_ERROR "TINYXML could not be found")
    endif()
    include_directories(${TINYXML_INCLUDE_DIRS}) 
    link_directories(${TINYXML_LIBRARY_DIRS})     
endif()


############################# LOAD EIGEN #############################
if(PKG_CONFIG_FOUND)
    pkg_check_modules(EIGEN eigen3)
    if(NOT EIGEN_FOUND)
       message(FATAL_ERROR "EIGEN could not be found")
    endif()
    include_directories(${EIGEN_INCLUDE_DIRS})
    link_directories(${EIGEN_LIBRARY_DIRS})    
endif()

add_library(robots SHARED 
            ${CMAKE_CURRENT_SOURCE_DIR}/src/robot.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/ManipulatorRobot.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/DubinRobot.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/uav.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/propagator.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/Kinematics.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/ManipulatorPropagator.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/DubinPropagator.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/integrate.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/ActionSpace.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/DiscreteActionSpace.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/ContinuousActionSpace.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/ObservationSpace.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/DiscreteObservationSpace.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/ContinuousObservationSpace.cpp)
            
target_link_libraries (robots
                       ${Boost_LIBRARIES}
                       ${PYTHON_LIBRARIES}
                       ${TINYXML_LIBRARIES}
                       ${FCL_LIBRARIES}
                       ${RBDL_INTERFACE_LIBRARIES}
                       ${VIEWER_LIBRARIES})
                       
set(pkg_conf_file_in "${CMAKE_CURRENT_SOURCE_DIR}/robots.pc.in")
set(pkg_conf_file_out "${CMAKE_CURRENT_BINARY_DIR}/robots.pc")
configure_file("${pkg_conf_file_in}" "${pkg_conf_file_out}" @ONLY)

install(TARGETS robots DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(FILES ${INCLUDE_DIR}/robot.hpp  DESTINATION include/robots)
install(FILES ${INCLUDE_DIR}/propagator.hpp  DESTINATION include/robots)
install(FILES ${INCLUDE_DIR}/integrate.hpp  DESTINATION include/robots)
install(FILES ${INCLUDE_DIR}/Kinematics.hpp  DESTINATION include/robots)
install(FILES ${INCLUDE_DIR}/propagator.hpp  DESTINATION include/robots)
install(FILES ${INCLUDE_DIR}/ManipulatorPropagator.hpp  DESTINATION include/robots)
install(FILES ${INCLUDE_DIR}/ManipulatorRobot.hpp  DESTINATION include/robots)
install(FILES ${INCLUDE_DIR}/DubinPropagator.hpp  DESTINATION include/robots)
install(FILES ${INCLUDE_DIR}/DubinRobot.hpp  DESTINATION include/robots)
install(FILES ${INCLUDE_DIR}/uav.hpp  DESTINATION include/robots)
install(FILES ${INCLUDE_DIR}/ActionSpace.hpp  DESTINATION include/robots)
install(FILES ${INCLUDE_DIR}/DiscreteActionSpace.hpp  DESTINATION include/robots)
install(FILES ${INCLUDE_DIR}/ContinuousActionSpace.hpp  DESTINATION include/robots)
install(FILES ${INCLUDE_DIR}/ObservationSpace.hpp  DESTINATION include/robots)
install(FILES ${INCLUDE_DIR}/DiscreteObservationSpace.hpp  DESTINATION include/robots)
install(FILES ${INCLUDE_DIR}/ContinuousObservationSpace.hpp  DESTINATION include/robots)
install(FILES ${INCLUDE_DIR}/utils.hpp  DESTINATION include/robots)
install(FILES ${INCLUDE_DIR}/MultNormal.hpp  DESTINATION include/robots)
install(FILES "${pkg_conf_file_out}" DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig/ COMPONENT pkgconfig)
