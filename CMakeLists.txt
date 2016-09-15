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

############################# LOAD FRAPU_CORE #############################
if(PKG_CONFIG_FOUND)
    pkg_check_modules(FRAPU_CORE frapu_core)
    if(NOT FRAPU_CORE_FOUND)
       message(FATAL_ERROR "FRAPU_CORE could not be found")
    endif()
    include_directories(${FRAPU_CORE_INCLUDE_DIRS})
    link_directories(${FRAPU_CORE_LIBRARY_DIRS}) 
    message("-- FRAPU_CORE LIB DIRS ${FRAPU_CORE_LIBRARY_DIRS}")    
endif()

############################# LOAD ROBOT_HEADERS #############################
if(PKG_CONFIG_FOUND)
    pkg_check_modules(ROBOT_HEADERS robot_headers)
    if(NOT ROBOT_HEADERS_FOUND)
       message(FATAL_ERROR "ROBOT_HEADERS could not be found")
    endif()
    include_directories(${ROBOT_HEADERS_INCLUDE_DIRS})
    link_directories(${ROBOT_HEADERS_LIBRARY_DIRS}) 
    message("-- ROBOT_HEADERS LIB DIRS ${ROBOT_HEADERS_LIBRARY_DIRS}")    
endif()
 
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
#if(PKG_CONFIG_FOUND)    
#    pkg_check_modules(PYTHON python-2.7)
#    if(NOT PYTHON_FOUND)
#       message(FATAL_ERROR "Python could not be found")
#    endif()
#    include_directories(${PYTHON_INCLUDE_DIRS})
#    link_directories(${PYTHON_LIBRARY_DIRS})    
#endif()

############################# LOAD BOOST #############################
find_package(Boost ${BOOST_MIN_VERSION} REQUIRED COMPONENTS system thread timer)
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
            ${CMAKE_CURRENT_SOURCE_DIR}/src/Manipulator/ManipulatorRobot.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/Manipulator/ManipulatorRobotLinear.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/Manipulator/Kinematics.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/Manipulator/ManipulatorPropagator.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/Manipulator/ManipulatorPropagatorLinear.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/Manipulator/ManipulatorIntegrator.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/Dubin/DubinRobot.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/Dubin/DubinPropagator.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/AUV/auv.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/AUV/AUVPropagator.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/Airplane/AirplanePropagator.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/Airplane/AirplaneIntegrator.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/Homecare/Homecare.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/propagator.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/ActionSpace.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/DiscreteActionSpace.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/DiscreteVectorActionSpace.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/ContinuousActionSpace.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/ContinuousVectorActionSpace.cpp
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

install(DIRECTORY ${INCLUDE_DIR}/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/robots
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hxx" PATTERN "*.hpp"
  PATTERN ".DS_Store" EXCLUDE
)

install(TARGETS robots DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES "${pkg_conf_file_out}" DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig/ COMPONENT pkgconfig)

