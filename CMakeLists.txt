cmake_minimum_required(VERSION 2.8.3)
project(rosneuro_integrator)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(ROSNEURO_DATA_MIN_VERSION "1.0.0")

##############################################################################
# Find dependencies
##############################################################################

find_package(catkin REQUIRED COMPONENTS 
			 roscpp 
			 std_msgs
			 pluginlib
			 rosneuro_msgs
			 artifacts_cvsa)

find_package(Eigen3 REQUIRED)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)


##############################################################################
# Define package
##############################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  ${PROJECT_NAME}
	
  CATKIN_DEPENDS
  	roscpp
	std_msgs
	pluginlib
	rosneuro_msgs
	artifacts_cvsa
  DEPENDS
)

##############################################################################
# TEST
##############################################################################

catkin_add_gtest(TestIntegrator test/TestIntegrator.cpp)
target_link_libraries(TestIntegrator ${GTEST_BOTH_LIBRARIES} pthread gmock)
target_link_libraries(TestIntegrator ${PROJECT_NAME} ${catkin_LIBRARIES})

catkin_add_gtest(TestGenericIntegrator test/TestGenericIntegrator.cpp)
target_link_libraries(TestGenericIntegrator ${GTEST_BOTH_LIBRARIES} pthread gmock)
target_link_libraries(TestGenericIntegrator ${PROJECT_NAME} ${catkin_LIBRARIES})

##############################################################################
# Build
##############################################################################

include_directories(${catkin_INCLUDE_DIRS} 
                    ${EIGEN3_INCLUDE_DIRS}
					include)

add_definitions(${EIGEN3_DEFINITIONS})

add_library(${PROJECT_NAME} src/GenericIntegrator.cpp
							src/Integrator.cpp)

target_link_libraries(${PROJECT_NAME} ${Eigen3_LIBRARIES} 
									  ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} 
				 ${catkin_EXPORTED_TARGETS}
)

add_executable(integrator src/integrator.cpp)
target_link_libraries(integrator ${PROJECT_NAME} ${catkin_LIBRARIES}) 


#################
##   Install   ##
#################
install(TARGETS ${PROJECT_NAME}

	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS integrator
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

