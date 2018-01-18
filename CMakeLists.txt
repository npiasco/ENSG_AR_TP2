cmake_minimum_required(VERSION 2.8)
project( ARTP2 )
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

set(ORBSLAM_PATH ~/Cours/D2/AR/ORB_SLAM2)

add_definitions(-DCOMPILEDWITHC11)
set(ALL_LIBS
	${OpenCV_LIBS}
	${OPENGL_LIBRARY}
	${OPENGL_gl_LIBS}
	${ORBSLAM_PATH}/lib/libORB_SLAM2.so
	glfw
	SOIL
	assimp
)

include_directories(
	${PROJECT_SOURCE_DIR}
	${EIGEN3_INCLUDE_DIR}
	${GLEW_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIR}
	${Pangolin_INCLUDE_DIRS}
	${ORBSLAM_PATH}/include
	${ORBSLAM_PATH}
	opengl_code/
)


add_definitions(
	-DTW_STATIC
	-DTW_NO_LIB_PRAGMA
	-DTW_NO_DIRECT3D
	-DGLEW_STATIC
	-D_CRT_SECURE_NO_WARNINGS
)	

add_executable(test_orbslam test_orbslam.cpp)
target_link_libraries( orbslam ${ALL_LIBS} )
