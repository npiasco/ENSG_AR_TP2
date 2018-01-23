cmake_minimum_required(VERSION 2.8)
project( ENSG_AR_TP1 )
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

set(ORBSLAM_PATH ~/RA/ORB_SLAM2)

include_directories(
	${PROJECT_SOURCE_DIR}
	${Pangolin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
	${GLEW_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIR}
	${ORBSLAM_PATH}/include
	${ORBSLAM_PATH}
	opengl_code/
)

MESSAGE(${OPENGL_LIBRARY})
set(ALL_LIBS
	${OpenCV_LIBS}
	${OPENGL_LIBRARY}
	${OPENGL_gl_LIBS}
	${GLEW_LIBRARY}
	${EIGEN3_LIBS}
	${ORBSLAM_PATH}/lib/libORB_SLAM2.so
	glfw
	SOIL
	assimp
	${Pangolin_LIBRARIES}
)

add_definitions(
	-DTW_STATIC
	-DTW_NO_LIB_PRAGMA
	-DTW_NO_DIRECT3D
	-DGLEW_STATIC
	-D_CRT_SECURE_NO_WARNINGS
)	

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
add_executable(main main.cpp
		opengl_code/Shader.cpp 
		opengl_code/shaders/default.vertexshader 
		opengl_code/shaders/default.fragmentshader)
target_link_libraries( main ${ALL_LIBS} )
