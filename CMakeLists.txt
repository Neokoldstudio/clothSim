cmake_minimum_required(VERSION 3.12)
project(MTI855-devoir01)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenGL REQUIRED)

if (APPLE)
  add_definitions( -DGL_SILENCE_DEPRECATION )
endif()

include(FetchContent)

FetchContent_Declare(
  polyscope
  GIT_REPOSITORY https://github.com/nmwsharp/polyscope.git
  GIT_TAG v2.2.1
  GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(polyscope)
FetchContent_GetProperties(polyscope SOURCE_DIR polyscope_SRC_DIR BINARY_DIR polyscope_BIN_DIR)

FetchContent_Declare(
  Eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE
  SOURCE_SUBDIR cmake
)
FetchContent_MakeAvailable(Eigen)
FetchContent_GetProperties(Eigen SOURCE_DIR Eigen_SRC_DIR BINARY_DIR Eigen_BIN_DIR)

set(tissu_HEADERS include/Cloth.h 
            include/ClothFactory.h
            include/ClothViewer.h
            include/Integrators/ExplicitEuler.hpp
            include/Integrators/ImplicitEuler.hpp
            include/Integrators/Integrator.h
            include/Integrators/Midpoint.hpp
            include/Integrators/SemiImplicitEuler.hpp
			include/Solvers/MatrixFreePGS.h
            include/ParticleSystem.h )
set(tissu_SOURCE src/ClothFactory.cpp 
		src/ClothViewer.cpp 
		src/ParticleSystem.cpp 
		src/Solvers/MatrixFreePGS.cpp )

add_executable (tissu main.cpp ${tissu_HEADERS} ${tissu_SOURCE})
		 
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${Eigen_SRC_DIR} ${COMMON_INCLUDES})

target_link_libraries(tissu OpenGL::GL polyscope)

source_group(src FILES ${tissu_SOURCE})
source_group(include FILES ${tissu_HEADERS})

if(MSVC) 
  set_property(TARGET tissu PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
endif()
