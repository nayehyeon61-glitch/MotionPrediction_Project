# -*- mode: cmake; -*-
cmake_minimum_required(VERSION 3.0)

project (QP_controller)

# include ../Common.cmake
set (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../taesooLib/Samples")
get_filename_component(CAR_SAMPLE_DIR ${CMAKE_SOURCE_DIR} PATH)
get_filename_component(PROJ_ROOT_DIR ${CAR_SAMPLE_DIR} PATH)
set(TAESOOLIB_DIR "${PROJ_ROOT_DIR}/taesooLib")
set(PhysX_DIR "${PROJ_ROOT_DIR}/PhysX_MABA") #https://github.com/arpspoof/MABA
set(SAMPLE_DIR "${PROJ_ROOT_DIR}/taesooLib/Samples")
set(BUILD_CONSOLE OFF CACHE BOOL "BuildConsole option")
set(BUILD_EXE OFF CACHE BOOL "BuildExe option")

if(BUILD_EXE)

else()
	#include(FindPkgConfig)
	#pkg_search_module(PYTHON3 REQUIRED python3)

	execute_process(
		COMMAND "python3" -c "from sysconfig import get_paths as gp; print(gp()['include'])"
		OUTPUT_VARIABLE PYTHON3_INCLUDE_DIRS
		OUTPUT_STRIP_TRAILING_WHITESPACE
	)

endif()


	if(BUILD_CONSOLE)
		add_definitions(-DSEGA)				
		add_definitions(-DNO_GUI -DNO_OGRE)  
		set(NO_GUI TRUE)
	endif()
	if (TEST_SNIPPET_ARTICULATION)
		add_definitions(-DRENDER_SNIPPET)				
	endif()

	if(NOT BUILD_EXE)
		set(useShared TRUE)
	endif()
	set(NoDevil TRUE)
	if(NOT WIN32)
		set(NoFreeImage FALSE)
	endif()
	set(NoLapack TRUE)
	set(NoUMFpack TRUE)

	include(Common_baselib) 
	if(BUILD_CONSOLE)
    	add_subdirectory(../../taesooLib/BaseLib "./BaseLib/build_shared_console_${CMAKE_BUILD_TYPE}" )
    else()
    	add_subdirectory(../../taesooLib/BaseLib "./BaseLib/build_shared_${CMAKE_BUILD_TYPE}" )
    endif()

	if(BUILD_CONSOLE)
    	add_subdirectory(../../taesooLib/MainLib "./MainLib/build_shared_console_${CMAKE_BUILD_TYPE}" )
    else()
    	add_subdirectory(../../taesooLib/MainLib "./MainLib/build_shared_${CMAKE_BUILD_TYPE}" )
    endif()


	include(Common_mainlib) 

	# PhysicsLib settings
	set(NoAISTsim TRUE) # use TRL_LCP instead!
	set(NoGMBSsim FALSE) # use TRL_LCP instead!
	set(NoUTsim TRUE) # use TRL_LCP instead!
	set(NoBulletSim TRUE) # use TRL_LCP instead!
	set(EXCLUDE_RBDL_SIMULATOR FALSE)

	if(BUILD_CONSOLE)
    	add_subdirectory(../../taesooLib/PhysicsLib "./PhysicsLib/build_shared_console_${CMAKE_BUILD_TYPE}" )
    	add_subdirectory(../../taesooLib/ClassificationLib "./ClassificationLib/build_shared_console_${CMAKE_BUILD_TYPE}" )
    else()
    	add_subdirectory(../../taesooLib/PhysicsLib "./PhysicsLib/build_shared_${CMAKE_BUILD_TYPE}" )
    	add_subdirectory(../../taesooLib/ClassificationLib "./ClassificationLib/build_shared_${CMAKE_BUILD_TYPE}" )
    endif()
	add_subdirectory(../../taesooLib/Samples/QP_controller/qpOASES-3.0beta "./PhysicsLib/build_qpOASES_${CMAKE_BUILD_TYPE}" )


	#add_subdirectory(../../taesooLib/PhysicsLib/PhysX_implementation "./PhysicsLib/build_physX_${CMAKE_BUILD_TYPE}" )

add_definitions(-D_USRDLL)
add_definitions(-DMAINLIBPYTHON_EXPORTS)
add_definitions(-DML_LIBRARY)

if(BUILD_EXE)
	set(PYTHON3_INCLUDE_DIRS)
	set(NUMPY_INCLUDE_DIR)
else()
execute_process(
	COMMAND "python3" -c "import numpy; print(numpy.get_include())"
	OUTPUT_VARIABLE NUMPY_INCLUDE_DIR
	OUTPUT_STRIP_TRAILING_WHITESPACE
	)
message(${NUMPY_INCLUDE_DIR})
endif()


include_directories (	
	${PYTHON3_INCLUDE_DIRS}
	${NUMPY_INCLUDE_DIR}
	"/usr/include/python3.6"
	"/usr/include/python3.7m"
	"/usr/include/python3.8"
	"/usr/local/lib/python3.6/dist-packages/numpy/core/include" #numpy headers
	"/usr/local/lib/python3.8/site-packages/numpy/core/include"
	"/opt/homebrew/lib/python3.10/site-packages/numpy/core/include"
	.
	../../taesooLib/Samples/scripts
	../../taesooLib/PhysicsLib
	/usr/include/eigen3
	${TAESOOLIB_DIR}/Samples/QP_controller/qpOASES-3.0beta/include
	${CMAKE_SOURCE_DIR}/src/v-hacd/include
	${TAESOOLIB_DIR}
	)
set(src_python
# 	python only
  "MainlibPython.cpp"
  "MainlibPython.h"
  ${TAESOOLIB_DIR}/MainLib/python/MainlibPython.hpp
  ${TAESOOLIB_DIR}/MainLib/python/PythonExtendWin.hpp
  "PythonExtendWin.cpp"
  "PythonExtendWin.h"
  "./main.cpp"
  )

set(src_v_hacd
./v-hacd/app/wavefront.cpp
./v-hacd/app/InParser.cpp
#./v-hacd/app/TestVHACD.cpp
./v-hacd/app/FloatMath.cpp
./v-hacd/ConvexDecomp.cpp
./v-hacd/app/ScopedTime.h
./v-hacd/app/FloatMath.h
./v-hacd/app/wavefront.h
./v-hacd/app/InParser.h
./v-hacd/app/VHACD.h
./v-hacd/ConvexDecomp.h
./v-hacd/3Dthinning/itkBinaryThinningImageFilter3D.h
	)
set(src
  "${TAESOOLIB_DIR}/Samples/classification/cma/cmaes.c"
  "${TAESOOLIB_DIR}/Samples/classification/cma/CMAwrap.cpp"
  "${TAESOOLIB_DIR}/Samples/classification/cma/CMAwrap.h"
  "${TAESOOLIB_DIR}/Samples/QP_controller/eiquadprog.hpp"
  "${TAESOOLIB_DIR}/Samples/QP_controller/quadprog.cpp"
  "${TAESOOLIB_DIR}/Samples/QP_controller/quadprog.h"
  "${TAESOOLIB_DIR}/Samples/QP_controller/quadprog_qpOASES.cpp"

  )
set(src11 
	Lee2013MuscleDyn.cpp
	)

set(luna_script "${CMAKE_SOURCE_DIR}")

add_custom_command(
	OUTPUT "${luna_script}/luna_QP.cpp"
	DEPENDS "${luna_script}/luna_QP.lua" "${LUNA_GEN}" 
	PRE_BUILD
	COMMAND lua "${LUNA_GEN}" "${luna_script}/luna_QP.lua"
)

if(UNIX)
	#SET_SOURCE_FILES_PROPERTIES( ${src11} PROPERTIES COMPILE_FLAGS "--std=c++0x")
endif()

set(TAESOO_LIBS
	BaseLib
	MainLib
	PhysicsLib
	ClassificationLib
	#PhysX_sim
	)

if(BUILD_EXE)
	# build an executable that runs without python dependencies
	# for example, cd ../work; ./nopython aaa.lua
	set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CAR_SAMPLE_DIR}/work )

	add_executable(nopython 
		main_standalone.cpp
		${src}
		${src11}
		${src_v_hacd}
		"${CMAKE_SOURCE_DIR}/luna_QP.cpp" # important. without this, luna_gen doesn't run.
		${gmbs_src}
		)
	if(WIN32)
		set(systemLibs)
	else()
		set(systemLibs dl pthread)
	endif()
	target_link_libraries(nopython PUBLIC
		${TAESOO_LIBS}
		${FLTK_LIBS}
		${IMAGE_LIBS}
		${OGRE_LIBS}
		${QPOASES_LIB}
		${systemLibs}
		qpOASES
		)


	message(${OGRE_LIBS})
else()
	if(BUILD_CONSOLE )
	set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CAR_SAMPLE_DIR}/work/console )
	else()
	set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CAR_SAMPLE_DIR}/work )
	endif()


	set(usePybind11 TRUE)
	add_definitions(-DUSE_PYBIND11)
	add_subdirectory(pybind11)
	pybind11_add_module(libmainlib 
		${src}
		${src11}
		${src_v_hacd}
		${src_python}
		"${CMAKE_SOURCE_DIR}/luna_QP.cpp" # important. without this, luna_gen doesn't run.
		${gmbs_src}
		)

	if(BUILD_CONSOLE)
		target_link_libraries(libmainlib PUBLIC
			${TAESOO_LIBS}
			${IMAGE_LIBS}
			${QPOASES_LIB}
			dl
			qpOASES
			#boost_numpy
			)

	else()
		target_link_libraries(libmainlib PUBLIC
			${TAESOO_LIBS}
			${FLTK_LIBS}
			${IMAGE_LIBS}
			${OGRE_LIBS}
			${QPOASES_LIB}
			dl
			qpOASES
			#boost_numpy
			)
	endif()
	if(APPLE)
#		target_link_directories(mainlib  PUBLIC
#			${PYTHON3_LIBDIR}
#			${PHYSX_LIBRARIES}
#			)
#		target_link_libraries(mainlib 
#			python3.8
#			)
	else()
		target_link_libraries(libmainlib PUBLIC dl)
	endif()

endif()

