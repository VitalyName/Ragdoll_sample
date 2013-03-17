#/*
#-----------------------------------------------------------------------------
#Filename:    CMakeLists.txt 
#-----------------------------------------------------------------------------
#
#	Поддерживает только minGW, другие не пробовал, но думаю будут небольшие отличия в поиске библиотек, особенно под линуксом
#    Поправьте для своих сред этот файл если не лень и скиньте мне!    
#    
#*/

cmake_minimum_required(VERSION 2.6)
 
################################################################################
#   Settings (Конфигурируйте здесь!)
################################################################################

set (PROJECT Ragdoll_Sample)  # имя проекта

set(USE_BOOST TRUE)  # использовать буст? (зависит от того как был собран OGRE(с бустом для мультитрейдинга или нет))

set (OGRE_HOME $ENV{OGRE_HOME}) # Корневая папка OGRE (в данной момент считываеться из переменых среды windows)
set (OIS_HOME $ENV{OGRE_DEPENDENCIES_DIR}) # Корневая папка OIS (в данной момент считываеться из переменых среды windows)
set (BULLET_ROOT $ENV{BULLET_ROOT}) # Корневая папка Bullet (в данной момент считываеться из переменых среды windows)



# Здесь добавляем в проект свои файлы
set(HDRS
	./src/BaseApplication.h
	./src/DebugDraw.h
	./src/DynamicLineDrawer.h
	./src/MyMotionState.h
	./src/OgreBulletUtils.h
	./src/RagdollSample.h
)
 
set(SRCS
	./src/BaseApplication.cpp
	./src/DebugDraw.cpp
	./src/DynamicLineDrawer.cpp
	./src/RagdollSample.cpp
)

################################################################################
#   Cmake 
################################################################################

project(${PROJECT})

if (CMAKE_BUILD_TYPE STREQUAL "")
  # CMake defaults to leaving CMAKE_BUILD_TYPE empty. This screws up
  # differentiation between debug and release builds.
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build, options are: None (CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif ()

set(CMAKE_DEBUG_POSTFIX "_d")

set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_BINARY_DIR}")

###############################################################################
#   Ogre
###############################################################################
set ( OGRE_LIBRARY_NAMES OgreMain )

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
	set ( OGRE_LIBRARY_NAMES OgreMain_d )
endif ()

find_path ( OGRE_INCLUDE_DIRS
  	Ogre.h PATHS
  	"${OGRE_HOME}/include"
 	"${OGRE_HOME}/include/OGRE"
 )

find_library (OGRE_LIBRARIES
 ${OGRE_LIBRARY_NAMES} PATHS
 "${OGRE_HOME}/lib/${CMAKE_BUILD_TYPE}"
)

 #### message(STATUS !!!!!!!!!!!!!!! ${OGRE_LIBRARY_REL})
################################################################################
#   Boost
################################################################################
# Find Boost
if (USE_BOOST)
	set(Boost_USE_STATIC_LIBS TRUE)

	if (MINGW)
		# this is probably a bug in CMake: the boost find module tries to look for
		# boost libraries with name libboost_*, but CMake already prefixes library
		# search names with "lib". This is the workaround.
		set(CMAKE_FIND_LIBRARY_PREFIXES ${CMAKE_FIND_LIBRARY_PREFIXES} "")
	endif ()
	set(Boost_ADDITIONAL_VERSIONS "1.53" "1.49" "1.44" "1.44.0" "1.42" "1.42.0" "1.41.0" "1.41" "1.40.0" "1.40" "1.39.0" "1.39" "1.38.0" "1.38" "1.37.0" "1.37" )
	# Components that need linking (NB does not include header-only components like bind)
	set(REQ_BOOST_COMPONENTS thread date_time system)
	find_package(Boost COMPONENTS ${REQ_BOOST_COMPONENTS} QUIET)
	if (NOT Boost_FOUND)
		# Try again with the other type of libs
		set(Boost_USE_STATIC_LIBS NOT ${Boost_USE_STATIC_LIBS})
		find_package(Boost COMPONENTS ${REQ_BOOST_COMPONENTS} QUIET)
	endif()
	find_package(Boost QUIET)
 
	# Set up referencing of Boost
	include_directories(${Boost_INCLUDE_DIR})
	add_definitions(-DBOOST_ALL_NO_LIB)
	set(OGRE_LIBRARIES ${OGRE_LIBRARIES} ${Boost_LIBRARIES})
endif()

# message(STATUS !!!!!!!!!!!!!!! ${Boost_INCLUDE_DIR})
###############################################################################
#   finding OIS
###############################################################################

set ( OIS_LIBRARY_NAMES OIS)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
	set ( OIS_LIBRARY_NAMES OIS_d)
endif ()

find_path ( OIS_INCLUDE_DIRS
  	OIS.h PATHS
  	"${OIS_HOME}/include/OIS"
 )

find_library (OIS_LIBRARIES
 ${OIS_LIBRARY_NAMES} PATHS
 "${OIS_HOME}/lib/${CMAKE_BUILD_TYPE}"
)

 ################################################################################
 #   finding Bullet
 #################################################################################

find_package(BULLET QUIET)

##################################################################################
#  Building projects
##################################################################################
 
include_directories( ${OIS_INCLUDE_DIRS}
	${OGRE_INCLUDE_DIRS}
	${BULLET_INCLUDE_DIRS}
)
 
 # Для того чтобы в коде можно было поставить условие(#ifdef _DEBUG)
if ( CMAKE_BUILD_TYPE STREQUAL "Debug" )
	add_definitions( -D_DEBUG )          
endif() 								 

add_executable(${PROJECT} WIN32 ${HDRS} ${SRCS})
 
set_target_properties(${PROJECT} PROPERTIES DEBUG_POSTFIX _d)

target_link_libraries(${PROJECT} ${OGRE_LIBRARIES} ${OIS_LIBRARIES} ${BULLET_LIBRARIES})

if(MINGW)
	set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin/${CMAKE_BUILD_TYPE})
endif(MINGW)

##################################################################################
#  Installing projects
##################################################################################

if(WIN32)
 	install(TARGETS ${PROJECT}
		RUNTIME DESTINATION bin
		CONFIGURATIONS All
	)
 
	install(DIRECTORY ${CMAKE_SOURCE_DIR}/Media
		DESTINATION ./
		CONFIGURATIONS Release RelWithDebInfo Debug
	)
 
	install(FILES ${CMAKE_SOURCE_DIR}/conf_files/plugins.cfg
		${CMAKE_SOURCE_DIR}/conf_files/resources.cfg
		DESTINATION bin/${CMAKE_BUILD_TYPE}
		CONFIGURATIONS Release RelWithDebInfo
	)
 
	install(FILES ${CMAKE_SOURCE_DIR}/conf_files/plugins_d.cfg
		${CMAKE_SOURCE_DIR}/conf_files/resources_d.cfg
		DESTINATION bin/${CMAKE_BUILD_TYPE}
		CONFIGURATIONS Debug
	)
 
	install(FILES ${OGRE_HOME}/bin/${CMAKE_BUILD_TYPE}/OgreMain.dll
		${OGRE_HOME}/bin/${CMAKE_BUILD_TYPE}/RenderSystem_Direct3D9.dll
		${OGRE_HOME}/bin/${CMAKE_BUILD_TYPE}/RenderSystem_GL.dll
		${OGRE_HOME}/bin/${CMAKE_BUILD_TYPE}/OIS.dll
		DESTINATION bin/${CMAKE_BUILD_TYPE}
		CONFIGURATIONS Release RelWithDebInfo
	)
 
	install(FILES ${OGRE_HOME}/bin/${CMAKE_BUILD_TYPE}/OgreMain_d.dll
		${OGRE_HOME}/bin/${CMAKE_BUILD_TYPE}/RenderSystem_Direct3D9_d.dll
		${OGRE_HOME}/bin/${CMAKE_BUILD_TYPE}/RenderSystem_GL_d.dll
		${OGRE_HOME}/bin/${CMAKE_BUILD_TYPE}/OIS_d.dll
		DESTINATION bin/${CMAKE_BUILD_TYPE}
		CONFIGURATIONS Debug
	)
 
 	if(NOT Boost_USE_STATIC_LIBS)
		install(FILES ${Boost_DATE_TIME_LIBRARY_RELEASE}
	     	${Boost_THREAD_LIBRARY_RELEASE}
	      	DESTINATION bin
	     	 CONFIGURATIONS Release RelWithDebInfo
	    )
 
	   install(FILES ${Boost_DATE_TIME_LIBRARY_DEBUG}
	      ${Boost_THREAD_LIBRARY_DEBUG}
	      DESTINATION bin
	      CONFIGURATIONS Debug
	    )
    endif()

endif(WIN32)