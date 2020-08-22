project(rs-depth-source)


# Simple non robust way to find the librealsense library
if(WIN32)
    if("${CMAKE_SIZEOF_VOID_P}" STREQUAL "8")
        set(LIBRARY_DIR "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\lib\\x64")          # TODO: Update this variable to correct path - folder where realsense2.lib is found
        set(DLL_DIR "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\bin\\x64")              # TODO: Update this variable to correct path - folder where realsense2.dll is found
    else()
        set(LIBRARY_DIR "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\lib\\x86")          # TODO: Update this variable to correct path - folder where realsense2.lib is found
        set(DLL_DIR "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\bin\\x86")              # TODO: Update this variable to correct path - folder where realsense2.dll is found
    endif()
    set(PROJECT_BINARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/\$\(Configuration\)")                # TODO: Update this variable to correct path - folder to which your project will compile
    set(ADDITIONAL_INCLUDE_DIRS "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\include")   # TODO: Update this variable to correct path - folder where librealsense2 folder is found
endif()


find_library(REALSENSE2_FOUND realsense2 HINTS ${LIBRARY_DIR} REQUIRED)
if(NOT REALSENSE2_FOUND)
    SET(REALSENSE2_FOUND "realsense2")
    message(WARN "Failed to find_library(realsense2)")
endif()

if(MSVC)
	set(rs-depth-source_PLATFORM_DEPS
		w32-pthreads)
endif()

set(rs-depth-source_SOURCES
	rs-depth-source.cpp)

if(WIN32)
	set(MODULE_DESCRIPTION "OBS rs-depth module")
	configure_file(${CMAKE_SOURCE_DIR}/cmake/winrc/obs-module.rc.in rs-depth-source.rc)
	list(APPEND rs-depth-source_SOURCES
		rs-depth-source.rc)
endif()

add_library(rs-depth-source MODULE
	${rs-depth-source_SOURCES})

target_link_libraries(rs-depth-source
	#${realsense2_LIBRARY}
	${REALSENSE2_FOUND}
	${OpenCV_LIBS}
	libobs
	${rs-depth-source_PLATFORM_DEPS}
	)
set_target_properties(rs-depth-source PROPERTIES FOLDER "plugins")

install_obs_plugin_with_data(rs-depth-source data)
