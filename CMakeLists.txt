cmake_minimum_required(VERSION 3.1)
project(ImGuizmo)

set(IMGUIZMO_DIR ${CMAKE_CURRENT_SOURCE_DIR})

include_directories(${IMGUI_INCLUDE_DIR})
include_directories(${IMGUIZMO_DIR})

set(IMGUIZMO_SOURCES
	${IMGUIZMO_DIR}/ImGuizmo.cpp
	${IMGUIZMO_DIR}/ImGuizmo.h
	${IMGUIZMO_DIR}/ImGuizmo_internal.h
)

add_library(${PROJECT_NAME} STATIC ${IMGUIZMO_SOURCES})

if(APPLE)
	LIST(APPEND IMGUIZMO_CFLAGS -fvisibility=hidden)
	if(IOS)
		LIST(APPEND IMGUIZMO_CFLAGS -fembed-bitcode)
	endif(IOS)
	target_compile_options(${PROJECT_NAME} PUBLIC ${IMGUIZMO_CFLAGS})
endif()

set_target_properties (${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "d")

INSTALL(TARGETS ${PROJECT_NAME}
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
)

INSTALL(FILES
	${IMGUIZMO_DIR}/ImGuizmo.h
	DESTINATION include/ImGuizmo
)
