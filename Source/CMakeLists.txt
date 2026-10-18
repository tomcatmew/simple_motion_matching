cmake_minimum_required(VERSION 3.12)

######################################################

project(Runtime_demo_mm LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  # motion matching does a brute-force search every few frames, debug builds are far too slow
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif ()

if (MSVC)
  add_compile_options(/W3 /utf-8 /bigobj)
  add_compile_definitions(_USE_MATH_DEFINES NOMINMAX _CRT_SECURE_NO_WARNINGS)
  set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME})
else ()
  add_compile_options(-Wall)
endif ()

set(THIRD_PARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../3rd_party")

add_definitions(-DPATH_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

####################################
# glfw
#   1) prebuilt binaries in 3rd_party/libglfw (original setup), or
#   2) the 3rd_party/glfw git submodule, built from source (default)

if (EXISTS "${THIRD_PARTY_DIR}/libglfw/include/GLFW/glfw3.h")
  set(GLFW_ROOT ${THIRD_PARTY_DIR}/libglfw)
  include(${THIRD_PARTY_DIR}/FindGLFW.cmake)
elseif (EXISTS "${THIRD_PARTY_DIR}/glfw/CMakeLists.txt")
  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
  add_subdirectory(${THIRD_PARTY_DIR}/glfw ${CMAKE_BINARY_DIR}/glfw EXCLUDE_FROM_ALL)
  set(GLFW_INCLUDE_DIR "${THIRD_PARTY_DIR}/glfw/include")
  set(GLFW_LIBRARIES glfw)
else ()
  message(FATAL_ERROR "GLFW not found. Run `git submodule update --init` or put prebuilt GLFW in 3rd_party/libglfw")
endif ()

# opengl
set(OpenGL_GL_PREFERENCE LEGACY)
find_package(OpenGL REQUIRED)

# delfem2 (header only)
add_definitions(-DDFM2_HEADER_ONLY=ON)
set(DELFEM2_INCLUDE_DIR "${THIRD_PARTY_DIR}/delfem2/include")
if (NOT EXISTS "${DELFEM2_INCLUDE_DIR}/delfem2/rig_bvh.h")
  message(FATAL_ERROR "delfem2 not found. Run `git submodule update --init`")
endif ()

# imgui (glfw + legacy OpenGL2 backend)
set(IMGUIGLFW_INCLUDE_DIR "${THIRD_PARTY_DIR}/imguiglfw/include")
file(GLOB IMGUI_SOURCES
     "${IMGUIGLFW_INCLUDE_DIR}/*.h"
     "${IMGUIGLFW_INCLUDE_DIR}/*.cpp"
)
add_library(ImGui STATIC ${IMGUI_SOURCES})
target_include_directories(ImGui PUBLIC ${IMGUIGLFW_INCLUDE_DIR} ${GLFW_INCLUDE_DIR})
target_link_libraries(ImGui PUBLIC ${GLFW_LIBRARIES} ${OPENGL_LIBRARY})

# libtorch (optional) -- only needed by the neural network experiments in model.h,
# the motion matching demo itself does not use it
option(MM_USE_TORCH "Link libtorch from 3rd_party/libtorch" OFF)
if (MM_USE_TORCH)
  include(${THIRD_PARTY_DIR}/libtorch/share/cmake/Torch/TorchConfig.cmake)
endif ()

####################################################

add_executable(${PROJECT_NAME}
    main.cpp
    camera.h
    gamepad.h
    motion_matching.h
    path_editor.h
    render_util.h
    selftest.h
    )

target_include_directories(${PROJECT_NAME} PRIVATE
    ${OPENGL_INCLUDE_DIR}
    ${GLFW_INCLUDE_DIR}
    ${DELFEM2_INCLUDE_DIR}
    ${IMGUIGLFW_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}
    ${OPENGL_LIBRARY}
    ${GLFW_LIBRARIES}
    ImGui
    )

if (MM_USE_TORCH)
  target_include_directories(${PROJECT_NAME} PRIVATE ${TORCH_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} ${TORCH_LIBRARIES})
  target_compile_definitions(${PROJECT_NAME} PRIVATE MM_USE_TORCH)
  if (MSVC)
    file(GLOB TORCH_DLLS "${TORCH_INSTALL_PREFIX}/lib/*.dll")
    add_custom_command(TARGET ${PROJECT_NAME}
                       POST_BUILD
                       COMMAND ${CMAKE_COMMAND} -E copy_if_different
                       ${TORCH_DLLS}
                       $<TARGET_FILE_DIR:${PROJECT_NAME}>)
  endif (MSVC)
endif ()

if (MSVC)
  # run from the build directory so the cached feature files land next to the solution
  set_target_properties(${PROJECT_NAME} PROPERTIES
      VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROJECT_NAME}>")
endif ()
