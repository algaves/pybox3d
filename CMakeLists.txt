cmake_minimum_required(VERSION 3.18)
project(pybox3d LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)

add_subdirectory(libbox3d)

Python_add_library(_pybox3d MODULE WITH_SOABI
    src/pybox3d/_ext/module.c
    src/pybox3d/_ext/py_errors.c
    src/pybox3d/_ext/py_vec3.c
    src/pybox3d/_ext/py_quat.c
    src/pybox3d/_ext/py_box3d.c
    src/pybox3d/_ext/py_rigidbody.c
    src/pybox3d/_ext/py_world.c
)

target_link_libraries(_pybox3d PRIVATE box3d)
target_include_directories(_pybox3d PRIVATE src/pybox3d/_ext)

install(TARGETS _pybox3d DESTINATION pybox3d)
