add_library(box3d STATIC
    src/vec3.c
    src/quat.c
    src/box3d.c
    src/sphere.c
    src/capsule.c
    src/hull.c
    src/compound.c
    src/mesh.c
    src/heightfield.c
    src/gjk.c
    src/shape.c
    src/rigidbody.c
    src/world.c
    src/slotmap.c
    src/debugdraw.c
    src/character.c
)

target_include_directories(box3d PUBLIC include)
set_target_properties(box3d PROPERTIES POSITION_INDEPENDENT_CODE ON)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(box3d PRIVATE -Wall -Wextra)
endif()
