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

# Off by default so a source install on an untested toolchain/platform
# combination never gets blocked by an unrelated warning; CI turns this on
# (-DBOX3D_WERROR=ON) to catch warning regressions automatically.
option(BOX3D_WERROR "Treat compiler warnings as errors" OFF)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(box3d PRIVATE -Wall -Wextra)
    if(BOX3D_WERROR)
        target_compile_options(box3d PRIVATE -Werror)
    endif()
endif()
