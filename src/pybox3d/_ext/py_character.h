#ifndef PYBOX3D_PY_CHARACTER_H
#define PYBOX3D_PY_CHARACTER_H

#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "box3d/character.h"

/* A kinematic move-and-slide character controller -- standalone-owned,
 * like Vec3/Quat/Box3D (never "world-backed": it isn't added to any
 * World, just tested against one each move() call). */
typedef struct {
    PyObject_HEAD
    b3_CharacterMover value;
} PyCharacterMoverObject;

extern PyTypeObject PyCharacterMover_Type;

#define PyCharacterMover_Check(op) PyObject_TypeCheck((op), &PyCharacterMover_Type)

#endif /* PYBOX3D_PY_CHARACTER_H */
