# NumPy interop for Vec3/Quat

## Context

`Vec3`/`Quat` already accept NumPy arrays as input everywhere a `VecLike`/
`QuatLike` is expected (`RigidBody(position=arr, ...)`, joint anchors, etc.):
`PyVec3_Parse`/`PyQuat_Parse` go through `PySequence_Fast` +
`PyFloat_AsDouble`, and both work against ndarray elements via `__float__`.
Confirmed by hand against numpy 2.4.6.

What's missing is the *output* side: turning a `Vec3`/`Quat` back into a
NumPy array has no idiomatic path today (`to_tuple()` + `np.array()` only).
This spec covers closing that gap. Scope, per user decision: conversion
only (no batch/array-valued World/RigidBody APIs), and NumPy becomes a hard
runtime dependency (not an optional extra).

`b3_Vec3`/`b3_Quat` (`libbox3d/include/box3d/vec3.h`, `quat.h`) are POD
structs of contiguous `b3_real` (`typedef float`) — 3 and 4 fields
respectively, no padding. `Vec3.x/.y/.z` (and `Quat`'s) are settable in
Python and route through `pybox3d_require_finite` (added in the last polish
pass) to reject NaN/Inf.

## Design

### 1. Buffer protocol (`tp_as_buffer`)

Add a `bf_getbuffer` implementation to `PyVec3_Type` and `PyQuat_Type`
exposing the struct's own memory directly: format `"f"`, 1-D, shape `(3,)`
/ `(4,)`, strides `(sizeof(b3_real),)`. **Read-only** — reject any request
with `PyBUF_WRITABLE` set (`PyExc_BufferError`). A writable view would let
NumPy write raw floats into the struct without going through
`pybox3d_require_finite`, defeating the guarding added this cycle.

This alone makes `np.asarray(v)` a zero-copy read, and — free side effect —
`bytes(v)`, `memoryview(v)`, `array.array('f', v)` all work too, since they
all go through the same protocol. No `__array__` dunder needed: NumPy's
array constructors already recognize buffer-protocol objects.

`shape`/`strides` arrays are `static const Py_ssize_t` at file scope (same
for every instance of a fixed-size type — standard idiom for this
protocol). No `bf_releasebuffer` needed: with it left NULL, CPython's
default `PyBuffer_Release` just decrefs `view->obj`, which is all that's
needed here since the buffer aliases the object's own storage.

### 2. `to_numpy()` / `from_numpy()`

Explicit, discoverable methods mirroring the existing `to_tuple()` naming
convention:

- **`to_numpy()`** (`METH_NOARGS`) — `numpy.array(self, dtype="float32")`,
  called via `PyImport_ImportModule("numpy")` +
  `PyObject_CallMethod(numpy, "array", "Os", self, "float32")` at the call
  site. `numpy.array()`'s default `copy=True` gives the caller an
  independent, writable array — no risk of it becoming a back door around
  the buffer's read-only-ness. No numpy C-API/headers needed at build
  time; `PyImport_ImportModule` on an already-imported module is a cheap
  `sys.modules` lookup, so no caching layer is worth adding.
- **`from_numpy(arr)`** (`METH_O | METH_CLASS`) — thin sugar: parses via
  the existing `PyVec3_Parse`/`PyQuat_Parse` (which already handles
  ndarray input) and constructs. Kept for discoverability/symmetry with
  `to_numpy()`, not because it adds new capability.

dtype is `float32` throughout, matching `b3_real` exactly — this keeps
`to_numpy()` honest about the library's actual internal precision and
keeps it consistent with the buffer protocol's format character. Mixing
with a caller's own `float64` arrays upcasts automatically via normal
NumPy promotion rules; not a compatibility problem.

### 3. Packaging

- `pyproject.toml`: `dependencies = ["numpy>=1.24"]`. No CMake/build-time
  changes — nothing here touches numpy's C API or headers, only its
  Python-level API at runtime.
- Update the "dependency-free" framing (`README.md`, `docs/index.md`,
  `docs/limitations.md`) — `libbox3d` (the C core) stays zero-dependency;
  that claim is still true and should stay as-is. The *Python package*
  now has exactly one runtime dependency (NumPy). Reword the top-line
  pitch to keep the "no ctypes/cffi/pybind11/nanobind, hand-written
  CPython C-API" claim (still fully true) without asserting
  dependency-free at the package level.

### 4. Type stub / docs

- `src/pybox3d/_pybox3d.pyi`: add `to_numpy(self) -> npt.NDArray[np.float32]`
  and `from_numpy(cls, arr: npt.ArrayLike) -> Vec3/Quat` to both classes;
  broaden `VecLike`/`QuatLike` to explicitly include
  `npt.NDArray[np.floating[Any]]` (currently only `Sequence[float]`, which
  ndarray doesn't formally satisfy even though it works at runtime).
- New tutorial/example follow the existing pattern (`examples/*_demo.py` +
  `docs/tutorials/*.md` + `docs/examples/*.md` + `mkdocs.yml` nav) is
  **not** included in this pass — this spec is scoped to the conversion
  API itself; a demo script can follow separately if wanted.
- README/docs updates limited to the dependency-free wording fix above; no
  new tutorial page for this pass.

## Testing

- `tests/test_vec3.py` / `tests/test_quat.py`: `to_numpy()` returns a
  `float32` ndarray of the right shape/values; mutating the returned array
  does *not* affect the source `Vec3`; `from_numpy()` round-trips;
  `np.asarray(v)` is zero-copy (`.base is v` or shares the same buffer, via
  `np.shares_memory`); a writable buffer request
  (`memoryview(v).cast(...)` write attempt, or `np.asarray(v).flags.writeable`)
  is rejected/`False`; existing `VecLike`-accepts-ndarray behavior gets an
  explicit regression test (previously implicit/untested).
- Full suite + `ruff check .` + `mypy` + `mkdocs build --strict` (nav
  unchanged, but the reworded "dependency-free" prose still needs to build
  clean).

## Out of scope (this pass)

- Batch/array-valued World or RigidBody APIs (e.g. all-body positions as
  one ndarray) — separate, larger design if wanted later.
- New example/tutorial demo for NumPy interop specifically.
- Making NumPy optional/extras-based — explicitly rejected; hard
  dependency per user decision.
