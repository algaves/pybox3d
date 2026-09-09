#ifndef BOX3D_EXPORT_H
#define BOX3D_EXPORT_H

/* libbox3d is built as a static library and linked privately into the
 * pybox3d._pybox3d extension module, so no dllexport/visibility dance is
 * required today. This header exists as the single seam to add one later
 * (e.g. if libbox3d is ever built as a standalone shared library). */
#define B3_API

#endif /* BOX3D_EXPORT_H */
