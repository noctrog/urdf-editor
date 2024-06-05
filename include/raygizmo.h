#ifndef RAYGIZMO_H
#define RAYGIZMO_H

#include <raylib.h>

typedef enum RGizmoState {
    RGIZMO_STATE_COLD,

    RGIZMO_STATE_HOT,

    RGIZMO_STATE_HOT_ROT,
    RGIZMO_STATE_HOT_AXIS,
    RGIZMO_STATE_HOT_PLANE,

    RGIZMO_STATE_ACTIVE,

    RGIZMO_STATE_ACTIVE_ROT,
    RGIZMO_STATE_ACTIVE_AXIS,
    RGIZMO_STATE_ACTIVE_PLANE,
} RGizmoState;

typedef struct RGizmo {
    struct {
        Vector3 translation;
        Vector3 axis;
        float angle;
    } update;

    struct {
        float size;
        float handle_draw_thickness;
        float active_axis_draw_thickness;
        float axis_handle_length;
        float axis_handle_tip_length;
        float axis_handle_tip_radius;
        float plane_handle_offset;
        float plane_handle_size;
    } view;

    RGizmoState state;
} RGizmo;

void rgizmo_unload(void);

RGizmo rgizmo_create(void);

bool rgizmo_update(RGizmo *gizmo, Camera3D camera, Vector3 position);
void rgizmo_draw(RGizmo gizmo, Camera3D camera, Vector3 position);
Matrix rgizmo_get_transform(RGizmo gizmo, Vector3 position);

#endif  // RAYGIZMO_H
