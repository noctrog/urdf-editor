#ifndef RAYGIZMO_H
#define RAYGIZMO_H

#include <raylib.h>

// Identifies the gizmo's current hover or drag interaction mode.
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

// Stores gizmo interaction output, display settings, and current state.
typedef struct RGizmo {
    struct {
        // Translation, rotation axis, and rotation angle produced by a drag.
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

// Releases shared shaders and framebuffer resources used by all gizmos.
void rgizmo_unload(void);

// Creates a gizmo with the default view settings and no active interaction.
RGizmo rgizmo_create(void);

// Updates hover/drag state from mouse input within viewport.
//
// Returns true while the gizmo is actively being dragged.
bool rgizmo_update(RGizmo *gizmo, Camera3D camera, Vector3 position, Rectangle viewport);
// Draws the visible transform handles at position.
void rgizmo_draw(RGizmo gizmo, Camera3D camera, Vector3 position, Rectangle viewport);
// Returns the world-space transform represented by the current drag output.
Matrix rgizmo_get_transform(RGizmo gizmo, Vector3 position);

#endif  // RAYGIZMO_H
