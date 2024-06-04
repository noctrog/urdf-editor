#include <stdio.h>
#include "raygizmo.h"
#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include <stdlib.h>
#include <string.h>

#if defined(PLATFORM_DESKTOP)  // Shaders for PLATFORM_DESKTOP
static const char *SHADER_VERT = "\
#version 330 core\n\
in vec3 vertexPosition; \
in vec4 vertexColor; \
out vec4 fragColor; \
out vec3 fragPosition; \
uniform mat4 mvp; \
void main() \
{ \
    fragColor = vertexColor; \
    fragPosition = vertexPosition; \
    gl_Position = mvp * vec4(vertexPosition, 1.0); \
} \
";

static const char *SHADER_FRAG = "\
#version 330 core\n\
in vec4 fragColor; \
in vec3 fragPosition; \
uniform vec3 cameraPosition; \
uniform vec3 gizmoPosition; \
out vec4 finalColor; \
void main() \
{ \
    vec3 r = normalize(fragPosition - gizmoPosition); \
    vec3 c = normalize(fragPosition - cameraPosition); \
    if (dot(r, c) > 0.1) discard; \
    finalColor = fragColor; \
} \
";

#else  // Shaders for PLATFORM_ANDROID, PLATFORM_WEB

static const char *SHADER_VERT = "\
#version 100\n\
attribute vec3 vertexPosition; \
attribute vec4 vertexColor; \
varying vec4 fragColor; \
varying vec3 fragPosition; \
uniform mat4 mvp; \
void main() \
{ \
    fragColor = vertexColor; \
    fragPosition = vertexPosition; \
    gl_Position = mvp * vec4(vertexPosition, 1.0); \
} \
";

static const char *SHADER_FRAG = "\
#version 100\n\
precision mediump float; \
varying vec4 fragColor; \
varying vec3 fragPosition; \
uniform vec3 cameraPosition; \
uniform vec3 gizmoPosition; \
void main() { \
    vec3 r = normalize(fragPosition - gizmoPosition); \
    vec3 c = normalize(fragPosition - cameraPosition); \
    if (dot(r, c) > 0.1) discard; \
    gl_FragColor = fragColor; \
} \
";
#endif

#define PICKING_FBO_WIDTH 512
#define PICKING_FBO_HEIGHT 512

#define X_AXIS \
    (Vector3) { 1.0, 0.0, 0.0 }
#define Y_AXIS \
    (Vector3) { 0.0, 1.0, 0.0 }
#define Z_AXIS \
    (Vector3) { 0.0, 0.0, 1.0 }

#define SWAP(x, y) \
    do { \
        unsigned char \
            swap_temp[sizeof(x) == sizeof(y) ? (signed)sizeof(x) : -1]; \
        memcpy(swap_temp, &y, sizeof(x)); \
        memcpy(&y, &x, sizeof(x)); \
        memcpy(&x, swap_temp, sizeof(x)); \
    } while (0)

static bool IS_LOADED = false;
static Shader SHADER;
static int SHADER_CAMERA_POSITION_LOC;
static int SHADER_GIZMO_POSITION_LOC;

static unsigned int PICKING_FBO;
static unsigned int PICKING_TEXTURE;

typedef enum HandleId {
    HANDLE_X,

    ROT_HANDLE_X,
    AXIS_HANDLE_X,
    PLANE_HANDLE_X,

    HANDLE_Y,

    ROT_HANDLE_Y,
    AXIS_HANDLE_Y,
    PLANE_HANDLE_Y,

    HANDLE_Z,

    ROT_HANDLE_Z,
    AXIS_HANDLE_Z,
    PLANE_HANDLE_Z
} HandleId;

typedef struct Handle {
    Vector3 position;
    Vector3 axis;
    Color color;
    float distToCamera;
} Handle;

typedef struct XYZColors {
    Color x;
    Color y;
    Color z;
} XYZColors;

typedef struct HandleColors {
    XYZColors rot;
    XYZColors axis;
    XYZColors plane;
} HandleColors;

typedef struct Handles {
    Handle arr[3];
} Handles;

static Handles sort_handles(Handle h0, Handle h1, Handle h2) {
    if (h0.distToCamera < h1.distToCamera) SWAP(h0, h1);
    if (h1.distToCamera < h2.distToCamera) SWAP(h1, h2);
    if (h0.distToCamera < h1.distToCamera) SWAP(h0, h1);

    Handles handles = {.arr = {h0, h1, h2}};
    return handles;
}

static XYZColors get_xyz_colors(Vector3 current_axis, bool is_hot) {
    Color x = is_hot && current_axis.x == 1.0f ? WHITE : RED;
    Color y = is_hot && current_axis.y == 1.0f ? WHITE : GREEN;
    Color z = is_hot && current_axis.z == 1.0f ? WHITE : BLUE;
    XYZColors colors = {x, y, z};
    return colors;
}

static void draw_gizmo(
    RGizmo gizmo, Camera3D camera, Vector3 position, HandleColors colors
) {
    float radius = gizmo.view.size * Vector3Distance(camera.position, position);

    BeginMode3D(camera);
    rlSetLineWidth(gizmo.view.handle_draw_thickness);
    rlDisableDepthTest();

    // ---------------------------------------------------------------
    // Draw plane handles
    {
        float offset = radius * gizmo.view.plane_handle_offset;
        float size = radius * gizmo.view.plane_handle_size;

        Vector3 px = Vector3Add(position, (Vector3){0.0f, offset, offset});
        Vector3 py = Vector3Add(position, (Vector3){offset, 0.0f, offset});
        Vector3 pz = Vector3Add(position, (Vector3){offset, offset, 0.0f});

        Handle hx = {
            px,
            Z_AXIS,
            colors.plane.x,
            Vector3DistanceSqr(px, camera.position)};
        Handle hy = {
            py,
            Y_AXIS,
            colors.plane.y,
            Vector3DistanceSqr(py, camera.position)};
        Handle hz = {
            pz,
            X_AXIS,
            colors.plane.z,
            Vector3DistanceSqr(pz, camera.position)};
        Handles handles = sort_handles(hx, hy, hz);

        rlDisableBackfaceCulling();
        for (int i = 0; i < 3; ++i) {
            Handle *h = &handles.arr[i];
            rlPushMatrix();
            rlTranslatef(h->position.x, h->position.y, h->position.z);
            rlRotatef(90.0f, h->axis.x, h->axis.y, h->axis.z);
            DrawPlane(
                Vector3Zero(), Vector2Scale(Vector2One(), size), h->color
            );
            rlPopMatrix();
        }
    }

    // ---------------------------------------------------------------
    // Draw rotation handles
    {
        BeginShaderMode(SHADER);
        SetShaderValue(
            SHADER,
            SHADER_CAMERA_POSITION_LOC,
            &camera.position,
            SHADER_UNIFORM_VEC3
        );
        SetShaderValue(
            SHADER, SHADER_GIZMO_POSITION_LOC, &position, SHADER_UNIFORM_VEC3
        );
        DrawCircle3D(position, radius, Y_AXIS, 90.0f, colors.rot.x);
        DrawCircle3D(position, radius, X_AXIS, 90.0f, colors.rot.y);
        DrawCircle3D(position, radius, X_AXIS, 0.0f, colors.rot.z);
        EndShaderMode();
    }

    // ---------------------------------------------------------------
    // Draw axis handles
    {
        float length = radius * gizmo.view.axis_handle_length;
        float tip_length = radius * gizmo.view.axis_handle_tip_length;
        float tip_radius = radius * gizmo.view.axis_handle_tip_radius;

        Vector3 px = Vector3Add(position, Vector3Scale(X_AXIS, length));
        Vector3 py = Vector3Add(position, Vector3Scale(Y_AXIS, length));
        Vector3 pz = Vector3Add(position, Vector3Scale(Z_AXIS, length));

        Handle hx = {
            px, X_AXIS, colors.axis.x, Vector3DistanceSqr(px, camera.position)};
        Handle hy = {
            py, Y_AXIS, colors.axis.y, Vector3DistanceSqr(py, camera.position)};
        Handle hz = {
            pz, Z_AXIS, colors.axis.z, Vector3DistanceSqr(pz, camera.position)};
        Handles handles = sort_handles(hx, hy, hz);

        for (int i = 0; i < 3; ++i) {
            Handle *h = &handles.arr[i];
            Vector3 tip_end = Vector3Add(
                h->position, Vector3Scale(h->axis, tip_length)
            );
            DrawLine3D(position, h->position, h->color);
            DrawCylinderEx(
                h->position, tip_end, tip_radius, 0.0f, 16, h->color
            );
        }
    }
    EndMode3D();

    // ---------------------------------------------------------------
    // Draw long white line which represents current active axis
    if (gizmo.state == RGIZMO_STATE_ACTIVE_ROT
        || gizmo.state == RGIZMO_STATE_ACTIVE_AXIS) {
        BeginMode3D(camera);
        rlSetLineWidth(gizmo.view.active_axis_draw_thickness);
        Vector3 halfAxisLine = Vector3Scale(gizmo.update.axis, 1000.0f);
        DrawLine3D(
            Vector3Subtract(position, halfAxisLine),
            Vector3Add(position, halfAxisLine),
            WHITE
        );
        EndMode3D();
    }

    // ---------------------------------------------------------------
    // Draw white line from the gizmo's center to the mouse cursor when rotating
    if (gizmo.state == RGIZMO_STATE_ACTIVE_ROT) {
        rlSetLineWidth(gizmo.view.active_axis_draw_thickness);
        DrawLineV(
            GetWorldToScreen(position, camera), GetMousePosition(), WHITE
        );
    }
}

static void rgizmo_load(void) {
    if (IS_LOADED) {
        TraceLog(LOG_WARNING, "RAYGIZMO: Gizmo is already loaded, skip");
        return;
    }

    // -------------------------------------------------------------------
    // Load shader
    SHADER = LoadShaderFromMemory(SHADER_VERT, SHADER_FRAG);
    SHADER_CAMERA_POSITION_LOC = GetShaderLocation(SHADER, "cameraPosition");
    SHADER_GIZMO_POSITION_LOC = GetShaderLocation(SHADER, "gizmoPosition");

    // -------------------------------------------------------------------
    // Load picking fbo
    PICKING_FBO = rlLoadFramebuffer(PICKING_FBO_WIDTH, PICKING_FBO_HEIGHT);
    if (!PICKING_FBO) {
        TraceLog(LOG_ERROR, "RAYGIZMO: Failed to create picking fbo");
        exit(1);
    }
    rlEnableFramebuffer(PICKING_FBO);

    PICKING_TEXTURE = rlLoadTexture(
        NULL,
        PICKING_FBO_WIDTH,
        PICKING_FBO_HEIGHT,
        RL_PIXELFORMAT_UNCOMPRESSED_R8G8B8A8,
        1
    );
    rlActiveDrawBuffers(1);
    rlFramebufferAttach(
        PICKING_FBO,
        PICKING_TEXTURE,
        RL_ATTACHMENT_COLOR_CHANNEL0,
        RL_ATTACHMENT_TEXTURE2D,
        0
    );
    if (!rlFramebufferComplete(PICKING_FBO)) {
        TraceLog(LOG_ERROR, "RAYGIZMO: Picking fbo is not complete");
        exit(1);
    }

    IS_LOADED = true;
    TraceLog(LOG_INFO, "RAYGIZMO: Gizmo loaded");
}

void rgizmo_unload(void) {
    if (!IS_LOADED) {
        TraceLog(LOG_WARNING, "RAYGIZMO: Gizmo is already unloaded, skip");
        return;
    }

    UnloadShader(SHADER);
    rlUnloadFramebuffer(PICKING_FBO);
    rlUnloadTexture(PICKING_TEXTURE);

    IS_LOADED = false;
    TraceLog(LOG_INFO, "RAYGIZMO: Gizmo unloaded");
}

RGizmo rgizmo_create(void) {
    if (!IS_LOADED) rgizmo_load();
    RGizmo gizmo = {0};
    gizmo.view.size = 0.12f;
    gizmo.view.handle_draw_thickness = 5.0f;
    gizmo.view.active_axis_draw_thickness = 2.0f;
    gizmo.view.axis_handle_length = 1.2f;
    gizmo.view.axis_handle_tip_length = 0.3f;
    gizmo.view.axis_handle_tip_radius = 0.1f;
    gizmo.view.plane_handle_offset = 0.4f;
    gizmo.view.plane_handle_size = 0.2f;

    return gizmo;
}

bool rgizmo_update(RGizmo *gizmo, Camera3D camera, Vector3 position) {
    bool updated = false;

    if (!IS_LOADED) {
        TraceLog(LOG_ERROR, "RAYGIZMO: Gizmo is not loaded");
        exit(1);
    }

    // -------------------------------------------------------------------
    // Draw gizmo into the picking fbo for the mouse pixel-picking
    rlEnableFramebuffer(PICKING_FBO);
    rlViewport(0, 0, PICKING_FBO_WIDTH, PICKING_FBO_HEIGHT);
    rlClearColor(0, 0, 0, 0);
    rlClearScreenBuffers();
    rlDisableColorBlend();

    HandleColors colors = {
        {(Color){ROT_HANDLE_X, 0, 0, 0},
         (Color){ROT_HANDLE_Y, 0, 0, 0},
         (Color){ROT_HANDLE_Z, 0, 0, 0}},
        {(Color){AXIS_HANDLE_X, 0, 0, 0},
         (Color){AXIS_HANDLE_Y, 0, 0, 0},
         (Color){AXIS_HANDLE_Z, 0, 0, 0}},
        {(Color){PLANE_HANDLE_X, 0, 0, 0},
         (Color){PLANE_HANDLE_Y, 0, 0, 0},
         (Color){PLANE_HANDLE_Z, 0, 0, 0}}};

    draw_gizmo(*gizmo, camera, position, colors);

    rlDisableFramebuffer();
    rlEnableColorBlend();
    Vector2 dpi = GetWindowScaleDPI();
    rlViewport(0, 0, (int)(GetScreenWidth() * dpi.x), (int)(GetScreenHeight() * dpi.y));

    // -------------------------------------------------------------------
    // Pick the pixel under the mouse cursor
    Vector2 mouse_position = GetMousePosition();
    unsigned char *pixels = (unsigned char *)rlReadTexturePixels(
        PICKING_TEXTURE,
        PICKING_FBO_WIDTH,
        PICKING_FBO_HEIGHT,
        RL_PIXELFORMAT_UNCOMPRESSED_R8G8B8A8
    );

    float x_fract = Clamp(mouse_position.x / (float)GetScreenWidth(), 0.0, 1.0);
    float y_fract = Clamp(
        1.0 - (mouse_position.y / (float)GetScreenHeight()), 0.0, 1.0
    );
    int x = (int)(PICKING_FBO_WIDTH * x_fract);
    int y = (int)(PICKING_FBO_HEIGHT * y_fract);
    int idx = 4 * (y * PICKING_FBO_WIDTH + x);
    unsigned char picked_id = pixels[idx];

    free(pixels);

    // -------------------------------------------------------------------
    // Update gizmo
    gizmo->update.angle = 0.0;
    gizmo->update.translation = Vector3Zero();

    bool is_lmb_down = IsMouseButtonDown(0);
    if (!is_lmb_down) gizmo->state = RGIZMO_STATE_COLD;

    if (gizmo->state < RGIZMO_STATE_ACTIVE) {
        if (picked_id < HANDLE_Y) gizmo->update.axis = X_AXIS;
        else if (picked_id < HANDLE_Z) gizmo->update.axis = Y_AXIS;
        else gizmo->update.axis = Z_AXIS;

        if (picked_id % 4 == 1)
            gizmo->state = is_lmb_down ? RGIZMO_STATE_ACTIVE_ROT
                                       : RGIZMO_STATE_HOT_ROT;
        else if (picked_id % 4 == 2)
            gizmo->state = is_lmb_down ? RGIZMO_STATE_ACTIVE_AXIS
                                       : RGIZMO_STATE_HOT_AXIS;
        else if (picked_id % 4 == 3)
            gizmo->state = is_lmb_down ? RGIZMO_STATE_ACTIVE_PLANE
                                       : RGIZMO_STATE_HOT_PLANE;
    }

    Vector2 delta = GetMouseDelta();
    bool is_mouse_moved = (fabs(delta.x) + fabs(delta.y)) > EPSILON;
    if (!is_mouse_moved) return updated;

    switch (gizmo->state) {
        case RGIZMO_STATE_ACTIVE_ROT: {
            Vector2 p1 = Vector2Subtract(
                GetMousePosition(), GetWorldToScreen(position, camera)
            );
            Vector2 p0 = Vector2Subtract(p1, GetMouseDelta());

            // Get angle between two vectors:
            float angle = 0.0f;
            float dot = Vector2DotProduct(
                Vector2Normalize(p1), Vector2Normalize(p0)
            );
            if (1.0 - fabs(dot) > EPSILON) {
                angle = acos(dot);
                float z = p1.x * p0.y - p1.y * p0.x;

                if (fabs(z) < EPSILON) angle = 0.0;
                else if (z <= 0) angle *= -1.0;
            }

            // If we look at the gizmo from behind, we should flip the rotation
            if (Vector3DotProduct(gizmo->update.axis, position)
                > Vector3DotProduct(gizmo->update.axis, camera.position)) {
                angle *= -1;
            }

            gizmo->update.angle = angle;
            updated = true;
            break;
        };
        case RGIZMO_STATE_ACTIVE_AXIS: {
            Vector2 p = Vector2Add(
                GetWorldToScreen(position, camera), GetMouseDelta()
            );
            Ray r = GetMouseRay(p, camera);

            // Get two lines nearest point
            Vector3 line0point0 = camera.position;
            Vector3 line0point1 = Vector3Add(line0point0, r.direction);
            Vector3 line1point0 = position;
            Vector3 line1point1 = Vector3Add(line1point0, gizmo->update.axis);
            Vector3 vec0 = Vector3Subtract(line0point1, line0point0);
            Vector3 vec1 = Vector3Subtract(line1point1, line1point0);
            Vector3 plane_vec = Vector3Normalize(Vector3CrossProduct(vec0, vec1)
            );
            Vector3 plane_normal = Vector3Normalize(
                Vector3CrossProduct(vec0, plane_vec)
            );

            // Intersect line and plane
            float dot = Vector3DotProduct(plane_normal, vec1);
            if (fabs(dot) > EPSILON) {
                Vector3 w = Vector3Subtract(line1point0, line0point0);
                float k = -Vector3DotProduct(plane_normal, w) / dot;
                Vector3 isect = Vector3Add(line1point0, Vector3Scale(vec1, k));
                gizmo->update.translation = Vector3Subtract(isect, position);
            }
            updated = true;
            break;
        }
        case RGIZMO_STATE_ACTIVE_PLANE: {
            Vector2 p = Vector2Add(
                GetWorldToScreen(position, camera), GetMouseDelta()
            );
            Ray r = GetMouseRay(p, camera);

            // Collide ray and plane
            float denominator = r.direction.x * gizmo->update.axis.x
                                + r.direction.y * gizmo->update.axis.y
                                + r.direction.z * gizmo->update.axis.z;

            if (fabs(denominator) > EPSILON) {
                float t = ((position.x - r.position.x) * gizmo->update.axis.x
                           + (position.y - r.position.y) * gizmo->update.axis.y
                           + (position.z - r.position.z) * gizmo->update.axis.z)
                          / denominator;

                if (t > 0) {
                    Vector3 c = Vector3Add(
                        r.position, Vector3Scale(r.direction, t)
                    );
                    gizmo->update.translation = Vector3Subtract(c, position);
                }
            }
            updated = true;
            break;
        }
        default: break;
    }

    return updated;
}

void rgizmo_draw(RGizmo gizmo, Camera3D camera, Vector3 position) {
    if (!IS_LOADED) {
        TraceLog(LOG_ERROR, "RAYGIZMO: Gizmo is not loaded");
        exit(1);
    }

    HandleColors colors;
    colors.rot = get_xyz_colors(
        gizmo.update.axis,
        gizmo.state == RGIZMO_STATE_HOT_ROT
            || gizmo.state == RGIZMO_STATE_ACTIVE_ROT
    );
    colors.axis = get_xyz_colors(
        gizmo.update.axis,
        gizmo.state == RGIZMO_STATE_HOT_AXIS
            || gizmo.state == RGIZMO_STATE_ACTIVE_AXIS
    );
    colors.plane = get_xyz_colors(
        gizmo.update.axis,
        gizmo.state == RGIZMO_STATE_HOT_PLANE
            || gizmo.state == RGIZMO_STATE_ACTIVE_PLANE
    );

    draw_gizmo(gizmo, camera, position, colors);
}

Matrix rgizmo_get_tranform(RGizmo gizmo, Vector3 position) {
    Matrix translation = MatrixTranslate(
        gizmo.update.translation.x,
        gizmo.update.translation.y,
        gizmo.update.translation.z
    );

    Matrix rotation = MatrixMultiply(
        MatrixMultiply(
            MatrixTranslate(-position.x, -position.y, -position.z),
            MatrixRotate(gizmo.update.axis, gizmo.update.angle)
        ),
        MatrixTranslate(position.x, position.y, position.z)
    );

    Matrix transform = MatrixMultiply(translation, rotation);

    return transform;
}
