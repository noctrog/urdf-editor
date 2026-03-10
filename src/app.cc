#include <app.h>
#include <fmt/format.h>
#include <glad.h>
#include <imgui.h>
#include <misc/cpp/imgui_stdlib.h>
#include <raygizmo.h>
#include <raymath.h>
#include <rcamera.h>
#include <rlImGui.h>
#include <rlgl.h>
#include <rlights.h>
#include <robot.h>

#include <algorithm>
#include <array>
#include <functional>
#include <loguru.hpp>
#include <nfd.hpp>
#include <pugixml.hpp>

// Camera control speeds
constexpr float kCameraRotSpeed = 0.003F;
constexpr float kCameraMoveSpeed = 0.01F;
constexpr float kCameraZoomSpeed = 1.0F;

// Window defaults
constexpr int kDefaultScreenWidth = 1200;
constexpr int kDefaultScreenHeight = 800;
constexpr int kTargetFps = 120;

// Lighting
constexpr float kAmbientLightIntensity = 3.0F;
constexpr float kDefaultCameraFov = 45.0F;
const Vector3 kDefaultCameraPosition = {0.0F, 1.5F, 2.5F};
const Vector3 kDefaultCameraTarget = {0.0F, 0.0F, 0.0F};
const Vector3 kDefaultCameraUp = {0.0F, 0.0F, 1.0F};
const Vector3 kLightDirection = {1.0F, 1.0F, 1.0F};

// Grid
constexpr int kGridSlices = 10;
constexpr float kGridSpacing = 1.0F;
constexpr float kGridAxisColor = 0.5F;
constexpr float kGridLineColor = 0.75F;

// UI
constexpr float kSidePanelWidth = 350.0F;
constexpr float kRobotTableHeight = 300.0F;
constexpr float kMaterialEditorReserveHeight = 200.0F;
constexpr float kMinPropertiesHeight = 80.0F;
constexpr int kSupersamplingScale = 2;

// Selection outline
constexpr float kOutlineWidthPx = 5.0F;
constexpr float kOutlineColor[4] = {1.0F, 0.65F, 0.0F, 1.0F};

// Joint axis visualization
constexpr float kAxisLength = 0.3F;
constexpr float kAxisTipLength = 0.04F;
constexpr float kAxisTipRadius = 0.015F;
constexpr float kAxisSphereRadius = 0.015F;
const Vector3 kUrdfDefaultAxis = {1.0F, 0.0F, 0.0F};

// Platform-aware modifier label for menu shortcuts
#ifdef __APPLE__
constexpr const char* kModName = "Cmd";
#else
constexpr const char* kModName = "Ctrl";
#endif

void drawGridZUp(int slices, float spacing) {
    int half_slices = slices / 2;

    rlBegin(RL_LINES);
    for (int i = -half_slices; i <= half_slices; i++) {
        if (i == 0) {
            rlColor3f(kGridAxisColor, kGridAxisColor, kGridAxisColor);
            rlColor3f(kGridAxisColor, kGridAxisColor, kGridAxisColor);
            rlColor3f(kGridAxisColor, kGridAxisColor, kGridAxisColor);
            rlColor3f(kGridAxisColor, kGridAxisColor, kGridAxisColor);
        } else {
            rlColor3f(kGridLineColor, kGridLineColor, kGridLineColor);
            rlColor3f(kGridLineColor, kGridLineColor, kGridLineColor);
            rlColor3f(kGridLineColor, kGridLineColor, kGridLineColor);
            rlColor3f(kGridLineColor, kGridLineColor, kGridLineColor);
        }

        rlVertex3f(static_cast<float>(i) * spacing, static_cast<float>(-half_slices) * spacing,
                   0.0F);
        rlVertex3f(static_cast<float>(i) * spacing, static_cast<float>(half_slices) * spacing,
                   0.0F);

        rlVertex3f(static_cast<float>(-half_slices) * spacing, static_cast<float>(i) * spacing,
                   0.0F);
        rlVertex3f(static_cast<float>(half_slices) * spacing, static_cast<float>(i) * spacing,
                   0.0F);
    }
    rlEnd();
}

// Pan the camera right and up by the given amounts, keeping the target in sync.
static void cameraPan(Camera3D *camera, float dx, float dy) {
    CameraMoveRight(camera, dx, true);
    Vector3 right = GetCameraRight(camera);
    Vector3 up = Vector3CrossProduct(Vector3Subtract(camera->position, camera->target), right);
    up = Vector3Scale(Vector3Normalize(up), dy);
    camera->position = Vector3Add(camera->position, up);
    camera->target = Vector3Add(camera->target, up);
}

// Orbit the camera around its target by yaw and pitch deltas.
static void cameraOrbit(Camera3D *camera, float dx, float dy) {
    CameraYaw(camera, dx, true);
    CameraPitch(camera, dy, true, true, false);
}

// Compute the center of the combined bounding box of all meshes in a model.
static Vector3 getModelBoundingBoxCenter(const Model& model) {
    BoundingBox bbox = GetMeshBoundingBox(model.meshes[0]);
    for (int i = 1; i < model.meshCount; i++) {
        BoundingBox mb = GetMeshBoundingBox(model.meshes[i]);
        if (mb.min.x < bbox.min.x) bbox.min.x = mb.min.x;
        if (mb.min.y < bbox.min.y) bbox.min.y = mb.min.y;
        if (mb.min.z < bbox.min.z) bbox.min.z = mb.min.z;
        if (mb.max.x > bbox.max.x) bbox.max.x = mb.max.x;
        if (mb.max.y > bbox.max.y) bbox.max.y = mb.max.y;
        if (mb.max.z > bbox.max.z) bbox.max.z = mb.max.z;
    }
    return {(bbox.min.x + bbox.max.x) * 0.5F,
            (bbox.min.y + bbox.max.y) * 0.5F,
            (bbox.min.z + bbox.max.z) * 0.5F};
}

static void updateCamera(Camera3D *camera) {
    bool is_mmb_down = IsMouseButtonDown(MOUSE_BUTTON_MIDDLE);
    bool is_shift_down = IsKeyDown(KEY_LEFT_SHIFT);

    // MMB controls (mouse users)
    if (is_mmb_down) {
        Vector2 mouse_delta = GetMouseDelta();
        if (is_shift_down) {
            cameraPan(camera, -kCameraMoveSpeed * mouse_delta.x,
                      -kCameraMoveSpeed * mouse_delta.y);
        } else {
            cameraOrbit(camera, -kCameraRotSpeed * mouse_delta.x,
                        -kCameraRotSpeed * mouse_delta.y);
        }
    }

    // Scroll-based controls (trackpad + mouse wheel)
    Vector2 wheel = Vector2Scale(GetMouseWheelMoveV(), 3.0F);
    bool is_ctrl_down = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);

    if (is_ctrl_down) {
        CameraMoveToTarget(camera, -wheel.y * kCameraZoomSpeed);
    } else if (is_shift_down) {
        cameraPan(camera, -wheel.x * kCameraMoveSpeed, wheel.y * kCameraMoveSpeed);
    } else {
        cameraOrbit(camera, -wheel.x * kCameraRotSpeed, -wheel.y * kCameraRotSpeed);
    }
}

App::App(int argc, char *argv[]) : bShowGrid_(true), selected_link_origin_{nullptr} {
    loguru::init(argc, argv);
}

void App::run() {
    setup();

    while (not bWindowShouldClose_) {
        update();
        draw();
        bWindowShouldClose_ |= WindowShouldClose();
    }

    cleanup();
}

void App::setup() {
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE | FLAG_WINDOW_HIGHDPI);
    InitWindow(kDefaultScreenWidth, kDefaultScreenHeight, "URDF Editor");

    shader_ = LoadShader("./resources/shaders/lighting.vs", "./resources/shaders/lighting.fs");
    shader_.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(shader_, "viewPos");
    shader_.locs[SHADER_LOC_COLOR_DIFFUSE] = GetShaderLocation(shader_, "colDiffuse");
    shader_.locs[SHADER_LOC_COLOR_AMBIENT] = GetShaderLocation(shader_, "ambient");

    SetShaderValue(shader_, shader_.locs[SHADER_LOC_COLOR_AMBIENT],
                   std::array<float, 4>({kAmbientLightIntensity, kAmbientLightIntensity,
                                         kAmbientLightIntensity, 1.0F})
                       .data(),
                   SHADER_UNIFORM_VEC4);

    // Create lights
    std::array<Light, MAX_LIGHTS> lights{};
    lights[0] = CreateLight(LIGHT_DIRECTIONAL, kLightDirection, Vector3Zero(), WHITE, shader_);

    // Define the camera to look into our 3d world
    camera_ = {kDefaultCameraPosition, kDefaultCameraTarget, kDefaultCameraUp, kDefaultCameraFov,
               0};
    bWindowShouldClose_ = false;

    outline_shader_ =
        LoadShader("./resources/shaders/outline.vs", "./resources/shaders/outline.fs");
    outline_loc_width_ = GetShaderLocation(outline_shader_, "outlineWidth");
    outline_loc_viewport_ = GetShaderLocation(outline_shader_, "viewportSize");
    outline_loc_color_ = GetShaderLocation(outline_shader_, "outlineColor");
    outline_loc_center_ = GetShaderLocation(outline_shader_, "objectCenter");

    gizmo_ = rgizmo_create();

    SetTargetFPS(kTargetFps);
    rlImGuiSetup(true);
}

void App::update() {
    command_buffer_.execute();

    // Update camera only when mouse is over the viewport
    if (viewport_hovered_) {
        updateCamera(&camera_);
    }

    // Viewport hover & click-to-select
    if (viewport_hovered_ && gizmo_.state < RGIZMO_STATE_HOT && robot_) {
        Vector2 mouse = GetMousePosition();
        int vp_w = GetScreenWidth() - static_cast<int>(kSidePanelWidth);
        int vp_h = GetScreenHeight() - menubar_height_;
        Vector2 viewport_mouse = {mouse.x - kSidePanelWidth,
                                  mouse.y - static_cast<float>(menubar_height_)};
        Ray ray = GetScreenToWorldRayEx(viewport_mouse, camera_, vp_w, vp_h);

        auto hit = robot_->getLink(ray);

        hovered_node_ = hit ? hit->link : nullptr;

        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            if (!hit) {
                selected_node_ = nullptr;
                selected_link_origin_ = nullptr;
                pending_tab_ = -1;
            } else if (hit->type == urdf::HitResult::kVisual) {
                selected_node_ = hit->link;
                pending_tab_ = hit->index;
                auto& origin = hit->link->link.visual[hit->index].origin;
                selected_link_origin_ = origin ? &*origin : nullptr;
            } else {
                selected_node_ = hit->link;
                int num_visuals = static_cast<int>(hit->link->link.visual.size());
                pending_tab_ = num_visuals + hit->index;
                auto& origin = hit->link->link.collision[hit->index].origin;
                selected_link_origin_ = origin ? &*origin : nullptr;
            }
        }
    }

    //----------------------------------------------------------------------------------
}

void App::drawViewport() {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    ImGui::SetNextWindowPos(ImVec2(kSidePanelWidth, static_cast<float>(menubar_height_)),
                            ImGuiCond_Always);
    ImGui::SetNextWindowSize(
        ImVec2(static_cast<float>(GetScreenWidth()) - kSidePanelWidth,
               static_cast<float>(GetScreenHeight()) - static_cast<float>(menubar_height_)),
        ImGuiCond_Always);

    ImGui::Begin("##Viewport", nullptr,
                 ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                     ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar |
                     ImGuiWindowFlags_NoScrollWithMouse |
                     ImGuiWindowFlags_NoBringToFrontOnFocus);

    int vp_w = GetScreenWidth() - static_cast<int>(kSidePanelWidth);
    int vp_h = GetScreenHeight() - menubar_height_;
    rlImGuiImageRect(
        &scene_texture_.texture, vp_w, vp_h,
        {0, 0, static_cast<float>(scene_texture_.texture.width),
         -static_cast<float>(scene_texture_.texture.height)});
    viewport_hovered_ = ImGui::IsWindowHovered();

    ImGui::End();
    ImGui::PopStyleVar();
}

void App::drawMenu() {
    rlImGuiBegin();

    handleShortcuts();
    drawToolbar();
    drawSideMenu();
    drawViewport();

    // Validation popup (triggered by saveFile)
    if (pending_save_) {
        ImGui::OpenPopup("Validation");
        pending_save_ = false;
    }
    if (ImGui::BeginPopupModal("Validation", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        bool has_errors = false;
        for (const auto& msg : validation_results_) {
            if (msg.level == urdf::ValidationMessage::kError) {
                has_errors = true;
                ImGui::TextColored(ImVec4(1, 0.2f, 0.2f, 1), "Error: %s", msg.message.c_str());
            } else {
                ImGui::TextColored(ImVec4(1, 0.8f, 0, 1), "Warning: %s", msg.message.c_str());
            }
        }
        ImGui::Separator();
        if (has_errors) {
            ImGui::Text("Errors must be fixed before saving.");
            if (ImGui::Button("OK")) {
                ImGui::CloseCurrentPopup();
            }
        } else {
            if (ImGui::Button("Save anyway")) {
                exportRobot(*robot_, pending_save_path_);
                LOG_F(INFO, "Saved with warnings: %s", pending_save_path_.c_str());
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel")) {
                ImGui::CloseCurrentPopup();
            }
        }
        ImGui::EndPopup();
    }

    rlImGuiEnd();
}

void App::drawSelectionOutline(const std::shared_ptr<urdf::LinkNode>& link,
                               Rectangle viewport) {
    if (link->visual_models.empty()) return;

    rlDrawRenderBatchActive();

    // Pass 1: Fill stencil buffer with 1 where the selected meshes are drawn
    glEnable(GL_STENCIL_TEST);
    glDisable(GL_DEPTH_TEST);
    glStencilFunc(GL_ALWAYS, 1, 0xFF);
    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
    glStencilMask(0xFF);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    glDepthMask(GL_FALSE);

    rlDrawRenderBatchActive();
    for (const Model& vm : link->visual_models) {
        DrawModel(vm, Vector3Zero(), 1.0F, WHITE);
    }
    rlDrawRenderBatchActive();

    // Pass 2: Draw expanded mesh only where stencil != 1 (the border region)
    glStencilFunc(GL_NOTEQUAL, 1, 0xFF);
    glStencilMask(0x00);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

    for (Model& vm : link->visual_models) {
        if (vm.meshCount <= 0) continue;

        // Set outline shader uniforms (locations cached at load time)
        float vp_size[2] = {viewport.width, viewport.height};
        Vector3 center = getModelBoundingBoxCenter(vm);

        SetShaderValue(outline_shader_, outline_loc_width_, &kOutlineWidthPx, SHADER_UNIFORM_FLOAT);
        SetShaderValue(outline_shader_, outline_loc_viewport_, vp_size, SHADER_UNIFORM_VEC2);
        SetShaderValue(outline_shader_, outline_loc_color_, kOutlineColor, SHADER_UNIFORM_VEC4);
        SetShaderValue(outline_shader_, outline_loc_center_, &center, SHADER_UNIFORM_VEC3);

        // Temporarily swap to outline shader for all materials
        for (int i = 0; i < vm.materialCount; i++) {
            vm.materials[i].shader = outline_shader_;
        }

        DrawModel(vm, Vector3Zero(), 1.0F, WHITE);

        for (int i = 0; i < vm.materialCount; i++) {
            vm.materials[i].shader = shader_;
        }
    }

    rlDrawRenderBatchActive();

    // Restore GL state
    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_STENCIL_TEST);
    glStencilMask(0xFF);

    rlDrawRenderBatchActive();
}

void App::drawJointAxis(const urdf::JointNodePtr& joint) {
    // Only revolute, continuous, and prismatic joints have a meaningful axis
    switch (joint->joint.type) {
        case urdf::Joint::kRevolute:
        case urdf::Joint::kContinuous:
        case urdf::Joint::kPrismatic:
            break;
        default:
            return;
    }

    // Compute world-space joint transform: w_T_j = w_T_p * p_T_j
    const Matrix& w_t_p = joint->parent->w_T_l;
    Matrix p_t_j = joint->joint.origin ? joint->joint.origin->toMatrix() : MatrixIdentity();
    Matrix w_t_j = MatrixMultiply(p_t_j, w_t_p);

    Vector3 origin = urdf::PosFromMatrix(w_t_j);

    // Rotate local axis into world space (strip translation to get rotation only)
    Vector3 local_axis = joint->joint.axis ? joint->joint.axis->xyz : kUrdfDefaultAxis;
    Matrix w_rot_j = w_t_j;
    w_rot_j.m12 = 0.0F;
    w_rot_j.m13 = 0.0F;
    w_rot_j.m14 = 0.0F;
    Vector3 world_axis = Vector3Normalize(Vector3Transform(local_axis, w_rot_j));

    // Draw on top of everything (like gizmos)
    rlDrawRenderBatchActive();
    rlDisableDepthTest();

    switch (joint->joint.type) {
        case urdf::Joint::kRevolute:
        case urdf::Joint::kContinuous: {
            // Arrow: line + cone tip
            float line_len = kAxisLength - kAxisTipLength;
            Vector3 line_end = Vector3Add(origin, Vector3Scale(world_axis, line_len));
            Vector3 tip_end = Vector3Add(origin, Vector3Scale(world_axis, kAxisLength));

            DrawLine3D(origin, line_end, YELLOW);
            DrawCylinderEx(line_end, tip_end, kAxisTipRadius, 0.0F, 8, YELLOW);
            break;
        }
        case urdf::Joint::kPrismatic: {
            // Line segment along axis between lower/upper limits, with spheres at endpoints
            float lower = 0.0F;
            float upper = kAxisLength;
            if (joint->joint.limit && joint->joint.limit->upper > joint->joint.limit->lower) {
                lower = joint->joint.limit->lower;
                upper = joint->joint.limit->upper;
            }

            Vector3 start = Vector3Add(origin, Vector3Scale(world_axis, lower));
            Vector3 end = Vector3Add(origin, Vector3Scale(world_axis, upper));

            DrawLine3D(start, end, YELLOW);
            DrawSphere(start, kAxisSphereRadius, YELLOW);
            DrawSphere(end, kAxisSphereRadius, YELLOW);
            break;
        }
        default:
            break;
    }

    rlDrawRenderBatchActive();
    rlEnableDepthTest();
}

void App::drawScene(Rectangle viewport) {
    ClearBackground(LIGHTGRAY);
    glClear(GL_STENCIL_BUFFER_BIT);

    // TODO(ramon): links can't be selected. Select instead visual, collision,
    // inertial and modify it's origin, if it exists. const auto selected_link =
    const auto selected_link = std::dynamic_pointer_cast<urdf::LinkNode>(selected_node_);
    if (selected_link and selected_link_origin_) {
        // Compose world-frame origin transform: w_T_o = w_T_l * l_T_o
        const Matrix &w_t_l = selected_link->w_T_l;
        Matrix l_t_o = selected_link_origin_->toMatrix();
        Matrix w_t_o = MatrixMultiply(l_t_o, w_t_l);

        const Vector3 position{urdf::PosFromMatrix(w_t_o)};
        if (rgizmo_update(&gizmo_, camera_, position, viewport)) {
            // Apply gizmo delta in world space, then recover local origin:
            // l_T_o = w_T_o' * inv(w_T_l)
            w_t_o = MatrixMultiply(w_t_o, rgizmo_get_transform(gizmo_, position));
            l_t_o = MatrixMultiply(w_t_o, MatrixInvert(w_t_l));

            // Save new origin
            selected_link_origin_->xyz.x = l_t_o.m12;
            selected_link_origin_->xyz.y = l_t_o.m13;
            selected_link_origin_->xyz.z = l_t_o.m14;
            selected_link_origin_->rpy = urdf::MatrixToXYZ(l_t_o);

            // Update the robot
            robot_->forwardKinematics();
        }
        // Re-bind scene FBO after rgizmo_update's internal picking pass
        rlEnableFramebuffer(scene_texture_.id);
        rlViewport(0, 0, scene_texture_.texture.width, scene_texture_.texture.height);
    }

    const auto selected_joint = std::dynamic_pointer_cast<urdf::JointNode>(selected_node_);
    if (selected_joint and selected_joint->joint.origin) {
        urdf::Origin &joint_origin = *selected_joint->joint.origin;

        // Compose world-frame joint transform: w_T_j = w_T_parent * p_T_j
        const Matrix &w_t_p = selected_joint->parent->w_T_l;
        Matrix p_t_j = joint_origin.toMatrix();
        Matrix w_t_j = MatrixMultiply(p_t_j, w_t_p);

        const Vector3 position{urdf::PosFromMatrix(w_t_j)};
        if (rgizmo_update(&gizmo_, camera_, position, viewport)) {
            // Apply gizmo delta in world space, then recover local joint origin:
            // p_T_j = w_T_j' * inv(w_T_parent)
            w_t_j = MatrixMultiply(w_t_j, rgizmo_get_transform(gizmo_, position));
            p_t_j = MatrixMultiply(w_t_j, MatrixInvert(w_t_p));

            // Save new origin {xyz, rpy}
            joint_origin.xyz.x = p_t_j.m12;
            joint_origin.xyz.y = p_t_j.m13;
            joint_origin.xyz.z = p_t_j.m14;
            joint_origin.rpy = urdf::MatrixToXYZ(p_t_j);

            // Update the robot
            robot_->forwardKinematics();
        }
        // Re-bind scene FBO after rgizmo_update's internal picking pass
        rlEnableFramebuffer(scene_texture_.id);
        rlViewport(0, 0, scene_texture_.texture.width, scene_texture_.texture.height);
    }

    bool was_active = prev_gizmo_state_ >= RGIZMO_STATE_ACTIVE;
    bool is_active = gizmo_.state >= RGIZMO_STATE_ACTIVE;

    if (!was_active && is_active) {
        // Drag just started — snapshot the origin
        if (selected_link && selected_link_origin_) {
            gizmo_drag_origin_ = selected_link_origin_;
            snapshot_gizmo_origin_ = *selected_link_origin_;
        } else if (selected_joint && selected_joint->joint.origin) {
            gizmo_drag_origin_ = &(*selected_joint->joint.origin);
            snapshot_gizmo_origin_ = *selected_joint->joint.origin;
        }
    }

    if (was_active && !is_active && gizmo_drag_origin_ && snapshot_gizmo_origin_) {
        // Drag just ended — create undo command
        auto fk = [this]() { robot_->forwardKinematics(); };
        command_buffer_.add(std::make_shared<UpdatePropertyCommand<urdf::Origin>>(
            *gizmo_drag_origin_, *snapshot_gizmo_origin_, *gizmo_drag_origin_, fk));
        gizmo_drag_origin_ = nullptr;
        snapshot_gizmo_origin_.reset();
    }

    prev_gizmo_state_ = gizmo_.state;

    BeginMode3D(camera_);

    if (bShowGrid_) drawGridZUp(kGridSlices, kGridSpacing);

    if (robot_) {
        const auto hovered_node = std::dynamic_pointer_cast<urdf::LinkNode>(hovered_node_);
        robot_->draw(hovered_node, selected_link);
    }

    if (selected_link) {
        drawSelectionOutline(selected_link, viewport);
    }

    if (selected_joint) {
        drawJointAxis(selected_joint);
    }

    if (selected_link and selected_link_origin_) {
        const Matrix &w_t_l = selected_link->w_T_l;
        Matrix l_t_o = selected_link_origin_->toMatrix();
        Matrix w_t_o = MatrixMultiply(l_t_o, w_t_l);

        const Vector3 position{urdf::PosFromMatrix(w_t_o)};
        rgizmo_draw(gizmo_, camera_, position, viewport);
    }

    if (selected_joint and selected_joint->joint.origin) {
        const Matrix &w_t_p = selected_joint->parent->w_T_l;
        const Matrix p_t_j = selected_joint->joint.origin->toMatrix();
        const Matrix w_t_j = MatrixMultiply(p_t_j, w_t_p);

        const Vector3 position{urdf::PosFromMatrix(w_t_j)};
        rgizmo_draw(gizmo_, camera_, position, viewport);
    }

    EndMode3D();
}

void App::recreateSceneTexture(int width, int height) {
    // Clean up existing resources
    if (scene_texture_.id != 0) {
        rlUnloadTexture(scene_texture_.texture.id);
        rlUnloadFramebuffer(scene_texture_.id);
    }
    if (depth_stencil_rbo_ != 0) {
        glDeleteRenderbuffers(1, &depth_stencil_rbo_);
    }

    scene_texture_ = {};
    depth_stencil_rbo_ = 0;

    scene_texture_.id = rlLoadFramebuffer();
    if (scene_texture_.id > 0) {
        rlEnableFramebuffer(scene_texture_.id);

        // Color texture
        scene_texture_.texture.id =
            rlLoadTexture(nullptr, width, height, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8, 1);
        scene_texture_.texture.width = width;
        scene_texture_.texture.height = height;
        scene_texture_.texture.format = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8;
        scene_texture_.texture.mipmaps = 1;

        // Combined depth-stencil renderbuffer (required for stencil outline pass)
        glGenRenderbuffers(1, &depth_stencil_rbo_);
        glBindRenderbuffer(GL_RENDERBUFFER, depth_stencil_rbo_);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, width, height);
        glBindRenderbuffer(GL_RENDERBUFFER, 0);

        rlFramebufferAttach(scene_texture_.id, scene_texture_.texture.id,
                            RL_ATTACHMENT_COLOR_CHANNEL0, RL_ATTACHMENT_TEXTURE2D, 0);
        rlFramebufferAttach(scene_texture_.id, depth_stencil_rbo_,
                            RL_ATTACHMENT_DEPTH, RL_ATTACHMENT_RENDERBUFFER, 0);
        rlFramebufferAttach(scene_texture_.id, depth_stencil_rbo_,
                            RL_ATTACHMENT_STENCIL, RL_ATTACHMENT_RENDERBUFFER, 0);

        scene_texture_.depth.id = depth_stencil_rbo_;
        scene_texture_.depth.width = width;
        scene_texture_.depth.height = height;

        rlFramebufferComplete(scene_texture_.id);
        rlDisableFramebuffer();
    }

    SetTextureFilter(scene_texture_.texture, TEXTURE_FILTER_BILINEAR);
}

void App::draw() {
    // Compute viewport rect (right of side panel, below menu bar)
    int vp_w = GetScreenWidth() - static_cast<int>(kSidePanelWidth);
    int vp_h = GetScreenHeight() - menubar_height_;
    if (vp_w < 1) vp_w = 1;
    if (vp_h < 1) vp_h = 1;

    // Render at 2x resolution when supersampling is enabled
    int scale = bSupersampling_ ? kSupersamplingScale : 1;
    int tex_w = vp_w * scale;
    int tex_h = vp_h * scale;

    // Recreate FBO if viewport size changed
    if (tex_w != scene_texture_.texture.width || tex_h != scene_texture_.texture.height) {
        recreateSceneTexture(tex_w, tex_h);
    }

    // Viewport in window coordinates (for gizmo mouse mapping)
    Rectangle viewport = {kSidePanelWidth, static_cast<float>(menubar_height_),
                          static_cast<float>(vp_w), static_cast<float>(vp_h)};

    // 1. Render 3D scene into texture
    BeginTextureMode(scene_texture_);
    drawScene(viewport);
    EndTextureMode();

    // 2. Draw ImGui overlay (includes viewport image)
    BeginDrawing();
    ClearBackground(DARKGRAY);
    drawMenu();
    EndDrawing();
}

void App::openFile() {
    NFD::UniquePath out_path;
    nfdfilteritem_t filter_item[2] = {{"URDF file", "urdf,xml"}};
    nfdresult_t result = NFD::OpenDialog(out_path, filter_item, 1);
    if (result == NFD_OKAY) {
        command_buffer_.add(std::make_shared<LoadRobotCommand>(out_path.get(), robot_, shader_));
    } else if (result == NFD_CANCEL) {
        LOG_F(INFO, "User pressed cancel.");
    } else {
        LOG_F(ERROR, "Error: %s", NFD::GetError());
    }
}

void App::saveFile() {
    NFD::UniquePath out_path;
    nfdfilteritem_t filter_item[2] = {{"URDF file", "urdf"}};
    nfdresult_t result = NFD::SaveDialog(out_path, filter_item, 1);
    if (result == NFD_OKAY) {
        validation_results_ = robot_->validate();
        if (validation_results_.empty()) {
            LOG_F(INFO, "Success! %s", out_path.get());
            exportRobot(*robot_, out_path.get());
        } else {
            pending_save_ = true;
            pending_save_path_ = out_path.get();
        }
    } else if (result == NFD_CANCEL) {
        LOG_F(INFO, "User pressed cancel.");
    } else {
        LOG_F(ERROR, "Error: %s", NFD::GetError());
    }
}

void App::handleShortcuts() {
    ImGuiIO& io = ImGui::GetIO();
    if (io.WantTextInput) return;

    if (ImGui::IsKeyChordPressed(ImGuiMod_Shortcut | ImGuiKey_Z)) {
        command_buffer_.undo();
    }
    if (ImGui::IsKeyChordPressed(ImGuiMod_Shortcut | ImGuiKey_Y)) {
        command_buffer_.redo();
    }
    if (ImGui::IsKeyChordPressed(ImGuiMod_Shortcut | ImGuiMod_Shift | ImGuiKey_Z)) {
        command_buffer_.redo();
    }
    if (ImGui::IsKeyChordPressed(ImGuiMod_Shortcut | ImGuiKey_O)) {
        openFile();
    }
    if (ImGui::IsKeyChordPressed(ImGuiMod_Shortcut | ImGuiKey_S)) {
        if (robot_) saveFile();
    }
    if (ImGui::IsKeyChordPressed(ImGuiMod_Shortcut | ImGuiKey_N)) {
        command_buffer_.add(std::make_shared<CreateRobotCommand>(robot_, shader_));
    }
    if (ImGui::IsKeyChordPressed(ImGuiMod_Shortcut | ImGuiKey_J)) {
        auto link_node = std::dynamic_pointer_cast<urdf::LinkNode>(selected_node_);
        if (link_node) {
            command_buffer_.add(
                std::make_shared<CreateJointCommand>("New Joint", link_node, robot_));
        }
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Delete) || ImGui::IsKeyPressed(ImGuiKey_Backspace)) {
        deleteSelectedJoint();
    }
}

void App::deleteSelectedJoint() {
    if (auto joint_node = std::dynamic_pointer_cast<urdf::JointNode>(selected_node_)) {
        command_buffer_.add(std::make_shared<DeleteJointCommand>(
            joint_node, robot_, selected_node_, selected_link_origin_));
    } else if (auto link_node = std::dynamic_pointer_cast<urdf::LinkNode>(selected_node_)) {
        if (link_node->parent) {
            command_buffer_.add(std::make_shared<DeleteJointCommand>(
                link_node->parent, robot_, selected_node_, selected_link_origin_));
        }
    }
}

void App::drawToolbar() {
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("Open", fmt::format("{}+O", kModName).c_str())) {
                openFile();
            }

            if (ImGui::MenuItem("Save", fmt::format("{}+S", kModName).c_str(), false,
                                static_cast<bool>(robot_))) {
                saveFile();
            }
            if (ImGui::MenuItem("Exit", "Alt+F4")) {
                bWindowShouldClose_ = true;
            }

            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Edit")) {
            if (ImGui::MenuItem("Undo", fmt::format("{}+Z", kModName).c_str(), false,
                                command_buffer_.canUndo())) {
                command_buffer_.undo();
            }
            if (ImGui::MenuItem("Redo", fmt::format("{}+Y", kModName).c_str(), false,
                                command_buffer_.canRedo())) {
                command_buffer_.redo();
            }
            ImGui::Separator();
            if (ImGui::MenuItem("New Robot", fmt::format("{}+N", kModName).c_str())) {
                command_buffer_.add(std::make_shared<CreateRobotCommand>(robot_, shader_));
            }
            auto link_node = std::dynamic_pointer_cast<urdf::LinkNode>(selected_node_);
            if (ImGui::MenuItem("Create Joint", fmt::format("{}+J", kModName).c_str(), false,
                                link_node != nullptr)) {
                command_buffer_.add(
                    std::make_shared<CreateJointCommand>("New Joint", link_node, robot_));
            }
            bool can_duplicate = link_node && link_node->parent;
            if (ImGui::MenuItem("Duplicate", nullptr, false, can_duplicate)) {
                command_buffer_.add(
                    std::make_shared<CloneSubtreeCommand>(link_node, robot_, shader_));
            }
            // Enable delete when a joint is selected, or a non-root link
            bool is_joint = std::dynamic_pointer_cast<urdf::JointNode>(selected_node_) != nullptr;
            bool can_delete = is_joint || (link_node && link_node->parent);
            if (ImGui::MenuItem("Delete", "Del", false, can_delete)) {
                deleteSelectedJoint();
            }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("View")) {
            ImGui::MenuItem("Show Grid", nullptr, &bShowGrid_);
            ImGui::MenuItem("Supersampling AA", nullptr, &bSupersampling_);
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Help")) {
            ImGui::MenuItem("About");
            ImGui::EndMenu();
        }
    }

    menubar_height_ = static_cast<int>(ImGui::GetWindowSize().y);
    ImGui::EndMainMenuBar();
}

void App::drawRobotTree() {
    ImGuiTableFlags table_flags = ImGuiTableFlags_BordersV | ImGuiTableFlags_BordersOuterH |
                                  ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable |
                                  ImGuiTableFlags_Hideable | ImGuiTableFlags_NoBordersInBody |
                                  ImGuiTableFlags_ScrollY;

    if (ImGui::BeginTable("robot table", 2, table_flags, ImVec2(0, kRobotTableHeight))) {
        ImGui::TableSetupColumn(
            "Name", ImGuiTableColumnFlags_NoHide | ImGuiTableColumnFlags_WidthStretch, 0.75F);
        ImGui::TableSetupColumn(
            "Type", ImGuiTableColumnFlags_NoHide | ImGuiTableColumnFlags_WidthStretch, 0.25F);
        ImGui::TableHeadersRow();

        if (not robot_) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("--");
            ImGui::TableNextColumn();
            ImGui::Text("--");
        } else {
            ImGuiTreeNodeFlags tree_flags = ImGuiTreeNodeFlags_DefaultOpen |
                                            ImGuiTreeNodeFlags_OpenOnArrow |
                                            ImGuiTreeNodeFlags_SpanAllColumns;

            float orig_indent = ImGui::GetStyle().IndentSpacing;
            ImGui::GetStyle().IndentSpacing = 10.0F;

            urdf::LinkNodePtr current_link = robot_->getRoot();

            std::function<void(const urdf::LinkNodePtr &)> recursion = [&](auto link) {
                if (not link) return;

                // Unique ID for drag and drop
                void *node_id = static_cast<void *>(link.get());

                ImGui::TableNextRow();
                ImGui::TableNextColumn();

                bool open = ImGui::TreeNodeEx(
                    link->link.name.c_str(),
                    tree_flags | (link->children.empty() ? ImGuiTreeNodeFlags_Leaf : 0) |
                        (link.get() == selected_node_.get() ? ImGuiTreeNodeFlags_Selected : 0));

                if (ImGui::IsItemHovered()) {
                    hovered_node_ = link;
                }

                if (ImGui::IsItemClicked() and selected_node_ != link) {
                    selected_node_ = link;
                    selected_link_origin_ = nullptr;
                }

                // Drag source for link node
                if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_None)) {
                    ImGui::SetDragDropPayload("LINK_NODE", &node_id, sizeof(void *));
                    ImGui::Text("Moving %s", link->link.name.c_str());
                    ImGui::EndDragDropSource();
                }

                // Drop target for joint node
                if (ImGui::BeginDragDropTarget()) {
                    if (const ImGuiPayload *payload = ImGui::AcceptDragDropPayload("JOINT_NODE")) {
                        urdf::JointNodePtr dropped_joint_node =
                            *static_cast<urdf::JointNodePtr *>(payload->Data);
                        LOG_F(INFO, "Dropped joint %s onto link %s",
                              dropped_joint_node->joint.name.c_str(), link->link.name.c_str());
                        command_buffer_.add(std::make_shared<JointChangeParentCommand>(
                            dropped_joint_node, link, robot_));
                    }
                    ImGui::EndDragDropTarget();
                }

                ImGui::TableNextColumn();
                ImGui::Text("Link");

                if (open) {
                    for (urdf::JointNodePtr &joint : link->children) {
                        ImGui::TableNextRow();
                        ImGui::TableNextColumn();

                        bool open =
                            ImGui::TreeNodeEx(joint->joint.name.c_str(),
                                              tree_flags | (joint.get() == selected_node_.get()
                                                                ? ImGuiTreeNodeFlags_Selected
                                                                : 0));

                        if (ImGui::IsItemHovered()) {
                            hovered_node_ = nullptr;
                        }

                        if (ImGui::IsItemClicked()) {
                            selected_node_ = joint;
                        }

                        // Drag source for joint node
                        if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_None)) {
                            ImGui::SetDragDropPayload("JOINT_NODE", &joint,
                                                      sizeof(urdf::JointNodePtr *));
                            ImGui::Text("Moving %s", joint->joint.name.c_str());
                            ImGui::EndDragDropSource();
                        }

                        // Drop target for link node
                        if (ImGui::BeginDragDropTarget()) {
                            if (const ImGuiPayload *payload =
                                    ImGui::AcceptDragDropPayload("LINK_NODE")) {
                                // void* dropped_link_node_id =
                                // *static_cast<void**>(payload->Data); Handle the drop for
                                // a link node (update parent/child relationships)
                            }
                            ImGui::EndDragDropTarget();
                        }

                        ImGui::TableNextColumn();
                        ImGui::Text("Joint");

                        if (open) {
                            recursion(joint->child);
                            ImGui::TreePop();
                        }
                    }
                    ImGui::TreePop();
                }
            };

            recursion(current_link);

            ImGui::GetStyle().IndentSpacing = orig_indent;
        }

        ImGui::EndTable();
    }
}

void App::drawSideMenu() {
    ImGui::SetNextWindowPos(ImVec2(0, ImGui::GetFrameHeight()), ImGuiCond_Always);
    ImGui::SetNextWindowSize(
        ImVec2(kSidePanelWidth, static_cast<float>(GetScreenHeight()) - ImGui::GetFrameHeight()),
        ImGuiCond_Always);

    ImGui::Begin("Robot Tree", nullptr,
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar);

    drawRobotTree();
    ImGui::Separator();

    // Scrollable properties region that fills space above the material editor
    float avail = ImGui::GetContentRegionAvail().y;
    float props_height = std::max(avail - kMaterialEditorReserveHeight, kMinPropertiesHeight);

    if (ImGui::BeginChild("Properties", ImVec2(0, props_height))) {
        drawNodeProperties();
    }
    ImGui::EndChild();

    ImGui::Separator();
    drawMaterialEditor();

    ImGui::End();
}

void App::menuPropertiesInertial(urdf::LinkNodePtr link_node) {
    if (auto &inertial = link_node->link.inertial) {
        selected_link_origin_ = &inertial->origin;

        inputFloatUndoable("Mass", inertial->mass);

        originGui(inertial->origin);

        if (ImGui::TreeNode("Inertia")) {
            inputFloatUndoable("ixx", inertial->inertia.ixx);
            inputFloatUndoable("iyy", inertial->inertia.iyy);
            inputFloatUndoable("izz", inertial->inertia.izz);
            inputFloatUndoable("ixy", inertial->inertia.ixy);
            inputFloatUndoable("ixz", inertial->inertia.ixz);
            inputFloatUndoable("iyz", inertial->inertia.iyz);
            ImGui::TreePop();
        }
    } else {
        selected_link_origin_ = nullptr;
        if (ImGui::Button("Create inertial component")) {
            command_buffer_.add(
                std::make_shared<CreateInertialCommand>(link_node, selected_link_origin_));
        }
    }
}

void App::menuPropertiesCollisions(urdf::LinkNodePtr link_node, int i) {
    urdf::Collision &col = link_node->link.collision[i];

    selected_link_origin_ = col.origin.has_value() ? &col.origin.value() : nullptr;

    menuName(col.name, "collision");
    menuOrigin(col.origin);
    menuGeometry(col.geometry, link_node->collision_models[i]);
}

void App::drawNodeProperties() {
    if (not selected_node_ or not robot_) return;

    if (auto link_node = std::dynamic_pointer_cast<urdf::LinkNode>(selected_node_)) {
        // Link name (with rename command that cascades to joints)
        static char link_name_buf[256];
        strncpy(link_name_buf, link_node->link.name.c_str(), sizeof(link_name_buf) - 1);
        link_name_buf[sizeof(link_name_buf) - 1] = '\0';
        if (ImGui::InputText("Name##link", link_name_buf, sizeof(link_name_buf),
                             ImGuiInputTextFlags_EnterReturnsTrue)) {
            std::string new_name(link_name_buf);
            if (!new_name.empty() && new_name != link_node->link.name) {
                bool duplicate = false;
                robot_->forEveryLink([&](const urdf::LinkNodePtr& l) {
                    if (l != link_node && l->link.name == new_name) duplicate = true;
                });
                if (!duplicate) {
                    command_buffer_.add(std::make_shared<RenameLinkCommand>(
                        robot_, link_node, link_node->link.name, new_name));
                }
            }
        }

        if (ImGui::BeginTabBar("PropertiesBar",
                               ImGuiTabBarFlags_FittingPolicyScroll |
                                   ImGuiTabBarFlags_NoCloseWithMiddleMouseButton)) {
            if (ImGui::BeginTabItem("Inertial##PropMenuInertial")) {
                menuPropertiesInertial(link_node);
                ImGui::EndTabItem();
            }

            // "+" button for adding visuals and collisions
            if (ImGui::TabItemButton("+##AddVisCol", ImGuiTabItemFlags_Trailing)) {
                ImGui::OpenPopup("AddVisColPopup");
            }
            if (ImGui::BeginPopup("AddVisColPopup")) {
                if (ImGui::MenuItem("Add Visual")) {
                    command_buffer_.add(
                        std::make_shared<CreateVisualCommand>(link_node, robot_, shader_));
                }
                if (ImGui::MenuItem("Add Collision")) {
                    command_buffer_.add(std::make_shared<AddCollisionCommand>(link_node));
                }
                ImGui::EndPopup();
            }

            int num_visuals = static_cast<int>(link_node->link.visual.size());
            for (int i = 0; i < num_visuals; ++i) {
                bool open = true;
                char name_buffer[256];
                snprintf(name_buffer, 256, "Vis %d##VisTabItem%d", i, i);
                ImGuiTabItemFlags vis_flags = (pending_tab_ == i)
                                                   ? ImGuiTabItemFlags_SetSelected : 0;
                if (ImGui::BeginTabItem(name_buffer, &open, vis_flags)) {
                    urdf::Visual &vis = link_node->link.visual[i];
                    if (vis.origin.has_value()) {
                        selected_link_origin_ = &*vis.origin;
                    }
                    menuName(vis.name, "visual");
                    menuOrigin(vis.origin);
                    menuGeometry(vis.geometry, link_node->visual_models[i]);
                    menuMaterial(vis.material_name, link_node);
                    ImGui::EndTabItem();
                }

                if (not open) {
                    command_buffer_.add(
                        std::make_shared<DeleteVisualCommand>(link_node, i, robot_));
                }
            }

            int num_collisions = static_cast<int>(link_node->link.collision.size());
            for (int i = 0; i < num_collisions; ++i) {
                bool open = true;
                char name_buffer[256];
                snprintf(name_buffer, 256, "Col %d##ColTabItem%d", i, i);
                ImGuiTabItemFlags col_flags = (pending_tab_ == num_visuals + i)
                                                   ? ImGuiTabItemFlags_SetSelected : 0;
                if (ImGui::BeginTabItem(name_buffer, &open, col_flags)) {
                    menuPropertiesCollisions(link_node, i);
                    ImGui::EndTabItem();
                }

                if (not open) {
                    command_buffer_.add(std::make_shared<DeleteCollisionCommand>(link_node, i));
                }
            }

            ImGui::EndTabBar();
            pending_tab_ = -1;
        }

        // Non-root links can be deleted or duplicated
        if (link_node->parent) {
            ImGui::Separator();
            if (ImGui::Button("Delete link")) {
                deleteSelectedJoint();
            }
            ImGui::SameLine();
            if (ImGui::Button("Duplicate link")) {
                command_buffer_.add(
                    std::make_shared<CloneSubtreeCommand>(link_node, robot_, shader_));
            }
        }
    } else if (auto joint_node = std::dynamic_pointer_cast<urdf::JointNode>(selected_node_)) {
        inputTextUndoable("Name##joint", joint_node->joint.name);

        static const char *joint_types[] = {"revolute", "continuous", "prismatic",
                                            "fixed",    "floating",   "planar"};
        int choice = joint_node->joint.type;
        urdf::Joint::Type old_type = joint_node->joint.type;
        if (ImGui::Combo("dropdown", &choice, joint_types, IM_ARRAYSIZE(joint_types),
                         urdf::Joint::kNumJointTypes)) {
            joint_node->joint.type = static_cast<urdf::Joint::Type>(choice);
            command_buffer_.add(std::make_shared<UpdatePropertyCommand<urdf::Joint::Type>>(
                joint_node->joint.type, old_type, joint_node->joint.type));
        }

        ImGui::Text("Parent link: %s", joint_node->parent->link.name.c_str());
        ImGui::Text("Child link: %s", joint_node->child->link.name.c_str());

        menuOrigin(joint_node->joint.origin);
        if (choice != urdf::Joint::kFixed) {
            menuAxis(joint_node->joint.axis);
            menuDynamics(joint_node->joint.dynamics);
            menuLimit(joint_node->joint.limit);
            menuMimic(joint_node->joint.mimic);
            menuCalibration(joint_node->joint.calibration);
        }

        ImGui::Separator();
        if (ImGui::Button("Delete joint")) {
            deleteSelectedJoint();
        }
    }
}

void App::originGui(urdf::Origin &origin) {
    if (ImGui::TreeNode("Origin")) {
        auto fk = [this]() { robot_->forwardKinematics(); };

        auto originField = [&](const char *label, Vector3 &vec) {
            Vector3 pre = vec;
            ImGui::InputFloat3(label, &vec.x);
            if (ImGui::IsItemActivated()) snapshot_origin_ = origin;
            if (ImGui::IsItemDeactivatedAfterEdit() && snapshot_origin_) {
                command_buffer_.add(std::make_shared<UpdatePropertyCommand<urdf::Origin>>(
                    origin, *snapshot_origin_, origin, fk));
                snapshot_origin_.reset();
            }
            if (pre.x != vec.x || pre.y != vec.y || pre.z != vec.z) robot_->forwardKinematics();
        };

        originField("Position", origin.xyz);
        originField("Orientation", origin.rpy);

        if (ImGui::IsItemClicked() and selected_node_) {
            selected_link_origin_ = &origin;
        }

        ImGui::TreePop();
    }
}

void App::menuName(std::optional<std::string> &name, const char *label) {
    if (name) {
        inputTextUndoable("Name", *name);
    } else {
        if (ImGui::Button("Create name")) {
            command_buffer_.add(std::make_shared<CreateNameCommand>(name));
        }
    }
}

void App::menuOrigin(std::optional<urdf::Origin> &origin) {
    if (origin) {
        originGui(*origin);
    } else {
        if (ImGui::Button("Create origin")) {
            command_buffer_.add(
                std::make_shared<CreateOriginCommand>(origin, selected_link_origin_));
        }
    }
}

void App::menuMaterial(std::optional<std::string> &material_name,
                       const urdf::LinkNodePtr &link) {
    if (!robot_) return;

    const auto &materials = robot_->getMaterials();

    // Build list: index 0 = "(None)", then one entry per material
    std::vector<std::string> names;
    names.reserve(materials.size() + 1);
    names.emplace_back("(None)");
    for (const auto &[k, _] : materials) names.push_back(k);

    int current = 0;
    if (material_name) {
        for (int i = 1; i < static_cast<int>(names.size()); ++i) {
            if (names[i] == *material_name) {
                current = i;
                break;
            }
        }
    }

    auto post = [this, link]() { robot_->updateMaterial(link); };

    if (ImGui::Combo("Material", &current,
                      [](void *data, int idx) -> const char * {
                          return (*static_cast<std::vector<std::string> *>(data))[idx].c_str();
                      },
                      &names, static_cast<int>(names.size()))) {
        auto old_val = material_name;
        if (current == 0) {
            material_name = std::nullopt;
        } else {
            material_name = names[current];
        }
        robot_->updateMaterial(link);
        command_buffer_.add(
            std::make_shared<UpdatePropertyCommand<std::optional<std::string>>>(
                material_name, old_val, material_name, post));
    }
}

void App::menuAxis(std::optional<urdf::Axis> &axis) {
    if (axis) {
        if (ImGui::TreeNode("Axis")) {
            inputFloat3Undoable("xyz", axis->xyz);
            if (ImGui::Button("Delete axis")) {
                auto old_axis = *axis;
                axis = std::nullopt;
                command_buffer_.add(
                    std::make_shared<UpdatePropertyCommand<std::optional<urdf::Axis>>>(
                        axis, old_axis, std::nullopt));
            }
            ImGui::TreePop();
        }
    } else {
        if (ImGui::Button("Create axis")) {
            command_buffer_.add(std::make_shared<CreateAxisCommand>(axis));
        }
    }
}

void App::menuDynamics(std::optional<urdf::Dynamics> &dynamics) {
    if (dynamics) {
        if (ImGui::TreeNode("Dynamics")) {
            inputFloatUndoable("Damping", dynamics->damping);
            inputFloatUndoable("Friction", dynamics->friction);
            if (ImGui::Button("Delete dynamics")) {
                auto old_dynamics = *dynamics;
                dynamics = std::nullopt;
                command_buffer_.add(
                    std::make_shared<UpdatePropertyCommand<std::optional<urdf::Dynamics>>>(
                        dynamics, old_dynamics, std::nullopt));
            }
            ImGui::TreePop();
        }
    } else {
        if (ImGui::Button("Create dynamics")) {
            command_buffer_.add(std::make_shared<CreateDynamicsCommand>(dynamics));
        }
    }
}

void App::menuLimit(std::optional<urdf::Limit> &limit) {
    if (limit) {
        if (ImGui::TreeNode("Limit")) {
            inputFloatUndoable("Lower", limit->lower);
            inputFloatUndoable("Upper", limit->upper);
            inputFloatUndoable("Effort", limit->effort);
            inputFloatUndoable("Velocity", limit->velocity);
            if (ImGui::Button("Delete limit")) {
                auto old_limit = *limit;
                limit = std::nullopt;
                command_buffer_.add(
                    std::make_shared<UpdatePropertyCommand<std::optional<urdf::Limit>>>(
                        limit, old_limit, std::nullopt));
            }
            ImGui::TreePop();
        }
    } else {
        if (ImGui::Button("Create limit")) {
            command_buffer_.add(std::make_shared<CreateLimitCommand>(limit));
        }
    }
}

void App::menuMimic(std::optional<urdf::Mimic> &mimic) {
    if (mimic) {
        if (ImGui::TreeNode("Mimic")) {
            inputTextUndoable("Joint##mimic", mimic->joint);
            inputFloatUndoable("Multiplier", mimic->multiplier);
            inputFloatUndoable("Offset", mimic->offset);
            if (ImGui::Button("Delete mimic")) {
                auto old_mimic = *mimic;
                mimic = std::nullopt;
                command_buffer_.add(
                    std::make_shared<UpdatePropertyCommand<std::optional<urdf::Mimic>>>(
                        mimic, old_mimic, std::nullopt));
            }
            ImGui::TreePop();
        }
    } else {
        if (ImGui::Button("Create mimic")) {
            command_buffer_.add(std::make_shared<CreateMimicCommand>(mimic));
        }
    }
}

void App::menuCalibration(std::optional<urdf::Calibration> &calibration) {
    if (calibration) {
        if (ImGui::TreeNode("Calibration")) {
            inputFloatUndoable("Rising", calibration->rising);
            inputFloatUndoable("Falling", calibration->falling);
            if (ImGui::Button("Delete calibration")) {
                auto old_cal = *calibration;
                calibration = std::nullopt;
                command_buffer_.add(
                    std::make_shared<UpdatePropertyCommand<std::optional<urdf::Calibration>>>(
                        calibration, old_cal, std::nullopt));
            }
            ImGui::TreePop();
        }
    } else {
        if (ImGui::Button("Create calibration")) {
            command_buffer_.add(std::make_shared<CreateCalibrationCommand>(calibration));
        }
    }
}

void App::menuGeometry(urdf::Geometry &geometry, Model &model) {
    if (ImGui::TreeNode("Geometry")) {
        if (urdf::GeometryTypePtr &type = geometry.type) {
            static const char *geom_types[] = {"Box", "Cylinder", "Sphere", "Mesh"};

            int choice = 0;
            if (std::dynamic_pointer_cast<urdf::Box>(type)) {
                choice = 0;
            } else if (std::dynamic_pointer_cast<urdf::Cylinder>(type)) {
                choice = 1;
            } else if (std::dynamic_pointer_cast<urdf::Sphere>(type)) {
                choice = 2;
            } else if (std::dynamic_pointer_cast<urdf::Mesh>(type)) {
                choice = 3;
            }

            if (ImGui::Combo("dropdown", &choice, geom_types, IM_ARRAYSIZE(geom_types), 4)) {
                switch (choice) {
                    case 0:
                        command_buffer_.add(std::make_shared<ChangeGeometryCommand>(
                            type, std::make_shared<urdf::Box>(), geometry, model, robot_));
                        break;
                    case 1:
                        command_buffer_.add(std::make_shared<ChangeGeometryCommand>(
                            type, std::make_shared<urdf::Cylinder>(), geometry, model, robot_));
                        break;
                    case 2:
                        command_buffer_.add(std::make_shared<ChangeGeometryCommand>(
                            type, std::make_shared<urdf::Sphere>(), geometry, model, robot_));
                        break;
                    case 3:
                        command_buffer_.add(std::make_shared<ChangeGeometryCommand>(
                            type, std::make_shared<urdf::Mesh>(""), geometry, model, robot_));
                        break;
                    default:
                        LOG_F(ERROR, "Invalid geometry type");
                }
            }

            switch (choice) {
                case 0:
                    if (auto box = std::dynamic_pointer_cast<urdf::Box>(type)) {
                        Vector3 pre = box->size;
                        ImGui::InputFloat3("Size", &box->size.x, "%.3f");
                        if (ImGui::IsItemActivated()) snapshot_vec3_ = pre;
                        if (ImGui::IsItemDeactivatedAfterEdit() && snapshot_vec3_) {
                            command_buffer_.add(std::make_shared<UpdateGeometryBoxCommand>(
                                box, *snapshot_vec3_, model, shader_));
                            snapshot_vec3_.reset();
                        }
                    }
                    break;
                case 1:
                    if (auto cylinder = std::dynamic_pointer_cast<urdf::Cylinder>(type)) {
                        float pre_r = cylinder->radius;
                        ImGui::InputFloat("Radius", &cylinder->radius, 0.01F, 0.1F, "%.3f");
                        if (ImGui::IsItemActivated()) {
                            snapshot_float_ = pre_r;
                        }
                        if (ImGui::IsItemDeactivatedAfterEdit() && snapshot_float_) {
                            command_buffer_.add(std::make_shared<UpdateGeometryCylinderCommand>(
                                cylinder, *snapshot_float_, cylinder->length, model, shader_));
                            snapshot_float_.reset();
                        }

                        float pre_l = cylinder->length;
                        ImGui::InputFloat("Length", &cylinder->length, 0.01F, 0.1F, "%.3f");
                        if (ImGui::IsItemActivated()) {
                            snapshot_float_ = pre_l;
                        }
                        if (ImGui::IsItemDeactivatedAfterEdit() && snapshot_float_) {
                            command_buffer_.add(std::make_shared<UpdateGeometryCylinderCommand>(
                                cylinder, cylinder->radius, *snapshot_float_, model, shader_));
                            snapshot_float_.reset();
                        }
                    }
                    break;
                case 2:
                    if (auto sphere = std::dynamic_pointer_cast<urdf::Sphere>(type)) {
                        float pre_r = sphere->radius;
                        ImGui::InputFloat("Radius", &sphere->radius, 0.01F, 0.1F, "%.3f");
                        if (ImGui::IsItemActivated()) {
                            snapshot_float_ = pre_r;
                        }
                        if (ImGui::IsItemDeactivatedAfterEdit() && snapshot_float_) {
                            command_buffer_.add(std::make_shared<UpdateGeometrySphereCommand>(
                                sphere, *snapshot_float_, model, shader_));
                            snapshot_float_.reset();
                        }
                    }
                    break;
                case 3:
                    if (auto gmesh = std::dynamic_pointer_cast<urdf::Mesh>(type)) {
                        ImGui::Text("Filename: %s", gmesh->filename.c_str());
                        ImGui::SameLine();
                        if (ImGui::Button("...")) {
                            NFD::UniquePath out_path;

                            // prepare filters for the dialog
                            nfdfilteritem_t filter_item[2] = {{"Mesh filename", "dae,stl"}};

                            // show the dialog
                            nfdresult_t result = NFD::OpenDialog(out_path, filter_item, 1);
                            if (result == NFD_OKAY) {
                                command_buffer_.add(std::make_shared<UpdateGeometryMeshCommand>(
                                    gmesh, out_path.get(), model, shader_));
                            } else if (result == NFD_CANCEL) {
                                LOG_F(INFO, "User pressed cancel.");
                            } else {
                                LOG_F(WARNING, "Warn: %s", NFD::GetError());
                            }
                        }
                        if (ImGui::InputText("Filename", &gmesh->filename,
                                             ImGuiInputTextFlags_EnterReturnsTrue)) {
                            // TODO(ramon): try to load mesh and update
                        }
                        auto regen = [&model, &gmesh, this]() {
                            MaterialMap mat_map = model.materials[0].maps[MATERIAL_MAP_DIFFUSE];
                            const Matrix t = model.transform;
                            UnloadModel(model);
                            model = gmesh->generateGeometry();
                            model.materials[0].shader = shader_;
                            model.transform = t;
                            model.materials[0].maps[MATERIAL_MAP_DIFFUSE] = mat_map;
                        };
                        inputFloat3Undoable("Scale", gmesh->scale, "%.3f", regen);
                    }
                    break;
                default:
                    LOG_F(ERROR, "Invalid geometry type");
            }
        } else {
            if (ImGui::Button("Create geometry")) {
                geometry.type = std::make_shared<urdf::Box>();
                model = geometry.type->generateGeometry();
            }
        }
        ImGui::TreePop();
    }
}

void App::inputFloatUndoable(const char *label, float &value, float step, float step_fast,
                             const char *fmt, std::function<void()> post_action) {
    float pre = value;
    ImGui::InputFloat(label, &value, step, step_fast, fmt);
    if (ImGui::IsItemActivated()) snapshot_float_ = pre;
    if (ImGui::IsItemDeactivatedAfterEdit() && snapshot_float_) {
        command_buffer_.add(std::make_shared<UpdatePropertyCommand<float>>(value, *snapshot_float_,
                                                                           value, post_action));
        snapshot_float_.reset();
    }
}

void App::inputFloat3Undoable(const char *label, Vector3 &vec, const char *fmt,
                              std::function<void()> post_action) {
    Vector3 pre = vec;
    ImGui::InputFloat3(label, &vec.x, fmt);
    if (ImGui::IsItemActivated()) snapshot_vec3_ = pre;
    if (ImGui::IsItemDeactivatedAfterEdit() && snapshot_vec3_) {
        command_buffer_.add(std::make_shared<UpdatePropertyCommand<Vector3>>(vec, *snapshot_vec3_,
                                                                             vec, post_action));
        snapshot_vec3_.reset();
    }
}

void App::inputTextUndoable(const char *label, std::string &str,
                            std::function<void()> post_action) {
    std::string pre = str;
    ImGui::InputText(label, &str);
    if (ImGui::IsItemActivated()) snapshot_string_ = pre;
    if (ImGui::IsItemDeactivatedAfterEdit() && snapshot_string_) {
        command_buffer_.add(std::make_shared<UpdatePropertyCommand<std::string>>(
            str, *snapshot_string_, str, post_action));
        snapshot_string_.reset();
    }
}

void App::inputColorEdit4Undoable(const char *label, Vector4 &color,
                                  std::function<void()> post_action) {
    float c[4] = {color.x, color.y, color.z, color.w};
    if (ImGui::ColorEdit4(label, c, ImGuiColorEditFlags_Float)) {
        color = Vector4{c[0], c[1], c[2], c[3]};
        if (post_action) post_action();
    }
    if (ImGui::IsItemActivated()) snapshot_vec4_ = color;
    if (ImGui::IsItemDeactivatedAfterEdit() && snapshot_vec4_) {
        command_buffer_.add(std::make_shared<UpdatePropertyCommand<Vector4>>(
            color, *snapshot_vec4_, color, post_action));
        snapshot_vec4_.reset();
    }
}

void App::updateLinksUsingMaterial(const std::string &material_name) {
    robot_->forEveryLink([&](const urdf::LinkNodePtr &link) {
        for (const auto& vis : link->link.visual) {
            if (vis.material_name && *vis.material_name == material_name) {
                robot_->updateMaterial(link);
                break;
            }
        }
    });
}

std::function<void()> App::materialUpdateAction(const std::string &material_name) {
    return [this, material_name]() { updateLinksUsingMaterial(material_name); };
}

void App::drawMaterialEditor() {
    if (!robot_) return;

    if (!ImGui::CollapsingHeader("Materials", ImGuiTreeNodeFlags_DefaultOpen)) return;

    auto& materials = robot_->getMutableMaterials();

    if (ImGui::Button("+ New Material")) {
        int idx = 0;
        std::string name;
        do {
            name = fmt::format("Material_{}", idx++);
        } while (materials.count(name) > 0);
        urdf::Material mat;
        mat.name = name;
        mat.rgba = Vector4{0.5f, 0.5f, 0.5f, 1.0f};
        command_buffer_.add(std::make_shared<AddMaterialCommand>(robot_, mat));
    }

    ImGui::Separator();

    // Snapshot keys to iterate safely (commands may modify the map)
    std::vector<std::string> keys;
    keys.reserve(materials.size());
    for (const auto& [k, _] : materials) keys.push_back(k);

    for (const auto& key : keys) {
        auto it = materials.find(key);
        if (it == materials.end()) continue;
        urdf::Material& mat = it->second;

        ImGui::PushID(key.c_str());

        bool want_delete = false;
        bool header_open = ImGui::CollapsingHeader("##mat", ImGuiTreeNodeFlags_AllowOverlap);

        ImGui::SameLine();
        ImGui::Text("%s", key.c_str());

        ImGui::SameLine(ImGui::GetContentRegionAvail().x - 40);
        if (ImGui::SmallButton("Delete")) {
            want_delete = true;
        }

        if (header_open) {
            // --- Name (rename)
            static char name_buf[256];
            strncpy(name_buf, mat.name.c_str(), sizeof(name_buf) - 1);
            name_buf[sizeof(name_buf) - 1] = '\0';
            if (ImGui::InputText("Name", name_buf, sizeof(name_buf),
                                 ImGuiInputTextFlags_EnterReturnsTrue)) {
                std::string new_name(name_buf);
                if (!new_name.empty() && new_name != mat.name && materials.count(new_name) == 0) {
                    command_buffer_.add(
                        std::make_shared<RenameMaterialCommand>(robot_, mat.name, new_name));
                }
            }

            // --- Color
            auto update_action = materialUpdateAction(key);
            if (mat.rgba) {
                inputColorEdit4Undoable("Color", *mat.rgba, update_action);

                if (ImGui::Button("Remove Color")) {
                    auto old_rgba = mat.rgba;
                    mat.rgba = std::nullopt;
                    command_buffer_.add(
                        std::make_shared<UpdatePropertyCommand<std::optional<Vector4>>>(
                            mat.rgba, old_rgba, std::nullopt, update_action));
                }
            } else {
                if (ImGui::Button("Add Color")) {
                    mat.rgba = Vector4{0.5f, 0.5f, 0.5f, 1.0f};
                    command_buffer_.add(
                        std::make_shared<UpdatePropertyCommand<std::optional<Vector4>>>(
                            mat.rgba, std::nullopt, mat.rgba, update_action));
                }
            }

            // --- Texture
            auto openTextureDialog = [&]() -> std::optional<std::string> {
                NFD::UniquePath out_path;
                nfdfilteritem_t filter_item[1] = {{"Image", "png,jpg,jpeg,bmp,tga"}};
                nfdresult_t result = NFD::OpenDialog(out_path, filter_item, 1);
                if (result == NFD_OKAY) return std::string(out_path.get());
                return std::nullopt;
            };

            if (mat.texture_file) {
                ImGui::Text("Texture: %s", mat.texture_file->c_str());
                ImGui::SameLine();
                if (ImGui::SmallButton("...##tex")) {
                    if (auto path = openTextureDialog()) {
                        auto old_tex = mat.texture_file;
                        mat.texture_file = *path;
                        command_buffer_.add(
                            std::make_shared<UpdatePropertyCommand<std::optional<std::string>>>(
                                mat.texture_file, old_tex, mat.texture_file));
                    }
                }
                ImGui::SameLine();
                if (ImGui::SmallButton("x##tex")) {
                    auto old_tex = mat.texture_file;
                    mat.texture_file = std::nullopt;
                    command_buffer_.add(
                        std::make_shared<UpdatePropertyCommand<std::optional<std::string>>>(
                            mat.texture_file, old_tex, std::nullopt));
                }
            } else {
                if (ImGui::Button("Add Texture")) {
                    if (auto path = openTextureDialog()) {
                        mat.texture_file = *path;
                        command_buffer_.add(
                            std::make_shared<UpdatePropertyCommand<std::optional<std::string>>>(
                                mat.texture_file, std::nullopt, mat.texture_file));
                    }
                }
            }

            // --- Used by
            std::vector<std::string> used_by;
            robot_->forEveryLink([&](const urdf::LinkNodePtr& link) {
                for (const auto& vis : link->link.visual) {
                    if (vis.material_name && *vis.material_name == key) {
                        used_by.push_back(link->link.name);
                        break;
                    }
                }
            });
            if (!used_by.empty()) {
                std::string usage = "Used by: ";
                for (size_t i = 0; i < used_by.size(); ++i) {
                    if (i > 0) usage += ", ";
                    usage += used_by[i];
                }
                ImGui::TextWrapped("%s", usage.c_str());
            }
        }

        if (want_delete) {
            command_buffer_.add(std::make_shared<DeleteMaterialCommand>(robot_, key));
        }

        ImGui::PopID();
    }
}

void App::cleanup() {
    if (scene_texture_.id != 0) {
        rlUnloadTexture(scene_texture_.texture.id);
        rlUnloadFramebuffer(scene_texture_.id);
    }
    if (depth_stencil_rbo_ != 0) {
        glDeleteRenderbuffers(1, &depth_stencil_rbo_);
    }
    UnloadShader(shader_);
    UnloadShader(outline_shader_);
    rgizmo_unload();
    rlImGuiShutdown();
    CloseWindow();
}
