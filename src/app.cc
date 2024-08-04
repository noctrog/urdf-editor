#include <app.h>

#include <functional>
#include <array>

#include <raygizmo.h>

#include <rcamera.h>
#include <raymath.h>
#include <rlgl.h>
#include <rlights.h>

#include <imgui.h>
#include <misc/cpp/imgui_stdlib.h>

#include <rlImGui.h>

#include <pugixml.hpp>
#include <loguru.hpp>
#include <fmt/format.h>

#include <nfd.hpp>

#include <robot.h>

void drawGridZUp(int slices, float spacing)
{
    int half_slices = slices/2;

    rlBegin(RL_LINES);
        for (int i = -half_slices; i <= half_slices; i++)
        {
            if (i == 0)
            {
                rlColor3f(0.5F, 0.5F, 0.5F);
                rlColor3f(0.5F, 0.5F, 0.5F);
                rlColor3f(0.5F, 0.5F, 0.5F);
                rlColor3f(0.5F, 0.5F, 0.5F);
            }
            else
            {
                rlColor3f(0.75F, 0.75F, 0.75F);
                rlColor3f(0.75F, 0.75F, 0.75F);
                rlColor3f(0.75F, 0.75F, 0.75F);
                rlColor3f(0.75F, 0.75F, 0.75F);
            }

            rlVertex3f(static_cast<float>(i)*spacing, static_cast<float>(-half_slices)*spacing, 0.0F);
            rlVertex3f(static_cast<float>(i)*spacing, static_cast<float>(half_slices)*spacing, 0.0F);

            rlVertex3f(static_cast<float>(-half_slices)*spacing, static_cast<float>(i)*spacing, 0.0F);
            rlVertex3f(static_cast<float>(half_slices)*spacing, static_cast<float>(i)*spacing, 0.0F);
        }
    rlEnd();
}

#define CAMERA_ROT_SPEED 0.003f
#define CAMERA_MOVE_SPEED 0.01f
#define CAMERA_ZOOM_SPEED 1.0f

static void updateCamera(Camera3D *camera) {
    bool is_mmb_down = IsMouseButtonDown(MOUSE_BUTTON_MIDDLE);
    bool is_shift_down = IsKeyDown(KEY_LEFT_SHIFT);
    Vector2 mouse_delta = GetMouseDelta();

    if (is_mmb_down && is_shift_down) {
        CameraMoveRight(camera, -CAMERA_MOVE_SPEED * mouse_delta.x, true);

        Vector3 right = GetCameraRight(camera);
        Vector3 up = Vector3CrossProduct(
            Vector3Subtract(camera->position, camera->target), right
        );
        up = Vector3Scale(
            Vector3Normalize(up), -CAMERA_MOVE_SPEED * mouse_delta.y
        );
        camera->position = Vector3Add(camera->position, up);
        camera->target = Vector3Add(camera->target, up);
    } else if (is_mmb_down) {
        CameraYaw(camera, -CAMERA_ROT_SPEED * mouse_delta.x, true);
        CameraPitch(
            camera, -CAMERA_ROT_SPEED * mouse_delta.y, true, true, false
        );
    }

    CameraMoveToTarget(camera, -GetMouseWheelMove() * CAMERA_ZOOM_SPEED);
}


App::App(int argc, char* argv[])
    : bShowGrid_(true), selected_link_origin_{nullptr}
{
    loguru::init(argc, argv);
}

void App::run()
{
    setup();

    while (not bWindowShouldClose_)
    {
        update();
        draw();
        bWindowShouldClose_ |= WindowShouldClose();
    }

    cleanup();
}

void App::setup()
{
    const int screen_width = 1200;
    const int screen_height = 800;
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE | FLAG_WINDOW_HIGHDPI);
    InitWindow(screen_width, screen_height, "URDF Editor");

    shader_ = LoadShader("./resources/shaders/lighting.vs", "./resources/shaders/lighting.fs");
    shader_.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(shader_, "viewPos");
    shader_.locs[SHADER_LOC_COLOR_DIFFUSE] = GetShaderLocation(shader_, "colDiffuse");
    shader_.locs[SHADER_LOC_COLOR_AMBIENT] = GetShaderLocation(shader_, "ambient");

    SetShaderValue(shader_,
                   shader_.locs[SHADER_LOC_COLOR_AMBIENT],
                   std::array<float, 4>({3.0F, 3.0F, 3.0F, 1.0F}).data(),
                   SHADER_UNIFORM_VEC4);

    // Create lights
    std::array<Light, MAX_LIGHTS> lights{};
    lights[0] = CreateLight(LIGHT_DIRECTIONAL, Vector3{ 1, 1, 1 }, Vector3Zero(), WHITE, shader_);

    // Define the camera to look into our 3d world
    camera_ = { { 0.0F, 1.5F, 2.5F }, { 0.0F, 0.0F, 0.0F }, { 0.0F, 0.0F, 1.0F }, 45.0F, 0 };
    bWindowShouldClose_ = false;

    gizmo_ = rgizmo_create();

    SetTargetFPS(120);
    rlImGuiSetup(true);
}

void App::update()
{
    command_buffer_.execute();

    // Update
    ImGuiIO& io = ImGui::GetIO();
    if (not io.WantCaptureMouse) {
        updateCamera(&camera_);
    }

    //----------------------------------------------------------------------------------

    // Input
    if (IsKeyDown(KEY_LEFT_CONTROL) && IsKeyPressed(KEY_Z)) {
        command_buffer_.undo();
    }
    if (IsKeyDown(KEY_LEFT_CONTROL) && IsKeyPressed(KEY_Y)) {
        command_buffer_.redo();
    }
}

void App::drawMenu()
{
    rlImGuiBegin();

    drawToolbar();
    drawSideMenu();

    rlImGuiEnd();
}


void App::drawScene()
{
    ClearBackground(LIGHTGRAY);

    // TODO(ramon): links can't be selected. Select instead visual, collision,
    // inertial and modify it's origin, if it exists. const auto selected_link =
    const auto selected_link = std::dynamic_pointer_cast<urdf::LinkNode>(selected_node_);
    if (selected_link and selected_link_origin_) {
        // Create matrix w_T_o
        const Matrix& w_t_l = selected_link->w_T_l;
        Matrix l_t_o = selected_link_origin_->toMatrix();
        Matrix w_t_o = MatrixMultiply(l_t_o, w_t_l);

        const Vector3 position {urdf::PosFromMatrix(w_t_o)};
        if (rgizmo_update(&gizmo_, camera_, position)) {
            // Update matrix with gizmo
            w_t_o = MatrixMultiply(w_t_o, rgizmo_get_transform(gizmo_, position));

            // Get matrix l_t_o
            l_t_o = MatrixMultiply(w_t_o, MatrixInvert(w_t_l));

            // Save new origin
            selected_link_origin_->xyz.x = l_t_o.m12;
            selected_link_origin_->xyz.y = l_t_o.m13;
            selected_link_origin_->xyz.z = l_t_o.m14;
            selected_link_origin_->rpy = urdf::MatrixToXYZ(l_t_o);

            // Update the robot
            robot_->forwardKinematics();
        }
    }

    const auto selected_joint = std::dynamic_pointer_cast<urdf::JointNode>(selected_node_);
    if (selected_joint and selected_joint->joint.origin) {
        urdf::Origin& joint_origin = *selected_joint->joint.origin;

        // Create matrix w_T_j
        const Matrix& w_t_p = selected_joint->parent->w_T_l;
        Matrix p_t_j = joint_origin.toMatrix();
        Matrix w_t_j = MatrixMultiply(p_t_j, w_t_p);

        const Vector3 position {urdf::PosFromMatrix(w_t_j)};
        if (rgizmo_update(&gizmo_, camera_, position)) {
            // Update matrix with gizmo
            w_t_j = MatrixMultiply(w_t_j, rgizmo_get_transform(gizmo_, position));

            // Get matrix p_T_j
            p_t_j = MatrixMultiply(w_t_j, MatrixInvert(w_t_p));

            // Save new origin {xyz, rpy}
            joint_origin.xyz.x = p_t_j.m12;
            joint_origin.xyz.y = p_t_j.m13;
            joint_origin.xyz.z = p_t_j.m14;
            joint_origin.rpy = urdf::MatrixToXYZ(p_t_j);

            // Update the robot
            robot_->forwardKinematics();
        }
    }

    BeginMode3D(camera_);

    if (bShowGrid_) drawGridZUp(10, 1.0F);

    if (robot_) {
        const auto hovered_node = std::dynamic_pointer_cast<urdf::LinkNode>(hovered_node_);
        robot_->draw(hovered_node);
    }

    if (selected_link and selected_link_origin_) {
        const Matrix& w_t_l = selected_link->w_T_l;
        Matrix l_t_o = selected_link_origin_->toMatrix();
        Matrix w_t_o = MatrixMultiply(l_t_o, w_t_l);

        const Vector3 position {urdf::PosFromMatrix(w_t_o)};
        rgizmo_draw(gizmo_, camera_, position);
    }

    if (selected_joint and selected_joint->joint.origin) {
        const Matrix& w_t_p = selected_joint->parent->w_T_l;
        const Matrix p_t_j = selected_joint->joint.origin->toMatrix();
        const Matrix w_t_j = MatrixMultiply(p_t_j, w_t_p);

        const Vector3 position {urdf::PosFromMatrix(w_t_j)};
        rgizmo_draw(gizmo_, camera_, position);
    }

    EndMode3D();
}

void App::draw()
{
    BeginDrawing();
    drawScene(); // Draw to render texture
    drawMenu();  // Draw to screen
    EndDrawing();
}

void App::drawToolbar()
{
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            // File menu items go here
            if (ImGui::MenuItem("Open", "Ctrl+O")) {
                NFD::UniquePath out_path;

                // prepare filters for the dialog
                nfdfilteritem_t filter_item[2] = {{"URDF file", "urdf,xml"}};

                // show the dialog
                nfdresult_t result = NFD::OpenDialog(out_path, filter_item, 1);
                if (result == NFD_OKAY) {
                    command_buffer_.add(std::make_shared<LoadRobotCommand>(out_path.get(), robot_, shader_));
                } else if (result == NFD_CANCEL) {
                    LOG_F(INFO, "User pressed cancel.");
                } else {
                    LOG_F(ERROR, "Error: %s", NFD::GetError());
                }
            }

            if (ImGui::MenuItem("Save", "Ctrl+S", false, static_cast<bool>(robot_))) {
                NFD::UniquePath out_path;
                nfdfilteritem_t filter_item[2] = {{"URDF file", "urdf"}};
                nfdresult_t result = NFD::SaveDialog(out_path, filter_item, 1);
                if (result == NFD_OKAY) {
                    LOG_F(INFO, "Success! %s", out_path.get());
                    exportRobot(*robot_, out_path.get());
                } else if (result == NFD_CANCEL) {
                    LOG_F(INFO, "User pressed cancel.");
                } else {
                    LOG_F(ERROR, "Error: %s", NFD::GetError());
                }
            }
            if (ImGui::MenuItem("Exit", "Alt+F4")) {
                bWindowShouldClose_ = true;
            }

            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Edit")) {
            if (ImGui::MenuItem("Undo", "Ctrl+Z", false, command_buffer_.canUndo())) {
                command_buffer_.undo();
            }
            if (ImGui::MenuItem("Redo", "Ctrl+Y", false, command_buffer_.canRedo())) {
                command_buffer_.redo();
            }
            ImGui::Separator();
            if (ImGui::MenuItem("New Robot", "Ctrl+N")) {
                command_buffer_.add(std::make_shared<CreateRobotCommand>(robot_, shader_));
            }
            auto link_node = std::dynamic_pointer_cast<urdf::LinkNode>(selected_node_);
            if (ImGui::MenuItem("Create Joint", "Ctrl+J", false, link_node != nullptr)) {
                command_buffer_.add(std::make_shared<CreateJointCommand>("New Joint", link_node, robot_));
            }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("View")) {
            ImGui::MenuItem("Show Grid", nullptr, &bShowGrid_);
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

void App::drawRobotTree()
{
    ImGuiTableFlags table_flags = ImGuiTableFlags_BordersV | ImGuiTableFlags_BordersOuterH |
        ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable | ImGuiTableFlags_Hideable |
        ImGuiTableFlags_NoBordersInBody | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY;

    if (ImGui::BeginTable("robot table", 2, table_flags, ImVec2(0, 300))) {
        ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_NoHide | ImGuiTableColumnFlags_WidthStretch, 0.9F);
        ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_NoHide | ImGuiTableColumnFlags_WidthFixed, 0.1F);
        ImGui::TableHeadersRow();

        if (not robot_) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("--");
            ImGui::TableNextColumn();
            ImGui::Text("--");
        } else {
            ImGuiTreeNodeFlags tree_flags = ImGuiTreeNodeFlags_DefaultOpen |
                ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_SpanAllColumns;

            urdf::LinkNodePtr current_link = robot_->getRoot();

            std::function<void (const urdf::LinkNodePtr&)> recursion = [&](auto link){
                if (not link) return;

                // Unique ID for drag and drop
                void* node_id = static_cast<void*>(link.get());

                ImGui::TableNextRow();
                ImGui::TableNextColumn();

                bool open = ImGui::TreeNodeEx(link->link.name.c_str(),
                                      tree_flags | (link->children.empty() ? ImGuiTreeNodeFlags_Leaf : 0)
                                      | (link.get() == selected_node_.get() ? ImGuiTreeNodeFlags_Selected : 0));

                if (ImGui::IsItemHovered()) {
                    hovered_node_ = link;
                }

                if (ImGui::IsItemClicked() and selected_node_ != link) {
                    selected_node_ = link;
                    selected_link_origin_ = nullptr;
                }

                // Drag source for link node
                if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_None)) {
                    ImGui::SetDragDropPayload("LINK_NODE", &node_id, sizeof(void*));
                    ImGui::Text("Moving %s", link->link.name.c_str());
                    ImGui::EndDragDropSource();
                }

                // Drop target for joint node
                if (ImGui::BeginDragDropTarget()) {
                    if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("JOINT_NODE")) {
                        urdf::JointNodePtr dropped_joint_node = *static_cast<urdf::JointNodePtr*>(payload->Data);
                        LOG_F(INFO, "Dropped joint %s onto link %s", dropped_joint_node->joint.name.c_str(), link->link.name.c_str());
                        command_buffer_.add(std::make_shared<JointChangeParentCommand>(dropped_joint_node, link, robot_));
                    }
                    ImGui::EndDragDropTarget();
                }

                ImGui::TableNextColumn();
                ImGui::Text("Link");

                if (open) {
                    for (urdf::JointNodePtr& joint : link->children) {
                        ImGui::TableNextRow();
                        ImGui::TableNextColumn();

                        bool open = ImGui::TreeNodeEx(joint->joint.name.c_str(),
                                                      tree_flags | (joint.get() == selected_node_.get() ? ImGuiTreeNodeFlags_Selected : 0));

                        if (ImGui::IsItemHovered()) {
                            hovered_node_ = nullptr;
                        }

                        if (ImGui::IsItemClicked()) {
                            selected_node_ = joint;
                        }

                        // Drag source for joint node
                        if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_None)) {
                            ImGui::SetDragDropPayload("JOINT_NODE", &joint, sizeof(urdf::JointNodePtr*));
                            ImGui::Text("Moving %s", joint->joint.name.c_str());
                            ImGui::EndDragDropSource();
                        }

                        // Drop target for link node
                        if (ImGui::BeginDragDropTarget()) {
                            if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("LINK_NODE")) {
                                // void* dropped_link_node_id = *static_cast<void**>(payload->Data);
                                // Handle the drop for a link node (update parent/child relationships)
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
        }

        ImGui::EndTable();
    }
}

void App::drawSideMenu()
{
    ImGui::SetNextWindowPos(ImVec2(0, ImGui::GetFrameHeight()), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(350, static_cast<float>(GetScreenHeight()) - ImGui::GetFrameHeight()), ImGuiCond_Always);

    ImGui::Begin("Robot Tree", nullptr, ImGuiWindowFlags_NoMove |
                 ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar);

    drawRobotTree();
    ImGui::Separator();
    drawNodeProperties();

    ImGui::End();
}

void App::menuPropertiesInertial(urdf::LinkNodePtr link_node)
{
    if (auto& inertial = link_node->link.inertial) {
        selected_link_origin_ = &inertial->origin;

        ImGui::InputFloat("Mass", &inertial->mass);

        originGui(inertial->origin);

        if (ImGui::TreeNode("Inertia")) {
            ImGui::InputFloat("ixx", &inertial->inertia.ixx);
            ImGui::InputFloat("iyy", &inertial->inertia.iyy);
            ImGui::InputFloat("izz", &inertial->inertia.izz);
            ImGui::InputFloat("ixy", &inertial->inertia.ixy);
            ImGui::InputFloat("ixz", &inertial->inertia.ixz);
            ImGui::InputFloat("iyz", &inertial->inertia.iyz);
            ImGui::TreePop();
        }
    } else {
        selected_link_origin_ = nullptr;
        if (ImGui::Button("Create inertial component")) {
            command_buffer_.add(std::make_shared<CreateInertialCommand>(link_node, selected_link_origin_));
        }
    }
}

void App::menuPropertiesVisual(urdf::LinkNodePtr link_node)
{
    auto& visual = link_node->link.visual;
    if (visual.has_value() and visual->origin.has_value()) {
        selected_link_origin_ = &*visual->origin;
        menuName(visual->name, "visual");
        menuOrigin(visual->origin);
        menuGeometry(visual->geometry, link_node->visual_model);
        menuMaterial(visual->material_name);
        ImGui::Separator();
        if (ImGui::Button("Delete visual component")) {
            command_buffer_.add(std::make_shared<DeleteVisualCommand>(link_node, robot_));
        }
    } else {
        selected_link_origin_ = nullptr;
        if (ImGui::Button("Create visual component")) {
            command_buffer_.add(std::make_shared<CreateVisualCommand>(link_node, robot_, shader_));
        }
    }
}

void App::menuPropertiesCollisions(urdf::LinkNodePtr link_node, int i)
{
    urdf::Collision& col = link_node->link.collision[i];

    selected_link_origin_ = col.origin.has_value() ? &col.origin.value() : nullptr;

    menuName(col.name, "collision");
    menuOrigin(col.origin);
    menuGeometry(col.geometry, link_node->collision_models[i]);
}

void App::drawNodeProperties()
{
    if (not selected_node_ or not robot_) return;

    if (auto link_node = std::dynamic_pointer_cast<urdf::LinkNode>(selected_node_)) {
        if (ImGui::BeginTabBar("PropertiesBar", ImGuiTabBarFlags_FittingPolicyScroll | ImGuiTabBarFlags_NoCloseWithMiddleMouseButton)) {
            if (ImGui::BeginTabItem("Inertial##PropMenuInertial")) {
                menuPropertiesInertial(link_node);
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("Visual##PropMenuVisual")) {
                menuPropertiesVisual(link_node);
                ImGui::EndTabItem();
            }

            if (ImGui::TabItemButton("+", ImGuiTabItemFlags_Trailing)) {
                command_buffer_.add(std::make_shared<AddCollisionCommand>(link_node));
            }

            // TODO(ramon) fix bug: when changing name, the tab looses focus
            for (int i = 0; i < link_node->link.collision.size(); ++i) {
                bool open = true;
                const urdf::Collision& col = link_node->link.collision[i];
                char name_buffer[256];
                if (col.name.has_value()) {
                    snprintf(name_buffer, 256, "%s##ColTabItem%d", col.name.value().c_str(), i);
                } else {
                    snprintf(name_buffer, 256, "Col %d##ColTabItem%d", i, i);
                }
                if (ImGui::BeginTabItem(name_buffer, &open)) {
                    menuPropertiesCollisions(link_node, i);
                    ImGui::EndTabItem();
                }

                if (not open) {
                    command_buffer_.add(std::make_shared<DeleteCollisionCommand>(link_node, i));
                }
            }

            ImGui::EndTabBar();
        }
    } else if (auto joint_node = std::dynamic_pointer_cast<urdf::JointNode>(selected_node_)) {
        ImGui::InputText("Name##joint", &joint_node->joint.name, ImGuiInputTextFlags_None);

        static const char* joint_types[] = {"revolute", "continuous", "prismatic", "fixed", "floating", "planar"};
        int choice = joint_node->joint.type;
        if (ImGui::Combo("dropdown", &choice, joint_types, IM_ARRAYSIZE(joint_types), urdf::Joint::kNumJointTypes)) {
            joint_node->joint.type = static_cast<urdf::Joint::Type>(choice);
        }

        ImGui::Text("Parent link: %s", joint_node->parent->link.name.c_str());
        ImGui::Text("Child link: %s", joint_node->child->link.name.c_str());

        menuOrigin(joint_node->joint.origin);
        if (choice != urdf::Joint::kFixed) {
            menuAxis(joint_node->joint.axis);
            menuDynamics(joint_node->joint.dynamics);
            menuLimit(joint_node->joint.limit);
        }
    }
}

void App::originGui(urdf::Origin& origin)
{
    urdf::Origin old_origin = origin;

    if (ImGui::TreeNode("Origin")) {
        if (ImGui::InputFloat3("Position", &origin.xyz.x)) {
            command_buffer_.add(std::make_shared<UpdateOriginCommand>(old_origin, origin, origin, robot_));
        }

        if (ImGui::InputFloat3("Orientation", &origin.rpy.x)) {
            command_buffer_.add(std::make_shared<UpdateOriginCommand>(old_origin, origin, origin, robot_));
        }

        if (ImGui::IsItemClicked() and selected_node_) {
            selected_link_origin_ = &origin;
        }

        ImGui::TreePop();
    }
}

void App::menuName(std::optional<std::string>& name, const char *label)
{
    if (name) {
        ImGui::InputText("Name", &(*name));
    } else {
        if (ImGui::Button("Create name")) {
            command_buffer_.add(std::make_shared<CreateNameCommand>(name));
        }
    }
}

void App::menuOrigin(std::optional<urdf::Origin>& origin)
{
    if (origin) {
        originGui(*origin);
    } else {
        if (ImGui::Button("Create origin")) {
            command_buffer_.add(std::make_shared<CreateOriginCommand>(origin, selected_link_origin_));
        }
    }
}

void App::menuMaterial(std::optional<std::string>& material_name, const char *label)
{
    if (material_name) {
        ImGui::InputText("Material name", &(*material_name));
    } else {
        if (ImGui::Button("Create material")) {
            command_buffer_.add(std::make_shared<CreateNameCommand>(material_name));
        }
    }
}

void App::menuAxis(std::optional<urdf::Axis>& axis)
{
    if (axis) {
        if (ImGui::TreeNode("Axis")) {
            ImGui::InputFloat3("xyz", &axis->xyz.x);
            if (ImGui::Button("Delete axis")) {
                axis = std::nullopt;
            }
            ImGui::TreePop();
        }
    } else {
        if (ImGui::Button("Create axis")) {
            command_buffer_.add(std::make_shared<CreateAxisCommand>(axis));
        }
    }
}

void App::menuDynamics(std::optional<urdf::Dynamics>& dynamics)
{
    if (dynamics) {
        if (ImGui::TreeNode("Dynamics")) {
            ImGui::InputFloat("Damping", &dynamics->damping);
            ImGui::InputFloat("Friction", &dynamics->friction);
            if (ImGui::Button("Delete dynamics")) {
                dynamics = std::nullopt;
            }
            ImGui::TreePop();
        }
    } else {
        if (ImGui::Button("Create dynamics")) {
            command_buffer_.add(std::make_shared<CreateDynamicsCommand>(dynamics));
        }
    }
}

void App::menuLimit(std::optional<urdf::Limit>& limit)
{
    if (limit) {
        if (ImGui::TreeNode("Limit")) {
            ImGui::InputFloat("Lower", &limit->lower);
            ImGui::InputFloat("Upper", &limit->upper);
            ImGui::InputFloat("Effort", &limit->effort);
            ImGui::InputFloat("Velocity", &limit->velocity);
            if (ImGui::Button("Delete limit")) {
                limit = std::nullopt;
            }
            ImGui::TreePop();
        }
    } else {
        if (ImGui::Button("Create limit")) {
            command_buffer_.add(std::make_shared<CreateLimitCommand>(limit));
        }
    }
}

void App::menuGeometry(urdf::Geometry& geometry, Model& model)
{
    if (ImGui::TreeNode("Geometry")) {
        if (urdf::GeometryTypePtr& type = geometry.type) {
            static const char* geom_types[] = {"Box", "Cylinder", "Sphere", "Mesh"};

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
                            type, std::make_shared<urdf::Box>(), geometry, model));
                        break;
                    case 1:
                        command_buffer_.add(std::make_shared<ChangeGeometryCommand>(
                            type, std::make_shared<urdf::Cylinder>(), geometry, model));
                        break;
                    case 2:
                        command_buffer_.add(std::make_shared<ChangeGeometryCommand>(
                            type, std::make_shared<urdf::Sphere>(), geometry, model));
                        break;
                    case 3:
                        command_buffer_.add(std::make_shared<ChangeGeometryCommand>(
                            type, std::make_shared<urdf::Mesh>(""), geometry, model));
                        break;
                    default:
                        LOG_F(ERROR, "Invalid geometry type");
                }
            }

            switch (choice) {
                case 0:
                    if (auto box = std::dynamic_pointer_cast<urdf::Box>(type)) {
                        static Vector3 old_size = box->size;
                        if (ImGui::InputFloat3("Size", &box->size.x, "%.3f")) {
                            command_buffer_.add(std::make_shared<UpdateGeometryBoxCommand>(box, old_size, model, shader_));
                        }
                    }
                    break;
                case 1:
                    if (auto cylinder = std::dynamic_pointer_cast<urdf::Cylinder>(type)) {
                        float old_radius = cylinder->radius;
                        float old_length = cylinder->length;
                        ImGui::InputFloat("Radius", &cylinder->radius, 0.01F, 0.1F, "%.3f");
                        ImGui::InputFloat("Length", &cylinder->length, 0.01F, 0.1F, "%.3f");

                        if (old_radius != cylinder->radius or old_length != cylinder->length) {
                            command_buffer_.add(std::make_shared<UpdateGeometryCylinderCommand>(
                                cylinder, cylinder->radius, cylinder->length, model, shader_));
                        }
                    }
                    break;
                case 2:
                    if (auto sphere = std::dynamic_pointer_cast<urdf::Sphere>(type)) {
                        float old_radius = sphere->radius;
                        ImGui::InputFloat("Radius", &sphere->radius, 0.01F, 0.1F, "%.3f");
                        if (old_radius != sphere->radius)
                        command_buffer_.add(std::make_shared<UpdateGeometrySphereCommand>(
                            sphere, sphere->radius, model, shader_));
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
                        if (ImGui::InputText("Filename", &gmesh->filename, ImGuiInputTextFlags_EnterReturnsTrue)) {
                            // TODO(ramon): try to load mesh and update
                        }
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

void App::cleanup()
{
    UnloadShader(shader_);
    rgizmo_unload();
    rlImGuiShutdown();      // Close rl gui
    CloseWindow();          // Close window and OpenGL context
}
