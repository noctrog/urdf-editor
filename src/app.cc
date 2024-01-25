#include <app.h>

#include <functional>
#include <array>

#include <raylib.h>
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

void DrawGridZUp(int slices, float spacing)
{
    int halfSlices = slices/2;

    rlBegin(RL_LINES);
        for (int i = -halfSlices; i <= halfSlices; i++)
        {
            if (i == 0)
            {
                rlColor3f(0.5f, 0.5f, 0.5f);
                rlColor3f(0.5f, 0.5f, 0.5f);
                rlColor3f(0.5f, 0.5f, 0.5f);
                rlColor3f(0.5f, 0.5f, 0.5f);
            }
            else
            {
                rlColor3f(0.75f, 0.75f, 0.75f);
                rlColor3f(0.75f, 0.75f, 0.75f);
                rlColor3f(0.75f, 0.75f, 0.75f);
                rlColor3f(0.75f, 0.75f, 0.75f);
            }

            rlVertex3f((float)i*spacing, (float)-halfSlices*spacing, 0.0f);
            rlVertex3f((float)i*spacing, (float)halfSlices*spacing, 0.0f);

            rlVertex3f((float)-halfSlices*spacing, (float)i*spacing, 0.0f);
            rlVertex3f((float)halfSlices*spacing, (float)i*spacing, 0.0f);
        }
    rlEnd();
}


App::App(int argc, char* argv[])
{
    loguru::init(argc, argv);
}

App::~App()
{

}

void App::run()
{
    setup();

    while (not bWindowShouldClose_)
    {
        update();
        BeginDrawing();
            ClearBackground(LIGHTGRAY);
            draw();
        EndDrawing();

        bWindowShouldClose_ |= WindowShouldClose();
    }

    cleanup();
}

void App::setup()
{
    const int screenWidth = 1200;
    const int screenHeight = 800;
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(screenWidth, screenHeight, "URDF Editor");

    shader_ = LoadShader("./resources/shaders/lighting.vs", "./resources/shaders/lighting.fs");
    shader_.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(shader_, "viewPos");
    shader_.locs[SHADER_LOC_COLOR_DIFFUSE] = GetShaderLocation(shader_, "colDiffuse");
    shader_.locs[SHADER_LOC_COLOR_AMBIENT] = GetShaderLocation(shader_, "ambient");

    SetShaderValue(shader_,
                   shader_.locs[SHADER_LOC_COLOR_AMBIENT],
                   std::array<float, 4>({3.0f, 3.0f, 3.0f, 1.0f}).data(),
                   SHADER_UNIFORM_VEC4);

    // Create lights
    Light lights[MAX_LIGHTS] = { };
    lights[0] = CreateLight(LIGHT_DIRECTIONAL, Vector3{ 1, 1, 1 }, Vector3Zero(), WHITE, shader_);

    // Define the camera to look into our 3d world
    camera_ = { { 0.0f, 1.5f, 2.5f }, { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }, 45.0f, 0 };
    bOrbiting_ = false;
    bWindowShouldClose_ = false;

    SetTargetFPS(120);
    rlImGuiSetup(true);
}

void App::update()
{
    // Update
    if (bOrbiting_) {
        UpdateCamera(&camera_, CAMERA_THIRD_PERSON);
    }

    //----------------------------------------------------------------------------------

    // Input
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        bOrbiting_ = true;
    } else {
        bOrbiting_ = false;
    }
}

void App::draw()
{
    BeginMode3D(camera_);

        DrawGridZUp(10, 1.0f);

        if (robot_) {
            auto hovered_node = std::dynamic_pointer_cast<urdf::LinkNode>(hovered_node_);
            robot_->draw(hovered_node);
        }

    EndMode3D();

    rlImGuiBegin();

    drawToolbar();
    drawSideMenu();

    rlImGuiEnd();
}

void App::drawToolbar()
{
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            // File menu items go here
            if (ImGui::MenuItem("Open", "Ctrl+O")) {
                NFD::UniquePath outPath;

                // prepare filters for the dialog
                nfdfilteritem_t filterItem[2] = {{"URDF file", "urdf,xml"}};

                // show the dialog
                nfdresult_t result = NFD::OpenDialog(outPath, filterItem, 1);
                if (result == NFD_OKAY) {
                    LOG_F(INFO, "Success! %s", outPath.get());
                    urdf::Parser parser;
                    robot_ = parser.build_robot(outPath.get());
                    robot_->set_shader(shader_);
                    robot_->build_geometry();
                    robot_->forward_kinematics();
                    LOG_F(INFO, "Robot loaded succesfully.");
                } else if (result == NFD_CANCEL) {
                    LOG_F(INFO, "User pressed cancel.");
                } else {
                    LOG_F(ERROR, "Error: %s", NFD::GetError());
                }
            }

            if (ImGui::MenuItem("Save", "Ctrl+S", false, static_cast<bool>(robot_))) {
                NFD::UniquePath outPath;
                nfdfilteritem_t filterItem[2] = {{"URDF file", "urdf"}};
                nfdresult_t result = NFD::SaveDialog(outPath, filterItem, 1);
                if (result == NFD_OKAY) {
                    LOG_F(INFO, "Success! %s", outPath.get());
                    urdf::Parser parser;
                    parser.export_robot(*robot_, outPath.get());
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
            // Edit menu items go here
            ImGui::MenuItem("Undo", "Ctrl+Z");
            ImGui::MenuItem("Redo", "Ctrl+Y");
            ImGui::MenuItem("Copy", "Ctrl+C");
            ImGui::MenuItem("Paste", "Ctrl+V");
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Help")) {
            // Help menu items go here
            ImGui::MenuItem("About");
            ImGui::EndMenu();
        }

        ImGui::EndMainMenuBar();
    }
}

void App::drawRobotTree()
{
    ImGuiTableFlags table_flags = ImGuiTableFlags_BordersV | ImGuiTableFlags_BordersOuterH | ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable | ImGuiTableFlags_Hideable | ImGuiTableFlags_NoBordersInBody | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY;

    if (ImGui::BeginTable("robot table", 2, table_flags, ImVec2(0, 300))) {
        ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_NoHide | ImGuiTableColumnFlags_NoResize, 0.9f);
        ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_NoHide, 0.1f);
        ImGui::TableHeadersRow();

        if (not robot_) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("--");
            ImGui::TableNextColumn();
            ImGui::Text("--");
        } else {
            ImGuiTreeNodeFlags tree_flags = ImGuiTreeNodeFlags_DefaultOpen | ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_SpanAllColumns;

            urdf::LinkNodePtr current_link = robot_->get_root();

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

                if (ImGui::IsItemClicked()) {
                    selected_node_ = link;
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
                        void* dropped_node_id = *(void**)payload->Data;
                        // TODO: Handle the drop for a joint node (update parent/child relationships)
                    }
                    ImGui::EndDragDropTarget();
                }

                ImGui::TableNextColumn();
                ImGui::Text("Link");

                if (open) {
                    for (const urdf::JointNodePtr& joint : link->children) {
                        ImGui::TableNextRow();
                        ImGui::TableNextColumn();

                        bool open = ImGui::TreeNodeEx(joint->joint.name.c_str(),
                                                      tree_flags | (joint.get() == selected_node_.get() ? ImGuiTreeNodeFlags_Selected : 0));

                        void* joint_node_id = static_cast<void*>(joint.get());

                        if (ImGui::IsItemHovered()) {
                            hovered_node_ = nullptr;
                        }

                        if (ImGui::IsItemClicked()) {
                            selected_node_ = joint;
                        }

                        // Drag source for joint node
                        if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_None)) {
                            ImGui::SetDragDropPayload("JOINT_NODE", &joint_node_id, sizeof(void*));
                            ImGui::Text("Moving %s", joint->joint.name.c_str());
                            ImGui::EndDragDropSource();
                        }

                        // Drop target for link node
                        if (ImGui::BeginDragDropTarget()) {
                            if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("LINK_NODE")) {
                                void* dropped_link_node_id = *(void**)payload->Data;
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
    ImGui::SetNextWindowSize(ImVec2(350, 1000), ImGuiCond_Always);

    ImGui::Begin("Robot Tree", nullptr, ImGuiWindowFlags_NoMove |
                 ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar);

    drawRobotTree();
    ImGui::Separator();
    drawNodeProperties();

    ImGui::End();
}

void App::drawNodeProperties(void)
{
    if (not selected_node_) return;

    if (auto link_node = std::dynamic_pointer_cast<urdf::LinkNode>(selected_node_)) {
        ImGui::InputText("Name", &link_node->link.name, ImGuiInputTextFlags_None);

        if (ImGui::CollapsingHeader("Inertial")) {
            if (auto& inertial = link_node->link.inertial) {
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
                if (ImGui::Button("Create inertial component")) {
                    link_node->link.inertial = urdf::Inertial();
                }
            }
        }
        if (ImGui::CollapsingHeader("Visual")) {
            if (auto& visual = link_node->link.visual) {
                menuName(visual->name);
                menuOrigin(visual->origin);
                menuGeometry(visual->geometry, link_node->visual_mesh, link_node->visual_model);
            } else {
                if (ImGui::Button("Create visual component")) {
                    link_node->link.visual = urdf::Visual();
                }
            }
        }
        if (ImGui::CollapsingHeader("Collision")) {
            if (link_node->link.collision.size() > 0) {
                for (size_t i = 0; i < link_node->link.collision.size(); ++i) {
                    urdf::Collision& col = link_node->link.collision[i];
                    if (ImGui::TreeNode(fmt::format("Collision {}", i).c_str())) { // name origin geometry
                        menuName(col.name);
                        menuOrigin(col.origin);
                        menuGeometry(col.geometry, link_node->collision_mesh[i], link_node->collision_models[i]);
                        if (ImGui::Button("Delete collision component")) {
                            link_node->DeleteCollision(i);
                        }
                        ImGui::TreePop();
                    }
                }
                ImGui::Separator();
                if (ImGui::Button("Add collision component")) {
                    link_node->AddCollision();
                }
            } else {
                if (ImGui::Button("Create collision component")) {
                    link_node->link.collision.clear();
                    link_node->AddCollision();
                }
            }
        }
        ImGui::Separator();
    } else if (auto joint_node = std::dynamic_pointer_cast<urdf::JointNode>(selected_node_)) {
        ImGui::Text("Joint name: %s", joint_node->joint.name.c_str());

        static const char* joint_types[] = {"revolute", "continuous", "prismatic", "fixed", "floating", "planar"};
        int choice = joint_node->joint.type;
        LOG_F(INFO, "Choice: %d", choice);
        if (ImGui::Combo("dropdown", &choice, joint_types, IM_ARRAYSIZE(joint_types), urdf::Joint::NUM_JOINT_TYPES)) {
            joint_node->joint.type = static_cast<urdf::Joint::Type>(choice);
        }

        // TODO: parent link
        // TODO: child link

        menuOrigin(joint_node->joint.origin);
        if (choice != urdf::Joint::FIXED) {
            menuAxis(joint_node->joint.axis);
            menuDynamics(joint_node->joint.dynamics);
            menuLimit(joint_node->joint.limit);
        }
    }
}

void App::originGui(urdf::Origin& origin)
{
    if (ImGui::TreeNode("Origin")) {
        float xyz[3] = {origin.xyz.x, origin.xyz.y, origin.xyz.z};
        float rpy[3] = {origin.rpy.x, origin.rpy.y, origin.rpy.z};

        ImGui::InputFloat3("Position", xyz);
        ImGui::InputFloat3("Orientation", rpy);

        origin.xyz.x = xyz[0]; origin.xyz.y = xyz[1]; origin.xyz.z = xyz[2];
        origin.rpy.x = rpy[0]; origin.rpy.y = rpy[1]; origin.rpy.z = rpy[2];

        ImGui::TreePop();
    }
}

void App::menuName(std::optional<std::string>& name)
{
    if (name) {
        ImGui::InputText("Name", &(*name));
    } else {
        if (ImGui::Button("Create name")) {
            name = std::string();
        }
    }
}

void App::menuOrigin(std::optional<urdf::Origin>& origin)
{
    if (origin) {
        originGui(*origin);
    } else {
        if (ImGui::Button("Create origin")) {
            origin = urdf::Origin();
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
            axis = urdf::Axis();
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
            dynamics = urdf::Dynamics();
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
            limit = urdf::Limit(0.0f, 1.0f, 1.0f, 1.0f);
        }
    }
}

void App::menuGeometry(urdf::Geometry& geometry, ::Mesh& mesh, Model& model)
{
    if (ImGui::TreeNode("Geometry")) {
        if (auto& type = geometry.type) {
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
                        type = std::make_shared<urdf::Box>();
                        break;
                    case 1:
                        type = std::make_shared<urdf::Cylinder>();
                        break;
                    case 2:
                        type = std::make_shared<urdf::Sphere>();
                        break;
                    case 3:
                        type = std::make_shared<urdf::Mesh>("");
                        break;
                    default:
                        LOG_F(ERROR, "Invalid geometry type");
                }

                UnloadMesh(mesh);
                mesh = type->generateGeometry();
                model.meshes[0] = mesh;
            }

            switch (choice) {
                case 0:
                    if (auto box = std::dynamic_pointer_cast<urdf::Box>(type)) {
                        if (ImGui::InputFloat3("Size", &box->size.x, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue)) {
                            // TODO update geometry
                            mesh = type->generateGeometry();
                        }
                    }
                    break;
                case 1:
                    if (auto cylinder = std::dynamic_pointer_cast<urdf::Cylinder>(type)) {
                        bool update = false;
                        if (ImGui::InputFloat("Radius", &cylinder->radius,
                            0.01f, 0.1f, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue)) {
                            update = true;
                        }
                        if (ImGui::InputFloat("Length", &cylinder->length,
                            0.01f, 0.1f, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue)) {
                            update = true;
                        }
                        if (update) {
                            // TODO update geometry
                            mesh = type->generateGeometry();
                        }
                    }
                    break;
                case 2:
                    if (auto sphere = std::dynamic_pointer_cast<urdf::Sphere>(type)) {
                        if (ImGui::InputFloat("Radius", &sphere->radius,
                            0.01f, 0.1f, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue)) {
                            // TODO update geometry
                        }
                    }
                    break;
                case 3:
                    if (auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(type)) {
                        if (ImGui::InputText("Filename", &mesh->filename, ImGuiInputTextFlags_EnterReturnsTrue)) {
                            // TODO try to load mesh and update
                        }
                    }
                    break;
                default:
                    LOG_F(ERROR, "Invalid geometry type");
            }
        } else {
            if (ImGui::Button("Create geometry")) {
                geometry.type = std::make_shared<urdf::Box>();
                mesh = geometry.type->generateGeometry();
                model.meshes[0] = mesh;
            }
        }
        ImGui::TreePop();
    }
}

void App::cleanup()
{
    UnloadShader(shader_);
    rlImGuiShutdown();      // Close rl gui
    CloseWindow();          // Close window and OpenGL context
}
