#include <app.h>
#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>
#include <rlights.h>

#include <imgui.h>
#include <rlImGui.h>

#include <pugixml.hpp>
#include <loguru.hpp>

#include <nfd.hpp>

#include <robot.h>

#include <array>

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
        {
            ClearBackground(RAYWHITE);
            draw();
        }
        EndDrawing();

        bWindowShouldClose_ |= WindowShouldClose();
    }

    cleanup();
}

void App::setup()
{
    const int screenWidth = 800;
    const int screenHeight = 450;
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
    camera_ = { { 0.0f, 10.0f, 10.0f }, { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }, 45.0f, 0 };
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

        if (robot_) robot_->draw();

    EndMode3D();

    rlImGuiBegin();

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
                    robot_ = std::make_shared<urdf::Robot>(parser.build_robot(outPath.get()));
                    robot_->set_shader(shader_);
                    robot_->build_geometry();
                    robot_->forward_kinematics();
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

    // end ImGui Content
    rlImGuiEnd();
}

void App::cleanup()
{
    UnloadShader(shader_);
    rlImGuiShutdown();      // Close rl gui
    CloseWindow();          // Close window and OpenGL context
}
