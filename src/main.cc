#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>
#include <rlights.h>

#include <pugixml.hpp>
#include <loguru.hpp>

#include <urdf_parser.h>

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

//------------------------------------------------------------------------------------
// Program main entry point
//------------------------------------------------------------------------------------
int main(int argc, char* argv[])
{
    loguru::init(argc, argv);

    // Initialization
    //--------------------------------------------------------------------------------------
    const int screenWidth = 800;
    const int screenHeight = 450;
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(screenWidth, screenHeight, "URDF Editor");

    Shader shader = LoadShader("./resources/shaders/lighting.vs", "./resources/shaders/lighting.fs");
    shader.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(shader, "viewPos");
    shader.locs[SHADER_LOC_COLOR_DIFFUSE] = GetShaderLocation(shader, "colDiffuse");
    shader.locs[SHADER_LOC_COLOR_AMBIENT] = GetShaderLocation(shader, "ambient");

    SetShaderValue(shader,
                   shader.locs[SHADER_LOC_COLOR_AMBIENT],
                   std::array<float, 4>({0.5f, 0.5f, 0.5f, 1.0f}).data(),
                   SHADER_UNIFORM_VEC4);

    // Create lights
    Light lights[MAX_LIGHTS] = { };
    lights[0] = CreateLight(LIGHT_DIRECTIONAL, Vector3{ 1, 1, 1 }, Vector3Zero(), WHITE, shader);

    const char *urdf_file = "./resources/simple.urdf";
    urdf::Parser urdf_parser(urdf_file);
    urdf::Robot robot = urdf_parser.build_robot();
    robot.set_shader(shader);

    robot.print_tree();
    robot.build_geometry();
    robot.forward_kinematics();

    // Define the camera to look into our 3d world
    Camera camera = { { 0.0f, 10.0f, 10.0f }, { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }, 45.0f, 0 };
    bool bOrbiting = false;

    SetTargetFPS(120);
    //--------------------------------------------------------------------------------------

    // Main game loop
    while (!WindowShouldClose())    // Detect window close button or ESC key
    {
        // Update
        if (bOrbiting) {
            UpdateCamera(&camera, CAMERA_THIRD_PERSON);
        }

        //----------------------------------------------------------------------------------

        // Input
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            bOrbiting = true;
        } else {
            bOrbiting = false;
        }

        //----------------------------------------------------------------------------------

        // Compute
        //----------------------------------------------------------------------------------

        //----------------------------------------------------------------------------------
        //
        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();

            ClearBackground(RAYWHITE);

            BeginMode3D(camera);

                DrawGridZUp(10, 1.0f);

                robot.draw();

            EndMode3D();

            DrawFPS(10, 10);

        EndDrawing();
        //----------------------------------------------------------------------------------
    }

    UnloadShader(shader);

    // De-Initialization
    //--------------------------------------------------------------------------------------
    CloseWindow();        // Close window and OpenGL context
    //--------------------------------------------------------------------------------------

    return 0;
}
