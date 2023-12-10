/*******************************************************************************************
*
*   raylib [models] example - Detect basic 3d collisions (box vs sphere vs box)
*
*   Example originally created with raylib 1.3, last time updated with raylib 3.5
*
*   Example licensed under an unmodified zlib/libpng license, which is an OSI-certified,
*   BSD-like license that allows static linking with closed source software
*
*   Copyright (c) 2015-2023 Ramon Santamaria (@raysan5)
*
********************************************************************************************/

#include <raylib.h>
#include <pugixml.hpp>
#include <loguru.hpp>

#include <urdf_parser.h>

//------------------------------------------------------------------------------------
// Program main entry point
//------------------------------------------------------------------------------------
int main(int argc, char* argv[])
{
    loguru::init(argc, argv);

    // Initialization
    //--------------------------------------------------------------------------------------
    const char *urdf_file = "./resources/simple.urdf";
    urdf::Parser urdf_parser(urdf_file);

    const int screenWidth = 800;
    const int screenHeight = 450;

    InitWindow(screenWidth, screenHeight, "URDF Editor");

    // Define the camera to look into our 3d world
    Camera camera = { { 0.0f, 10.0f, 10.0f }, { 0.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, 45.0f, 0 };
    bool bOrbiting = false;

    SetTargetFPS(60);               // Set our game to run at 60 frames-per-second
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

                DrawGrid(10, 1.0f);        // Draw a grid

            EndMode3D();

            DrawFPS(10, 10);

        EndDrawing();
        //----------------------------------------------------------------------------------
    }

    // De-Initialization
    //--------------------------------------------------------------------------------------
    CloseWindow();        // Close window and OpenGL context
    //--------------------------------------------------------------------------------------

    return 0;
}
