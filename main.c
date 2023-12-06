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

#include <string.h>

#include "raylib.h"
#include "libxml2/libxml/parser.h"
#include "libxml2/libxml/tree.h"

typedef struct UOrigin {
    Vector3 xyz;
    Vector3 rpy;
} UOrigin;

typedef struct UInertia {
    float ixx;
    float ixy;
    float ixz;
    float iyy;
    float iyz;
    float izz;
} UInertia;

typedef struct UMaterial {
    char *name;
    Vector4 *color_rgba;
    char *texture_filename;
} UMaterial;

typedef struct UVisual {
    UOrigin origin;
    char *material_name;
    // TODO: geometry
} UVisual;

typedef struct UCollision {
    UOrigin origin;
    // TODO: geometry
} UCollision;

typedef struct UInertial {
    UOrigin origin;
    float mass;
    UInertia inertia;
} UInertial;

typedef struct ULimit {
    float effort;
    float lower;
    float upper;
    float velocity;
} ULimit;

typedef struct ULink {
    char *link_name;
    UInertial inertial;
    UVisual *visuals;
} ULink;

typedef struct UJoint {
    char* joint_name;
    ULink* parent;
    ULink* child;

    UOrigin origin;
    Vector3 axis;

    ULimit limit;
} UJoint;

void deleteVisual(UVisual *visual) {

}

void deleteVisuals(UVisual **visuals) {

}

ULink* createLink() {
    ULink *link = (ULink *)malloc(sizeof(ULink));
    if (link == NULL) {
        return NULL;
    }


    return link;
}

void deleteLink(ULink **link) {
    deleteVisuals((*link)->visuals);
    free((*link)->link_name);
    free(*link);
    *link = NULL;
}

#ifdef LIBXML_TREE_ENABLED
void read_urdf(void)
{
    xmlDoc *doc = xmlReadFile("./resources/simple.urdf", NULL, 0);
    if (doc == NULL) {
        fprintf(stderr, "XML file could not be read!\n");
        return;
    }

    xmlNode *root_element = xmlDocGetRootElement(doc);
    if (root_element == NULL) {
        fprintf(stderr, "Cannot find the root element in the XML file!\n");
        return;
    }
    fprintf(stdout, "root_element->name: %s", root_element->name);

    // Find all the root links
    // For the time being, only support one tree
    for (xmlNode* it = root_element->children; it; it = it->next) {
        if (it->type == XML_ELEMENT_NODE) {
            if (strcmp((const char *)it->name, "link") == 0) {

            } else if (strcmp((const char *)it->name, "joint") == 0) {

            } else if (strcmp((const char *)it->name, "material") == 0) {

            } else {
                fprintf(stdout, "Unexpected node under <robot>: %s\n", it->name);
            }
        }
    }

    // print_element_names(root_element);

    xmlFreeDoc(doc);
    xmlCleanupParser();
}
#else
void read_urdf(void)
{
    fprintf(stderr, "XML Tree functionality not enabled!\n");
}
#endif

//------------------------------------------------------------------------------------
// Program main entry point
//------------------------------------------------------------------------------------
int main(void)
{
    // Initialization
    //--------------------------------------------------------------------------------------
    const int screenWidth = 800;
    const int screenHeight = 450;

    InitWindow(screenWidth, screenHeight, "URDF Editor");

    // Define the camera to look into our 3d world
    Camera camera = { { 0.0f, 10.0f, 10.0f }, { 0.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, 45.0f, 0 };
    bool bOrbiting = false;

    LIBXML_TEST_VERSION

    read_urdf();


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
