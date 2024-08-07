#pragma once

#define RAYGIZMO_IMPLEMENTATION

#include <raylib.h>
#include <memory>
#include <nfd.hpp>
#include <robot.h>
#include <command.h>
#include <raygizmo.h>

class App
{
public:
    App(int argc, char* argv[]);
    ~App() = default;

    void run();

private:

    void setup();
    void update();
    void draw();
    void drawMenu();
    void drawScene();
    void cleanup();

    void drawToolbar();
    void drawRobotTree();
    void drawNodeProperties();
    void drawSideMenu();

    void originGui(urdf::Origin& origin);

    void menuName(std::optional<std::string>& name, const char *label = "");
    void menuOrigin(std::optional<urdf::Origin>& origin);
    void menuMaterial(std::optional<std::string>& material_name, const char *label = "");
    void menuGeometry(urdf::Geometry& geometry, Model& model);
    void menuAxis(std::optional<urdf::Axis>& axis);
    void menuDynamics(std::optional<urdf::Dynamics>& dynamics);
    void menuLimit(std::optional<urdf::Limit>& limit);

    void menuPropertiesInertial(urdf::LinkNodePtr link_node);
    void menuPropertiesVisual(urdf::LinkNodePtr link_node);
    void menuPropertiesCollisions(urdf::LinkNodePtr link_node, int i);

    CommandBuffer command_buffer_;

    Camera camera_;
    Shader shader_;
    std::shared_ptr<urdf::Robot> robot_;

    NFD::Guard nfdguard_;

    bool bShowGrid_;
    bool bWindowShouldClose_;
    int menubar_height_;

    urdf::TreeNodePtr hovered_node_;
    urdf::TreeNodePtr selected_node_;
    urdf::OriginRawPtr selected_link_origin_;

    RGizmo gizmo_;
};
