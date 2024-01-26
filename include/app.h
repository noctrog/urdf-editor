#pragma once

#include <raylib.h>
#include <memory>
#include <nfd.hpp>
#include <robot.h>

class App
{
public:
    App(int agrc, char* argv[]);
    ~App();

    void run();

private:

    void setup();
    void update();
    void draw();
    void draw_menu();
    void draw_scene();
    void cleanup();

    void drawToolbar();
    void drawRobotTree();
    void drawNodeProperties();
    void drawSideMenu();

    void originGui(urdf::Origin& origin);

    void menuName(std::optional<std::string>& name);
    void menuOrigin(std::optional<urdf::Origin>& origin);
    void menuGeometry(urdf::Geometry& geometry, ::Mesh& mesh, Model& model);
    void menuAxis(std::optional<urdf::Axis>& axis);
    void menuDynamics(std::optional<urdf::Dynamics>& dynamics);
    void menuLimit(std::optional<urdf::Limit>& limit);

    Camera camera_;
    Shader shader_;
    urdf::Parser urdf_parser_;
    std::shared_ptr<urdf::Robot> robot_;

    NFD::Guard nfdguard_;

    bool bOrbiting_; // TODO
    bool bWindowShouldClose_;
    int menubar_height_;

    urdf::TreeNodePtr hovered_node_;
    urdf::TreeNodePtr selected_node_;

    RenderTexture view_texture_;
};
