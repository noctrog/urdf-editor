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
    void cleanup();

    Camera camera_;
    Shader shader_;
    urdf::Parser urdf_parser_;
    std::shared_ptr<urdf::Robot> robot_;

    NFD::Guard nfdguard_;

    bool bOrbiting_; // TODO
    bool bWindowShouldClose_;
};
