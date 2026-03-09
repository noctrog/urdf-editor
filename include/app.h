#pragma once

#define RAYGIZMO_IMPLEMENTATION

#include <command.h>
#include <raygizmo.h>
#include <raylib.h>
#include <robot.h>

#include <functional>
#include <memory>
#include <nfd.hpp>
#include <optional>

class App {
   public:
    App(int argc, char* argv[]);
    ~App() = default;

    void run();

   private:
    void setup();
    void update();
    void draw();
    void drawMenu();
    void drawScene(Rectangle viewport);
    void drawSelectionOutline(const std::shared_ptr<urdf::LinkNode>& link, Rectangle viewport);
    void drawJointAxis(const urdf::JointNodePtr& joint);
    void drawViewport();
    void recreateSceneTexture(int width, int height);
    void cleanup();

    void drawToolbar();
    void drawRobotTree();
    void drawNodeProperties();
    void drawSideMenu();
    void drawMaterialEditor();

    // Update visual materials on all links that reference the given material name.
    void updateLinksUsingMaterial(const std::string& material_name);
    // Return a post_action closure that calls updateLinksUsingMaterial for the given name.
    std::function<void()> materialUpdateAction(const std::string& material_name);

    void handleShortcuts();
    void openFile();
    void saveFile();

    void originGui(urdf::Origin& origin);

    void menuName(std::optional<std::string>& name, const char* label = "");
    void menuOrigin(std::optional<urdf::Origin>& origin);
    void menuMaterial(std::optional<std::string>& material_name, const urdf::LinkNodePtr& link);
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
    Shader outline_shader_;
    int outline_loc_width_ = -1;
    int outline_loc_viewport_ = -1;
    int outline_loc_color_ = -1;
    int outline_loc_center_ = -1;
    std::shared_ptr<urdf::Robot> robot_;

    NFD::Guard nfdguard_;

    RenderTexture2D scene_texture_{};
    unsigned int depth_stencil_rbo_ = 0;
    bool viewport_hovered_{false};

    bool bShowGrid_;
    bool bSupersampling_{true};
    bool bWindowShouldClose_;
    int menubar_height_{0};

    urdf::TreeNodePtr hovered_node_;
    urdf::TreeNodePtr selected_node_;
    urdf::OriginRawPtr selected_link_origin_;

    int pending_tab_ = -1;  // -1=none, 0=visual, 1+=collision index + 1

    RGizmo gizmo_;

    // Gizmo undo tracking
    RGizmoState prev_gizmo_state_ = RGIZMO_STATE_COLD;
    urdf::Origin* gizmo_drag_origin_ = nullptr;
    std::optional<urdf::Origin> snapshot_gizmo_origin_;

    // ImGui continuous edit snapshots (only one widget active at a time)
    std::optional<float> snapshot_float_;
    std::optional<Vector3> snapshot_vec3_;
    std::optional<std::string> snapshot_string_;
    std::optional<urdf::Origin> snapshot_origin_;
    std::optional<Vector4> snapshot_vec4_;

    void inputFloatUndoable(const char* label, float& value, float step = 0, float step_fast = 0,
                            const char* fmt = "%.3f", std::function<void()> post_action = nullptr);
    void inputFloat3Undoable(const char* label, Vector3& vec, const char* fmt = "%.3f",
                             std::function<void()> post_action = nullptr);
    void inputTextUndoable(const char* label, std::string& str,
                           std::function<void()> post_action = nullptr);
    void inputColorEdit4Undoable(const char* label, Vector4& color,
                                 std::function<void()> post_action = nullptr);
};
