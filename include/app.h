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

// Owns the editor window, UI state, rendering resources, and active robot.
//
// The application coordinates input, the undo/redo command buffer, and the
// off-screen scene render used by the ImGui viewport.
class App {
   public:
    // Creates an application configured from the process command-line arguments.
    App(int argc, char* argv[]);
    ~App() = default;

    // Runs the application event loop until the user closes the window.
    void run();

   private:
    // Initializes the window, renderer, UI integration, and initial robot.
    void setup();
    // Processes one frame of input and updates editor state.
    void update();
    // Renders one complete editor frame.
    void draw();
    // Draws the menu bar and any modal dialogs it owns.
    void drawMenu();
    // Draws the robot scene into the supplied viewport rectangle.
    void drawScene(Rectangle viewport);
    // Draws a stencil-based outline around a selected link.
    void drawSelectionOutline(const std::shared_ptr<urdf::LinkNode>& link, Rectangle viewport);
    // Draws the selected joint's axis and, when applicable, its motion limits.
    void drawJointAxis(const urdf::JointNodePtr& joint);
    // Draws the scene texture and handles viewport-specific interactions.
    void drawViewport();
    // Recreates the off-screen color and depth-stencil attachments at the given size.
    void recreateSceneTexture(int width, int height);
    // Releases graphics and UI resources before the window closes.
    void cleanup();

    // Draws the primary editing controls.
    void drawToolbar();
    // Draws the searchable link/joint hierarchy and drag-and-drop targets.
    void drawRobotTree();
    // Draws properties for the currently selected tree node.
    void drawNodeProperties();
    // Draws the sidebar containing properties and materials.
    void drawSideMenu();
    // Draws the material list and editor.
    void drawMaterialEditor();
    // Draws validation results before a pending save.
    void drawValidationPanel();

    // Updates visual materials on links that reference a material.
    //
    // Args:
    //   material_name: Name of the material whose GPU-backed visuals changed.
    void updateLinksUsingMaterial(const std::string& material_name);
    // Creates a callback that refreshes links using a material after an edit.
    //
    // Args:
    //   material_name: Name captured by the returned callback.
    //
    // Returns:
    //   A post-edit callback suitable for an undoable property widget.
    std::function<void()> materialUpdateAction(const std::string& material_name);

    // Handles application-wide keyboard shortcuts.
    void handleShortcuts();
    // Opens a URDF file after confirming that unsaved work can be discarded.
    void openFile();
    // Validates and exports the active robot, or opens validation results.
    void saveFile();
    // Deletes the selected joint or the parent joint of a selected non-root link.
    void deleteSelectedJoint();
    // Runs an action immediately or after confirmation when the document is dirty.
    void guardUnsavedChanges(std::function<void()> action);

    // Draws position and Euler-angle controls for an origin.
    void originGui(urdf::Origin& origin);

    // Draws controls for optional URDF properties and creates/removes them on demand.
    void menuName(std::optional<std::string>& name);
    void menuOrigin(std::optional<urdf::Origin>& origin);
    void menuMaterial(std::optional<std::string>& material_name, const urdf::LinkNodePtr& link);
    void menuGeometry(urdf::Geometry& geometry, Model& model);
    void menuAxis(std::optional<urdf::Axis>& axis);
    void menuDynamics(std::optional<urdf::Dynamics>& dynamics);
    void menuLimit(std::optional<urdf::Limit>& limit);
    void menuMimic(std::optional<urdf::Mimic>& mimic);
    void menuCalibration(std::optional<urdf::Calibration>& calibration);
    void menuSafetyController(std::optional<urdf::SafetyController>& safety_controller);

    void menuPropertiesInertial(urdf::LinkNodePtr link_node);
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
    bool bShowCollisions_{true};
    bool bShowValidation_{false};
    bool bWindowShouldClose_;
    int menubar_height_{0};

    char tree_filter_[128] = {};

    urdf::TreeNodePtr hovered_node_;
    urdf::TreeNodePtr selected_node_;
    urdf::OriginRawPtr selected_link_origin_;

    // Encodes a requested properties tab: -1 is none, 0..N-1 are visuals,
    // and subsequent values are collision indices offset by the visual count.
    int pending_tab_ = -1;

    // Tracks validation that must be acknowledged before a save proceeds.
    std::vector<urdf::ValidationMessage> validation_results_;
    bool pending_save_ = false;
    std::string pending_save_path_;

    // Stores an action deferred until the user confirms discarding unsaved work.
    bool pending_discard_ = false;
    std::function<void()> discard_action_;

    RGizmo gizmo_;

    // Captures the origin once per gizmo drag so the complete drag is undoable.
    RGizmoState prev_gizmo_state_ = RGIZMO_STATE_COLD;
    urdf::Origin* gizmo_drag_origin_ = nullptr;
    std::optional<urdf::Origin> snapshot_gizmo_origin_;

    // Captures an ImGui widget's value on activation; only one widget is active at a time.
    std::optional<float> snapshot_float_;
    std::optional<Vector3> snapshot_vec3_;
    std::optional<std::string> snapshot_string_;
    std::optional<urdf::Origin> snapshot_origin_;
    std::optional<Vector4> snapshot_vec4_;

    // Draws an undoable numeric input and invokes post_action after each committed edit.
    void inputFloatUndoable(const char* label, float& value, float step = 0, float step_fast = 0,
                            const char* fmt = "%.3f", std::function<void()> post_action = nullptr);
    // Draws an undoable numeric slider bounded by min_val and max_val.
    void sliderFloatUndoable(const char* label, float& value, float min_val, float max_val,
                             const char* fmt = "%.3f", std::function<void()> post_action = nullptr);
    // Draws an undoable three-component floating-point input.
    void inputFloat3Undoable(const char* label, Vector3& vec, const char* fmt = "%.3f",
                             std::function<void()> post_action = nullptr);
    // Draws an undoable text input.
    void inputTextUndoable(const char* label, std::string& str,
                           std::function<void()> post_action = nullptr);
    // Draws an undoable RGBA color editor.
    void inputColorEdit4Undoable(const char* label, Vector4& color,
                                 std::function<void()> post_action = nullptr);
};
