#pragma once

#include <import_mesh.h>
#include <robot.h>

#include <functional>
#include <string>

// Represents a reversible editor operation.
//
// Implementations must make execute() safe to call again after undo() so the
// command buffer can redo the operation.
class Command {
   public:
    virtual ~Command() = default;
    // Applies this operation.
    virtual void execute() = 0;
    // Restores the state that existed before execute().
    virtual void undo() = 0;
};

using CommandPtr = std::shared_ptr<Command>;

// Changes one value and optionally refreshes dependent state after each change.
//
// Args:
//   T: Value type copied into the command's before and after snapshots.
template <typename T>
class UpdatePropertyCommand : public Command {
   public:
    UpdatePropertyCommand(T& target, T old_value, T new_value,
                          std::function<void()> post_action = nullptr)
        : target_(target),
          old_value_(std::move(old_value)),
          new_value_(std::move(new_value)),
          post_action_(std::move(post_action)) {}

    void execute() override {
        target_ = new_value_;
        if (post_action_) post_action_();
    }
    void undo() override {
        target_ = old_value_;
        if (post_action_) post_action_();
    }

   private:
    T& target_;
    T old_value_;
    T new_value_;
    std::function<void()> post_action_;
};

// Maintains the undo/redo history and the document's saved-state marker.
//
// Commands before current_command_ have been applied; later entries can be
// redone. Adding a command after undo discards that redo branch.
class CommandBuffer {
   public:
    CommandBuffer();

    // Queues a command to be applied by execute().
    void add(CommandPtr command);
    // Executes the next queued command and records it in history.
    void execute();
    // Reverts the most recently executed command, if any.
    void undo();
    // Reapplies the next reverted command, if any.
    void redo();

    // Returns whether there is an applied command to undo.
    bool canUndo() const;
    // Returns whether there is a reverted command to redo.
    bool canRedo() const;

    // Returns whether the history position differs from the saved position.
    bool isDirty() const;
    // Marks the current history position as the saved document state.
    void setSavePoint();
    // Drops all command history and marks the result as saved.
    void reset();

   private:
    std::vector<CommandPtr> executed_commands_;
    std::vector<CommandPtr> new_commands_;
    size_t current_command_;
    size_t save_point_ = 0;
};

// Replaces the active robot with a new empty robot while preserving undo state.
class CreateRobotCommand : public Command {
   public:
    CreateRobotCommand(std::shared_ptr<urdf::Robot>& robot, const Shader& shader);
    void execute() override;
    void undo() override;

   private:
    std::shared_ptr<urdf::Robot>& robot_;
    std::shared_ptr<urdf::Robot> old_robot_;
    const Shader& shader_;
};

// Loads a robot from disk and swaps it into the active document.
class LoadRobotCommand : public Command {
   public:
    LoadRobotCommand(std::string filename, std::shared_ptr<urdf::Robot>& robot, Shader& shader);
    void execute() override;
    void undo() override;

   private:
    const std::string filename_;
    std::shared_ptr<urdf::Robot>& robot_;
    std::shared_ptr<urdf::Robot> old_robot_;
    std::shared_ptr<urdf::Robot> future_robot_;
    const Shader& shader_;
};

// Reparents a joint and its child subtree to a different link.
class JointChangeParentCommand : public Command {
   public:
    JointChangeParentCommand(const urdf::JointNodePtr& node, const urdf::LinkNodePtr& new_parent,
                             const urdf::RobotPtr& robot);
    void execute() override;
    void undo() override;

   private:
    urdf::JointNodePtr joint_;
    urdf::LinkNodePtr new_parent_;
    urdf::LinkNodePtr old_parent_;
    urdf::RobotPtr robot_;
    std::vector<urdf::JointNodePtr>::iterator old_position_;
};

// Adds a uniquely named fixed joint and child link beneath a parent link.
class CreateJointCommand : public Command {
   public:
    CreateJointCommand(const char* joint_name, urdf::LinkNodePtr& parent, urdf::RobotPtr& robot);
    void execute() override;
    void undo() override;

   private:
    std::string joint_name_;
    urdf::LinkNodePtr parent_;
    urdf::JointNodePtr new_joint_;
    urdf::RobotPtr& robot_;
};

// Deletes a joint and its entire child subtree from the tree.
class DeleteJointCommand : public Command {
   public:
    DeleteJointCommand(urdf::JointNodePtr joint, urdf::RobotPtr& robot,
                       urdf::TreeNodePtr& selected_node, urdf::OriginRawPtr& selected_origin);
    void execute() override;
    void undo() override;

   private:
    urdf::JointNodePtr joint_;
    urdf::LinkNodePtr parent_;
    urdf::RobotPtr& robot_;
    urdf::TreeNodePtr& selected_node_;
    urdf::OriginRawPtr& selected_origin_;
    // Index used to restore the joint to its original sibling order on undo.
    int position_;
};

// Creates an empty optional name property.
class CreateNameCommand : public Command {
   public:
    explicit CreateNameCommand(std::optional<std::string>& target);
    void execute() override;
    void undo() override;

   private:
    std::optional<std::string>& target_;
};

// Creates an origin property and selects it for viewport editing.
class CreateOriginCommand : public Command {
   public:
    explicit CreateOriginCommand(std::optional<urdf::Origin>& target,
                                 urdf::OriginRawPtr& selected_origin);
    void execute() override;
    void undo() override;

   private:
    std::optional<urdf::Origin>& target_;
    urdf::OriginRawPtr& selected_origin_;
};

// Creates an axis property with its URDF default value.
class CreateAxisCommand : public Command {
   public:
    explicit CreateAxisCommand(std::optional<urdf::Axis>& target);
    void execute() override;
    void undo() override;

   private:
    std::optional<urdf::Axis>& target_;
};

// Creates a joint dynamics property with default damping and friction.
class CreateDynamicsCommand : public Command {
   public:
    explicit CreateDynamicsCommand(std::optional<urdf::Dynamics>& target);
    void execute() override;
    void undo() override;

   private:
    std::optional<urdf::Dynamics>& target_;
};

// Creates a joint limit property with editable default bounds.
class CreateLimitCommand : public Command {
   public:
    explicit CreateLimitCommand(std::optional<urdf::Limit>& target);
    void execute() override;
    void undo() override;

   private:
    std::optional<urdf::Limit>& target_;
};

// Creates a joint mimic property with the URDF default multiplier and offset.
class CreateMimicCommand : public Command {
   public:
    explicit CreateMimicCommand(std::optional<urdf::Mimic>& target);
    void execute() override;
    void undo() override;

   private:
    std::optional<urdf::Mimic>& target_;
};

// Creates a joint calibration property.
class CreateCalibrationCommand : public Command {
   public:
    explicit CreateCalibrationCommand(std::optional<urdf::Calibration>& target);
    void execute() override;
    void undo() override;

   private:
    std::optional<urdf::Calibration>& target_;
};

// Creates a joint safety-controller property.
class CreateSafetyControllerCommand : public Command {
   public:
    explicit CreateSafetyControllerCommand(std::optional<urdf::SafetyController>& target);
    void execute() override;
    void undo() override;

   private:
    std::optional<urdf::SafetyController>& target_;
};

// Replaces a visual or collision geometry and regenerates its render model.
class ChangeGeometryCommand : public Command {
   public:
    ChangeGeometryCommand(urdf::GeometryTypePtr old_geometry, urdf::GeometryTypePtr new_geometry,
                          urdf::Geometry& target, Model& model, urdf::RobotPtr& robot);
    void execute() override;
    void undo() override;

   private:
    urdf::GeometryTypePtr old_geometry_;
    urdf::GeometryTypePtr new_geometry_;
    urdf::Geometry& target_;
    Model& model_;
    urdf::RobotPtr robot_;
};

// Adds an inertial block to a link and selects its origin for editing.
class CreateInertialCommand : public Command {
   public:
    explicit CreateInertialCommand(urdf::LinkNodePtr& link, urdf::OriginRawPtr& selected_origin);
    void execute() override;
    void undo() override;

   private:
    urdf::LinkNodePtr link_;
    urdf::OriginRawPtr& selected_origin_;
};

// Appends a visual element to a link and creates its render model.
class CreateVisualCommand : public Command {
   public:
    CreateVisualCommand(urdf::LinkNodePtr& link, const urdf::RobotPtr& robot,
                        const Shader& shader_);
    void execute() override;
    void undo() override;

   private:
    urdf::LinkNodePtr link_;
    const urdf::RobotPtr robot_;
    const Shader shader_;
};

// Removes a visual element while retaining its data for undo.
class DeleteVisualCommand : public Command {
   public:
    DeleteVisualCommand(urdf::LinkNodePtr& link, int i, urdf::RobotPtr& robot);
    void execute() override;
    void undo() override;

   private:
    urdf::LinkNodePtr link_;
    int i_;
    Shader shader_;
    urdf::Visual old_visual_;
    const urdf::RobotPtr robot_;
};

// Appends a collision element to a link.
class AddCollisionCommand : public Command {
   public:
    explicit AddCollisionCommand(urdf::LinkNodePtr& link);
    void execute() override;
    void undo() override;

   private:
    urdf::LinkNodePtr link_;
};

// Removes a collision element while retaining its data for undo.
class DeleteCollisionCommand : public Command {
   public:
    DeleteCollisionCommand(urdf::LinkNodePtr& link, int i);
    void execute() override;
    void undo() override;

   private:
    urdf::LinkNodePtr link_;
    int i_;
    urdf::Collision old_collision_;
};

// Updates a box's dimensions and rebuilds its model.
class UpdateGeometryBoxCommand : public Command {
   public:
    UpdateGeometryBoxCommand(std::shared_ptr<urdf::Box>& box, const Vector3& old_size, Model& model,
                             const Shader& shader);
    void execute() override;
    void undo() override;

   private:
    Vector3 new_size_;
    Vector3 old_size_;
    std::shared_ptr<urdf::Box> box_;
    Model& model_;
    const Shader& shader_;
};

// Updates a cylinder's radius and length and rebuilds its model.
class UpdateGeometryCylinderCommand : public Command {
   public:
    UpdateGeometryCylinderCommand(std::shared_ptr<urdf::Cylinder>& cylinder, float old_radius,
                                  float old_height, Model& model, const Shader& shader);
    void execute() override;
    void undo() override;

   private:
    float new_radius_;
    float new_length_;
    float old_radius_;
    float old_length_;
    std::shared_ptr<urdf::Cylinder> cylinder_;
    Model& model_;
    const Shader& shader_;
};

// Updates a sphere's radius and rebuilds its model.
class UpdateGeometrySphereCommand : public Command {
   public:
    UpdateGeometrySphereCommand(std::shared_ptr<urdf::Sphere>& sphere, float old_radius,
                                Model& model, const Shader& shader);
    void execute() override;
    void undo() override;

   private:
    float new_radius_;
    float old_radius_;
    std::shared_ptr<urdf::Sphere> sphere_;
    Model& model_;
    const Shader& shader_;
};

// Adds a global material definition to the active robot.
class AddMaterialCommand : public Command {
   public:
    AddMaterialCommand(urdf::RobotPtr& robot, urdf::Material material);
    void execute() override;
    void undo() override;

   private:
    urdf::RobotPtr& robot_;
    urdf::Material material_;
};

// Removes a material definition and refreshes visuals that reference it.
class DeleteMaterialCommand : public Command {
   public:
    DeleteMaterialCommand(urdf::RobotPtr& robot, const std::string& name);
    void execute() override;
    void undo() override;

   private:
    urdf::RobotPtr& robot_;
    std::string name_;
    urdf::Material saved_material_;
};

// Renames a link and updates parent/child references in every joint.
class RenameLinkCommand : public Command {
   public:
    RenameLinkCommand(urdf::RobotPtr& robot, urdf::LinkNodePtr link, const std::string& old_name,
                      const std::string& new_name);
    void execute() override;
    void undo() override;

   private:
    void renameLink(const std::string& from, const std::string& to);

    urdf::RobotPtr& robot_;
    urdf::LinkNodePtr link_;
    std::string old_name_;
    std::string new_name_;
};

// Renames a global material and updates visual references to it.
class RenameMaterialCommand : public Command {
   public:
    RenameMaterialCommand(urdf::RobotPtr& robot, const std::string& old_name,
                          const std::string& new_name);
    void execute() override;
    void undo() override;

   private:
    void renameMaterial(const std::string& from, const std::string& to);

    urdf::RobotPtr& robot_;
    std::string old_name_;
    std::string new_name_;
};

// Clones a non-root link subtree and attaches the clone beside its source.
class CloneSubtreeCommand : public Command {
   public:
    CloneSubtreeCommand(urdf::LinkNodePtr source, urdf::RobotPtr& robot, const Shader& shader);
    void execute() override;
    void undo() override;

   private:
    urdf::LinkNodePtr source_;
    urdf::RobotPtr& robot_;
    Shader shader_;
    urdf::JointNodePtr cloned_joint_;
    urdf::LinkNodePtr attach_parent_;
};

// Replaces a mesh file reference and reloads the render model.
class UpdateGeometryMeshCommand : public Command {
   public:
    UpdateGeometryMeshCommand(std::shared_ptr<urdf::Mesh>& mesh, const std::string& new_filename,
                              Model& model, const Shader& shader);
    void execute() override;
    void undo() override;

   private:
    std::string new_filename_;
    std::string old_filename_;
    std::string old_resolved_path_;
    std::shared_ptr<urdf::Mesh> mesh_;
    Model& model_;
    const Shader& shader_;
};
