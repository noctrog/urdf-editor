#pragma once

#include <import_mesh.h>
#include <robot.h>

#include <functional>
#include <string>

class Command {
   public:
    virtual ~Command() = default;
    virtual void execute() = 0;
    virtual void undo() = 0;
};

using CommandPtr = std::shared_ptr<Command>;

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

class CommandBuffer {
   public:
    CommandBuffer();

    void add(CommandPtr command);
    void execute();
    void undo();
    void redo();

    bool canUndo();
    bool canRedo();

   private:
    std::vector<CommandPtr> executed_commands_;
    std::vector<CommandPtr> new_commands_;
    size_t current_command_;
};

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
    int position_;  // index in parent's children vector
};

class CreateNameCommand : public Command {
   public:
    explicit CreateNameCommand(std::optional<std::string>& target);
    void execute() override;
    void undo() override;

   private:
    std::optional<std::string>& target_;
};

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

class CreateAxisCommand : public Command {
   public:
    explicit CreateAxisCommand(std::optional<urdf::Axis>& target);
    void execute() override;
    void undo() override;

   private:
    std::optional<urdf::Axis>& target_;
};

class CreateDynamicsCommand : public Command {
   public:
    explicit CreateDynamicsCommand(std::optional<urdf::Dynamics>& target);
    void execute() override;
    void undo() override;

   private:
    std::optional<urdf::Dynamics>& target_;
};

class CreateLimitCommand : public Command {
   public:
    explicit CreateLimitCommand(std::optional<urdf::Limit>& target);
    void execute() override;
    void undo() override;

   private:
    std::optional<urdf::Limit>& target_;
};

class CreateMimicCommand : public Command {
   public:
    explicit CreateMimicCommand(std::optional<urdf::Mimic>& target);
    void execute() override;
    void undo() override;

   private:
    std::optional<urdf::Mimic>& target_;
};

class CreateCalibrationCommand : public Command {
   public:
    explicit CreateCalibrationCommand(std::optional<urdf::Calibration>& target);
    void execute() override;
    void undo() override;

   private:
    std::optional<urdf::Calibration>& target_;
};

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

class CreateInertialCommand : public Command {
   public:
    explicit CreateInertialCommand(urdf::LinkNodePtr& link, urdf::OriginRawPtr& selected_origin);
    void execute() override;
    void undo() override;

   private:
    urdf::LinkNodePtr link_;
    urdf::OriginRawPtr& selected_origin_;
};

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

class AddCollisionCommand : public Command {
   public:
    explicit AddCollisionCommand(urdf::LinkNodePtr& link);
    void execute() override;
    void undo() override;

   private:
    urdf::LinkNodePtr link_;
};

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

class AddMaterialCommand : public Command {
   public:
    AddMaterialCommand(urdf::RobotPtr& robot, urdf::Material material);
    void execute() override;
    void undo() override;

   private:
    urdf::RobotPtr& robot_;
    urdf::Material material_;
};

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
