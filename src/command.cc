#include <deque>
#include <algorithm>
#include <utility>

#include <fmt/format.h>
#include <robot.h>
#include <command.h>

CommandBuffer::CommandBuffer()
    : current_command_(0)
{

}

void CommandBuffer::add(CommandPtr command)
{
    if (command) {
        new_commands_.emplace_back(std::move(command));
    }
}

void CommandBuffer::execute()
{
    if (new_commands_.empty()) return;

    // Delete all commands that were executed after the current command
    // This will only happen after undoing some commands
    if (executed_commands_.begin() + current_command_ != executed_commands_.end()) {
        executed_commands_.erase(executed_commands_.begin() + current_command_,
                                 executed_commands_.end());
    }

    for (const auto& command : new_commands_) {
        command->execute();
        executed_commands_.push_back(command);
    }

    current_command_ = executed_commands_.size();
    new_commands_.clear();
}

void CommandBuffer::undo()
{
    if (executed_commands_.begin() + current_command_ == executed_commands_.begin()) return;

    (*(executed_commands_.begin() + current_command_ - 1))->undo();
    current_command_--;
}

void CommandBuffer::redo()
{
    if (executed_commands_.begin() + current_command_ == executed_commands_.end()) return;

    (*(executed_commands_.begin() + current_command_))->execute();
    current_command_++;
}

bool CommandBuffer::canUndo()
{
    return executed_commands_.begin() + current_command_ != executed_commands_.begin();
}

bool CommandBuffer::canRedo()
{
    return executed_commands_.begin() + current_command_ != executed_commands_.end();
}

CreateRobotCommand::CreateRobotCommand(std::shared_ptr<urdf::Robot>& robot,
                                       const Shader& shader)
    : robot_(robot), shader_(shader)
{

}

void CreateRobotCommand::execute()
{
    old_robot_ = robot_;
    urdf::LinkNodePtr root = std::make_shared<urdf::LinkNode>(urdf::Link("root"), nullptr);
    robot_ = std::make_shared<urdf::Robot>(root);
    robot_->setShader(shader_);
}

void CreateRobotCommand::undo()
{
    robot_ = old_robot_;
}

LoadRobotCommand::LoadRobotCommand(std::string filename,
                                   std::shared_ptr<urdf::Robot>& robot,
                                   Shader& shader)
    : filename_(std::move(filename)), robot_(robot), shader_(shader)
{

}

void LoadRobotCommand::execute()
{
    if (future_robot_) {
        old_robot_ = robot_;
        robot_ = future_robot_;
    } else {
        old_robot_ = robot_;
        robot_ = urdf::buildRobot(filename_.c_str());
        robot_->buildGeometry();
        robot_->forwardKinematics();
        robot_->setShader(shader_);
    }
}

void LoadRobotCommand::undo()
{
    future_robot_ = robot_;
    robot_ = old_robot_;
}

JointChangeParentCommand::JointChangeParentCommand(const urdf::JointNodePtr& node,
                                                   const urdf::LinkNodePtr& new_parent,
                                                   const urdf::RobotPtr& robot)
    : joint_(node), new_parent_(new_parent), old_parent_(node->parent), robot_(robot)
{

}

void JointChangeParentCommand::execute()
{
    old_position_ = std::find(joint_->parent->children.begin(),
                              joint_->parent->children.end(),
                              joint_);
    joint_->parent->children.erase(old_position_);
    new_parent_->children.push_back(joint_);

    joint_->parent = new_parent_;
    joint_->joint.parent = new_parent_->link.name;
    robot_->forwardKinematics();
}

void JointChangeParentCommand::undo()
{
    joint_->parent->children.erase(std::find(joint_->parent->children.begin(),
                                             joint_->parent->children.end(),
                                             joint_));
    old_parent_->children.insert(old_position_, joint_);

    joint_->parent = old_parent_;
    joint_->joint.parent = old_parent_->link.name;
    robot_->forwardKinematics();
}

CreateJointCommand::CreateJointCommand(const char *joint_name,
                                       urdf::LinkNodePtr& parent,
                                       urdf::RobotPtr& robot)
    : joint_name_(joint_name), parent_(parent), robot_(robot)
{

}

void CreateJointCommand::execute()
{
    // Get number of overlapping link and joint names
    int new_link_count = 0;
    int new_joint_count = 0;
    std::deque<urdf::LinkNodePtr> deq { robot_->getRoot() };
    while (not deq.empty()) {
        const auto& current_link = deq.front();
        deq.pop_front();

        new_link_count += static_cast<int>(current_link->link.name.find("New link") != std::string::npos);

        for (const auto& joint : current_link->children) {
            new_joint_count += static_cast<int>(joint->joint.name.find(joint_name_) != std::string::npos);
            deq.push_back(joint->child);
        }
    }

    // Rename in case of name collisions
    if (new_joint_count > 0) joint_name_ += fmt::format(" {}", new_joint_count);

    std::string new_link_name = (new_link_count == 0) ? "New link" : fmt::format("New link {}", new_link_count);
    urdf::LinkNodePtr child = std::make_shared<urdf::LinkNode>(urdf::Link{ new_link_name }, nullptr);

    // Create new joint and link
    urdf::JointNodePtr new_joint = std::make_shared<urdf::JointNode>(urdf::JointNode {
        urdf::Joint(joint_name_.c_str(), parent_->link.name.c_str(), new_link_name.c_str(), "fixed"),
        parent_,
        child});

    child->parent = new_joint;

    parent_->children.push_back(new_joint);
}

void CreateJointCommand::undo()
{
    parent_->children.erase(std::find(parent_->children.begin(),
                                      parent_->children.end(),
                                      new_joint_));
    new_joint_.reset();
}

ChangeNameCommand::ChangeNameCommand(const std::string& old_name,
                                     const std::string& new_name,
                                     std::string& target)
    : old_name_(old_name), new_name_(new_name), target_(target)
{

}

void ChangeNameCommand::execute()
{
    target_ = new_name_;
}

void ChangeNameCommand::undo()
{
    target_ = old_name_;
}

CreateNameCommand::CreateNameCommand(std::optional<std::string>& target)
    : target_(target)
{

}

void CreateNameCommand::execute()
{
    target_ = std::string();
}

void CreateNameCommand::undo()
{
    target_ = std::nullopt;
}

CreateOriginCommand::CreateOriginCommand(std::optional<urdf::Origin>& target)
    : target_(target)
{

}

void CreateOriginCommand::execute()
{
    target_ = urdf::Origin();
}

void CreateOriginCommand::undo()
{
    target_ = std::nullopt;
}

UpdateOriginCommand::UpdateOriginCommand(urdf::Origin& old_origin,
                                         urdf::Origin& new_origin,
                                         urdf::Origin& target,
                                         urdf::RobotPtr& robot)
    : old_origin_(old_origin), new_origin_(new_origin), target_(target), robot_(robot)
{

}

void UpdateOriginCommand::execute()
{
    target_ = new_origin_;
    robot_->forwardKinematics();
}

void UpdateOriginCommand::undo()
{
    target_ = old_origin_;
    robot_->forwardKinematics();
}

CreateAxisCommand::CreateAxisCommand(std::optional<urdf::Axis>& target)
    : target_(target)
{

}

void CreateAxisCommand::execute()
{
    target_ = urdf::Axis();
}

void CreateAxisCommand::undo()
{
    target_ = std::nullopt;
}

CreateDynamicsCommand::CreateDynamicsCommand(std::optional<urdf::Dynamics>& target)
    : target_(target)
{

}

void CreateDynamicsCommand::execute()
{
    target_ = urdf::Dynamics();
}

void CreateDynamicsCommand::undo()
{
    target_ = std::nullopt;
}

CreateLimitCommand::CreateLimitCommand(std::optional<urdf::Limit>& target)
    : target_(target)
{

}

void CreateLimitCommand::execute()
{
    target_ = urdf::Limit(0.0F, 1.0F, 1.0F, 1.0F);
}

void CreateLimitCommand::undo()
{
    target_ = std::nullopt;
}

ChangeGeometryCommand::ChangeGeometryCommand(urdf::GeometryTypePtr old_geometry,
                                             urdf::GeometryTypePtr new_geometry,
                                             urdf::Geometry& target,
                                             Model& model)
    : old_geometry_(std::move(old_geometry)), new_geometry_(std::move(new_geometry)), target_(target), model_(model)
{

}

void ChangeGeometryCommand::execute()
{
    target_.type = new_geometry_;

    UnloadModel(model_);
    model_ = target_.type->generateGeometry();
    // mesh_ = target_.type->generateGeometry();
    // model_.meshes[0] = mesh_; // TODO: might need to update material
}

void ChangeGeometryCommand::undo()
{
    target_.type = old_geometry_;

    UnloadModel(model_);
    model_ = target_.type->generateGeometry();
    // model_.meshes[0] = mesh_;
}

CreateInertialCommand::CreateInertialCommand(urdf::LinkNodePtr& link)
    : link_(link)
{

}

void CreateInertialCommand::execute()
{
    link_->link.inertial = urdf::Inertial();
}

void CreateInertialCommand::undo()
{
    link_->link.inertial = std::nullopt;
}

CreateVisualCommand::CreateVisualCommand(urdf::LinkNodePtr& link,
                                         const urdf::RobotPtr& robot,
                                         const Shader& shader)
    : link_(link), robot_(robot), shader_(shader)
{

}

void CreateVisualCommand::execute()
{
    link_->link.visual = urdf::Visual{
        std::nullopt, std::nullopt,
        urdf::Geometry{std::make_shared<urdf::Box>()},
        std::nullopt};
    link_->visual_model = link_->link.visual->geometry.type->generateGeometry();
    link_->visual_model.materials[0].shader = shader_;
    robot_->forwardKinematics();
}

void CreateVisualCommand::undo()
{
    UnloadModel(link_->visual_model);
    link_->link.visual = std::nullopt;
}

DeleteVisualCommand::DeleteVisualCommand(urdf::LinkNodePtr& link, urdf::RobotPtr& robot)
    : link_(link), robot_(robot)
{

}

void DeleteVisualCommand::execute()
{
    old_visual_ = *link_->link.visual;
    shader_ = link_->visual_model.materials[0].shader;

    UnloadModel(link_->visual_model);
    link_->link.visual = std::nullopt;
}

void DeleteVisualCommand::undo()
{
    link_->link.visual = old_visual_;
    old_visual_ = urdf::Visual();
    UnloadModel(link_->visual_model);
    link_->visual_model = link_->link.visual->geometry.type->generateGeometry();
    link_->visual_model.materials[0].shader = shader_;
    robot_->forwardKinematics();
    robot_->updateMaterial(link_);
}

AddCollisionCommand::AddCollisionCommand(urdf::LinkNodePtr& link)
    : link_(link)
{

}

void AddCollisionCommand::execute()
{
    link_->addCollision();
}

void AddCollisionCommand::undo()
{
    link_->deleteCollision(link_->link.collision.size() - 1);
}

DeleteCollisionCommand::DeleteCollisionCommand(urdf::LinkNodePtr& link, int i)
    : link_(link), i_(i)
{

}

void DeleteCollisionCommand::execute()
{
    old_collision_ = link_->link.collision[i_];
    link_->deleteCollision(i_);
}

void DeleteCollisionCommand::undo()
{
    link_->link.collision.insert(link_->link.collision.begin() + i_, old_collision_);
    link_->collision_models.insert(link_->collision_models.begin() + i_,
                                   old_collision_.geometry.type->generateGeometry());
}

UpdateGeometryBoxCommand::UpdateGeometryBoxCommand(std::shared_ptr<urdf::Box>& box,
                                                   const Vector3& old_size,
                                                   Model& model,
                                                   const Shader& shader)
    : new_size_(box->size), old_size_(old_size), box_(box), model_(model), shader_(shader)
{

}

void UpdateGeometryBoxCommand::execute()
{
    MaterialMap mat_map = model_.materials[0].maps[MATERIAL_MAP_DIFFUSE];
    const Matrix t = model_.transform;
    box_->size = new_size_;

    UnloadModel(model_);
    model_ = box_->generateGeometry();

    model_.materials[0].shader = shader_;
    model_.transform = t;
    model_.materials[0].maps[MATERIAL_MAP_DIFFUSE] = mat_map;
}

void UpdateGeometryBoxCommand::undo()
{
    MaterialMap mat_map = model_.materials[0].maps[MATERIAL_MAP_DIFFUSE];
    const Matrix t = model_.transform;
    box_->size = old_size_;

    UnloadModel(model_);
    model_ = box_->generateGeometry();

    model_.materials[0].shader = shader_;
    model_.transform = t;
    model_.materials[0].maps[MATERIAL_MAP_DIFFUSE] = mat_map;
}

UpdateGeometryCylinderCommand::UpdateGeometryCylinderCommand(std::shared_ptr<urdf::Cylinder>& cylinder,
                                                             const float old_radius,
                                                             const float old_height,
                                                             Model& model,
                                                             const Shader& shader)
    : new_radius_(cylinder->radius), new_length_(cylinder->length), old_radius_(old_radius), old_length_(old_height),
      cylinder_(cylinder), model_(model), shader_(shader)
{

}

void UpdateGeometryCylinderCommand::execute()
{
    MaterialMap mat_map = model_.materials[0].maps[MATERIAL_MAP_DIFFUSE];
    const Matrix t = model_.transform;
    cylinder_->radius = new_radius_;
    cylinder_->length = new_length_;

    UnloadModel(model_);
    model_ = cylinder_->generateGeometry();

    model_.materials[0].shader = shader_;
    model_.transform = t;
    model_.materials[0].maps[MATERIAL_MAP_DIFFUSE] = mat_map;
}

void UpdateGeometryCylinderCommand::undo()
{
    MaterialMap mat_map = model_.materials[0].maps[MATERIAL_MAP_DIFFUSE];
    const Matrix t = model_.transform;
    cylinder_->radius = old_radius_;
    cylinder_->length = old_length_;

    UnloadModel(model_);
    model_ = cylinder_->generateGeometry();

    model_.materials[0].shader = shader_;
    model_.transform = t;
    model_.materials[0].maps[MATERIAL_MAP_DIFFUSE] = mat_map;
}

UpdateGeometrySphereCommand::UpdateGeometrySphereCommand(std::shared_ptr<urdf::Sphere>& sphere,
                                                         const float old_radius,
                                                         Model& model,
                                                         const Shader& shader)
    : new_radius_(sphere->radius), old_radius_(old_radius), sphere_(sphere), model_(model), shader_(shader)
{

}

void UpdateGeometrySphereCommand::execute()
{
    MaterialMap mat_map = model_.materials[0].maps[MATERIAL_MAP_DIFFUSE];
    const Matrix t = model_.transform;
    sphere_->radius = new_radius_;

    UnloadModel(model_);
    model_ = sphere_->generateGeometry();

    model_.materials[0].shader = shader_;
    model_.transform = t;
    model_.materials[0].maps[MATERIAL_MAP_DIFFUSE] = mat_map;
}

void UpdateGeometrySphereCommand::undo()
{
    MaterialMap mat_map = model_.materials[0].maps[MATERIAL_MAP_DIFFUSE];
    const Matrix t = model_.transform;

    UnloadModel(model_);
    model_ = sphere_->generateGeometry();

    model_.materials[0].shader = shader_;
    model_.transform = t;
    model_.materials[0].maps[MATERIAL_MAP_DIFFUSE] = mat_map;
}

UpdateGeometryMeshCommand::UpdateGeometryMeshCommand(std::shared_ptr<urdf::Mesh>& mesh,
                                                     const std::string& new_filename,
                                                     Model& model,
                                                     const Shader& shader)
    : new_filename_(new_filename), mesh_(mesh), model_(model), shader_(shader)
{

}

void UpdateGeometryMeshCommand::execute()
{
    MaterialMap mat_map = model_.materials[0].maps[MATERIAL_MAP_DIFFUSE];
    const Matrix t = model_.transform;
    old_filename_ = mesh_->filename;
    mesh_->filename = new_filename_;

    UnloadModel(model_);
    model_ = mesh_->generateGeometry();

    model_.materials[0].shader = shader_;
    model_.transform = t;
    model_.materials[0].maps[MATERIAL_MAP_DIFFUSE] = mat_map;
}

void UpdateGeometryMeshCommand::undo()
{
    MaterialMap mat_map = model_.materials[0].maps[MATERIAL_MAP_DIFFUSE];
    const Matrix t = model_.transform;
    mesh_->filename = old_filename_;

    UnloadModel(model_);
    model_ = mesh_->generateGeometry();

    model_.materials[0].shader = shader_;
    model_.transform = t;
    model_.materials[0].maps[MATERIAL_MAP_DIFFUSE] = mat_map;
}

