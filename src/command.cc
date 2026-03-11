#include <command.h>
#include <fmt/format.h>
#include <robot.h>

#include <algorithm>
#include <deque>
#include <filesystem>
#include <utility>

CommandBuffer::CommandBuffer() : current_command_(0) {}

void CommandBuffer::add(CommandPtr command) {
    if (command) {
        new_commands_.emplace_back(std::move(command));
    }
}

void CommandBuffer::execute() {
    if (new_commands_.empty()) return;

    // Discard any commands after the current position (happens after undo)
    if (current_command_ < executed_commands_.size()) {
        executed_commands_.erase(executed_commands_.begin() + current_command_,
                                 executed_commands_.end());
        // If the save point was in the erased range, the saved state is unreachable
        if (save_point_ > current_command_) save_point_ = current_command_;
    }

    for (const auto& command : new_commands_) {
        command->execute();
        executed_commands_.push_back(command);
    }

    current_command_ = executed_commands_.size();
    new_commands_.clear();
}

void CommandBuffer::undo() {
    if (current_command_ == 0) return;

    executed_commands_[current_command_ - 1]->undo();
    current_command_--;
}

void CommandBuffer::redo() {
    if (current_command_ == executed_commands_.size()) return;

    executed_commands_[current_command_]->execute();
    current_command_++;
}

bool CommandBuffer::canUndo() const { return current_command_ > 0; }

bool CommandBuffer::canRedo() const { return current_command_ < executed_commands_.size(); }

bool CommandBuffer::isDirty() const { return current_command_ != save_point_; }

void CommandBuffer::setSavePoint() { save_point_ = current_command_; }

void CommandBuffer::reset() {
    executed_commands_.clear();
    new_commands_.clear();
    current_command_ = 0;
    save_point_ = 0;
}

CreateRobotCommand::CreateRobotCommand(std::shared_ptr<urdf::Robot>& robot, const Shader& shader)
    : robot_(robot), shader_(shader) {}

void CreateRobotCommand::execute() {
    old_robot_ = robot_;
    urdf::LinkNodePtr root = std::make_shared<urdf::LinkNode>(urdf::Link("root"), nullptr);
    robot_ = std::make_shared<urdf::Robot>(root);
    robot_->setShader(shader_);
}

void CreateRobotCommand::undo() { robot_ = old_robot_; }

LoadRobotCommand::LoadRobotCommand(std::string filename, std::shared_ptr<urdf::Robot>& robot,
                                   Shader& shader)
    : filename_(std::move(filename)), robot_(robot), shader_(shader) {}

void LoadRobotCommand::execute() {
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

void LoadRobotCommand::undo() {
    future_robot_ = robot_;
    robot_ = old_robot_;
}

JointChangeParentCommand::JointChangeParentCommand(const urdf::JointNodePtr& node,
                                                   const urdf::LinkNodePtr& new_parent,
                                                   const urdf::RobotPtr& robot)
    : joint_(node), new_parent_(new_parent), old_parent_(node->parent), robot_(robot) {}

void JointChangeParentCommand::execute() {
    old_position_ =
        std::find(joint_->parent->children.begin(), joint_->parent->children.end(), joint_);
    joint_->parent->children.erase(old_position_);
    new_parent_->children.push_back(joint_);

    joint_->parent = new_parent_;
    joint_->joint.parent = new_parent_->link.name;
    robot_->forwardKinematics();
}

void JointChangeParentCommand::undo() {
    joint_->parent->children.erase(
        std::find(joint_->parent->children.begin(), joint_->parent->children.end(), joint_));
    old_parent_->children.insert(old_position_, joint_);

    joint_->parent = old_parent_;
    joint_->joint.parent = old_parent_->link.name;
    robot_->forwardKinematics();
}

CreateJointCommand::CreateJointCommand(const char* joint_name, urdf::LinkNodePtr& parent,
                                       urdf::RobotPtr& robot)
    : joint_name_(joint_name), parent_(parent), robot_(robot) {}

void CreateJointCommand::execute() {
    // Get number of overlapping link and joint names
    int new_link_count = 0;
    int new_joint_count = 0;
    std::deque<urdf::LinkNodePtr> deq{robot_->getRoot()};
    while (not deq.empty()) {
        const auto& current_link = deq.front();
        deq.pop_front();

        new_link_count +=
            static_cast<int>(current_link->link.name.find("New link") != std::string::npos);

        for (const auto& joint : current_link->children) {
            new_joint_count +=
                static_cast<int>(joint->joint.name.find(joint_name_) != std::string::npos);
            deq.push_back(joint->child);
        }
    }

    // Rename in case of name collisions
    if (new_joint_count > 0) joint_name_ += fmt::format(" {}", new_joint_count);

    std::string new_link_name =
        (new_link_count == 0) ? "New link" : fmt::format("New link {}", new_link_count);
    urdf::LinkNodePtr child = std::make_shared<urdf::LinkNode>(urdf::Link{new_link_name}, nullptr);

    // Create new joint and link
    urdf::JointNodePtr new_joint = std::make_shared<urdf::JointNode>(
        urdf::JointNode{urdf::Joint(joint_name_.c_str(), parent_->link.name.c_str(),
                                    new_link_name.c_str(), "fixed"),
                        parent_, child});

    child->parent = new_joint;

    parent_->children.push_back(new_joint);
}

void CreateJointCommand::undo() {
    parent_->children.erase(
        std::find(parent_->children.begin(), parent_->children.end(), new_joint_));
    new_joint_.reset();
}

DeleteJointCommand::DeleteJointCommand(urdf::JointNodePtr joint, urdf::RobotPtr& robot,
                                       urdf::TreeNodePtr& selected_node,
                                       urdf::OriginRawPtr& selected_origin)
    : joint_(std::move(joint)),
      parent_(joint_->parent),
      robot_(robot),
      selected_node_(selected_node),
      selected_origin_(selected_origin),
      position_(0) {}

void DeleteJointCommand::execute() {
    auto& children = parent_->children;
    auto it = std::find(children.begin(), children.end(), joint_);
    position_ = static_cast<int>(std::distance(children.begin(), it));
    children.erase(it);

    selected_node_ = nullptr;
    selected_origin_ = nullptr;

    robot_->forwardKinematics();
}

void DeleteJointCommand::undo() {
    auto& children = parent_->children;
    children.insert(children.begin() + position_, joint_);

    selected_node_ = joint_;
    selected_origin_ = nullptr;

    robot_->forwardKinematics();
}

CreateNameCommand::CreateNameCommand(std::optional<std::string>& target) : target_(target) {}

void CreateNameCommand::execute() { target_ = std::string(); }

void CreateNameCommand::undo() { target_ = std::nullopt; }

CreateOriginCommand::CreateOriginCommand(std::optional<urdf::Origin>& target,
                                         urdf::OriginRawPtr& selected_origin)
    : target_(target), selected_origin_(selected_origin) {}

void CreateOriginCommand::execute() {
    target_ = urdf::Origin();
    selected_origin_ = &*target_;
}

void CreateOriginCommand::undo() {
    target_ = std::nullopt;
    selected_origin_ = nullptr;
}

CreateAxisCommand::CreateAxisCommand(std::optional<urdf::Axis>& target) : target_(target) {}

void CreateAxisCommand::execute() { target_ = urdf::Axis(); }

void CreateAxisCommand::undo() { target_ = std::nullopt; }

CreateDynamicsCommand::CreateDynamicsCommand(std::optional<urdf::Dynamics>& target)
    : target_(target) {}

void CreateDynamicsCommand::execute() { target_ = urdf::Dynamics(); }

void CreateDynamicsCommand::undo() { target_ = std::nullopt; }

CreateLimitCommand::CreateLimitCommand(std::optional<urdf::Limit>& target) : target_(target) {}

void CreateLimitCommand::execute() { target_ = urdf::Limit(0.0F, 1.0F, 1.0F, 1.0F); }

void CreateLimitCommand::undo() { target_ = std::nullopt; }

CreateMimicCommand::CreateMimicCommand(std::optional<urdf::Mimic>& target) : target_(target) {}

void CreateMimicCommand::execute() { target_ = urdf::Mimic(); }

void CreateMimicCommand::undo() { target_ = std::nullopt; }

CreateCalibrationCommand::CreateCalibrationCommand(std::optional<urdf::Calibration>& target)
    : target_(target) {}

void CreateCalibrationCommand::execute() { target_ = urdf::Calibration(); }

void CreateCalibrationCommand::undo() { target_ = std::nullopt; }

CreateSafetyControllerCommand::CreateSafetyControllerCommand(
    std::optional<urdf::SafetyController>& target)
    : target_(target) {}

void CreateSafetyControllerCommand::execute() { target_ = urdf::SafetyController(); }

void CreateSafetyControllerCommand::undo() { target_ = std::nullopt; }

ChangeGeometryCommand::ChangeGeometryCommand(urdf::GeometryTypePtr old_geometry,
                                             urdf::GeometryTypePtr new_geometry,
                                             urdf::Geometry& target, Model& model,
                                             urdf::RobotPtr& robot)
    : old_geometry_(std::move(old_geometry)),
      new_geometry_(std::move(new_geometry)),
      target_(target),
      model_(model),
      robot_(robot) {}

void ChangeGeometryCommand::execute() {
    target_.type = new_geometry_;

    UnloadModel(model_);
    model_ = target_.type->generateGeometry();

    robot_->forwardKinematics();
}

void ChangeGeometryCommand::undo() {
    target_.type = old_geometry_;

    UnloadModel(model_);
    model_ = target_.type->generateGeometry();

    robot_->forwardKinematics();
}

CreateInertialCommand::CreateInertialCommand(urdf::LinkNodePtr& link,
                                             urdf::OriginRawPtr& selected_origin)
    : link_(link), selected_origin_(selected_origin) {}

void CreateInertialCommand::execute() {
    link_->link.inertial = urdf::Inertial();
    selected_origin_ = &link_->link.inertial->origin;
}

void CreateInertialCommand::undo() {
    link_->link.inertial = std::nullopt;
    selected_origin_ = nullptr;
}

CreateVisualCommand::CreateVisualCommand(urdf::LinkNodePtr& link, const urdf::RobotPtr& robot,
                                         const Shader& shader)
    : link_(link), robot_(robot), shader_(shader) {}

void CreateVisualCommand::execute() {
    link_->link.visual.push_back(
        urdf::Visual{std::nullopt, std::nullopt,
                     urdf::Geometry{std::make_shared<urdf::Box>()}, std::nullopt});
    Model model = link_->link.visual.back().geometry.type->generateGeometry();
    model.materials[0].shader = shader_;
    link_->visual_models.push_back(model);
    robot_->forwardKinematics();
}

void CreateVisualCommand::undo() {
    link_->deleteVisual(static_cast<int>(link_->link.visual.size()) - 1);
}

DeleteVisualCommand::DeleteVisualCommand(urdf::LinkNodePtr& link, int i, urdf::RobotPtr& robot)
    : link_(link), i_(i), robot_(robot) {}

void DeleteVisualCommand::execute() {
    old_visual_ = link_->link.visual[i_];
    shader_ = link_->visual_models[i_].materials[0].shader;
    link_->deleteVisual(i_);
}

void DeleteVisualCommand::undo() {
    link_->link.visual.insert(link_->link.visual.begin() + i_, old_visual_);
    Model model = old_visual_.geometry.type->generateGeometry();
    model.materials[0].shader = shader_;
    link_->visual_models.insert(link_->visual_models.begin() + i_, model);
    robot_->forwardKinematics();
    robot_->updateMaterial(link_);
}

AddCollisionCommand::AddCollisionCommand(urdf::LinkNodePtr& link) : link_(link) {}

void AddCollisionCommand::execute() {
    link_->addCollision();
    if (link_->parent and link_->parent->parent) {
        urdf::Robot::forwardKinematics(link_->parent->parent);
    }
}

void AddCollisionCommand::undo() { link_->deleteCollision(link_->link.collision.size() - 1); }

DeleteCollisionCommand::DeleteCollisionCommand(urdf::LinkNodePtr& link, int i)
    : link_(link), i_(i) {}

void DeleteCollisionCommand::execute() {
    old_collision_ = link_->link.collision[i_];
    link_->deleteCollision(i_);
}

void DeleteCollisionCommand::undo() {
    link_->link.collision.insert(link_->link.collision.begin() + i_, old_collision_);
    link_->collision_models.insert(link_->collision_models.begin() + i_,
                                   old_collision_.geometry.type->generateGeometry());
}

AddMaterialCommand::AddMaterialCommand(urdf::RobotPtr& robot, urdf::Material material)
    : robot_(robot), material_(std::move(material)) {}

void AddMaterialCommand::execute() { robot_->getMutableMaterials()[material_.name] = material_; }

void AddMaterialCommand::undo() { robot_->getMutableMaterials().erase(material_.name); }

DeleteMaterialCommand::DeleteMaterialCommand(urdf::RobotPtr& robot, const std::string& name)
    : robot_(robot), name_(name) {}

void DeleteMaterialCommand::execute() {
    auto& materials = robot_->getMutableMaterials();
    auto it = materials.find(name_);
    if (it != materials.end()) {
        saved_material_ = it->second;
        materials.erase(it);
    }
    // Update visuals so affected links fall back to default grey
    robot_->forEveryLink([&](const urdf::LinkNodePtr& link) {
        for (const auto& vis : link->link.visual) {
            if (vis.material_name && *vis.material_name == name_) {
                robot_->updateMaterial(link);
                break;
            }
        }
    });
}

void DeleteMaterialCommand::undo() {
    robot_->getMutableMaterials()[saved_material_.name] = saved_material_;
    // Restore the material color on affected links
    robot_->forEveryLink([&](const urdf::LinkNodePtr& link) {
        for (const auto& vis : link->link.visual) {
            if (vis.material_name && *vis.material_name == name_) {
                robot_->updateMaterial(link);
                break;
            }
        }
    });
}

RenameLinkCommand::RenameLinkCommand(urdf::RobotPtr& robot, urdf::LinkNodePtr link,
                                     const std::string& old_name, const std::string& new_name)
    : robot_(robot), link_(std::move(link)), old_name_(old_name), new_name_(new_name) {}

void RenameLinkCommand::renameLink(const std::string& from, const std::string& to) {
    link_->link.name = to;
    robot_->forEveryJoint([&](const urdf::JointNodePtr& joint) {
        if (joint->joint.parent == from) joint->joint.parent = to;
        if (joint->joint.child == from) joint->joint.child = to;
    });
}

void RenameLinkCommand::execute() { renameLink(old_name_, new_name_); }

void RenameLinkCommand::undo() { renameLink(new_name_, old_name_); }

RenameMaterialCommand::RenameMaterialCommand(urdf::RobotPtr& robot, const std::string& old_name,
                                             const std::string& new_name)
    : robot_(robot), old_name_(old_name), new_name_(new_name) {}

void RenameMaterialCommand::renameMaterial(const std::string& from, const std::string& to) {
    auto& materials = robot_->getMutableMaterials();
    auto it = materials.find(from);
    if (it == materials.end()) return;

    urdf::Material mat = it->second;
    mat.name = to;
    materials.erase(it);
    materials[to] = mat;

    robot_->forEveryLink([&](const urdf::LinkNodePtr& link) {
        for (auto& vis : link->link.visual) {
            if (vis.material_name && *vis.material_name == from) {
                vis.material_name = to;
            }
        }
        robot_->updateMaterial(link);
    });
}

void RenameMaterialCommand::execute() { renameMaterial(old_name_, new_name_); }

void RenameMaterialCommand::undo() { renameMaterial(new_name_, old_name_); }

CloneSubtreeCommand::CloneSubtreeCommand(urdf::LinkNodePtr source, urdf::RobotPtr& robot,
                                         const Shader& shader)
    : source_(std::move(source)), robot_(robot), shader_(shader) {}

void CloneSubtreeCommand::execute() {
    if (!source_->parent) return;
    attach_parent_ = source_->parent->parent;

    if (cloned_joint_) {
        // Re-execute (redo): re-insert the previously cloned joint
        attach_parent_->children.push_back(cloned_joint_);
    } else {
        cloned_joint_ = urdf::cloneSubtree(source_, attach_parent_, robot_);
        if (!cloned_joint_) return;
        attach_parent_->children.push_back(cloned_joint_);

        // Generate geometry for all cloned links
        std::deque<urdf::LinkNodePtr> deq{cloned_joint_->child};
        while (!deq.empty()) {
            auto link = deq.front();
            deq.pop_front();

            for (const auto& vis : link->link.visual) {
                Model model = vis.geometry.type->generateGeometry();
                model.materials[0].shader = shader_;
                link->visual_models.push_back(model);
            }
            robot_->updateMaterial(link);
            for (const auto& col : link->link.collision) {
                link->collision_models.push_back(col.geometry.type->generateGeometry());
            }
            for (const auto& j : link->children) {
                deq.push_back(j->child);
            }
        }
    }

    robot_->forwardKinematics();
}

void CloneSubtreeCommand::undo() {
    if (!cloned_joint_ || !attach_parent_) return;
    auto& children = attach_parent_->children;
    children.erase(std::find(children.begin(), children.end(), cloned_joint_));
    robot_->forwardKinematics();
}

UpdateGeometryBoxCommand::UpdateGeometryBoxCommand(std::shared_ptr<urdf::Box>& box,
                                                   const Vector3& old_size, Model& model,
                                                   const Shader& shader)
    : new_size_(box->size), old_size_(old_size), box_(box), model_(model), shader_(shader) {}

void UpdateGeometryBoxCommand::execute() {
    MaterialMap mat_map = model_.materials[0].maps[MATERIAL_MAP_DIFFUSE];
    const Matrix t = model_.transform;
    box_->size = new_size_;

    UnloadModel(model_);
    model_ = Model{};
    model_ = box_->generateGeometry();

    model_.materials[0].shader = shader_;
    model_.transform = t;
    model_.materials[0].maps[MATERIAL_MAP_DIFFUSE] = mat_map;
}

void UpdateGeometryBoxCommand::undo() {
    MaterialMap mat_map = model_.materials[0].maps[MATERIAL_MAP_DIFFUSE];
    const Matrix t = model_.transform;
    box_->size = old_size_;

    UnloadModel(model_);
    model_ = Model{};
    model_ = box_->generateGeometry();

    model_.materials[0].shader = shader_;
    model_.transform = t;
    model_.materials[0].maps[MATERIAL_MAP_DIFFUSE] = mat_map;
}

UpdateGeometryCylinderCommand::UpdateGeometryCylinderCommand(
    std::shared_ptr<urdf::Cylinder>& cylinder, const float old_radius, const float old_height,
    Model& model, const Shader& shader)
    : new_radius_(cylinder->radius),
      new_length_(cylinder->length),
      old_radius_(old_radius),
      old_length_(old_height),
      cylinder_(cylinder),
      model_(model),
      shader_(shader) {}

void UpdateGeometryCylinderCommand::execute() {
    MaterialMap mat_map = model_.materials[0].maps[MATERIAL_MAP_DIFFUSE];
    const Matrix t = model_.transform;
    cylinder_->radius = new_radius_;
    cylinder_->length = new_length_;

    UnloadModel(model_);
    model_ = Model{};
    model_ = cylinder_->generateGeometry();

    model_.materials[0].shader = shader_;
    model_.transform = t;
    model_.materials[0].maps[MATERIAL_MAP_DIFFUSE] = mat_map;
}

void UpdateGeometryCylinderCommand::undo() {
    MaterialMap mat_map = model_.materials[0].maps[MATERIAL_MAP_DIFFUSE];
    const Matrix t = model_.transform;
    cylinder_->radius = old_radius_;
    cylinder_->length = old_length_;

    UnloadModel(model_);
    model_ = Model{};
    model_ = cylinder_->generateGeometry();

    model_.materials[0].shader = shader_;
    model_.transform = t;
    model_.materials[0].maps[MATERIAL_MAP_DIFFUSE] = mat_map;
}

UpdateGeometrySphereCommand::UpdateGeometrySphereCommand(std::shared_ptr<urdf::Sphere>& sphere,
                                                         const float old_radius, Model& model,
                                                         const Shader& shader)
    : new_radius_(sphere->radius),
      old_radius_(old_radius),
      sphere_(sphere),
      model_(model),
      shader_(shader) {}

void UpdateGeometrySphereCommand::execute() {
    MaterialMap mat_map = model_.materials[0].maps[MATERIAL_MAP_DIFFUSE];
    const Matrix t = model_.transform;
    sphere_->radius = new_radius_;

    UnloadModel(model_);
    model_ = Model{};
    model_ = sphere_->generateGeometry();

    model_.materials[0].shader = shader_;
    model_.transform = t;
    model_.materials[0].maps[MATERIAL_MAP_DIFFUSE] = mat_map;
}

void UpdateGeometrySphereCommand::undo() {
    MaterialMap mat_map = model_.materials[0].maps[MATERIAL_MAP_DIFFUSE];
    const Matrix t = model_.transform;
    sphere_->radius = old_radius_;

    UnloadModel(model_);
    model_ = sphere_->generateGeometry();

    model_.materials[0].shader = shader_;
    model_.transform = t;
    model_.materials[0].maps[MATERIAL_MAP_DIFFUSE] = mat_map;
}

UpdateGeometryMeshCommand::UpdateGeometryMeshCommand(std::shared_ptr<urdf::Mesh>& mesh,
                                                     const std::string& new_filename, Model& model,
                                                     const Shader& shader)
    : new_filename_(new_filename),
      old_filename_(mesh->filename),
      old_resolved_path_(mesh->resolved_path),
      mesh_(mesh),
      model_(model),
      shader_(shader) {}

void UpdateGeometryMeshCommand::execute() {
    MaterialMap mat_map = model_.materials[0].maps[MATERIAL_MAP_DIFFUSE];
    const Matrix t = model_.transform;
    mesh_->filename = new_filename_;
    mesh_->resolved_path = new_filename_;

    UnloadModel(model_);
    model_ = Model{};
    if (std::filesystem::exists(mesh_->resolved_path)) {
        model_ = mesh_->generateGeometry();
    }

    model_.materials[0].shader = shader_;
    model_.transform = t;
    model_.materials[0].maps[MATERIAL_MAP_DIFFUSE] = mat_map;
}

void UpdateGeometryMeshCommand::undo() {
    MaterialMap mat_map = model_.materials[0].maps[MATERIAL_MAP_DIFFUSE];
    const Matrix t = model_.transform;
    mesh_->filename = old_filename_;
    mesh_->resolved_path = old_resolved_path_;

    UnloadModel(model_);
    model_ = mesh_->generateGeometry();

    model_.materials[0].shader = shader_;
    model_.transform = t;
    model_.materials[0].maps[MATERIAL_MAP_DIFFUSE] = mat_map;
}
