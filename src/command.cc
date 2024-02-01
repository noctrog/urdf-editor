#include <deque>
#include <algorithm>

#include <fmt/format.h>
#include <robot.h>
#include <command.h>

CommandBuffer::CommandBuffer()
    : executed_commands_(), new_commands_(), current_command_(0)
{

}

void CommandBuffer::add(CommandPtr command)
{
    new_commands_.push_back(command);
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

bool CommandBuffer::can_undo()
{
    return executed_commands_.begin() + current_command_ != executed_commands_.begin();
}

bool CommandBuffer::can_redo()
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
    robot_->set_shader(shader_);
}

void CreateRobotCommand::undo()
{
    robot_ = old_robot_;
}

LoadRobotCommand::LoadRobotCommand(const std::string& filename,
                                   std::shared_ptr<urdf::Robot>& robot,
                                   Shader& shader)
    : filename_(filename), robot_(robot), shader_(shader)
{

}

void LoadRobotCommand::execute()
{
    old_robot_ = robot_;
    robot_ = urdf::build_robot(filename_.c_str());
    robot_->build_geometry();
    robot_->forward_kinematics();
    robot_->set_shader(shader_);
}

void LoadRobotCommand::undo()
{
    robot_ = old_robot_;
}

JointChangeParentCommand::JointChangeParentCommand(urdf::JointNodePtr node,
                                                   urdf::LinkNodePtr new_parent,
                                                   urdf::RobotPtr robot)
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
    robot_->forward_kinematics();
}

void JointChangeParentCommand::undo()
{
    joint_->parent->children.erase(std::find(joint_->parent->children.begin(),
                                             joint_->parent->children.end(),
                                             joint_));
    old_parent_->children.insert(old_position_, joint_);

    joint_->parent = old_parent_;
    joint_->joint.parent = old_parent_->link.name;
    robot_->forward_kinematics();
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
    std::deque<urdf::LinkNodePtr> deq;
    while (not deq.empty()) {
        const auto& current_link = deq.front();
        deq.pop_front();

        new_link_count += current_link->link.name.find("New link") != std::string::npos;

        for (const auto& joint : current_link->children) {
            new_joint_count += joint->joint.name.find(joint_name_) != std::string::npos;
            deq.push_back(joint->child);
        }
    }

    // Rename in case of name collisions
    if (new_joint_count > 0) joint_name_ += fmt::format(" {}", new_joint_count);

    std::string new_link_name = (new_link_count == 0) ? "New link" : fmt::format("New link {}", new_link_count);
    urdf::LinkNodePtr child = std::make_shared<urdf::LinkNode>(urdf::Link{ new_link_name }, nullptr);

    // Create new joint and link
    urdf::JointNodePtr new_joint_ = std::make_shared<urdf::JointNode>(urdf::JointNode {
        urdf::Joint(joint_name_.c_str(), parent_->link.name.c_str(), new_link_name.c_str(), "fixed"),
        parent_,
        child});

    child->parent = new_joint_;

    parent_->children.push_back(new_joint_);
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
    target_ = urdf::Limit(0.0f, 1.0f, 1.0f, 1.0f);
}

void CreateLimitCommand::undo()
{
    target_ = std::nullopt;
}

ChangeGeometryCommand::ChangeGeometryCommand(urdf::GeometryTypePtr old_geometry,
                                             urdf::GeometryTypePtr new_geometry,
                                             urdf::Geometry& target,
                                             ::Mesh& mesh,
                                             Model& model)
    : old_geometry_(old_geometry), new_geometry_(new_geometry), target_(target), mesh_(mesh), model_(model)
{

}

void ChangeGeometryCommand::execute()
{
    target_.type = new_geometry_;

    UnloadMesh(mesh_);
    mesh_ = target_.type->generateGeometry();
    model_.meshes[0] = mesh_;
}

void ChangeGeometryCommand::undo()
{
    target_.type = old_geometry_;

    UnloadMesh(mesh_);
    mesh_ = target_.type->generateGeometry();
    model_.meshes[0] = mesh_;
}
