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