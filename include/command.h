#pragma once

#include <string>
#include <robot.h>

class Command {
    public:
        virtual ~Command() = default;
        virtual void execute() = 0;
        virtual void undo() = 0;
};

using CommandPtr = std::shared_ptr<Command>;

class CommandBuffer
{
public:
    CommandBuffer();

    void add(CommandPtr command);
    void execute();
    void undo();
    void redo();
private:
    std::vector<CommandPtr> executed_commands_;
    std::vector<CommandPtr> new_commands_;
    size_t current_command_;
};

class LoadRobotCommand : public Command {
public:
    LoadRobotCommand(const std::string& filename,
                     std::shared_ptr<urdf::Robot>& robot,
                     Shader& shader);
    void execute() override;
    void undo() override;
private:
    const std::string filename_;
    std::shared_ptr<urdf::Robot>& robot_;
    std::shared_ptr<urdf::Robot> old_robot_;
    const Shader& shader_;
};

class ChangeNameCommand : public Command {
public:
    ChangeNameCommand(const std::string& old_name,
                      const std::string& new_name,
                      std::string& target);
    void execute() override;
    void undo() override;
private:
    const std::string old_name_;
    const std::string new_name_;
    std::string& target_;
};

class CreateNameCommand : public Command {
public:
    CreateNameCommand(std::optional<std::string>& target);
    void execute() override;
    void undo() override;

private:
    std::optional<std::string>& target_;
};

class CreateOriginCommand : public Command {
public:
    CreateOriginCommand(std::optional<urdf::Origin>& target);
    void execute() override;
    void undo() override;
private:
    std::optional<urdf::Origin>& target_;
};

class CreateAxisCommand : public Command {
public:
    CreateAxisCommand(std::optional<urdf::Axis>& target);
    void execute() override;
    void undo() override;
private:
    std::optional<urdf::Axis>& target_;
};

class CreateDynamicsCommand : public Command {
public:
    CreateDynamicsCommand(std::optional<urdf::Dynamics>& target);
    void execute() override;
    void undo() override;
private:
    std::optional<urdf::Dynamics>& target_;
};

class CreateLimitCommand : public Command {
public:
    CreateLimitCommand(std::optional<urdf::Limit>& target);
    void execute() override;
    void undo() override;
private:
    std::optional<urdf::Limit>& target_;
};

class ChangeGeometryCommand : public Command {
public:
    ChangeGeometryCommand(urdf::GeometryTypePtr old_geometry,
                          urdf::GeometryTypePtr new_geometry,
                          urdf::Geometry& target,
                          ::Mesh& mesh,
                          Model& model);
    void execute() override;
    void undo() override;
private:
    urdf::GeometryTypePtr old_geometry_;
    urdf::GeometryTypePtr new_geometry_;
    urdf::Geometry& target_;
    ::Mesh& mesh_;
    Model& model_;
};
