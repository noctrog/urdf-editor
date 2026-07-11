#pragma once

// Legacy declaration of the reversible-command interface.
//
// New code should include command.h, which also defines the concrete editor
// commands and the command buffer.
class Command {
    virtual ~Command() = default;
    // Applies this command.
    virtual void execute() = 0;
    // Restores the state that existed before execute().
    virtual void undo() = 0;
};
