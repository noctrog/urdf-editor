#pragma once


class Command
{
    virtual ~Command() = default;
    virtual void execute() = 0;
    virtual void undo() = 0;
};
