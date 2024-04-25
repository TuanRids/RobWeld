#pragma once
namespace ncommand
{
    // Forward declaration c?a class Command
    class Command {
    public:
        virtual void execute() = 0;
        virtual void undo() = 0;
        virtual void redo() = 0;
        virtual ~Command() = default;
    };
}