#pragma once
#include <vector>
#include <deque>

namespace ncommand
{
    // Forward declaration c?a class Command
    class Command {
    public:
        virtual void execute(std::deque<std::string> &cmdlogs) = 0;
        virtual void undo() = 0;
        virtual void redo() = 0;
        virtual ~Command() = default;
        
    };
}