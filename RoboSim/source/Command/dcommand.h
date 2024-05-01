#pragma once
#include <vector>
#include <deque>
#include <string>
namespace ncommand
{
    // Forward declaration c?a class Command
    class Command {
    public:
        virtual void execute(std::deque<std::string> &cmdlogs) = 0;
        virtual void undo(const std::string& lastlog) = 0;
        virtual void redo(const std::string& lastlog) = 0;
        virtual bool isValid() const { return true; } 
        virtual ~Command() = default;
        
    };
}