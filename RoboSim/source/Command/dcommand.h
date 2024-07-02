#pragma once
#include <vector>
#include <deque>
#include <string>
namespace ncommand
{
    // Forward declaration c?a class Command
    class Command {
    public:
        virtual void execute(std::deque<std::string> &cmdIDs, int reverse, std::deque<std::string> &cmdIDs_redo) = 0;
        virtual void undo(const std::string& lastlog, std::deque<std::string> cmdIDs) = 0;
        virtual void redo(const std::string& lastlog, std::deque<std::string> cmdIDs) = 0;
        virtual bool isValid() const { return true; } 
        virtual ~Command() = default;
        
    };
}