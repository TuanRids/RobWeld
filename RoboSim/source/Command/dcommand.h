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
        virtual bool isValid() const { return true; } 
        virtual ~Command() = default;
        
    };
}