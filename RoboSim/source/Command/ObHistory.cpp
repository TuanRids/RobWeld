#include "pch.h"
#include "ObHistory.h"

namespace ncommand
{
    void ObHistory::execmd(std::unique_ptr<Command> cmd) {
        if (cmd && cmd->isValid()) {
            cmd->execute( cmdIDs,0, cmdIDs_redo);
            // Store the command in the command stack
            commandStack.push_back(std::move(cmd));
            redoStack.clear(); // Clear redo stack when executing new command
            if (commandStack.size() > limhit) {
                commandStack.front().reset(); // FOCUS: Reset to avoid Memory Leaks 
                commandStack.pop_front(); // Remove oldest command if stack size exceeds limit
            }
        }
    }
    void ObHistory::undocmd() {
        if (!commandStack.empty()) {
            if (commandStack.empty()) {return;}
            auto lastCmd = std::move(commandStack.back());
            lastCmd->execute(cmdIDs,1, cmdIDs_redo);
            redoStack.push_back(std::move(lastCmd));
            commandStack.pop_back();
        }
    }
    void ObHistory::redocmd() {
        if (!redoStack.empty()) {
            auto lastRedoCmd = std::move(redoStack.back());
            lastRedoCmd->execute( cmdIDs,2, cmdIDs_redo);
            commandStack.push_back(std::move(lastRedoCmd));
            redoStack.pop_back();
        }
    }
}
