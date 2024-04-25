#pragma once
#include "pch.h"

#include "Command/dcommand.h"
#include "elems/mesh.h"
#include <vector>
#include <deque>
#include <memory>
#include <mutex>
#include <deque>


namespace ncommand
{

    
    class ObHistory {
    private:
        const int limhit = 10; // Maximum history that can be stored
        std::deque<std::unique_ptr<Command>> commandStack; // Store the command that has been executed
        std::deque<std::unique_ptr<Command>> redoStack; // Store the command that has been undone


    public:
        
        // Execute the command
        void execmd(std::unique_ptr<Command> cmd) {
            if (cmd) {
                cmd->execute();
                // Store the command in the command stack
                commandStack.push_back(std::move(cmd));
                redoStack.clear(); // Clear redo stack when executing new command

                if (commandStack.size() > limhit) {
                    commandStack.front().reset(); // FOCUS: Reset to avoid Memory Leaks 
                    commandStack.pop_front(); // Remove oldest command if stack size exceeds limit
                }
            }
        }

        // Undo the command
        void undocmd() {
            if (!commandStack.empty()) {
                auto lastCmd = std::move(commandStack.back());
                lastCmd->undo();

                redoStack.push_back(std::move(lastCmd));
                commandStack.pop_back();
            }
        }

        // Redo the command
        void redocmd() {
            if (!redoStack.empty()  ) {
                auto lastRedoCmd = std::move(redoStack.back());
                // Call execute 
                lastRedoCmd->execute();
                commandStack.push_back(std::move(lastRedoCmd));
                redoStack.pop_back();
            }
        }
    };



}
