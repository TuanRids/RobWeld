#pragma once
#include "pch.h"


#include "elems/mesh.h"
#include <vector>
#include <deque>
#include <memory>
#include <mutex>
#include "imgui.h"
#include <typeinfo> // Include for typeid
#include "Command/dcommand.h"
#include "Command/Transform.h"
#include "render/ui_context.h"
namespace ncommand
{
    class ObHistory {
    private:
        const int limhit = 30; // Maximum history that can be stored
        std::deque<std::unique_ptr<Command>> commandStack; // Store the command that has been executed
        std::deque<std::unique_ptr<Command>> redoStack; // Store the command that has been undone
        std::deque<std::string> cmdlogs; // Store the command logs
        int count{ 0 };
        // Private constructor

        std::string theme;
        ObHistory() { cmdlogs.push_back("Welcome to RoboSim. This Application is developed by OhLab."); }

    public:
        // Delete copy constructor and assignment operator
        ObHistory(const ObHistory&) = delete;
        ObHistory& operator=(const ObHistory&) = delete;

        // Static method to get the instance
        static ObHistory& getInstance() { static ObHistory instance; return instance; }

        void execmd(std::unique_ptr<Command> cmd);
        // Undo the command
        void undocmd();
        // Redo the command
        void redocmd();
        void Command_Logs();

	};
}