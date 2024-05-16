#include "pch.h"
#include "ObHistory.h"

namespace ncommand
{
    void ObHistory::execmd(std::unique_ptr<Command> cmd) {
        if (cmd && cmd->isValid()) {
            size_t sizebf = cmdlogs.size();
            cmd->execute(cmdlogs);
            // Check if there are nothing new in the command logs
            if (sizebf == cmdlogs.size()) {
                return;
            }
            count++;
            // Store the command in the command stack
            commandStack.push_back(std::move(cmd));
            redoStack.clear(); // Clear redo stack when executing new command
            if (commandStack.size() > limhit) {
                commandStack.front().reset(); // FOCUS: Reset to avoid Memory Leaks 
                commandStack.pop_front(); // Remove oldest command if stack size exceeds limit
                cmdlogs.pop_front();
            }
        }
    }
    void ObHistory::undocmd() {
        if (!commandStack.empty()) {
            auto lastCmd = std::move(commandStack.back());
            lastCmd->undo(cmdlogs.empty() ? "" : cmdlogs.back());
            redoStack.push_back(std::move(lastCmd));
            commandStack.pop_back();
            if (!cmdlogs.empty()) {
                cmdlogs.pop_back();
            }
            count--;
        }
    }
    void ObHistory::redocmd() {
        // ERROR: REDO MULTIPLE OBJECFTS IS FAILING
        if (!redoStack.empty()) {
            auto lastRedoCmd = std::move(redoStack.back());
            // Call execute 
            lastRedoCmd->redo(cmdlogs.empty() ? "" : cmdlogs.back());
            lastRedoCmd->execute(cmdlogs);

            cmdlogs.back().insert(0, std::to_string(count) + " :");
            count++;
            commandStack.push_back(std::move(lastRedoCmd));
            redoStack.pop_back();
        }
    }
    void ObHistory::Command_Logs() {
        // Can move Begin to the parrent function for checking active viewport 
        // if this is necessary

        if (ImGui::Begin("Command Logs")) {
            ImVec2 frameSize = ImGui::GetContentRegionAvail();
            ImVec4 textcolor;
            nrender::UIContext::get_theme(theme);
            if (theme == "dark")
            {
                ImGui::GetWindowDrawList()->AddRectFilled(ImGui::GetWindowPos(), ImVec2(ImGui::GetWindowPos().x + frameSize.x + 100, ImGui::GetWindowPos().y + frameSize.y + 100), IM_COL32(0, 0, 0, 255));
                textcolor = ImVec4(1.0f, 0.6f, 0.6f, 1.0f);
            }
            else if (theme == "light")
            {
                ImGui::GetWindowDrawList()->AddRectFilled(ImGui::GetWindowPos(), ImVec2(ImGui::GetWindowPos().x + frameSize.x + 100, ImGui::GetWindowPos().y + frameSize.y + 100), IM_COL32(255, 255, 255, 255));
                textcolor = ImVec4(0.0f, 0.0f, 0.0f, 1.0f);
            }

            ImGui::BeginChild("ScrollingRegion", ImVec2(0, frameSize.y),
                false, ImGuiWindowFlags_HorizontalScrollbar |
                ImGuiWindowFlags_AlwaysVerticalScrollbar);
            //font
            if (cmdlogs.size() > limhit * 30) { cmdlogs.pop_front(); }
            for (int i = 0; i < cmdlogs.size(); i++) {
                // Title
                if (cmdlogs[i].find("to RoboSim") != -1)
                {
                    ImGui::TextColored(textcolor, cmdlogs[i].c_str());
                    continue;
                }
                // Content
                //ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.6f, 1.0f), "Command");
                //ImGui::SameLine();
                if (ImGui::Selectable(cmdlogs[i].c_str(), false, ImGuiSelectableFlags_AllowDoubleClick)) {
                    ImGui::SetClipboardText(cmdlogs[i].c_str());
                }
            }
            // TODO ADJUST AUTO SCROLL LATER
            ImGui::SetScrollHereY(1.0f);
            ImGui::EndChild();
            ImGui::End();
        }

    }
}
