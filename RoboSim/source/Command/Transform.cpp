// MoveCommand.cpp
#include "pch.h"
#include "Transform.h"

// Move Command
namespace ncommand
{
    void MoveOb::execute(std::deque<std::string>& cmdlogs, std::deque<std::string> &cmdIDs, int reverse, std::deque<std::string> &cmdIDs_redo)
    {
        if (proMesh->check_selected() == 0 && reverse) { return; }
        std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now(); // Reset startTime
        // Calculate displacement vector
        std::string tempLog, list_ids, lastIDs;
        if (reverse == 1) { lastIDs = cmdIDs.back(); }
        else if (reverse == 2){ lastIDs = cmdIDs_redo.back(); }
        for (int i = 0; i < proMesh->size(); i++)
        {
            proMesh->get_mesh_ptr(i, mesh);
            if (reverse == 0)
            {
                if (!(mesh->selected)) { continue; }
                mesh->move(mx, my, mz);
                list_ids += std::to_string(mesh->ID) + ", ";
                mesh->create_buffers();
            }
            else if (reverse == 1) // undo
            {
                if (lastIDs.find(std::to_string(mesh->ID)) != std::string::npos)
                {
                    mesh->move(-mx, -my, -mz);
                    mesh->create_buffers();
                }
			}   
            else if (reverse == 2) // redo
            {
                if (lastIDs.find(std::to_string(mesh->ID)) != std::string::npos)
                {
                    mesh->move(mx, my, mz);
                    mesh->create_buffers();
                }
            }
        }
        recorded_cmdlogs(tempLog, startTime);
        size_t pos = tempLog.find("]:");
        if (reverse == 0)
        {
            cmdlogs.push_back(tempLog);
            cmdIDs.push_back(list_ids);
            cmdIDs_redo.clear();
        }
        else if (reverse == 1) // undo, give lastIDs for cmdIDs_redo
        {
            // insert ~[Undo] after "]:" in tempLog
            tempLog.insert(pos + 2, "~[Undo] ");
            cmdlogs.push_back(tempLog);
            cmdIDs.pop_back();
            cmdIDs_redo.push_back(lastIDs);
        }
        else if (reverse == 2) // redo, give lastIDs for cmdIDs
        {
            tempLog.insert(pos + 2, " ~[Redo] ");
            cmdlogs.push_back(tempLog);
            cmdIDs.push_back(lastIDs);
            cmdIDs_redo.pop_back();
        }
    }
    void MoveOb::recorded_cmdlogs(std::string& tempLog, const std::chrono::steady_clock::time_point startTime)
    {
        std::string currentTime;
        std::chrono::nanoseconds durationTime;

        std::time_t currentTimeStamp = std::time(nullptr);
        std::tm currentTimeStruct;
        if (localtime_s(&currentTimeStruct, &currentTimeStamp) == 0) {
            std::ostringstream currentTimeSS;
            currentTimeSS << std::put_time(&currentTimeStruct, "%H-%M-%S");
            currentTime = currentTimeSS.str();
        }
        else {
            currentTime = "Failed to get current time";
        }
        // Calculate duration time
        durationTime = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - startTime);

        // recored logs
        std::ostringstream oss;
        oss << "[" << currentTime << "]: "
            << "Move Object. Displacement Vector: (" << mx << ", " << my << ", " << mz << ");"
            << "Duration Time - ns: (" << durationTime.count() << ")";
        std::string logEntry = oss.str();
        tempLog += logEntry;
    }
}

// Rotate Command
namespace ncommand
{

    void RotateOb::execute(std::deque<std::string>& cmdlogs, std::deque<std::string> &cmdIDs, int reverse, std::deque<std::string> &cmdIDs_redo)
    {
        if (proMesh->check_selected() == 0 && reverse) { return; }
        std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now(); // Reset startTime
        // Calculate displacement vector
        std::string tempLog, list_ids, lastIDs;
        if (reverse == 1) { lastIDs = cmdIDs.back(); }
        else if (reverse == 2) { lastIDs = cmdIDs_redo.back(); }
        for (int i = 0; i < proMesh->size(); i++)
        {
            proMesh->get_mesh_ptr(i, mesh);
            if (reverse == 0)
            {
                if (!(mesh->selected)) { continue; }
                mesh->rotate(rx, ry, rz);
                list_ids += std::to_string(mesh->ID) + ", ";
                mesh->create_buffers();
            }
            else if (reverse == 1) // undo
            {
                if (lastIDs.find(std::to_string(mesh->ID)) != std::string::npos)
                {
                    mesh->rotate(-rx, -ry, -rz);
                    mesh->create_buffers();
                }
            }
            else if (reverse == 2) // redo
            {
                if (lastIDs.find(std::to_string(mesh->ID)) != std::string::npos)
                {
                    mesh->rotate(rx, ry, rz);
                    mesh->create_buffers();
                }
            }
        }
        recorded_cmdlogs(tempLog, startTime);
        size_t pos = tempLog.find("]:");
        if (reverse == 0)
        {
            cmdlogs.push_back(tempLog);
            cmdIDs.push_back(list_ids);
            cmdIDs_redo.clear();
        }
        else if (reverse == 1) // undo, give lastIDs for cmdIDs_redo
        {
            // insert ~[Undo] after "]:" in tempLog
            tempLog.insert(pos + 2, "~[Undo] ");
            cmdlogs.push_back(tempLog);
            cmdIDs.pop_back();
            cmdIDs_redo.push_back(lastIDs);
        }
        else if (reverse == 2) // redo, give lastIDs for cmdIDs
        {
            tempLog.insert(pos + 2, " ~[Redo] ");
            cmdlogs.push_back(tempLog);
            cmdIDs.push_back(lastIDs);
            cmdIDs_redo.pop_back();
        }
    }
    void RotateOb::recorded_cmdlogs(std::string& tempLog, const std::chrono::steady_clock::time_point startTime)
    {
        std::string currentTime;
        std::chrono::nanoseconds durationTime;

        std::time_t currentTimeStamp = std::time(nullptr);
        std::tm currentTimeStruct;
        if (localtime_s(&currentTimeStruct, &currentTimeStamp) == 0) {
            std::ostringstream currentTimeSS;
            currentTimeSS << std::put_time(&currentTimeStruct, "%H-%M-%S");
            currentTime = currentTimeSS.str();
        }
        else {
            currentTime = "Failed to get current time";
        }
        // Calculate duration time
        durationTime = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - startTime);

        // recored logs
        std::ostringstream oss;
        oss << "[" << currentTime << "]: "
            << "Rotate Object. Rotation degree: (" << rx << ", " << ry << ", " << rz << ");"
            << "Duration Time - ns: (" << durationTime.count() << ")";

        std::string logEntry = oss.str();
        tempLog += logEntry;
    }
    void RotateOb::undo(const std::string& lastlog, std::deque<std::string> cmdIDs)
    {
        if (!lastlog.empty() && lastlog.find("Rotate Object") != std::string::npos) {
            // Update vertex positions
            for (int i = 0; i < proMesh->size(); i++) {
                proMesh->get_mesh_ptr(i, mesh);
                if (lastlog.find(std::to_string(mesh->ID)) != std::string::npos) {
                    mesh->rotate(-rx, -ry, -rz);
                    mesh->create_buffers();
                }
            }
        }
    }
    void RotateOb::redo(const std::string& lastlog, std::deque<std::string> cmdIDs)
    {
        for (int i = 0; i < proMesh->size(); i++) {
            proMesh->get_mesh_ptr(i, mesh);
            if (lastlog.find(std::to_string(mesh->ID)) != std::string::npos)
            { mesh->selected = true; }
            else { mesh->selected = false; }
        }
    }
}

// Del command
namespace ncommand
{
    void delOb::execute(std::deque<std::string>& cmdlogs, std::deque<std::string> &cmdIDs, int reverse, std::deque<std::string> &cmdIDs_redo)
    {
        std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now(); // Reset startTime
        // Calculate displacement vector
        std::string tempLog;
        if (proMesh->check_selected() > 0)
        {
            recorded_cmdlogs(tempLog, startTime);
            proMesh->delete_selected();
            cmdlogs.push_back(tempLog);
        }
    }
    void delOb::recorded_cmdlogs(std::string& tempLog, const std::chrono::steady_clock::time_point startTime)
    {
        std::string currentTime;
        std::chrono::nanoseconds durationTime;

        std::time_t currentTimeStamp = std::time(nullptr);
        std::tm currentTimeStruct;
        if (localtime_s(&currentTimeStruct, &currentTimeStamp) == 0) {
            std::ostringstream currentTimeSS;
            currentTimeSS << std::put_time(&currentTimeStruct, "%H-%M-%S");
            currentTime = currentTimeSS.str();
        }
        else {
            currentTime = "Failed to get current time";
        }
        // Calculate duration time
        durationTime = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - startTime);

        // recored logs
        std::ostringstream oss;
        oss << "[" << currentTime << "]: "
            << "Delete Object. Duration Time - ns: (" << durationTime.count() << ")";

        std::string logEntry = oss.str();
        tempLog += logEntry;
    }
    void delOb::undo(const std::string& lastlog, std::deque<std::string> cmdIDs)
    {
    }
    void delOb::redo(const std::string& lastlog, std::deque<std::string> cmdIDs)
    {
    }
}