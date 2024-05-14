// MoveCommand.h
#pragma once
#include "Command/dcommand.h"
#include "elems/mesh.h"
#include <glm/glm.hpp>
#include <sstream>
#include <chrono>
#include <iomanip> 
namespace ncommand
{
    class MoveOb : public Command {
    private:
        nelems::oMesh* mesh;
        nelems::mMesh* proMesh;
        float mx,my,mz;
    public:
        MoveOb(nelems::mMesh* obj, float x, float y, float z) : proMesh(obj),
            mesh(nullptr),mx(x),my(y),mz(z) {}
        ~MoveOb() { mesh = nullptr; proMesh = nullptr; }
        void execute(std::deque<std::string>& cmdlogs) override
        {
            std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now(); // Reset startTime
            // Calculate displacement vector
            std::string tempLog;
            for (int i = 0; i < proMesh->size(); i++)
            {
                proMesh->get_mesh_ptr(i, mesh);
                if (mesh->selected)
                {
                    mesh->move(mx,my,mz);

                    // Update buffers
                    mesh->create_buffers();
                }
            }
            if (proMesh->check_selected()>0){ recorded_cmdlogs(mesh, tempLog, startTime); }
            if (tempLog.size() > 0)
            {
                cmdlogs.push_back(tempLog);
            }
        }
        bool isValid() const override {return (mx != 0.0f || my != 0.0f || mz != 0.0f);}
        void recorded_cmdlogs(nelems::oMesh* mesh, std::string& tempLog, const std::chrono::steady_clock::time_point startTime) {
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
            oss << "Move Object: \t"
                << "Displacement Vector: (" << mx << ", " << my << ", " << mz << ")];\t["
                << "Duration Time (ns): " << durationTime.count() << "];\t["
                << "Current Time: " << currentTime << "]\n";
            std::string logEntry = oss.str();
            tempLog += logEntry;
        }

        void undo(const std::string& lastlog) override {
            if (!lastlog.empty() && lastlog.find("Move Object") != std::string::npos) {
                // Update vertex positions
                for (int i = 0; i < proMesh->size(); i++) {
                    proMesh->get_mesh_ptr(i, mesh);
                    if (lastlog.find(std::to_string(mesh->ID)) != std::string::npos) {
                        mesh->move(-mx, -my, -mz);
                        mesh->create_buffers();
                    }
                }
            }
        }

        void redo(const std::string& lastlog) override
        {
            for (int i = 0; i < proMesh->size(); i++) {
                proMesh->get_mesh_ptr(i, mesh);
                if (lastlog.find(std::to_string(mesh->ID)) != std::string::npos)
                {mesh->selected = true;}
                else { mesh->selected = false; }
            }
        }
    };


    class RotateOb : public Command
    {
    private:
        nelems::oMesh* mesh;
        nelems::mMesh* proMesh;
        float rx, ry, rz;
    public:
        RotateOb(nelems::mMesh* obj, float x, float y, float z) : mesh(nullptr),rx(x),ry(y),rz(z),proMesh(obj)  {}
        ~RotateOb() { mesh = nullptr; proMesh = nullptr; }
        void execute(std::deque<std::string>& cmdlogs) override {
            std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now(); // Reset startTime
            // Calculate displacement vector
            std::string tempLog;
            for (int i = 0; i < proMesh->size(); i++)
            {
                proMesh->get_mesh_ptr(i, mesh);
                if (mesh->selected)
                {
                    mesh->rotate(rx, ry, rz);
                    // Update buffers
                    mesh->create_buffers();
                }
            }
            if (proMesh->check_selected() > 0) { recorded_cmdlogs(mesh, tempLog, startTime); }
            if (tempLog.size() > 0)
            {
                cmdlogs.push_back(tempLog);
            }
        }
        bool isValid() const override { return (rx != 0.0f || ry != 0.0f || rz != 0.0f); }
        void recorded_cmdlogs(nelems::oMesh* mesh, std::string& tempLog, const std::chrono::steady_clock::time_point startTime) {
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
            oss << "Rotate Object: \t"
                << "Rotation degree: (" << rx << ", " << ry << ", " << rz << ")];\t["
                << "Duration Time (ns): " << durationTime.count() << "];\t["
                << "Current Time: " << currentTime << "]\n";
            std::string logEntry = oss.str();
            tempLog += logEntry;
        }

        void undo(const std::string& lastlog) override 
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
        void redo(const std::string& lastlog) override 
        {
            for (int i = 0; i < proMesh->size(); i++) {
                proMesh->get_mesh_ptr(i, mesh);
                if (lastlog.find(std::to_string(mesh->ID)) != std::string::npos)
                { mesh->selected = true; }
                else { mesh->selected = false; }
            }
        }
    };
}
