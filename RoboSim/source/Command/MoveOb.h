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
        glm::vec3 originalPosition;
        glm::vec3 MoveVec3;
    public:
        MoveOb(nelems::mMesh* obj, float x, float y, float z) : proMesh(obj), originalPosition(0,0,0) {
            MoveVec3 = glm::vec3(x, y, z);
        }
        ~MoveOb() { mesh = nullptr; proMesh = nullptr; }
        void execute(std::deque<std::string>& cmdlogs) override 
        {
            std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now(); // Reset startTime
            // Calculate displacement vector
            glm::vec3 displacementVector = MoveVec3;
            std::string tempLog = "";
            for (int i = 0; i < proMesh->size(); i++)
            {
				proMesh->get_mesh_ptr(i, mesh);
                if (mesh->selected)
                {
					originalPosition = mesh->mVertices[0].mPos;
					// Update vertex positions
                    for (auto& vertex : mesh->mVertices) {
						vertex.mPos += displacementVector;
					}
					// Update buffers
					mesh->create_buffers();
					recorded_cmdlogs(mesh, tempLog, startTime);
				}
			}
            cmdlogs.push_back(tempLog);

            
        }
        
        void recorded_cmdlogs(nelems::oMesh* mesh,std::string& tempLog, const std::chrono::steady_clock::time_point startTime) {
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
                << "Object ID: " << mesh->ID << "];\t["
                << "Object Name: " << mesh->oname << "];\t["
                << "Original Position: (" << originalPosition.x << ", " << originalPosition.y << ", " << originalPosition.z << ")];\t["
                << "Displacement Vector: (" << MoveVec3.x << ", " << MoveVec3.y << ", " << MoveVec3.z << ")];\t["
                << "Duration Time (ns): " << durationTime.count() << "];\t["
                << "Current Time: " << currentTime << "]\n";
            std::string logEntry = oss.str();
            tempLog+= logEntry;
        }

        void undo() override {
            // Restore original position
            glm::vec3 displacementVector = MoveVec3;
            // Update vertex positions
            for (int i = 0; i < proMesh->size(); i++)
            {
                proMesh->get_mesh_ptr(i, mesh);
                if (mesh->selected)
                {
                    originalPosition = mesh->mVertices[0].mPos;
                    // Update vertex positions
                    for (auto& vertex : mesh->mVertices) {
                        vertex.mPos -= displacementVector;
                    }
                    // Update buffers
                    mesh->create_buffers();
                }
            }
        }
        void redo() override {}
    };


    class RotateOb : public Command
    {
    private:
        nelems::oMesh* mesh;
        glm::vec3 rotVec3;
    public:
        RotateOb(nelems::oMesh* obj, float x) : mesh(obj) {
            rotVec3 = glm::vec3(x);
        }
        void execute(std::deque<std::string>& cmdlogs) override {}
        void undo() override {}
        void redo() override {}
    };
}
