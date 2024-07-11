// MoveCommand.cpp
#include "pch.h"
#include "Transform.h"
// Move Command
namespace ncommand
{
    void MoveOb::execute( std::deque<std::string> &cmdIDs, int reverse, std::deque<std::string> &cmdIDs_redo)
    {
        if (proMesh->check_selected() == 0 && reverse) { return; }
        std::string tempLog, list_ids, lastIDs;
        if (reverse == 1) { lastIDs = cmdIDs.back(); }
        else if (reverse == 2){ lastIDs = cmdIDs_redo.back(); }
        for (auto it = proMesh->getMesh()->begin(); it != proMesh->getMesh()->end(); it++)
        {
            auto mesh = *it;
            if (reverse == 0)
            {
                if (!(mesh->selected)) { continue; }
                mesh->move(mx, my, mz);
                cmdIDs.push_back(std::to_string(mesh->ID));
                cmdIDs_redo.clear();
                mesh->create_buffers();

                *sttlogs << "Move Displacement vector: (" + std::to_string(mx) + ", "+ std::to_string(my) + ", " +std::to_string(mz )+ ");";

            }
            else if (reverse == 1) // undo
            {
                if (lastIDs.find(std::to_string(mesh->ID)) != std::string::npos)
                {
                    mesh->move(-mx, -my, -mz);
                    mesh->create_buffers();
                    cmdIDs.pop_back();
                    cmdIDs_redo.push_back(lastIDs);
                    *sttlogs << "Undo Move Displacement vector: (" + std::to_string(mx) + ", " + std::to_string(my) + ", " + std::to_string(mz) + ");";
                }
			}   
            else if (reverse == 2) // redo
            {
                if (lastIDs.find(std::to_string(mesh->ID)) != std::string::npos)
                {
                    mesh->move(mx, my, mz);
                    mesh->create_buffers();
                    cmdIDs.push_back(lastIDs);
                    cmdIDs_redo.pop_back();
                    *sttlogs << "Redo Move Displacement vector: (" + std::to_string(mx) + ", " + std::to_string(my) + ", " + std::to_string(mz) + ");";
                }
            }
        }
    }
}

// Rotate Command
namespace ncommand
{
    void RotateOb::execute( std::deque<std::string> &cmdIDs, int reverse, std::deque<std::string> &cmdIDs_redo)
    {
        if (proMesh->check_selected() == 0 && reverse) { return; }
        std::string tempLog, list_ids, lastIDs;
        if (reverse == 1) { lastIDs = cmdIDs.back(); }
        else if (reverse == 2) { lastIDs = cmdIDs_redo.back(); }
        for (auto it = proMesh->getMesh()->begin(); it != proMesh->getMesh()->end(); it++)
        {
            auto mesh = *it;
            if (reverse == 0)
            {
                if (!(mesh->selected)) { continue; }
                mesh->rotate(rx, ry, rz);
                cmdIDs.push_back(std::to_string(mesh->ID));
                cmdIDs_redo.clear();
                mesh->create_buffers();

                *sttlogs << "Rotate Rotation vector: (" + std::to_string(rx) + ", " + std::to_string(ry) + ", " + std::to_string(rz) + ");";

            }
            else if (reverse == 1) // undo
            {
                if (lastIDs.find(std::to_string(mesh->ID)) != std::string::npos)
                {
                    mesh->rotate(-rx, -ry, -rz);
                    mesh->create_buffers();
                    cmdIDs.pop_back();
                    cmdIDs_redo.push_back(lastIDs);
                    *sttlogs << "Undo Rotate Rotation vector: (" + std::to_string(rx) + ", " + std::to_string(ry) + ", " + std::to_string(rz) + ");";

                }
            }
            else if (reverse == 2) // redo
            {
                if (lastIDs.find(std::to_string(mesh->ID)) != std::string::npos)
                {
                    mesh->rotate(rx, ry, rz);
                    mesh->create_buffers();
                    cmdIDs.push_back(lastIDs);
                    cmdIDs_redo.pop_back();
                    *sttlogs << "Redo Rotate Rotation vector: (" + std::to_string(rx) + ", " + std::to_string(ry) + ", " + std::to_string(rz) + ");";

                }
            }
        }
    }
}

// Del command
namespace ncommand
{
    void delOb::execute( std::deque<std::string> &cmdIDs, int reverse, std::deque<std::string> &cmdIDs_redo)
    {

        if (proMesh->check_selected() > 0)
        {
            auto mMeshes = proMesh->getMesh();

            for (auto& mesh : *mMeshes) {
                if (mesh->selected) {
                    mesh->delete_buffers();
                }
            }
            mMeshes->erase(std::remove_if(mMeshes->begin(), mMeshes->end(),
                [](const std::shared_ptr<nelems::oMesh>& mesh) { return mesh->selected; }), mMeshes->end());

            *sttlogs << "Delete Object.";
        }
    }
}