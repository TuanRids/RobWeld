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
        std::shared_ptr<nelems::oMesh> mesh;
        nelems::mMesh* proMesh;
        float mx,my,mz;
    public:
        MoveOb(nelems::mMesh* obj, float x, float y, float z) : proMesh(obj),
            mesh(nullptr),mx(x),my(y),mz(z) {}
        ~MoveOb() { mesh = nullptr; proMesh = nullptr; }
        void execute(std::deque<std::string>& cmdlogs, std::deque<std::string> &cmdIDs, int reverse, std::deque<std::string> &cmdIDs_redo) override;
        bool isValid() const override {return (mx != 0.0f || my != 0.0f || mz != 0.0f);}
        void recorded_cmdlogs(std::string& tempLog, const std::chrono::steady_clock::time_point startTime) ;
        void undo(const std::string& lastlog, std::deque<std::string> cmdIDs) override {}
        void redo(const std::string& lastlog, std::deque<std::string> cmdIDs) override {}
    };

    class RotateOb : public Command
    {
    private:
        std::shared_ptr<nelems::oMesh> mesh;
        nelems::mMesh* proMesh;
        float rx, ry, rz;
    public:
        RotateOb(nelems::mMesh* obj, float x, float y, float z) : mesh(nullptr),rx(x),ry(y),rz(z),proMesh(obj)  {}
        ~RotateOb() { mesh = nullptr; proMesh = nullptr; }
        void execute(std::deque<std::string>& cmdlogs, std::deque<std::string> &cmdIDs, int reverse, std::deque<std::string> &cmdIDs_redo) override;
        bool isValid() const override { return (rx != 0.0f || ry != 0.0f || rz != 0.0f); }
        void recorded_cmdlogs(std::string& tempLog, const std::chrono::steady_clock::time_point startTime) ;
        void undo(const std::string& lastlog, std::deque<std::string> cmdIDs) override;
        void redo(const std::string& lastlog, std::deque<std::string> cmdIDs) override;
    };

    class delOb : public Command
    {
    private:
        nelems::oMesh* mesh;
        nelems::mMesh* proMesh;
    public:
        delOb(nelems::mMesh* obj) : proMesh(obj),
            mesh(nullptr) {}
        ~delOb() { mesh = nullptr; proMesh = nullptr; }
        void execute(std::deque<std::string>& cmdlogs, std::deque<std::string> &cmdIDs, int reverse, std::deque<std::string> &cmdIDs_redo) override;
        
        void recorded_cmdlogs(std::string& tempLog, const std::chrono::steady_clock::time_point startTime) ;

        void undo(const std::string& lastlog, std::deque<std::string> cmdIDs) override;

        void redo(const std::string& lastlog, std::deque<std::string> cmdIDs) override;
    };
}
