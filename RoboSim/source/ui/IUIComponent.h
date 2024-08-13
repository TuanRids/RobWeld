#pragma once
#include "pch.h"
#include "elems/camera.h"
#include "elems/mesh.h"
#include "elems/light.h"
#include "shader/shader_util.h"
#include <unordered_map>
#include <memory>
#include "Filemgr/RobInitFile.h"

namespace nui
{
    class IUIComponent
    {
    public:
        virtual ~IUIComponent() = default;

        virtual void render();
        virtual void resize(int32_t width, int32_t height);

        // Frame management methods
        virtual void setCrActiveGui(const std::string& crActiveGui, bool isActive);
        virtual void set3DSize(float& x, float& y); 
        virtual void SetViewPos(float& x, float& y);

        virtual bool getCrActiveGui(const std::string& frameName);
        virtual void get3DSize(float& x, float& y);
        virtual void GetViewPos(float& x, float& y);
        static std::unique_ptr<nshaders::Shader> mShader;

    protected:
        static nelems::mMesh* rdMesh;
        static std::unique_ptr<nelems::Camera> mCamera;
        static std::unique_ptr<nelems::Light> mLight;
        static RobInitFile* robinit;

        static std::string crActiveGui;
        static float viewport_x, viewport_y;
        static float vdsize_x, vdsize_y;
        std::unordered_map<std::string, bool> frameStatus;

        static std::map<std::string, float> Chartdata;

    };

}
