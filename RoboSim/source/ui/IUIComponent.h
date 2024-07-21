#pragma once
#include "pch.h"

#include "elems/camera.h"
#include "elems/mesh.h"
#include "elems/light.h"
#include "shader/shader_util.h"

namespace nui
{
    class IUIComponent
    {
    public:
        virtual ~IUIComponent() = default;

        virtual void render() = 0;
        virtual void resize(int32_t width, int32_t height) = 0;
        nelems::mMesh* rdMesh;
        std::shared_ptr<nelems::Camera> mCamera;
        std::shared_ptr<nrender::OpenGL_FrameBuffer> mFrameBuffer;
        std::shared_ptr<nshaders::Shader> mShader;
        std::shared_ptr<nelems::Light> mLight;
    };
}