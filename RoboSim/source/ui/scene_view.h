#pragma once

#include "elems/camera.h"
#include "elems/mesh.h"
#include "elems/light.h"
#include "shader/shader_util.h"
#include "render/opengl_buffer_manager.h"

#include "elems/input.h"
#include "FrameManage.h"
#include "imgui.h"
#include "functional"
#include "YMConnect/YMConnect.h"
namespace nui
{
    /*
    Use the Singleton pattern to create a SceneView 
    MAYBE IT COULD BE USE FOR OTHERS CLASSES
    */
    class SceneView
    {
    private:
        nelems::mMesh* rdMesh;
        std::unique_ptr<nelems::Camera> mCamera;
        std::unique_ptr<nrender::OpenGL_FrameBuffer> mFrameBuffer;
        std::unique_ptr<nshaders::Shader> mShader;
        std::unique_ptr<nelems::Light> mLight;
        std::shared_ptr<nui::SceneView> mSceneView;

        // camera parameters
        glm::vec2 mSize;
        float Fov = 45.0f;
        float Aspect = 1.3f;
        float Near = 0.1f;
        float Far = 100.0f;
        int zoom = 1;
        static std::string arg_render_mode;

        // create a file browser instance
        std::function<void(const std::string&)> mMeshLoadCallback;

        // Private constructor to prevent instantiation
        SceneView() :
            rdMesh(nullptr),mCamera(nullptr), mFrameBuffer(nullptr), mShader(nullptr),
            mLight(nullptr), mSize(800, 600),  mSceneView(nullptr)
        {
            mFrameBuffer = std::make_unique<nrender::OpenGL_FrameBuffer>();
            mFrameBuffer->create_buffers(800, 600);
            mShader = std::make_unique<nshaders::Shader>();
            mShader->load("shaders/vs.shader", "shaders/fs_pbr.shader");
            mLight = std::make_unique<nelems::Light>();
            mCamera = std::make_unique<nelems::Camera>(glm::vec3(0, 0, 3), Fov, Aspect, Near, Far);
        }

    public:
        // Static method to get the instance of SceneView
        static SceneView& getInstance(){ static SceneView instance;  return instance;    }

        // Prevent copy and assignment
        SceneView(const SceneView&) = delete;
        SceneView& operator=(const SceneView&) = delete;

        // Destructor to close the connection
        ~SceneView() {  mShader->unload(); rdMesh = nullptr; }

        //=======================================================================================================
        
        // Public methods
        nelems::Light* get_light() { return mLight.get(); }
        nshaders::Shader* get_shader() { return mShader.get(); }
        void resize(int32_t width, int32_t height);
        void render();
        void load_mesh(const std::string& filepath);
        std::shared_ptr<nui::SceneView> get_mesh_scene() { if (mSceneView) {return mSceneView;} else {return nullptr;}}

        //=======================================================================================================
        void on_mouse_move(double x, double y, nelems::EInputButton button);
        void on_mouse_wheel(double delta);
        void set_rotation_center();
        void reset_view() { mCamera->reset(); }
        void setlink_meshloadcallback(std::string& fpath) { mMeshLoadCallback(fpath); }
        void SetMeshLoadCallback(const std::function<void(const std::string&)>& callback){ mMeshLoadCallback = callback; }
        ///
        void render_mode();
        void set_render_mode(const std::string& mode) { arg_render_mode = mode; }
        //=======================================================================================================
        void setFov(float newFov);
        void setAspect(float newAspect);   // Setters Aspect
        void setNear(float newNear);  // Setters Near
        void setFar(float newFar);  // Setters Far
        void setZoom(int newZoom); // Setters Zoom



    };
}
