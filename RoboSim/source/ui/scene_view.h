#pragma once

#include "render/opengl_buffer_manager.h"
#include "elems/input.h"
//#include "FrameManage.h"
#include "imgui.h"
#include "functional"
#include "IUIComponent.h"
namespace nui
{
    /*
    Use the Singleton pattern to create a SceneView
    MAYBE IT COULD BE USE FOR OTHERS CLASSES
    */
    class SceneView: public IUIComponent
    {
    private:
        glm::vec2 mSize;
        int zoom = 1;
        static std::string arg_render_mode;
        SceneView();

        std::function<void(const std::string&)> mMeshLoadCallback;
        std::shared_ptr<nrender::OpenGL_FrameBuffer> mFrameBuffer;

    public:
        static SceneView& getInstance() { static SceneView instance; return instance; }
        SceneView(const SceneView&) = delete;
        SceneView& operator=(const SceneView&) = delete;

        void render() override;
        void resize(int32_t width, int32_t height) override;

        ~SceneView() { mShader->unload(); rdMesh = nullptr; }

        void load_mesh(const std::string& filepath, bool robot);

        void on_mouse_move(double x, double y, nelems::EInputButton button);
        void on_mouse_wheel(double delta);
        void set_rotation_center();

        void reset_view() { mCamera->reset(); }

        void setlink_meshloadcallback(std::string& fpath) { mMeshLoadCallback(fpath); }
        void SetMeshLoadCallback(const std::function<void(const std::string&)>& callback) { mMeshLoadCallback = callback; }

        void render_mode();
        void set_render_mode(const std::string& mode) { arg_render_mode = mode; }
        void setZoom(int newZoom);
        void reset_camera();
    };
}
