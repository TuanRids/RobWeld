#pragma once

#include "elems/camera.h"
#include "elems/mesh.h"
#include "elems/light.h"
#include "shader/shader_util.h"
#include "render/opengl_buffer_manager.h"
#include "elems/input.h"

namespace nui
{
  class SceneView
  {

  private:
      float Fov = 45.0f;
      float Aspect = 1.3f;
      float Near = 0.1f;
      float Far = 100.0f;
      int zoom = 1;
  public:
    SceneView() : 
        mCamera(nullptr), mFrameBuffer(nullptr), mShader(nullptr),
        mLight(nullptr), mSize(800, 600)
        {
            mFrameBuffer = std::make_unique<nrender::OpenGL_FrameBuffer>();
            mFrameBuffer->create_buffers(800, 600);
            mShader = std::make_unique<nshaders::Shader>();
            mShader->load("shaders/vs.shader", "shaders/fs_pbr.shader");
            mLight = std::make_unique<nelems::Light>();

            mCamera = std::make_unique<nelems::Camera>(glm::vec3(0, 0, 3), Fov, Aspect,Near,Far);
            
        }

    ~SceneView()
    {
      mShader->unload();
    }

    nelems::Light* get_light() { return mLight.get(); }

    void resize(int32_t width, int32_t height);


    void render();

    void load_mesh(const std::string& filepath);

    void set_mesh(std::shared_ptr<nelems::Mesh> mesh)
    {
      mMesh = mesh;
    }



    std::shared_ptr<nelems::Mesh> get_mesh() { return mMesh; }
    
    void on_mouse_move(double x, double y, nelems::EInputButton button);

    void on_mouse_wheel(double delta);

    //void on_mouse_click(double x, double y);


    void reset_view()
    {
      mCamera->reset();
    }
    
  private:
    int noMesh = 0;
    std::unique_ptr<nelems::Camera> mCamera;
    std::unique_ptr<nrender::OpenGL_FrameBuffer> mFrameBuffer;
    std::unique_ptr<nshaders::Shader> mShader;
    std::unique_ptr<nelems::Light> mLight;
    std::shared_ptr<nelems::Mesh> mMesh;
    glm::vec2 mSize;
  };
}

