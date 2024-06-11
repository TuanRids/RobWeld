#include "pch.h"

#include "jgl_window.h"

#include "elems/input.h"

#define STB_IMAGE_IMPLEMENTATION
#include "elems/stb_image.h"
namespace nwindow
{
  bool GLWindow::init( const std::string& title)
  {
    int offsize = 0;
    if (!glfwInit()) {
        Width = 1920 - offsize;
        Height = 1080 - offsize-30;
        return false;
    }
    else 
    {
        const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
        if (!mode) {
            Width = 1920 - offsize;
            Height = 1080 - offsize - 30;
            glfwTerminate();
        }
		else {
			Width = mode->width - offsize;
			Height = mode->height - offsize - 30;
		}
    }
    Title = title;


    mRenderCtx->init(this);

    mUICtx->init(this);

    mSceneView = &nui::SceneView::getInstance();

    mPropertyPanel = std::make_unique<Property_Panel>();

    mSceneView->SetMeshLoadCallback(
      [this](std::string filepath) { mSceneView->load_mesh(filepath,0); });

    
    
    // MSAA
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 4); // 4x MSAA
    glEnable(GL_MULTISAMPLE);


    return mIsRunning;
  }

  GLWindow::~GLWindow()
  {
    mUICtx->end();
    mRenderCtx->end();
  }

  void GLWindow::on_resize(int width, int height)
  {
      Width = width;
      Height = height;

      // Bind the default framebuffer object (FBO) to ensure correct resizing
      glBindFramebuffer(GL_FRAMEBUFFER, 0);

      // Resize the viewport to match the new window size
      glViewport(0, 0, width, height);

      // Call render to redraw the scene with the new size
      render();
  }

  void GLWindow::on_scroll(double delta)
  {
      if (nui::FrameManage::getCrActiveGui("ViewPort") == true)
      {
          mSceneView->on_mouse_wheel(delta);
      }
  }

  void GLWindow::on_key(int key, int scancode, int action, int mods)
  {
    if (action == GLFW_PRESS)
    {
    }
  }
  //TODO CLOSE PROGRAM TODOCLOSE TODOEXIT
  void GLWindow::on_close()
  {
    nelems::mMesh::getInstance().~mMesh();
    mIsRunning = false;
  }

  void GLWindow::set_icon(const char* filename)
  {
      GLFWimage images[1];
      images[0].pixels = stbi_load(filename, &images[0].width, &images[0].height, 0, 4); //rgba channels 
      glfwSetWindowIcon(mWindow, 1, images);
      stbi_image_free(images[0].pixels);
  }

 

  void GLWindow::render()
  {
    
    // Clear the view
    mRenderCtx->pre_render();
    
    // Initialize UI components
    mUICtx->pre_render();

    
    //handle_input();
    // render scene to framebuffer and add it to scene view

    mSceneView->render();
    mPropertyPanel->render(mSceneView,mWindow);

    // Render the UI 
    mUICtx->post_render();
    // Render end, swap buffers
    mRenderCtx->post_render();
    
    if (!mLoadRobot)
    {
        mLoadRobot = std::make_unique<LoadRobot>();
        mLoadRobot->trigger_GP8();
    }
  }

  
}
