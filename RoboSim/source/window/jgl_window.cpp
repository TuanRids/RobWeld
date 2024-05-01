#include "pch.h"

#include "jgl_window.h"

#include "elems/input.h"

namespace nwindow
{
  bool GLWindow::init( const std::string& title)
  {
      int offsize = 50;
    if (!glfwInit()) {
        Width = 1920 - offsize;
        Height = 1080 - offsize;
        return false;
    }
    else 
    {
        const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
        if (!mode) {
            Width = 1920 - offsize;
            Height = 1080 - offsize;
            glfwTerminate();
        }
		else {
			Width = mode->width - offsize;
			Height = mode->height - offsize;
		}
    }

    Title = title;

    mRenderCtx->init(this);

    mUICtx->init(this);

    mSceneView = &nui::SceneView::getInstance();


    mPropertyPanel = std::make_unique<Property_Panel>();

    mSceneView->SetMeshLoadCallback(
      [this](std::string filepath) { mSceneView->load_mesh(filepath); });
    
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
    mSceneView->resize(Width, Height);
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

    std::cout<<"Window close event\n"<<std::endl;
    nelems::mMesh::getInstance().~mMesh();
    mIsRunning = false;
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
    
  }

  
}
