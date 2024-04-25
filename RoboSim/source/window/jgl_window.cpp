#include "pch.h"

#include "jgl_window.h"
#include "elems/input.h"
#include "Command/cmdManage.h"
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


    mPropertyPanel = std::make_unique<Property_Panel>(obHistory);

    mPropertyPanel->SetMeshLoadCallback(
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
    mSceneView->on_mouse_wheel(delta);
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

    // render scene to framebuffer and add it to scene view
    mSceneView->render();
    mPropertyPanel->render(mSceneView);

    // Render the UI 
    mUICtx->post_render();
    // Render end, swap buffers
    mRenderCtx->post_render();

    handle_input();
  }

  void GLWindow::handle_input()
  {
      
    // TODO: move this and camera to scene UI component?
    if (nui::FrameManage::getCrActiveGui("ViewPort") == true)
      {

          if (glfwGetKey(mWindow, GLFW_KEY_W) == GLFW_PRESS)
          {
              mSceneView->on_mouse_wheel(0.4f);
          }

          if (glfwGetKey(mWindow, GLFW_KEY_S) == GLFW_PRESS)
          {
              mSceneView->on_mouse_wheel(-0.4f);
          }

          if (glfwGetKey(mWindow, GLFW_KEY_F) == GLFW_PRESS)
          {
              mSceneView->reset_view();
          }

          double x, y;
          glfwGetCursorPos(mWindow, &x, &y);

          mSceneView->on_mouse_move(x, y, Input::GetPressedButton(mWindow));

      }
  }
}
