#include "pch.h"
#include "scene_view.h"


namespace nui
{
  void SceneView::resize(int32_t width, int32_t height)
  {
    mSize.x = float(width);
    mSize.y = float(height);
    mCamera->set_aspect(mSize.x / mSize.y);
    mFrameBuffer->create_buffers((int32_t)mSize.x, (int32_t) mSize.y);
  }

  void SceneView::on_mouse_move(double x, double y, nelems::EInputButton button)
  {
    mCamera->on_mouse_move(x, y, button);
  }

  void SceneView::on_mouse_wheel(double delta)
  {
    mCamera->on_mouse_wheel(delta*double(zoom));
  }
  //================================================================================================
  void SceneView::setFov(float newFov)
  {
      Fov = newFov;
      mCamera->set_fov(newFov);
      mCamera->update_view_matrix();
      mCamera->update(mShader.get());
  }
  void SceneView::setFar(float newFar)
  {
      Far = newFar;
	  mCamera->set_far(newFar);
	  mCamera->update_view_matrix();
	  mCamera->update(mShader.get());
  }
  void SceneView::setNear(float newNear)
  {
	  Near = newNear;
	  mCamera->set_near(newNear);
	  mCamera->update_view_matrix();
	  mCamera->update(mShader.get());
  }
  void SceneView::setZoom(int newZoom)
  {
	  zoom = newZoom;
	  // mCamera->on_mouse_wheel(double(zoom));
  }
  void SceneView::setAspect(float newAspect)
  {
      Aspect = newAspect;
	  mCamera->set_aspect(newAspect);
	  mCamera->update_view_matrix();
	  mCamera->update(mShader.get());
  }
  //================================================================================================
  void SceneView::load_mesh(const std::string& filepath)
  {
    if(!rdMesh)
    {
        rdMesh = &nelems::mMesh::getInstance();
    }
    try
    {
        rdMesh->load(filepath);
    }
    catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}
  }

  void SceneView::render()
  {

    mShader->use();
    mLight->update(mShader.get());
    mFrameBuffer->bind();
    if (rdMesh)     { rdMesh->update(mShader.get());}
    else            { rdMesh = &nelems::mMesh::getInstance();}
    mFrameBuffer->unbind();

    ImGui::Begin("ViewPort");
    {
        nui::FrameManage::setCrActiveGui("ViewPort", ImGui::IsWindowFocused());
        ImVec2 viewportPanelSize = ImGui::GetContentRegionAvail();
        mSize = { viewportPanelSize.x, viewportPanelSize.y };
        mCamera->set_aspect(mSize.x / mSize.y);
        mCamera->update(mShader.get());
        // add rendered texture to ImGUI scene window
        uint64_t textureID = mFrameBuffer->get_texture();
        ImGui::Image(reinterpret_cast<void*>(textureID), ImVec2{ mSize.x, mSize.y }, ImVec2{ 0, 1 }, ImVec2{ 1, 0 });
    }
    ImGui::End();

  }
}
