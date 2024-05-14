#include "pch.h"
#include "scene_view.h"
#include <Windows.h>
namespace nui
{
  std::string SceneView::arg_render_mode = "WireFrame";


  void SceneView::resize(int32_t width, int32_t height)
  {
      mSize.x = float(width); 
      mSize.y = float(height);
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
  void SceneView::set_rotation_center()
  {
      nelems::oMesh* selMesh;
      for (int i = 0; i < rdMesh->size(); i++)
      {
          rdMesh->get_mesh_ptr(i, selMesh);
          if (selMesh->selected)
          { mCamera->set_rotation_center(selMesh->oMaterial.position);
          return; }
      }
      // not return that mean there are non selected objects
      mCamera->set_rotation_center({ 0.0f,0.0f,0.0f });
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
      mFrameBuffer->bind();

      if (!rdMesh) {rdMesh = &nelems::mMesh::getInstance();}
      else {render_mode(); }

      mFrameBuffer->unbind();
      //TODO OBJECTS ARE SCALED WHEN SCREENE IS RESIZED
      ImGui::Begin("ViewPort");
      {
          nui::FrameManage::setCrActiveGui("ViewPort", ImGui::IsWindowFocused() || ImGui::IsWindowHovered());
          ImVec2 viewportPanelSize = ImGui::GetContentRegionAvail();
          mSize = { viewportPanelSize.x, viewportPanelSize.y };
          mCamera->set_aspect(mSize.x / mSize.y);
          mCamera->update(mShader.get());

          ImVec2 viewportWindowPos = ImGui::GetWindowPos();
          nui::FrameManage::setViewportSize(viewportWindowPos.x, viewportWindowPos.y);

          // add rendered texture to ImGUI scene window
          uint64_t textureID = mFrameBuffer->get_texture();
          ImGui::Image(reinterpret_cast<void*>(textureID), ImVec2{ mSize.x, mSize.y }, ImVec2{ 0, 1 }, ImVec2{ 1, 0 });
      }
      ImGui::End();
  }



  void SceneView::render_mode()
  {
      static bool lightmode = false;
      if (mLight->mStrength == 0) { lightmode = false; }
	  else { lightmode = true; }
      /* 
      "Points", "WireFrame", "Surface"
      ,"Points-Wire", "Points-Face", "Wire-Face", "Point-Wire-Face"
      */
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      glEnable(GL_BLEND);
      glEnable(GL_MULTISAMPLE);
      glHint(GL_LINE_SMOOTH_HINT, GL_DONT_CARE);
      glEnable(GL_LINE_SMOOTH);

      glPointSize(2.0f);
      if (arg_render_mode == "Points") {
          glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
          rdMesh->update(mShader.get(), lightmode);
      }
      else if (arg_render_mode == "WireFrame") {
          glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
          rdMesh->update(mShader.get(), lightmode);
      }
      else if (arg_render_mode == "Surface") {
          glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
          rdMesh->update(mShader.get(), lightmode);
      }
      else if (arg_render_mode == "Points-Wire") {
          glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
          rdMesh->update(mShader.get(), lightmode);
          glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
          rdMesh->update(mShader.get(), lightmode);
	  }
      else if (arg_render_mode == "Points-Face") {
		  glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
		  rdMesh->update(mShader.get(), lightmode);
		  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		  rdMesh->update(mShader.get(), lightmode);
	  }
      else if (arg_render_mode == "Wire-Face") {
		  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		  rdMesh->update(mShader.get(), lightmode);
		  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		  rdMesh->update(mShader.get(), lightmode);
	  }
      else if (arg_render_mode == "Point-Wire-Face") {
		  glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
		  rdMesh->update(mShader.get(), lightmode);
		  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		  rdMesh->update(mShader.get(), lightmode);
		  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		  rdMesh->update(mShader.get(), lightmode);
	  }
      

  }


  
}
