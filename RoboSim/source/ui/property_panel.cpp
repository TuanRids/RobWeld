#include "pch.h"
#include "property_panel.h"

namespace nui
{
  void Property_Panel::render(nui::SceneView* scene_view)
  {
    MenuBar();
    auto mesh = scene_view->get_mesh();

    ImGui::Begin("Properties");
    if (ImGui::CollapsingHeader("Object", ImGuiTreeNodeFlags_DefaultOpen))
    {
      
      if (ImGui::Button("Open..."))
      {
          OpenFileDialog();
      }
      ImGui::SameLine(0, 5.0f);
      ImGui::Text(mCurrentFile.c_str());
    }

    if (ImGui::CollapsingHeader("Material") && mesh)
    {
        ImGui::ColorPicker3("Color", (float*)&mesh->mColor, ImGuiColorEditFlags_PickerHueWheel | ImGuiColorEditFlags_DisplayRGB);
        ImGui::SliderFloat("Roughness", &mesh->mRoughness, 0.0f, 1.0f);
        ImGui::SliderFloat("Metallic", &mesh->mMetallic, 0.0f, 1.0f);
    }

    if (ImGui::CollapsingHeader("Light"))
    {

      ImGui::Separator();
      ImGui::Text("Position");
      ImGui::Separator();
      nimgui::draw_vec3_widget("Position", scene_view->get_light()->mPosition, 80.0f);
    }
    if (ImGui::CollapsingHeader("iObject") && mesh)
    {

        ImGui::Separator();
        ImGui::Text("Name: %s", mCurrentFile.c_str());
        ImGui::Text("Indices: %d", mesh->get_vertex_indices_size());
        ImGui::Text("Vertices: %d", mesh->get_vertices_size());
    }
    ImGui::End();

  }
  void Property_Panel::OpenFileDialog()
  {
      OPENFILENAME ofn;
      char szFile[260] = { 0 };

      ZeroMemory(&ofn, sizeof(ofn));
      ofn.lStructSize = sizeof(ofn);
      ofn.hwndOwner = NULL;
      ofn.lpstrFile = szFile;
      ofn.nMaxFile = sizeof(szFile);
      ofn.lpstrFilter = "FBX Files (*.fbx)\0*.fbx\0All Files (*.*)\0*.*\0";
      ofn.nFilterIndex = 1;
      ofn.lpstrFileTitle = NULL;
      ofn.nMaxFileTitle = 0;
      ofn.lpstrInitialDir = mCurrentFile.c_str();
      ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;

      if (GetOpenFileName(&ofn) == TRUE)
      {
          std::string filePath = ofn.lpstrFile;
          std::cout << filePath << std::endl;
          mCurrentFile = filePath.substr(filePath.find_last_of("/\\") + 1);

          mMeshLoadCallback(filePath);
      }
  }
}
void nui::Property_Panel::MenuBar()
{
    if (ImGui::BeginMainMenuBar())
    {
     if (ImGui::BeginMenu("File"))
     {
         if (ImGui::MenuItem("New"))
         {
          //NewScene();
         }
         if (ImGui::MenuItem("Open"))
         {
          //OpenScene();
         }
         if (ImGui::MenuItem("Save"))
         {
          //SaveScene();
         }
         if (ImGui::MenuItem("Save As"))
         {
          //SaveSceneAs();
         }
         ImGui::EndMenu();
     }
     if (ImGui::BeginMenu("Edit"))
     {
         if (ImGui::MenuItem("Undo"))
         {
          //Undo();
         }
         if (ImGui::MenuItem("Redo"))
         {
          //Redo();
         }
         ImGui::EndMenu();
     }
     ImGui::EndMainMenuBar();
    }
}