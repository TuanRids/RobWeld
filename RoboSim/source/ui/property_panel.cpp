#include "pch.h"
#include "property_panel.h"

namespace nui
{
  void Property_Panel::render(nui::SceneView* scene_view)
  {
    MenuBar();
    std::vector<long long> IDs;

    ImGui::Begin("Properties");
    ///
    /// Layer Management
    static long long prevSelectedID = 0;
    ImGui::CollapsingHeader("Layer", ImGuiTreeNodeFlags_DefaultOpen);

    if (scene_view->check_mesh())
    {
        scene_view->get_mesh_ids(IDs);
    }
    if (IDs.size() > 0)
    {
        
        ImGui::BeginTable("MeshTable", 2, ImGuiTableFlags_Borders);
        ImGui::TableSetupColumn("IDs");
		//ImGui::TableSetupColumn("Name");
        for (const auto& id : IDs)
        {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("%lld", id);

            // Add other columns dynamically here when needed

            if (ImGui::IsItemClicked())
            {
                selectedID = id;
            }
            if (id == selectedID)
            {
                ImU32 yellowColorU32 = ImGui::ColorConvertFloat4ToU32(ImVec4(0.25f, 0.42f, 1.0f, 1.0f));
                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, yellowColorU32);

            }

        }
        ImGui::EndTable();
	}
    ImGui::Separator();
    

    /// Main Properties

    auto sceneMesh = scene_view->get_mesh_scene();
    if (sceneMesh && selectedID != prevSelectedID && selectedID != 0)
    {
        sceneMesh->get_mesh_ptr(selectedID, mesh); 
    }
    prevSelectedID = selectedID;
    /// Material
    if (ImGui::CollapsingHeader("Material", ImGuiTreeNodeFlags_DefaultOpen))
    {
        if (prevSelectedID)
        {
            ImGui::PushItemFlag(ImGuiItemFlags_Disabled, false);
            ImGui::ColorEdit3("Color", (float*)&(mesh->oMaterial.mColor));
            ImGui::SliderFloat("Roughness", &mesh->oMaterial.roughness, 0.0f, 1.0f);
            ImGui::SliderFloat("Metallic", &mesh->oMaterial.metallic, 0.0f, 1.0f);
        }
        else {
            ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
            static ImVec4 disabledColor = ImVec4(0.5f, 0.5f, 0.5f, 1.0f);// gray out the widgets
            static float disabledFloat = 0.5f; // 0.5f is the default value
            ImGui::ColorEdit3("Color", (float*)&disabledColor);
            ImGui::SliderFloat("Roughness", &disabledFloat, 0.0f, 1.0f);
            ImGui::SliderFloat("Metallic", &disabledFloat, 0.0f, 1.0f);
        }
    }
    /// Light
    if (ImGui::CollapsingHeader("Light", ImGuiTreeNodeFlags_DefaultOpen))
    {
      ImGui::Separator();
      nimgui::draw_vec3_widget("Position", scene_view->get_light()->mPosition, 80.0f);
      ImGui::SliderInt("Light Intensity", &scene_view->get_light()->mStrength, 1, 1000);
    }
    /// iObject
    if (ImGui::CollapsingHeader("iObject", ImGuiTreeNodeFlags_DefaultOpen) && mesh)
    {

        ImGui::Separator();
        ImGui::Text("Name:          %s", mCurrentFile.c_str());
        ImGui::Text("ID:            %lld", mesh->ID);
        ImGui::Text("Indices:       %d", mesh->get_vertex_indices_size());
        ImGui::Text("Vertices:      %d", mesh->get_vertices_size());

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
      ofn.lpstrFilter = "FBX Files (*.fbx)\0*.fbx\0OBJ Files (*.obj)\0*.obj\0STL Files (*.stl)\0*.stl\0All Files (*.*)\0*.*\0";
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
            if (ImGui::MenuItem("Import"))
            {
                OpenFileDialog();
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
            if (ImGui::MenuItem("Collapse"))
            {
                showMenuBar = false;
            }
            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }
        

}